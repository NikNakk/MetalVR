// The ALVR headset driver: a headset running the stock ALVR client, reached through ALVR's server_core C API.
//
// server_core owns discovery, the handshake, the sockets and the statistics; this file feeds it encoded video and
// turns its events into what metalvrd asks a Driver for. docs/ALVR.md describes the call order, the timestamps and
// what has not been verified against a real headset.
//
// Threads: ALVR's events are polled on one thread owned by this driver, which is also the only thread that
// initializes or shuts ALVR down after start(). tracking(), sendFrame() and the other Driver calls may come from
// any daemon thread.
#include "driver.h"
#include "headsets.h"

#include "alvr_server_core.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>

namespace metalvr {
namespace {

using std::string_view;

constexpr float kPi = 3.14159265358979323846f;
constexpr float radians(float degrees) { return degrees * kPi / 180.0f; }

// What OpenXR is told before any headset has connected to this config directory: a Quest 2 with ALVR's default
// 2144-wide views. Once a headset has connected, ALVR's session remembers what was negotiated and that is used.
constexpr uint32_t kDefaultEyeWidth = 2144;
constexpr uint32_t kDefaultEyeHeight = 2336;
constexpr float kDefaultRefreshHz = 72.0f;
constexpr float kDefaultIpd = 0.063f;
constexpr mvr_fov kDefaultFov[2] = {
    {radians(-52.0f), radians(45.0f), radians(48.0f), radians(-51.0f)},
    {radians(-45.0f), radians(52.0f), radians(48.0f), radians(-51.0f)},
};
constexpr float kDefaultHeadHeight = 1.6f;

// ALVR's own placeholder for "nothing negotiated yet" (alvr_session's SessionConfig::default)
constexpr uint32_t kPlaceholderEyeWidth = 800;
constexpr uint32_t kPlaceholderEyeHeight = 900;
constexpr uint32_t kPlaceholderRefreshHz = 60;

// ALVR's protocol carries one pose per hand, the OpenXR grip pose. The aim pose is that pose moved to the front
// of a Touch controller and pitched down to its pointing direction. Approximate, and not measured on a headset.
constexpr float kAimPitch = radians(-40.0f);
constexpr mvr_vec3 kAimOffset = {0.0f, -0.025f, -0.085f};

// a trigger or squeeze counts as pressed from its analog value when the headset reports no click for it
constexpr float kPressOn = 0.55f;
constexpr float kPressOff = 0.45f;

// tracking older than this means the headset is not showing the stream (taken off, paused, link stalled)
constexpr int64_t kTrackingFreshNs = 500'000'000;

// a frame's times are reported to ALVR as offsets into the past; older than this they say nothing
constexpr int64_t kOldestFrameNs = 1'000'000'000;

constexpr uint64_t kPollTimeoutNs = 10'000'000;
constexpr int64_t kEncoderParamsIntervalNs = 100'000'000;
constexpr int64_t kClientListIntervalNs = 1'000'000'000;
constexpr size_t kMostUntrustedReported = 16;

int64_t nowNs() { return static_cast<int64_t>(clock_gettime_nsec_np(CLOCK_UPTIME_RAW)); }

[[gnu::format(printf, 1, 2)]] void logLine(const char *format, ...) {
  char text[1024];
  va_list args;
  va_start(args, format);
  std::vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  std::fprintf(stderr, "[alvr] %s\n", text);
}

// --- poses ------------------------------------------------------------------------------------------------------

mvr_vec3 rotate(const mvr_quat &q, const mvr_vec3 &v) {
  // v + 2w(u x v) + 2u x (u x v), u the vector part of q
  const mvr_vec3 t = {2.0f * (q.y * v.z - q.z * v.y), 2.0f * (q.z * v.x - q.x * v.z), 2.0f * (q.x * v.y - q.y * v.x)};
  return {v.x + q.w * t.x + (q.y * t.z - q.z * t.y), v.y + q.w * t.y + (q.z * t.x - q.x * t.z),
          v.z + q.w * t.z + (q.x * t.y - q.y * t.x)};
}

mvr_quat multiply(const mvr_quat &a, const mvr_quat &b) {
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

// `local` expressed in the space `parent` is expressed in
mvr_pose compose(const mvr_pose &parent, const mvr_pose &local) {
  const mvr_vec3 offset = rotate(parent.orientation, local.position);
  return {multiply(parent.orientation, local.orientation),
          {parent.position.x + offset.x, parent.position.y + offset.y, parent.position.z + offset.z}};
}

mvr_pose toPose(const AlvrPose &pose) {
  return {{pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w},
          {pose.position[0], pose.position[1], pose.position[2]}};
}

mvr_vec3 toVec3(const float v[3]) { return {v[0], v[1], v[2]}; }

mvr_fov toFov(const AlvrFov &fov) { return {fov.left, fov.right, fov.up, fov.down}; }

bool sameFov(const mvr_fov &a, const mvr_fov &b) {
  constexpr float eps = 1e-4f;
  return std::fabs(a.left - b.left) < eps && std::fabs(a.right - b.right) < eps && std::fabs(a.up - b.up) < eps &&
         std::fabs(a.down - b.down) < eps;
}

bool usableFov(const mvr_fov &fov) {
  return std::isfinite(fov.left) && std::isfinite(fov.right) && std::isfinite(fov.up) && std::isfinite(fov.down) &&
         fov.left < fov.right && fov.down < fov.up;
}

// --- JSON -------------------------------------------------------------------------------------------------------
//
// ALVR keeps what this driver must set and read (settings, the negotiated stream) in JSON: session.json on disk
// and alvr_get_settings_json(). Values are located by object-key path in the text itself, so a setting can be
// replaced in session.json without disturbing the rest of the file. Keys are compared verbatim; none of the keys
// or values used here contain escapes.
namespace json {

size_t skipSpace(string_view s, size_t i) {
  while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) i++;
  return i;
}

// s[i] is the opening quote; leaves i after the closing one
bool skipString(string_view s, size_t &i) {
  for (i++; i < s.size(); i++) {
    if (s[i] == '\\') {
      i++;
    } else if (s[i] == '"') {
      i++;
      return true;
    }
  }
  return false;
}

bool skipValue(string_view s, size_t &i, int depth = 0) {
  if (i >= s.size() || depth > 64) return false;
  if (s[i] == '"') return skipString(s, i);
  if (s[i] == '{' || s[i] == '[') {
    const char close = s[i] == '{' ? '}' : ']';
    const bool object = s[i] == '{';
    i = skipSpace(s, i + 1);
    while (i < s.size() && s[i] != close) {
      if (object) {
        if (s[i] != '"' || !skipString(s, i)) return false;
        i = skipSpace(s, i);
        if (i >= s.size() || s[i] != ':') return false;
        i = skipSpace(s, i + 1);
      }
      if (!skipValue(s, i, depth + 1)) return false;
      i = skipSpace(s, i);
      if (i < s.size() && s[i] == ',') {
        i = skipSpace(s, i + 1);
      } else if (i >= s.size() || s[i] != close) {
        return false;
      }
    }
    if (i >= s.size()) return false;
    i++;
    return true;
  }
  const size_t begin = i;
  while (i < s.size() && s[i] != ',' && s[i] != '}' && s[i] != ']' && s[i] != ' ' && s[i] != '\t' && s[i] != '\r' &&
         s[i] != '\n')
    i++;
  return i > begin;
}

// the text of the value reached by following object keys from the root; empty if absent or the text is not JSON
string_view find(string_view s, std::span<const string_view> path) {
  auto key = path.begin();
  if (key == path.end()) return {};
  size_t i = skipSpace(s, 0);
  for (;;) {
    if (i >= s.size() || s[i] != '{') return {};
    i = skipSpace(s, i + 1);
    bool descended = false;
    while (i < s.size() && s[i] != '}') {
      if (s[i] != '"') return {};
      const size_t name_begin = i + 1;
      if (!skipString(s, i)) return {};
      const string_view name = s.substr(name_begin, i - 1 - name_begin);
      i = skipSpace(s, i);
      if (i >= s.size() || s[i] != ':') return {};
      i = skipSpace(s, i + 1);
      if (name == *key) {
        if (++key == path.end()) {
          const size_t begin = i;
          if (!skipValue(s, i)) return {};
          return s.substr(begin, i - begin);
        }
        descended = true;
        break;
      }
      if (!skipValue(s, i)) return {};
      i = skipSpace(s, i);
      if (i < s.size() && s[i] == ',') i = skipSpace(s, i + 1);
    }
    if (!descended) return {};
  }
}

string_view find(string_view s, std::initializer_list<string_view> path) {
  return find(s, std::span<const string_view>(path.begin(), path.size()));
}

std::optional<double> number(string_view raw) {
  if (raw.empty() || raw.size() > 63) return std::nullopt;
  char text[64];
  std::memcpy(text, raw.data(), raw.size());
  text[raw.size()] = 0;
  char *end = nullptr;
  const double value = std::strtod(text, &end);
  if (end != text + raw.size() || !std::isfinite(value)) return std::nullopt;
  return value;
}

std::optional<bool> boolean(string_view raw) {
  if (raw == "true") return true;
  if (raw == "false") return false;
  return std::nullopt;
}

// Whether two value texts mean the same: numbers by value (0 and 0.0), arrays of numbers element by element,
// anything else by its text.
bool same(string_view a, string_view b) {
  const auto x = number(a), y = number(b);
  if (x || y) return x && y && *x == *y;
  if (a.empty() || b.empty() || a.front() != '[' || b.front() != '[') return a == b;
  size_t i = skipSpace(a, 1), j = skipSpace(b, 1);
  for (;;) {
    const bool a_ends = i < a.size() && a[i] == ']', b_ends = j < b.size() && b[j] == ']';
    if (a_ends || b_ends) return a_ends && b_ends;
    const size_t a_begin = i, b_begin = j;
    if (!skipValue(a, i) || !skipValue(b, j)) return false;
    if (!same(a.substr(a_begin, i - a_begin), b.substr(b_begin, j - b_begin))) return false;
    i = skipSpace(a, i);
    j = skipSpace(b, j);
    if (i < a.size() && a[i] == ',') i = skipSpace(a, i + 1);
    if (j < b.size() && b[j] == ',') j = skipSpace(b, j + 1);
  }
}

// calls visit(name, value) with the text of each member of the object `object`; false if it is not an object
template <typename Visit> bool forEachMember(string_view object, Visit &&visit) {
  size_t i = skipSpace(object, 0);
  if (i >= object.size() || object[i] != '{') return false;
  i = skipSpace(object, i + 1);
  while (i < object.size() && object[i] != '}') {
    if (object[i] != '"') return false;
    const size_t name_begin = i + 1;
    if (!skipString(object, i)) return false;
    const string_view name = object.substr(name_begin, i - 1 - name_begin);
    i = skipSpace(object, i);
    if (i >= object.size() || object[i] != ':') return false;
    i = skipSpace(object, i + 1);
    const size_t value_begin = i;
    if (!skipValue(object, i)) return false;
    visit(name, object.substr(value_begin, i - value_begin));
    i = skipSpace(object, i);
    if (i < object.size() && object[i] == ',') i = skipSpace(object, i + 1);
  }
  return i < object.size();
}

// adds the member `"name":value` to the object at `path` (the document itself if the path is empty); false if
// there is no object there
bool insertMember(std::string &doc, std::span<const string_view> path, string_view name, string_view value) {
  const string_view object = path.empty() ? string_view(doc).substr(skipSpace(doc, 0)) : find(doc, path);
  if (object.empty() || object.front() != '{') return false;
  const size_t at = static_cast<size_t>(object.data() - doc.data()) + 1;
  const size_t next = skipSpace(doc, at);
  const bool has_members = next < doc.size() && doc[next] != '}';
  std::string member = "\"";
  member += name;
  member += "\":";
  member += value;
  if (has_members) member += ',';
  doc.insert(at, member);
  return true;
}

} // namespace json

std::optional<std::string> readFile(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) return std::nullopt;
  std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  if (file.bad()) return std::nullopt;
  return text;
}

bool writeFileAtomic(const std::filesystem::path &path, string_view text) {
  std::filesystem::path temp = path;
  temp += ".metalvr-tmp";
  {
    std::ofstream file(temp, std::ios::binary | std::ios::trunc);
    file.write(text.data(), static_cast<std::streamsize>(text.size()));
    file.flush();
    if (!file) return false;
  }
  std::error_code error;
  std::filesystem::rename(temp, path, error);
  return !error;
}

// --- ALVR settings MetalVR depends on ---------------------------------------------------------------------------
//
// metalvrd composes a plain side-by-side frame, encodes it as ordinary 8-bit video-range video, and returns the
// poses ALVR hands it unchanged. The ALVR settings that assume the SteamVR driver's own processing are therefore
// pinned, in session.json before ALVR loads it and checked again in the settings ALVR reports:
//   - foveated encoding off: the client would undo a distortion the frame never had
//   - 8-bit, no HDR, encoding gamma 1, and limited ("video") range whatever the client prefers: when full range is
//     negotiated the ALVR client rescales the decoded picture on the assumption that its decoder treated a
//     full-range stream as limited; with limited range it shows what a standard decoder outputs
//   - game audio and microphone off: not implemented; on macOS ALVR's capture of the default output device fails
//   - steamvr_pipeline_frames 0: the client would otherwise predict controllers that much short of the head and
//     leave the rest to SteamVR
//   - controller offsets zero: the defaults are tuned to SteamVR's controller models; zero is the OpenXR grip pose
//   - auto_trust_clients off: which headsets may connect is decided by the trusted list and nothing else
struct Pin {
  std::vector<string_view> session; // where it is in session.json, below "session_settings"
  string_view value;                // what it is set to there
  std::vector<string_view> setting; // where the same thing is in alvr_get_settings_json(); what is under an
                                    // "Enabled" there exists only while that switch is on
  string_view expect;               // how it reads there
};

// With --foveated-encoding the client is told to expect ALVR's foveated encoding, and metalvrd squeezes each eye
// to match (foveation.h): that pin turns on, and its shape is ALVR's own settings'.
bool g_foveated_encoding = false;

const std::vector<Pin> &pins() {
  static const std::vector<Pin> table = {
      {{"video", "foveated_encoding", "enabled"}, "false", {"video", "foveated_encoding"}, "\"Disabled\""},
      {{"video", "encoder_config", "use_10bit"}, "false", {"video", "encoder_config", "use_10bit"}, "false"},
      {{"video", "encoder_config", "server_overrides_use_10bit"},
       "true",
       {"video", "encoder_config", "server_overrides_use_10bit"},
       "true"},
      {{"video", "encoder_config", "use_full_range"}, "false", {"video", "encoder_config", "use_full_range"}, "false"},
      {{"video", "encoder_config", "server_overrides_use_full_range"},
       "true",
       {"video", "encoder_config", "server_overrides_use_full_range"},
       "true"},
      {{"video", "encoder_config", "encoding_gamma"}, "1.0", {"video", "encoder_config", "encoding_gamma"}, "1.0"},
      {{"video", "encoder_config", "server_overrides_encoding_gamma"},
       "true",
       {"video", "encoder_config", "server_overrides_encoding_gamma"},
       "true"},
      {{"video", "encoder_config", "hdr", "enable_hdr"},
       "false",
       {"video", "encoder_config", "hdr", "enable_hdr"},
       "false"},
      {{"video", "encoder_config", "hdr", "server_overrides_enable_hdr"},
       "true",
       {"video", "encoder_config", "hdr", "server_overrides_enable_hdr"},
       "true"},
      {{"audio", "game_audio", "enabled"}, "false", {"audio", "game_audio"}, "\"Disabled\""},
      {{"audio", "microphone", "enabled"}, "false", {"audio", "microphone"}, "\"Disabled\""},
      {{"headset", "controllers", "content", "steamvr_pipeline_frames"},
       "0.0",
       {"headset", "controllers", "Enabled", "steamvr_pipeline_frames"},
       "0.0"},
      {{"headset", "controllers", "content", "left_controller_position_offset", "content"},
       "[0.0, 0.0, 0.0]",
       {"headset", "controllers", "Enabled", "left_controller_position_offset"},
       "[0.0, 0.0, 0.0]"},
      {{"headset", "controllers", "content", "left_controller_rotation_offset", "content"},
       "[0.0, 0.0, 0.0]",
       {"headset", "controllers", "Enabled", "left_controller_rotation_offset"},
       "[0.0, 0.0, 0.0]"},
      {{"connection", "client_discovery", "content", "auto_trust_clients"},
       "false",
       {"connection", "client_discovery", "Enabled", "auto_trust_clients"},
       "false"},
  };
  // the same, foveated encoding on (its settings are an object then, which is not checked)
  static const std::vector<Pin> foveated = [] {
    std::vector<Pin> t = table;
    t[0] = {{"video", "foveated_encoding", "enabled"}, "true", {"video", "foveated_encoding"}, ""};
    return t;
  }();
  return g_foveated_encoding ? foveated : table;
}

std::string dotted(std::span<const string_view> path) {
  std::string text;
  for (const string_view key : path) {
    if (!text.empty()) text += '.';
    text += key;
  }
  return text;
}

// --- trusted headsets -------------------------------------------------------------------------------------------
//
// ALVR streams only to clients its session lists as trusted, and knows a client by the hostname it announces:
// "1234.client" from the stock client, which is also what the headset shows, or a "name.local." from a client found
// through mDNS. server_core's C API has no call for the list, and the list is read from session.json once per
// process, so the headsets to trust are written there before ALVR loads it. Nothing else trusts a client: a
// headset that announces itself unasked is only reported (see reportUntrusted).

// a hostname without what people and mDNS add to it: the trailing dot and ".local"
std::string bareHostname(string_view name) {
  if (!name.empty() && name.back() == '.') name.remove_suffix(1);
  constexpr string_view local = ".local";
  if (name.size() > local.size()) {
    const string_view tail = name.substr(name.size() - local.size());
    bool is_local = true;
    for (size_t i = 0; i < local.size(); i++)
      is_local = is_local && std::tolower(static_cast<unsigned char>(tail[i])) == local[i];
    if (is_local) name.remove_suffix(local.size());
  }
  return std::string(name);
}

bool sameHostname(string_view a, string_view b) {
  const std::string x = bareHostname(a), y = bareHostname(b);
  if (x.size() != y.size()) return false;
  for (size_t i = 0; i < x.size(); i++)
    if (std::tolower(static_cast<unsigned char>(x[i])) != std::tolower(static_cast<unsigned char>(y[i]))) return false;
  return true;
}

// it becomes a key in session.json, so nothing that needs escaping there
bool validHostname(string_view name) {
  const std::string bare = bareHostname(name);
  if (bare.empty() || bare.size() > 253) return false;
  for (const char c : bare)
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '.' && c != '-' && c != '_') return false;
  return true;
}

// The names ALVR may come to know each headset by, whichever way it finds it and however the name was typed.
std::vector<std::string> trustKeys(const std::vector<std::string> &hostnames) {
  std::vector<std::string> keys;
  const auto add = [&](std::string key) {
    for (const std::string &have : keys)
      if (have == key) return;
    keys.push_back(std::move(key));
  };
  for (const std::string &hostname : hostnames) {
    add(bareHostname(hostname));
    add(bareHostname(hostname) + ".local.");
  }
  return keys;
}

// an entry of ALVR's client list (alvr_session's ClientConnectionConfig) for a headset that may connect
constexpr string_view kTrustedClient =
    R"({"display_name":"Unknown","current_ip":null,"manual_ips":[],"trusted":true,"connection_state":"Disconnected"})";

struct ClientList {
  struct Client {
    std::string hostname;
    std::string display_name; // what the client calls its headset ("Quest 3"), "Unknown" until it has said
    std::string state;        // alvr_common's ConnectionState: "Disconnected", "Connecting", "Streaming", ...
  };
  std::vector<std::string> trusted;
  std::vector<std::string> untrusted;
  std::vector<Client> trusted_clients;
};

// a JSON string's text without its quotes; empty if it is not a string or holds escapes
std::string plainString(string_view raw) {
  if (raw.size() < 2 || raw.front() != '"' || raw.back() != '"') return {};
  raw = raw.substr(1, raw.size() - 2);
  if (raw.find('\\') != string_view::npos) return {};
  return std::string(raw);
}

std::optional<ClientList> readClientList(const std::filesystem::path &session_path) {
  const auto text = readFile(session_path);
  if (!text) return std::nullopt;
  ClientList list;
  const string_view clients = json::find(*text, {"client_connections"});
  const bool ok = json::forEachMember(clients, [&](string_view name, string_view client) {
    const bool trusted = json::boolean(json::find(client, {"trusted"})).value_or(false);
    (trusted ? list.trusted : list.untrusted).emplace_back(name);
    if (trusted)
      list.trusted_clients.push_back({std::string(name), plainString(json::find(client, {"display_name"})),
                                      plainString(json::find(client, {"connection_state"}))});
  });
  if (!ok) return std::nullopt;
  return list;
}

// ALVR's name for a headset on USB, reached through adb (alvr_sockets' WIRED_CLIENT_HOSTNAME)
constexpr string_view kWiredClient = "client.wired";

bool isExecutable(const std::filesystem::path &path) {
  struct stat st {};
  return ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode) && ::access(path.c_str(), X_OK) == 0;
}

// the adb on PATH, as ALVR would find it; empty if there is none
std::filesystem::path adbOnPath() {
  const char *path = std::getenv("PATH");
  if (!path) return {};
  string_view rest = path;
  while (!rest.empty()) {
    const size_t colon = rest.find(':');
    const string_view dir = rest.substr(0, colon);
    if (!dir.empty()) {
      const std::filesystem::path candidate = std::filesystem::path(dir) / "adb";
      if (isExecutable(candidate)) return candidate;
    }
    if (colon == string_view::npos) break;
    rest.remove_prefix(colon + 1);
  }
  return {};
}

// Writes the object holding entries[begin, end), which share their first `depth` keys and are grouped by key.
struct SeedEntry {
  std::vector<string_view> path;
  string_view value;
};

void appendObject(std::string &out, const std::vector<SeedEntry> &entries, size_t begin, size_t end, size_t depth) {
  out += '{';
  for (size_t i = begin; i < end;) {
    size_t group_end = i + 1;
    while (group_end < end && entries[group_end].path[depth] == entries[i].path[depth]) group_end++;
    if (i != begin) out += ',';
    out += '"';
    out += entries[i].path[depth];
    out += "\":";
    if (entries[i].path.size() == depth + 1) {
      out += entries[i].value;
    } else {
      appendObject(out, entries, i, group_end, depth + 1);
    }
    i = group_end;
  }
  out += '}';
}

// A session that does not exist yet: the pins, the trusted headsets, and HEVC instead of ALVR's default H.264
// (ALVR's default frame for a Quest is 4288 pixels wide, more macroblocks than H.264 allows below level 6). ALVR
// fills in everything else when it loads the file.
std::string sessionSeed(const std::vector<std::string> &trust_keys, uint32_t headset) {
  std::vector<SeedEntry> entries;
  entries.push_back({{"session_settings", "video", "preferred_codec", "variant"}, "\"Hevc\""});
  // a headset named on the command line asks for its usual PC VR rate; ALVR's settings decide it from then on
  char fps[16] = {};
  if (headsetSpec(headset).stream_refresh > 0) {
    std::snprintf(fps, sizeof(fps), "%.1f", headsetSpec(headset).stream_refresh);
    entries.push_back({{"session_settings", "video", "preferred_fps"}, fps});
  }
  for (const Pin &pin : pins()) {
    SeedEntry entry{{"session_settings"}, pin.value};
    entry.path.insert(entry.path.end(), pin.session.begin(), pin.session.end());
    entries.push_back(std::move(entry));
  }
  for (const std::string &key : trust_keys) entries.push_back({{"client_connections", key}, kTrustedClient});
  std::string seed;
  appendObject(seed, entries, 0, entries.size(), 0);
  seed += '\n';
  return seed;
}

// Runs before ALVR loads the session, which it does once per process.
bool prepareSession(const std::filesystem::path &session_path, const std::vector<std::string> &trust_keys,
                    uint32_t headset) {
  auto text = readFile(session_path);
  if (!text || text->find_first_not_of(" \t\r\n") == std::string::npos) {
    if (!writeFileAtomic(session_path, sessionSeed(trust_keys, headset))) {
      logLine("cannot write %s", session_path.c_str());
      return false;
    }
    return true;
  }

  std::string reset;
  for (const Pin &pin : pins()) {
    std::vector<string_view> path = {"session_settings"};
    path.insert(path.end(), pin.session.begin(), pin.session.end());
    // a pin the file does not have is left to the check after ALVR has loaded it
    const string_view old = json::find(*text, path);
    if (old.empty() || json::same(old, pin.value)) continue;
    text->replace(static_cast<size_t>(old.data() - text->data()), old.size(), pin.value);
    reset += ' ' + dotted(pin.session);
  }

  // as above, whatever cannot be written here is caught by the check of the list ALVR ends up with
  bool trusted = false;
  for (const std::string &key : trust_keys) {
    if (const string_view old = json::find(*text, {"client_connections", key, "trusted"}); !old.empty()) {
      if (old == "true") continue;
      text->replace(static_cast<size_t>(old.data() - text->data()), old.size(), "true");
      trusted = true;
    } else if (json::find(*text, {"client_connections", key}).empty()) {
      if (json::find(*text, {"client_connections"}).empty()) json::insertMember(*text, {}, "client_connections", "{}");
      const string_view list[] = {"client_connections"};
      trusted |= json::insertMember(*text, list, key, kTrustedClient);
    }
  }

  if (!reset.empty() || trusted) {
    if (!writeFileAtomic(session_path, *text)) {
      logLine("cannot update %s", session_path.c_str());
      return false;
    }
    if (!reset.empty()) logLine("reset ALVR settings MetalVR cannot honor (docs/ALVR.md):%s", reset.c_str());
  }
  return true;
}

// the parts of ALVR's effective settings this driver reads, from alvr_get_settings_json()
struct Settings {
  Codec codec = Codec::HEVC;
  bool av1 = false;
  float preferred_fps = kDefaultRefreshHz;
  bool controllers_enabled = false;
  int64_t minimum_idr_interval_ns = 100'000'000;
  std::string problems; // pins that do not hold; empty if all do
};

std::optional<Settings> readSettings() {
  const uint64_t size = alvr_get_settings_json(nullptr);
  if (size < 2) return std::nullopt;
  std::vector<char> buffer(size);
  alvr_get_settings_json(buffer.data());
  const string_view text(buffer.data(), size - 1);

  Settings settings;
  const string_view codec = json::find(text, {"video", "preferred_codec"});
  if (codec == "\"H264\"") {
    settings.codec = Codec::H264;
  } else if (codec == "\"Hevc\"") {
    settings.codec = Codec::HEVC;
  } else if (codec == "\"AV1\"") {
    settings.codec = Codec::HEVC;
    settings.av1 = true;
  } else {
    return std::nullopt;
  }
  if (const auto fps = json::number(json::find(text, {"video", "preferred_fps"})); fps && *fps >= 1.0)
    settings.preferred_fps = static_cast<float>(*fps);
  if (const auto ms = json::number(json::find(text, {"connection", "minimum_idr_interval_ms"})); ms && *ms >= 0.0)
    settings.minimum_idr_interval_ns = static_cast<int64_t>(*ms * 1e6);

  const string_view controllers = json::find(text, {"headset", "controllers"});
  if (controllers.empty()) return std::nullopt;
  settings.controllers_enabled = controllers != "\"Disabled\"";

  for (const Pin &pin : pins()) {
    bool switched_off = false;
    for (size_t i = 0; i < pin.setting.size() && !switched_off; i++)
      switched_off = pin.setting[i] == "Enabled" &&
                     json::find(text, std::span(pin.setting).first(i)) == "\"Disabled\"";
    if (switched_off || pin.expect.empty()) continue;
    if (!json::same(json::find(text, pin.setting), pin.expect)) settings.problems += ' ' + dotted(pin.session);
  }
  return settings;
}

// what the stream to the headset looks like: negotiated by ALVR when a client connects and kept in the
// openvr_config section of session.json, which is the only place server_core publishes all of it
struct StreamConfig {
  uint32_t stream_width = kDefaultEyeWidth; // per eye, as encoded
  uint32_t stream_height = kDefaultEyeHeight;
  uint32_t render_width = kDefaultEyeWidth; // per eye, as recommended to applications
  uint32_t render_height = kDefaultEyeHeight;
  float refresh_hz = kDefaultRefreshHz;
  Codec codec = Codec::HEVC;
  bool av1 = false;
  bool full_range = true;
  bool ten_bit = false;
  bool hdr = false;
  bool foveated = false;
  float foveation[6] = {}; // center size x, y, center shift x, y, edge ratio x, y
  float gamma = 1.0f;
};

std::optional<StreamConfig> readStreamConfig(const std::filesystem::path &session_path) {
  const auto text = readFile(session_path);
  if (!text) return std::nullopt;
  const string_view config = json::find(*text, {"openvr_config"});
  if (config.empty()) return std::nullopt;

  const auto integer = [&](string_view key) -> std::optional<uint32_t> {
    const auto value = json::number(json::find(config, {key}));
    if (!value || *value < 0.0 || *value > 65535.0) return std::nullopt;
    return static_cast<uint32_t>(*value);
  };
  const auto stream_width = integer("eye_resolution_width"), stream_height = integer("eye_resolution_height");
  const auto render_width = integer("target_eye_resolution_width"),
             render_height = integer("target_eye_resolution_height");
  const auto refresh = integer("refresh_rate"), codec = integer("codec");
  if (!stream_width || !stream_height || !render_width || !render_height || !refresh || !codec) return std::nullopt;
  if (*stream_width == 0 || *stream_height == 0 || *render_width == 0 || *render_height == 0 || *refresh == 0)
    return std::nullopt;

  StreamConfig stream;
  stream.stream_width = *stream_width;
  stream.stream_height = *stream_height;
  stream.render_width = *render_width;
  stream.render_height = *render_height;
  stream.refresh_hz = static_cast<float>(*refresh);
  stream.codec = *codec == 0 ? Codec::H264 : Codec::HEVC; // alvr_session's CodecType: H264, Hevc, AV1
  stream.av1 = *codec == 2;
  stream.full_range = json::boolean(json::find(config, {"use_full_range_encoding"})).value_or(true);
  stream.ten_bit = json::boolean(json::find(config, {"use_10bit_encoder"})).value_or(false);
  stream.hdr = json::boolean(json::find(config, {"enable_hdr"})).value_or(false);
  stream.foveated = json::boolean(json::find(config, {"enable_foveated_encoding"})).value_or(false);
  static const string_view foveation_keys[6] = {"foveation_center_size_x",  "foveation_center_size_y",
                                                "foveation_center_shift_x", "foveation_center_shift_y",
                                                "foveation_edge_ratio_x",   "foveation_edge_ratio_y"};
  for (int i = 0; i < 6; i++)
    stream.foveation[i] = static_cast<float>(json::number(json::find(config, {foveation_keys[i]})).value_or(0.0));
  stream.gamma = static_cast<float>(json::number(json::find(config, {"encoding_gamma"})).value_or(1.0));
  return stream;
}

bool isPlaceholder(const StreamConfig &stream) {
  return stream.stream_width == kPlaceholderEyeWidth && stream.stream_height == kPlaceholderEyeHeight &&
         stream.refresh_hz == static_cast<float>(kPlaceholderRefreshHz);
}

// --- video ------------------------------------------------------------------------------------------------------

// The stock ALVR server sends parameter sets only through the decoder config and strips them, and access unit
// delimiters, from the front of the frames it sends. Returns how many leading bytes of `data` are such NAL units.
size_t leadingConfigBytes(Codec codec, const uint8_t *data, size_t size) {
  const auto startCode = [&](size_t at) -> size_t { // length of the start code at `at`, 0 if none
    if (at + 3 <= size && data[at] == 0 && data[at + 1] == 0 && data[at + 2] == 1) return 3;
    if (at + 4 <= size && data[at] == 0 && data[at + 1] == 0 && data[at + 2] == 0 && data[at + 3] == 1) return 4;
    return 0;
  };
  size_t at = 0;
  for (;;) {
    const size_t prefix = startCode(at);
    if (prefix == 0 || at + prefix >= size) return at;
    const uint8_t header = data[at + prefix];
    const unsigned type = codec == Codec::H264 ? header & 0x1Fu : (header >> 1) & 0x3Fu;
    const bool config = codec == Codec::H264 ? (type >= 7 && type <= 9) : (type >= 32 && type <= 35);
    if (!config) return at;
    size_t next = at + prefix;
    while (next < size && startCode(next) == 0) next++;
    if (next >= size) return 0; // nothing but configuration: leave the frame alone
    at = next;
  }
}

// --- input ------------------------------------------------------------------------------------------------------

enum class InputKind { Click, Touch, TriggerClick, SqueezeClick, TriggerValue, SqueezeValue, ThumbstickX, ThumbstickY };

struct InputBinding {
  uint32_t hand;
  InputKind kind;
  uint32_t bit; // mvr_button, for Click and Touch
};

bool isScalar(InputKind kind) {
  return kind == InputKind::TriggerValue || kind == InputKind::SqueezeValue || kind == InputKind::ThumbstickX ||
         kind == InputKind::ThumbstickY;
}

struct ControllerInput {
  uint32_t clicks = 0;
  uint32_t touches = 0;
  float trigger = 0.0f;
  float squeeze = 0.0f;
  float thumbstick_x = 0.0f;
  float thumbstick_y = 0.0f;
  bool trigger_down = false; // from the analog value
  bool squeeze_down = false;
  bool trigger_click = false; // from a click input, when the headset has one
  bool squeeze_click = false;
  bool has_trigger_click = false;
  bool has_squeeze_click = false;
};

// Button paths of the Touch family (ALVR's Quest 2, Quest 3, Quest Pro and Rift S emulation modes), which is what
// the client's buttons are mapped to before ALVR reports them. A/B and X/Y are accepted on either hand.
std::unordered_map<uint64_t, InputBinding> makeInputBindings() {
  struct Suffix {
    const char *path;
    InputKind kind;
    uint32_t bit;
  };
  static constexpr Suffix suffixes[] = {
      {"a/click", InputKind::Click, MVR_BUTTON_PRIMARY},
      {"x/click", InputKind::Click, MVR_BUTTON_PRIMARY},
      {"b/click", InputKind::Click, MVR_BUTTON_SECONDARY},
      {"y/click", InputKind::Click, MVR_BUTTON_SECONDARY},
      {"menu/click", InputKind::Click, MVR_BUTTON_MENU},
      {"system/click", InputKind::Click, MVR_BUTTON_SYSTEM},
      {"thumbstick/click", InputKind::Click, MVR_BUTTON_THUMBSTICK},
      {"a/touch", InputKind::Touch, MVR_BUTTON_PRIMARY},
      {"x/touch", InputKind::Touch, MVR_BUTTON_PRIMARY},
      {"b/touch", InputKind::Touch, MVR_BUTTON_SECONDARY},
      {"y/touch", InputKind::Touch, MVR_BUTTON_SECONDARY},
      {"system/touch", InputKind::Touch, MVR_BUTTON_SYSTEM},
      {"trigger/touch", InputKind::Touch, MVR_BUTTON_TRIGGER},
      {"squeeze/touch", InputKind::Touch, MVR_BUTTON_SQUEEZE},
      {"thumbstick/touch", InputKind::Touch, MVR_BUTTON_THUMBSTICK},
      {"thumbrest/touch", InputKind::Touch, MVR_BUTTON_THUMBREST},
      {"trigger/click", InputKind::TriggerClick, 0},
      {"squeeze/click", InputKind::SqueezeClick, 0},
      {"trigger/value", InputKind::TriggerValue, 0},
      {"squeeze/value", InputKind::SqueezeValue, 0},
      {"thumbstick/x", InputKind::ThumbstickX, 0},
      {"thumbstick/y", InputKind::ThumbstickY, 0},
  };
  std::unordered_map<uint64_t, InputBinding> bindings;
  for (uint32_t hand = 0; hand < 2; hand++) {
    for (const Suffix &suffix : suffixes) {
      const std::string path =
          std::string(hand == 0 ? "/user/hand/left/input/" : "/user/hand/right/input/") + suffix.path;
      bindings.emplace(alvr_path_to_id(path.c_str()), InputBinding{hand, suffix.kind, suffix.bit});
    }
  }
  return bindings;
}

// one tracking packet from the headset, as ALVR returns it for the packet's timestamp
struct Sample {
  uint64_t timestamp_ns = 0; // ALVR's name for the packet; 0 means none yet
  int64_t received_ns = 0;   // daemon clock
  AlvrDeviceMotion head{};
  bool hand_valid[2] = {false, false};
  AlvrDeviceMotion hand[2]{};
  bool skeleton_valid[2] = {false, false}; // the client tracks that bare hand (no controller in it)
  AlvrPose skeleton[2][MVR_HAND_JOINTS]{};
};

// ALVR's server_core is a set of process-wide singletons: one environment (config directory, logger) for the life
// of the process and one running instance at a time.
std::mutex g_process_mutex;
bool g_environment_ready = false;
std::string g_environment_dir;
bool g_instance_running = false;

class AlvrDriver final : public Driver {
public:
  explicit AlvrDriver(const AlvrOptions &options) : options_(options) {
    system_.eye_width = kDefaultEyeWidth;
    system_.eye_height = kDefaultEyeHeight;
    system_.refresh_hz = kDefaultRefreshHz;
    system_.ipd = kDefaultIpd;
    system_.fov[0] = kDefaultFov[0];
    system_.fov[1] = kDefaultFov[1];
    setViewOffsets(kDefaultIpd);
    setModel(options_.headset, "named on the command line");
    if (options_.headset != MVR_HEADSET_UNKNOWN) {
      const HeadsetSpec &spec = headsetSpec(options_.headset);
      system_.eye_width = spec.eye_width;
      system_.eye_height = spec.eye_height;
      system_.refresh_hz = spec.default_refresh;
      system_.fov[0] = spec.fov[0];
      system_.fov[1] = spec.fov[1];
      video_.frame_width = spec.eye_width * 2;
      video_.frame_height = spec.eye_height;
      video_.fps = spec.default_refresh;
    }
  }

  ~AlvrDriver() override { stop(); }

  const char *name() const override { return "alvr"; }

  bool start() override {
    if (started_) return true;
    if (options_.config_dir.empty()) {
      logLine("no config directory given");
      return false;
    }
    std::error_code error;
    std::filesystem::create_directories(options_.config_dir, error);
    const std::filesystem::path dir = std::filesystem::absolute(options_.config_dir, error);
    if (error || !std::filesystem::is_directory(dir)) {
      logLine("cannot use config directory %s", options_.config_dir.c_str());
      return false;
    }
    session_path_ = dir / "session.json";
    for (const std::string &hostname : options_.trusted_clients) {
      if (validHostname(hostname)) continue;
      logLine("'%s' is not a hostname a headset can be trusted by", hostname.c_str());
      return false;
    }
    if (!prepareWired()) return false;
    g_foveated_encoding = options_.foveated_encoding;

    {
      std::lock_guard lock(g_process_mutex);
      if (g_instance_running) {
        logLine("ALVR is already running in this process");
        return false;
      }
      if (!g_environment_ready) {
        if (!prepareSession(session_path_, trustKeys(options_.trusted_clients), options_.headset)) return false;
        const std::string session_log = (dir / "session_log.txt").string();
        const std::string crash_log = (dir / "crash_log.txt").string();
        alvr_initialize_environment(dir.c_str(), dir.c_str());
        alvr_initialize_logging(session_log.c_str(), crash_log.c_str());
        g_environment_ready = true;
        g_environment_dir = dir.string();
      } else if (g_environment_dir != dir.string()) {
        logLine("ALVR was already initialized with %s in this process and cannot move to %s",
                g_environment_dir.c_str(), dir.c_str());
        return false;
      }
      g_instance_running = true;
    }

    if (bindings_.empty()) {
      bindings_ = makeInputBindings();
      head_id_ = alvr_path_to_id("/user/head");
      hand_id_[0] = alvr_path_to_id("/user/hand/left");
      hand_id_[1] = alvr_path_to_id("/user/hand/right");
    }

    if (!bringUp()) {
      std::lock_guard lock(g_process_mutex);
      g_instance_running = false;
      return false;
    }
    stop_ = false;
    thread_ = std::thread([this] { run(); });
    started_ = true;
    return true;
  }

  void stop() override {
    if (!started_) return;
    stop_ = true;
    thread_.join();
    bringDown();
    {
      std::lock_guard lock(g_process_mutex);
      g_instance_running = false;
    }
    started_ = false;
  }

  bool connected() override { return connected_; }

  mvr_system_info systemInfo() override {
    std::lock_guard lock(state_mutex_);
    mvr_system_info info = system_;
    info.flags = connected_ ? MVR_SYSTEM_HMD_CONNECTED : 0;
    // every Quest tracks bare hands, and ALVR passes their joints on; a Steam Frame has no hand tracking
    if (system_.headset != MVR_HEADSET_STEAM_FRAME) info.flags |= MVR_SYSTEM_HAND_TRACKING;
    return info;
  }

  uint64_t generation() override { return generation_; }

  void tracking(int64_t target_time_ns, mvr_tracking_response &out) override {
    const uint32_t result = out.result;
    out = {};
    out.result = result;

    const int64_t now = nowNs();
    const bool connected = connected_;
    std::lock_guard lock(state_mutex_);

    // ALVR's client predicts every pose for the moment it expects a frame drawn from this packet to be displayed,
    // so the poses are returned as they are, whatever time was asked for.
    out.time_ns = target_time_ns > 0 ? target_time_ns : now;
    out.sample_id = sample_.timestamp_ns;

    const bool have_sample = sample_.timestamp_ns != 0;
    if (connected) out.flags |= MVR_TRACKING_HMD_CONNECTED;
    if (have_sample) out.flags |= MVR_TRACKING_HEAD_VALID;
    if (connected && have_sample && now - sample_.received_ns < kTrackingFreshNs) out.flags |= MVR_TRACKING_PRESENTING;

    if (connected && battery_[0].valid) {
      out.flags |= MVR_TRACKING_BATTERY_VALID | (battery_[0].plugged ? MVR_TRACKING_CHARGING : 0u);
      out.head_battery = battery_[0].level;
    }
    out.head = have_sample ? toPose(sample_.head.pose) : mvr_pose{{0, 0, 0, 1}, {0, kDefaultHeadHeight, 0}};
    // ALVR's client sends the head pose already predicted and without velocities
    out.head_linear_velocity = have_sample ? toVec3(sample_.head.linear_velocity) : mvr_vec3{};
    out.head_angular_velocity =
        have_sample ? rotate(out.head.orientation, toVec3(sample_.head.angular_velocity)) : mvr_vec3{};
    for (int eye = 0; eye < 2; eye++) {
      out.views[eye].pose = compose(out.head, view_offset_[eye]);
      out.views[eye].fov = system_.fov[eye];
    }

    for (int hand = 0; hand < 2; hand++) {
      mvr_controller_state &controller = out.controllers[hand];
      const ControllerInput &input = input_[hand];
      const bool trigger = input.has_trigger_click ? input.trigger_click : input.trigger_down;
      const bool squeeze = input.has_squeeze_click ? input.squeeze_click : input.squeeze_down;
      controller.buttons = input.clicks | (trigger ? MVR_BUTTON_TRIGGER : 0u) | (squeeze ? MVR_BUTTON_SQUEEZE : 0u);
      controller.touches = input.touches;
      controller.trigger = input.trigger;
      controller.squeeze = input.squeeze;
      controller.thumbstick_x = input.thumbstick_x;
      controller.thumbstick_y = input.thumbstick_y;
      controller.grip = {{0, 0, 0, 1}, {0, 0, 0}};
      controller.aim = controller.grip;
      if (connected && battery_[1 + hand].valid) {
        controller.flags |= MVR_CONTROLLER_BATTERY_VALID | (battery_[1 + hand].plugged ? MVR_CONTROLLER_CHARGING : 0u);
        controller.battery = battery_[1 + hand].level;
      }
      if (!connected || !have_sample || !sample_.hand_valid[hand]) continue;

      const AlvrDeviceMotion &motion = sample_.hand[hand];
      controller.flags |= MVR_CONTROLLER_CONNECTED | MVR_CONTROLLER_POSE_VALID;
      controller.grip = toPose(motion.pose);
      controller.aim = compose(
          controller.grip, {{std::sin(kAimPitch * 0.5f), 0.0f, 0.0f, std::cos(kAimPitch * 0.5f)}, kAimOffset});
      controller.linear_velocity = toVec3(motion.linear_velocity);
      // ALVR turns a controller's angular velocity into the controller's own frame (what SteamVR wants) when
      // its controller settings are enabled; OpenXR wants it in the tracking space
      const mvr_vec3 angular = toVec3(motion.angular_velocity);
      controller.angular_velocity = controllers_enabled_ ? rotate(controller.grip.orientation, angular) : angular;
    }
  }

  // the hands the client tracks, as it predicted them for the newest packet, like the head
  void hands(int64_t, mvr_hand out[2]) override {
    std::lock_guard lock(state_mutex_);
    const bool fresh = connected_ && sample_.timestamp_ns != 0 && nowNs() - sample_.received_ns < kTrackingFreshNs;
    for (int hand = 0; hand < 2; hand++) {
      out[hand].flags = 0;
      if (!fresh || !sample_.skeleton_valid[hand]) continue;
      for (uint32_t j = 0; j < MVR_HAND_JOINTS; j++) {
        out[hand].joints[j] = toPose(sample_.skeleton[hand][j]);
        out[hand].radii[j] = handJointRadius(j);
      }
      out[hand].flags = MVR_HAND_TRACKED;
    }
  }

  void haptic(uint32_t hand, float duration_s, float frequency_hz, float amplitude) override {
    if (!ready_ || !connected_ || hand > 1) return;
    if (!std::isfinite(duration_s) || !std::isfinite(frequency_hz) || !std::isfinite(amplitude)) return;
    // ALVR has no stop; a silent pulse of its minimum duration replaces whatever is playing
    if (duration_s <= 0.0f) {
      duration_s = 0.0f;
      amplitude = 0.0f;
    }
    alvr_send_haptics(hand_id_[hand], duration_s, frequency_hz < 0.0f ? 0.0f : frequency_hz, amplitude);
  }

  VideoParams videoParams() override {
    std::lock_guard lock(state_mutex_);
    return video_;
  }

  bool consumeKeyframeRequest() override {
    if (!keyframe_wanted_) return false;
    // as the stock server does (connection.minimum_idr_interval_ms): a client without a decoder yet asks again
    // for every frame it receives
    const int64_t now = nowNs();
    if (now - last_keyframe_ns_ < minimum_idr_interval_ns_) return false;
    if (!keyframe_wanted_.exchange(false)) return false;
    last_keyframe_ns_ = now;
    return true;
  }

  void sendVideoConfig(Codec codec, const uint8_t *annexb, size_t size) override {
    if (annexb == nullptr || size == 0 || size > static_cast<size_t>(std::numeric_limits<int32_t>::max())) return;
    std::lock_guard lock(video_mutex_);
    if (codec == config_codec_ && size == config_.size() && std::memcmp(annexb, config_.data(), size) == 0) return;
    const bool replaces = !config_.empty();
    config_.assign(annexb, annexb + size);
    config_codec_ = codec;
    if (!ready_) return; // handed to ALVR when it comes up
    forwardConfigLocked();
    // ALVR sends the decoder config only in reply to a client's keyframe request, and a client that has a
    // decoder has no reason to ask. One that may already hold the previous parameter sets is made to reconnect,
    // which makes it ask.
    if (replaces && connected_) {
      logLine("the encoder's parameter sets changed while a headset was connected; reconnecting it");
      restart_wanted_ = true;
    }
  }

  void sendFrame(const FrameInfo &frame, const uint8_t *annexb, size_t size) override {
    if (!ready_ || !connected_ || frame.sample_id == 0 || annexb == nullptr || size == 0 ||
        size > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
      frames_dropped_++;
      return;
    }
    {
      // the client decodes with the parameter sets it was given, so a frame they do not belong to is no use
      std::lock_guard lock(video_mutex_);
      if (config_.empty() || config_codec_ != frame.codec) {
        frames_dropped_++;
        return;
      }
    }
    // Nothing a client can decode before the first keyframe of its connection. The client only asks for one in
    // response to video it receives, so keep asking the encoder until one comes.
    if (frame.keyframe) {
      awaiting_keyframe_ = false;
    } else if (awaiting_keyframe_) {
      keyframe_wanted_ = true;
      frames_dropped_++;
      return;
    }
    const size_t skip = leadingConfigBytes(frame.codec, annexb, size);

    // ALVR keeps, per tracking packet, when the application presented the frame drawn from it, when the
    // compositor was done with it and when the encoder was: it charges the application from the packet's arrival
    // to "present", the compositor from "present" to "composed", the encoder from "composed" to "encoded".
    // alvr_report_present and alvr_report_composed take how long ago that moment was (ALVR subtracts the offset
    // from its own clock at the call); "encoded" has no offset and is the moment of alvr_send_video_nal, which
    // also reads "composed", so both reports come first. An offset reaching back before the machine booted would
    // make ALVR's clock arithmetic panic, hence the bound.
    const int64_t now = nowNs();
    const auto ago = [&](int64_t then) -> uint64_t {
      if (then <= 0 || then >= now) return 0;
      return static_cast<uint64_t>(std::min(now - then, kOldestFrameNs));
    };
    const uint64_t composed_ago = ago(frame.composed_ns > 0 ? frame.composed_ns : frame.encoded_ns);
    const uint64_t presented_ago = std::max(ago(frame.submitted_ns), composed_ago);
    alvr_report_present(frame.sample_id, presented_ago);
    alvr_report_composed(frame.sample_id, composed_ago);
    // ALVR and the client both look the frame up by the timestamp of the tracking packet it was rendered with
    alvr_send_video_nal(frame.sample_id, const_cast<uint8_t *>(annexb + skip), static_cast<int32_t>(size - skip),
                        frame.keyframe);
    frames_sent_++;
  }

private:
  // --- ALVR lifetime; start() before the thread exists, then the event thread only ---

  bool bringUp() {
    const AlvrTargetConfig target = alvr_initialize();

    const auto settings = readSettings();
    if (!settings) {
      logLine("cannot read ALVR's settings");
      alvr_shutdown();
      return false;
    }
    if (!settings->problems.empty()) {
      logLine("ALVR settings MetalVR cannot honor:%s; restart metalvrd to have them reset in %s",
              settings->problems.c_str(), session_path_.c_str());
      alvr_shutdown();
      return false;
    }
    if (settings->av1) logLine("ALVR prefers AV1, which metalvrd does not encode; using HEVC");
    minimum_idr_interval_ns_ = settings->minimum_idr_interval_ns;

    // The headsets asked for must be trusted in the list ALVR loaded. They are not if this process had already
    // loaded it before they were asked for, or if ALVR did not take the file as written.
    const auto clients = readClientList(session_path_);
    if (!clients) {
      logLine("cannot read ALVR's client list from %s", session_path_.c_str());
      alvr_shutdown();
      return false;
    }
    for (const std::string &wanted : options_.trusted_clients) {
      const bool trusted = std::any_of(clients->trusted.begin(), clients->trusted.end(),
                                       [&](const std::string &name) { return sameHostname(name, wanted); });
      if (trusted) continue;
      logLine("headset '%s' is not in the client list ALVR loaded from %s and cannot be trusted now; ALVR reads "
              "the list once per process, restart metalvrd",
              wanted.c_str(), session_path_.c_str());
      alvr_shutdown();
      return false;
    }
    std::string names;
    std::vector<std::string> listed;
    for (const std::string &name : clients->trusted) {
      const bool repeated = std::any_of(listed.begin(), listed.end(),
                                        [&](const std::string &other) { return sameHostname(other, name); });
      if (repeated) continue;
      listed.push_back(name);
      names += (names.empty() ? "" : ", ") + bareHostname(name);
    }
    if (names.empty())
      logLine("no headset is trusted: one that announces itself will be named here and refused");
    else
      logLine("trusted headsets: %s", names.c_str());
    // Before a headset connects, the model its client reported last time, when every trusted headset that has
    // reported one is the same model. Anything else stays unknown until a headset connects.
    if (options_.headset == MVR_HEADSET_UNKNOWN) {
      uint32_t known = MVR_HEADSET_UNKNOWN;
      bool agree = true;
      for (const ClientList::Client &client : clients->trusted_clients) {
        const uint32_t model = headsetFromAlvrName(client.display_name);
        if (model == MVR_HEADSET_UNKNOWN) continue;
        if (known != MVR_HEADSET_UNKNOWN && known != model) agree = false;
        known = model;
      }
      if (agree && known != MVR_HEADSET_UNKNOWN) setModel(known, "reported by its ALVR client when it last connected");
    }

    StreamConfig stream;
    if (const uint32_t model = currentModel(); model != MVR_HEADSET_UNKNOWN) {
      const HeadsetSpec &spec = headsetSpec(model);
      stream.stream_width = stream.render_width = spec.eye_width;
      stream.stream_height = stream.render_height = spec.eye_height;
    }
    if (const auto stored = readStreamConfig(session_path_); stored && !isPlaceholder(*stored)) {
      stream = *stored;
    } else {
      if (!stored) logLine("cannot read the stream configuration from %s", session_path_.c_str());
      if (target.stream_width != kPlaceholderEyeWidth || target.stream_height != kPlaceholderEyeHeight) {
        stream.stream_width = target.stream_width;
        stream.stream_height = target.stream_height;
        stream.render_width = target.game_render_width;
        stream.render_height = target.game_render_height;
      }
      stream.refresh_hz = settings->preferred_fps;
      stream.codec = settings->codec;
    }
    {
      std::lock_guard lock(state_mutex_);
      controllers_enabled_ = settings->controllers_enabled;
    }
    applyStream(stream);

    {
      std::lock_guard lock(video_mutex_);
      if (!config_.empty()) forwardConfigLocked();
    }
    alvr_start_connection();
    encoder_params_polled_ns_ = 0;
    client_list_polled_ns_ = 0;
    ready_ = true;
    logLine("waiting for a headset: %ux%u per eye at %.0f Hz, %s, config in %s", stream.stream_width,
            stream.stream_height, stream.refresh_hz, stream.codec == Codec::H264 ? "H.264" : "HEVC",
            session_path_.parent_path().c_str());
    return true;
  }

  void bringDown() {
    ready_ = false;
    setDisconnected();
    alvr_shutdown(); // closes the connection and joins ALVR's threads
  }

  void restart(const char *reason) {
    logLine("restarting ALVR: %s", reason);
    bringDown();
    if (!bringUp()) logLine("ALVR did not come back; this driver is idle until metalvrd restarts");
  }

  void forwardConfigLocked() {
    alvr_set_video_config_nals(config_codec_ == Codec::H264 ? ALVR_CODEC_TYPE_H264 : ALVR_CODEC_TYPE_HEVC,
                               config_.data(), static_cast<int32_t>(config_.size()));
  }

  // --- the event thread ---

  void run() {
    while (!stop_) {
      if (restart_wanted_.exchange(false) && ready_) restart("new parameter sets for a connected headset");
      if (!ready_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      AlvrEvent event;
      std::memset(&event, 0xFF, sizeof(event)); // ALVR reports some events without writing anything
      const int64_t polled = nowNs();
      if (alvr_poll_event(&event, kPollTimeoutNs)) {
        handle(event);
      } else if (nowNs() - polled < static_cast<int64_t>(kPollTimeoutNs / 2)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5)); // no event queue to wait on
      }

      const int64_t now = nowNs();
      if (ready_ && now - encoder_params_polled_ns_ >= kEncoderParamsIntervalNs) {
        encoder_params_polled_ns_ = now;
        AlvrDynamicEncoderParams params{};
        if (alvr_get_dynamic_encoder_params(&params) && params.bitrate_bps >= 1.0f) {
          std::lock_guard lock(state_mutex_);
          video_.bitrate_bps = static_cast<uint64_t>(params.bitrate_bps);
        }
      }
      if (ready_ && now - client_list_polled_ns_ >= kClientListIntervalNs) {
        client_list_polled_ns_ = now;
        reportUntrusted();
      }
    }
  }

  // ALVR adds every client that announces itself to its list, untrusted, and saves the session; that file is the
  // only place the list can be read. Each such headset is named once, with what would let it in.
  void reportUntrusted() {
    std::error_code error;
    const auto written = std::filesystem::last_write_time(session_path_, error);
    if (error || (session_seen_ && written == session_written_)) return;
    const auto clients = readClientList(session_path_);
    if (!clients) return; // caught mid-write: look again next time
    session_seen_ = true;
    session_written_ = written;
    for (const std::string &name : clients->untrusted) {
      if (reported_.size() >= kMostUntrustedReported || !reported_.insert(name).second) continue;
      if (reported_.size() == kMostUntrustedReported)
        logLine("more untrusted headsets are announcing themselves; they are refused and no longer listed");
      else if (validHostname(name))
        logLine("headset '%s' wants to connect and is not trusted; start metalvrd with --trust %s to allow it",
                name.c_str(), name.c_str());
      else
        logLine("a headset announcing itself as '%s' wants to connect; that is not a name it can be trusted by",
                name.c_str());
    }
  }

  void handle(const AlvrEvent &event) {
    switch (event.tag) {
    case ALVR_EVENT_CLIENT_CONNECTED:
      onConnected();
      break;
    case ALVR_EVENT_CLIENT_DISCONNECTED:
      logLine("headset disconnected (%llu frames sent, %llu dropped)",
              static_cast<unsigned long long>(frames_sent_.load()),
              static_cast<unsigned long long>(frames_dropped_.load()));
      setDisconnected();
      break;
    case ALVR_EVENT_VIEWS_CONFIG:
      onViews(event.VIEWS_CONFIG);
      break;
    case ALVR_EVENT_TRACKING_UPDATED:
      onTracking(event.TRACKING_UPDATED.sample_timestamp_ns);
      break;
    case ALVR_EVENT_BUTTONS_UPDATED:
      onButtons();
      break;
    case ALVR_EVENT_REQUEST_IDR:
      keyframe_wanted_ = true;
      break;
    case ALVR_EVENT_RESTART_PENDING:
      restart("requested through ALVR's dashboard API");
      break;
    case ALVR_EVENT_SHUTDOWN_PENDING:
      logLine("shutdown requested through ALVR's dashboard API; this driver is idle until metalvrd restarts");
      bringDown();
      break;
    case ALVR_EVENT_BATTERY:
      onBattery(event.battery);
      break;
    case ALVR_EVENT_PLAYSPACE_SYNC: // ALVR recenters its tracking space itself; only the size is passed on
      onPlayArea(event.playspace_sync[0], event.playspace_sync[1]);
      break;
    case ALVR_EVENT_CAPTURE_FRAME:
    default:
      break;
    }
  }

  void onConnected() {
    // which headset this is: what its client reported, which ALVR saved with the client before this event
    if (const auto clients = readClientList(session_path_)) {
      // the client furthest into its connection; one stuck in "Connecting" from an earlier attempt ranks last
      const auto rank = [](const std::string &state) {
        return state == "Streaming" ? 3 : state == "Connected" ? 2 : state == "Connecting" ? 1 : 0;
      };
      const ClientList::Client *client = nullptr;
      for (const ClientList::Client &c : clients->trusted_clients)
        if (rank(c.state) > 0 && (!client || rank(c.state) > rank(client->state))) client = &c;
      const uint32_t reported = client ? headsetFromAlvrName(client->display_name) : MVR_HEADSET_UNKNOWN;
      if (client) logLine("headset '%s' reports itself as '%s'", client->hostname.c_str(), client->display_name.c_str());
      if (options_.headset != MVR_HEADSET_UNKNOWN) {
        if (reported != MVR_HEADSET_UNKNOWN && reported != options_.headset)
          logLine("warning: the headset reports itself as %s, but --headset names %s; using %s",
                  mvr_describe_headset(reported)->name, mvr_describe_headset(options_.headset)->name,
                  mvr_describe_headset(options_.headset)->name);
      } else {
        setModel(reported, reported != MVR_HEADSET_UNKNOWN ? "reported by its ALVR client"
                                                           : "its ALVR client reports no model MetalVR knows");
      }
    }
    // ALVR wrote what it negotiated with this client to session.json before reporting the connection
    if (const auto stream = readStreamConfig(session_path_)) {
      applyStream(*stream);
      logLine("headset connected: %ux%u per eye at %.0f Hz, %s", stream->stream_width, stream->stream_height,
              stream->refresh_hz, stream->codec == Codec::H264 ? "H.264" : "HEVC");
      if (stream->av1) logLine("the headset negotiated AV1, which metalvrd does not encode; sending HEVC");
      const HeadsetSpec &spec = headsetSpec(currentModel());
      if (!withinDecoder(spec, stream->stream_width * 2, stream->stream_height, stream->refresh_hz))
        logLine("warning: %ux%u at %.0f Hz is more than %s's video decoder is known to handle; lower the stream's "
                "resolution in ALVR's settings if the picture stutters",
                stream->stream_width * 2, stream->stream_height, stream->refresh_hz,
                mvr_describe_headset(spec.headset)->name);
      // the pins rule these out; they can only differ if the settings were changed while metalvrd was running
      const bool unexpected_foveation = stream->foveated != options_.foveated_encoding;
      if (unexpected_foveation || stream->full_range || stream->ten_bit || stream->hdr || stream->gamma != 1.0f)
        logLine("warning: the headset was told to expect%s%s%s%s%s, which metalvrd does not produce; restart metalvrd",
                unexpected_foveation ? (stream->foveated ? " foveated encoding" : " no foveated encoding") : "",
                stream->full_range ? " full-range video" : "",
                stream->ten_bit ? " 10-bit video" : "", stream->hdr ? " HDR" : "",
                stream->gamma != 1.0f ? " an encoding gamma" : "");
    } else {
      logLine("headset connected, but its stream configuration cannot be read from %s; keeping the previous one",
              session_path_.c_str());
    }
    {
      std::lock_guard lock(state_mutex_);
      sample_ = {};
      input_[0] = input_[1] = {};
    }
    frames_sent_ = 0;
    frames_dropped_ = 0;
    awaiting_keyframe_ = true;
    keyframe_wanted_ = true;
    connected_ = true;
    generation_++;
  }

  void setDisconnected() {
    if (!connected_.exchange(false)) return;
    {
      std::lock_guard lock(state_mutex_);
      input_[0] = input_[1] = {};
      sample_.hand_valid[0] = sample_.hand_valid[1] = false;
      battery_[0] = battery_[1] = battery_[2] = {};
    }
    generation_++;
  }

  void applyStream(const StreamConfig &stream) {
    // foveated: each eye is composed at the stream's view size and squeezed to ALVR's optimized size, which is what
    // the client decodes and stretches back
    Foveation foveation;
    if (stream.foveated && options_.foveated_encoding)
      foveation = computeFoveation(stream.stream_width, stream.stream_height, stream.foveation[0], stream.foveation[1],
                                   stream.foveation[2], stream.foveation[3], stream.foveation[4], stream.foveation[5]);
    const uint32_t frame_width = foveation.enabled ? foveation.optimized_eye_width * 2 : stream.stream_width * 2;
    const uint32_t frame_height = foveation.enabled ? foveation.optimized_eye_height : stream.stream_height;
    std::lock_guard lock(state_mutex_);
    const bool changed = system_.eye_width != stream.render_width || system_.eye_height != stream.render_height ||
                         system_.refresh_hz != stream.refresh_hz || video_.codec != stream.codec ||
                         video_.frame_width != frame_width || video_.frame_height != frame_height ||
                         video_.foveation.enabled != foveation.enabled;
    system_.eye_width = stream.render_width;
    system_.eye_height = stream.render_height;
    system_.refresh_hz = stream.refresh_hz;
    setRefreshRatesLocked();
    video_.codec = stream.codec;
    video_.frame_width = frame_width;
    video_.frame_height = frame_height;
    video_.foveation = foveation;
    if (foveation.enabled && changed)
      logLine("foveated encoding: %ux%u per eye sent as %ux%u", stream.stream_width, stream.stream_height,
              foveation.optimized_eye_width, foveation.optimized_eye_height);
    video_.fps = stream.refresh_hz;
    if (changed) generation_++;
  }

  // --- the headset's model ---

  uint32_t currentModel() {
    std::lock_guard lock(state_mutex_);
    return system_.headset;
  }

  // Names the headset (MVR_HEADSET_UNKNOWN: a headset MetalVR does not know). Its controllers are what ALVR
  // reports for every client, the Touch family's inputs, unless the model is known to have others.
  void setModel(uint32_t headset, const char *why) {
    std::lock_guard lock(state_mutex_);
    if (system_.headset == headset && system_.system_name[0]) return;
    const mvr_headset_description *description = mvr_describe_headset(headset);
    system_.headset = headset;
    system_.controllers = headset == MVR_HEADSET_UNKNOWN ? MVR_CONTROLLERS_TOUCH : description->controllers;
    std::snprintf(system_.system_name, sizeof(system_.system_name), "%s",
                  headset == MVR_HEADSET_UNKNOWN ? "MetalVR ALVR Headset" : description->name);
    setRefreshRatesLocked();
    generation_++;
    if (headset != MVR_HEADSET_UNKNOWN) logLine("headset: %s (%s)", description->name, why);
  }

  // the model's refresh rates, with the one the stream runs at among them
  void setRefreshRatesLocked() {
    const HeadsetSpec &spec = headsetSpec(system_.headset);
    std::vector<float> rates(spec.refresh_rates, spec.refresh_rates + spec.refresh_rate_count);
    if (system_.refresh_hz > 0 && std::none_of(rates.begin(), rates.end(), [&](float r) {
          return std::fabs(r - system_.refresh_hz) < 0.5f;
        }))
      rates.push_back(system_.refresh_hz);
    std::sort(rates.begin(), rates.end());
    if (rates.size() > MVR_MAX_REFRESH_RATES) rates.resize(MVR_MAX_REFRESH_RATES);
    system_.refresh_rate_count = static_cast<uint32_t>(rates.size());
    std::fill(std::begin(system_.refresh_rates), std::end(system_.refresh_rates), 0.0f);
    std::copy(rates.begin(), rates.end(), system_.refresh_rates);
  }

  // --- what the headset says of itself besides video and tracking ---

  struct Battery {
    bool valid = false;
    bool plugged = false;
    float level = 0.0f;
  };

  void onBattery(const AlvrBatteryInfo &info) {
    int slot = -1; // 0 head, 1 left hand, 2 right hand
    if (info.device_id == head_id_) slot = 0;
    if (info.device_id == hand_id_[0]) slot = 1;
    if (info.device_id == hand_id_[1]) slot = 2;
    if (slot < 0 || !std::isfinite(info.gauge_value)) return;
    std::lock_guard lock(state_mutex_);
    battery_[slot] = {true, info.is_plugged, std::clamp(info.gauge_value, 0.0f, 1.0f)};
  }

  void onPlayArea(float width, float depth) {
    if (!std::isfinite(width) || !std::isfinite(depth) || width < 0.0f || depth < 0.0f || width > 100.0f ||
        depth > 100.0f)
      return;
    std::lock_guard lock(state_mutex_);
    if (std::fabs(system_.play_area_width - width) < 1e-3f && std::fabs(system_.play_area_depth - depth) < 1e-3f)
      return;
    system_.play_area_width = width;
    system_.play_area_depth = depth;
    generation_++;
  }

  // A trusted "client.wired" is a headset on USB, which ALVR reaches with adb. ALVR would download Google's
  // platform-tools when it finds no adb; here an adb must exist before ALVR starts, so it never does.
  bool prepareWired() {
    const bool wired = std::any_of(options_.trusted_clients.begin(), options_.trusted_clients.end(),
                                   [](const std::string &name) { return sameHostname(name, kWiredClient); });
    if (!wired) return true;
    std::filesystem::path adb = options_.adb.empty() ? adbOnPath() : std::filesystem::path(options_.adb);
    if (adb.empty() || !isExecutable(adb)) {
      logLine("a headset on USB (%s) needs adb: install Android's platform-tools (brew install android-platform-tools) "
              "or name one with --adb; MetalVR does not let ALVR download it",
              std::string(kWiredClient).c_str());
      return false;
    }
    if (!options_.adb.empty()) {
      // ALVR runs "adb" from PATH: the one named goes first there
      if (adb.filename() != "adb") {
        logLine("--adb must name a program called adb, not %s", adb.c_str());
        return false;
      }
      const char *path = std::getenv("PATH");
      const std::string dir = std::filesystem::absolute(adb).parent_path().string();
      ::setenv("PATH", (dir + (path && *path ? ":" + std::string(path) : std::string())).c_str(), 1);
    }
    logLine("a headset on USB is reached with %s", adb.c_str());
    return true;
  }

  void setViewOffsets(float ipd) {
    view_offset_[0] = {{0, 0, 0, 1}, {-ipd * 0.5f, 0, 0}};
    view_offset_[1] = {{0, 0, 0, 1}, {ipd * 0.5f, 0, 0}};
  }

  void onViews(const ViewsConfig_Body &views) {
    const mvr_fov fov[2] = {toFov(views.fov[0]), toFov(views.fov[1])};
    const float ipd = views.local_view_transform[1].position[0] - views.local_view_transform[0].position[0];
    if (!usableFov(fov[0]) || !usableFov(fov[1]) || !std::isfinite(ipd) || ipd < 0.0f || ipd > 0.2f) {
      logLine("ignoring unusable view parameters from the headset");
      return;
    }
    std::lock_guard lock(state_mutex_);
    const bool changed =
        !sameFov(fov[0], system_.fov[0]) || !sameFov(fov[1], system_.fov[1]) || std::fabs(ipd - system_.ipd) > 1e-5f;
    system_.fov[0] = fov[0];
    system_.fov[1] = fov[1];
    system_.ipd = ipd;
    view_offset_[0] = toPose(views.local_view_transform[0]);
    view_offset_[1] = toPose(views.local_view_transform[1]);
    if (changed) generation_++;
  }

  void onTracking(uint64_t timestamp_ns) {
    Sample sample;
    sample.timestamp_ns = timestamp_ns;
    sample.received_ns = nowNs();
    if (timestamp_ns == 0 || !alvr_get_device_motion(head_id_, timestamp_ns, &sample.head)) return;
    for (int hand = 0; hand < 2; hand++) {
      sample.hand_valid[hand] = alvr_get_device_motion(hand_id_[hand], timestamp_ns, &sample.hand[hand]);
      // ALVR's 26 joints are OpenXR's, in its order
      sample.skeleton_valid[hand] = alvr_get_hand_skeleton(hand == 0 ? ALVR_HAND_TYPE_LEFT : ALVR_HAND_TYPE_RIGHT,
                                                           timestamp_ns, sample.skeleton[hand]);
    }
    std::lock_guard lock(state_mutex_);
    sample_ = sample;
  }

  void onButtons() {
    // one queued batch per event; a null buffer asks for its size, a buffer takes it off ALVR's queue
    const uint64_t count = alvr_get_buttons(nullptr);
    std::vector<AlvrButtonEntry> entries(count > 0 ? count : 1);
    std::memset(entries.data(), 0, entries.size() * sizeof(AlvrButtonEntry));
    const uint64_t taken = alvr_get_buttons(entries.data());

    std::lock_guard lock(state_mutex_);
    for (uint64_t i = 0; i < taken && i < entries.size(); i++) {
      const auto binding = bindings_.find(entries[i].id);
      if (binding == bindings_.end()) continue;
      ControllerInput &input = input_[binding->second.hand];
      // ALVR's union holds a bool for click and touch inputs and a float for analog ones; which one is implied by
      // the input
      const bool pressed = !isScalar(binding->second.kind) && entries[i].value.scalar;
      const float value = isScalar(binding->second.kind) ? entries[i].value.float_value : 0.0f;
      switch (binding->second.kind) {
      case InputKind::Click:
        input.clicks = pressed ? input.clicks | binding->second.bit : input.clicks & ~binding->second.bit;
        break;
      case InputKind::Touch:
        input.touches = pressed ? input.touches | binding->second.bit : input.touches & ~binding->second.bit;
        break;
      case InputKind::TriggerClick:
        input.has_trigger_click = true;
        input.trigger_click = pressed;
        break;
      case InputKind::SqueezeClick:
        input.has_squeeze_click = true;
        input.squeeze_click = pressed;
        break;
      case InputKind::TriggerValue:
        input.trigger = value;
        if (value >= kPressOn) input.trigger_down = true;
        if (value <= kPressOff) input.trigger_down = false;
        break;
      case InputKind::SqueezeValue:
        input.squeeze = value;
        if (value >= kPressOn) input.squeeze_down = true;
        if (value <= kPressOff) input.squeeze_down = false;
        break;
      case InputKind::ThumbstickX:
        input.thumbstick_x = value;
        break;
      case InputKind::ThumbstickY:
        input.thumbstick_y = value;
        break;
      }
    }
  }

  const AlvrOptions options_;
  std::filesystem::path session_path_;
  bool started_ = false; // start() and stop() come from one thread

  std::unordered_map<uint64_t, InputBinding> bindings_;
  uint64_t head_id_ = 0;
  uint64_t hand_id_[2] = {0, 0};

  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> ready_{false}; // ALVR is initialized and its connection loop runs
  std::atomic<bool> connected_{false};
  std::atomic<bool> restart_wanted_{false};
  std::atomic<bool> keyframe_wanted_{false};
  std::atomic<bool> awaiting_keyframe_{true};
  std::atomic<int64_t> last_keyframe_ns_{0};
  std::atomic<int64_t> minimum_idr_interval_ns_{100'000'000};
  std::atomic<uint64_t> generation_{1};
  std::atomic<uint64_t> frames_sent_{0};
  std::atomic<uint64_t> frames_dropped_{0};
  int64_t encoder_params_polled_ns_ = 0; // event thread, like the next four
  int64_t client_list_polled_ns_ = 0;
  bool session_seen_ = false;
  std::filesystem::file_time_type session_written_{};
  std::set<std::string> reported_; // the untrusted headsets already named in the log

  std::mutex state_mutex_; // everything below, up to video_mutex_
  mvr_system_info system_{};
  VideoParams video_{Codec::HEVC, kDefaultEyeWidth * 2, kDefaultEyeHeight, kDefaultRefreshHz, 30'000'000};
  mvr_pose view_offset_[2]{};
  Sample sample_;
  ControllerInput input_[2];
  Battery battery_[3]; // head, left hand, right hand
  bool controllers_enabled_ = false;

  std::mutex video_mutex_; // the parameter sets last given to sendVideoConfig
  std::vector<uint8_t> config_;
  Codec config_codec_ = Codec::HEVC;
};

} // namespace

std::unique_ptr<Driver> makeAlvrDriver(const AlvrOptions &options) { return std::make_unique<AlvrDriver>(options); }

} // namespace metalvr
