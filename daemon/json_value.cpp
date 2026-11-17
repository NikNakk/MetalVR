#include "json_value.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace metalvr {

namespace {
const JsonValue &nullValue() {
  static const JsonValue none;
  return none;
}
} // namespace

class JsonParser {
public:
  explicit JsonParser(std::string_view text) : s_(text) {}

  bool document(JsonValue &out) {
    skip();
    if (!value(out, 0)) return false;
    skip();
    return i_ == s_.size();
  }

private:
  static constexpr int kMaxDepth = 64;

  void skip() {
    while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\r' || s_[i_] == '\n')) i_++;
  }

  bool literal(std::string_view word) {
    if (s_.substr(i_, word.size()) != word) return false;
    i_ += word.size();
    return true;
  }

  bool value(JsonValue &out, int depth) {
    if (i_ >= s_.size() || depth > kMaxDepth) return false;
    switch (s_[i_]) {
    case '{':
      return object(out, depth);
    case '[':
      return array(out, depth);
    case '"':
      out.type_ = JsonValue::Type::String;
      return string(out.string_);
    case 't':
      out.type_ = JsonValue::Type::Bool;
      out.boolean_ = true;
      return literal("true");
    case 'f':
      out.type_ = JsonValue::Type::Bool;
      out.boolean_ = false;
      return literal("false");
    case 'n':
      out.type_ = JsonValue::Type::Null;
      return literal("null");
    default:
      return number(out);
    }
  }

  bool number(JsonValue &out) {
    const size_t begin = i_;
    if (i_ < s_.size() && s_[i_] == '-') i_++;
    while (i_ < s_.size() && (std::strchr("0123456789.eE+-", s_[i_]) != nullptr) && s_[i_] != '\0') i_++;
    if (i_ == begin || i_ - begin > 64) return false;
    char text[65];
    std::memcpy(text, s_.data() + begin, i_ - begin);
    text[i_ - begin] = 0;
    char *end = nullptr;
    const double v = std::strtod(text, &end);
    if (end != text + (i_ - begin) || !std::isfinite(v)) return false;
    out.type_ = JsonValue::Type::Number;
    out.number_ = v;
    return true;
  }

  static void appendUtf8(std::string &out, uint32_t c) {
    if (c < 0x80) {
      out += static_cast<char>(c);
    } else if (c < 0x800) {
      out += static_cast<char>(0xC0 | (c >> 6));
      out += static_cast<char>(0x80 | (c & 0x3F));
    } else {
      out += static_cast<char>(0xE0 | (c >> 12));
      out += static_cast<char>(0x80 | ((c >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (c & 0x3F));
    }
  }

  bool string(std::string &out) {
    if (i_ >= s_.size() || s_[i_] != '"') return false;
    i_++;
    while (i_ < s_.size()) {
      const char c = s_[i_++];
      if (c == '"') return true;
      if (c != '\\') {
        out += c;
        continue;
      }
      if (i_ >= s_.size()) return false;
      const char e = s_[i_++];
      switch (e) {
      case '"': out += '"'; break;
      case '\\': out += '\\'; break;
      case '/': out += '/'; break;
      case 'b': out += '\b'; break;
      case 'f': out += '\f'; break;
      case 'n': out += '\n'; break;
      case 'r': out += '\r'; break;
      case 't': out += '\t'; break;
      case 'u': {
        if (i_ + 4 > s_.size()) return false;
        uint32_t code = 0;
        for (int k = 0; k < 4; k++) {
          const char h = s_[i_++];
          code <<= 4;
          if (h >= '0' && h <= '9') code |= h - '0';
          else if (h >= 'a' && h <= 'f') code |= h - 'a' + 10;
          else if (h >= 'A' && h <= 'F') code |= h - 'A' + 10;
          else return false;
        }
        appendUtf8(out, code); // a surrogate pair stays two code units; nothing here needs more
        break;
      }
      default:
        return false;
      }
    }
    return false;
  }

  bool array(JsonValue &out, int depth) {
    out.type_ = JsonValue::Type::Array;
    i_++;
    skip();
    if (i_ < s_.size() && s_[i_] == ']') {
      i_++;
      return true;
    }
    for (;;) {
      JsonValue element;
      skip();
      if (!value(element, depth + 1)) return false;
      out.array_.push_back(std::move(element));
      skip();
      if (i_ >= s_.size()) return false;
      if (s_[i_] == ',') {
        i_++;
        continue;
      }
      if (s_[i_] != ']') return false;
      i_++;
      return true;
    }
  }

  bool object(JsonValue &out, int depth) {
    out.type_ = JsonValue::Type::Object;
    i_++;
    skip();
    if (i_ < s_.size() && s_[i_] == '}') {
      i_++;
      return true;
    }
    for (;;) {
      skip();
      std::string key;
      if (!string(key)) return false;
      skip();
      if (i_ >= s_.size() || s_[i_] != ':') return false;
      i_++;
      skip();
      JsonValue member;
      if (!value(member, depth + 1)) return false;
      out.object_.emplace_back(std::move(key), std::move(member));
      skip();
      if (i_ >= s_.size()) return false;
      if (s_[i_] == ',') {
        i_++;
        continue;
      }
      if (s_[i_] != '}') return false;
      i_++;
      return true;
    }
  }

  std::string_view s_;
  size_t i_ = 0;
};

std::optional<JsonValue> JsonValue::parse(std::string_view text) {
  // firmware blocks end in a NUL, which is not part of the document
  while (!text.empty() && text.back() == '\0') text.remove_suffix(1);
  JsonValue value;
  JsonParser parser(text);
  if (!parser.document(value)) return std::nullopt;
  return value;
}

const JsonValue &JsonValue::operator[](std::string_view key) const {
  if (type_ != Type::Object) return nullValue();
  for (const auto &member : object_)
    if (member.first == key) return member.second;
  return nullValue();
}

const JsonValue &JsonValue::operator[](size_t index) const {
  if (type_ != Type::Array || index >= array_.size()) return nullValue();
  return array_[index];
}

bool JsonValue::numbers(float *out, size_t n) const {
  if (type_ != Type::Array || array_.size() < n) return false;
  for (size_t i = 0; i < n; i++)
    if (!array_[i].isNumber()) return false;
  for (size_t i = 0; i < n; i++) out[i] = static_cast<float>(array_[i].number_);
  return true;
}

bool JsonValue::numbers(double *out, size_t n) const {
  if (type_ != Type::Array || array_.size() < n) return false;
  for (size_t i = 0; i < n; i++)
    if (!array_[i].isNumber()) return false;
  for (size_t i = 0; i < n; i++) out[i] = array_[i].number_;
  return true;
}

} // namespace metalvr
