// Smoke test for the ALVR driver without a headset: it links, starts, answers, stops, and starts again.
//
//   alvr_driver_smoke [config-dir] [seconds] [trusted-hostname...]
//
// With a headset running the ALVR client on the same network, and its hostname given, the same program shows the
// connection, the negotiated stream and live tracking; it sends no video.
#include "driver.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <time.h>

namespace {

void print(metalvr::Driver &driver, double elapsed) {
  const mvr_system_info system = driver.systemInfo();
  const metalvr::VideoParams video = driver.videoParams();
  mvr_tracking_response tracking{};
  tracking.result = 0xABCD1234u; // the driver must leave this alone
  driver.tracking(static_cast<int64_t>(clock_gettime_nsec_np(CLOCK_UPTIME_RAW)), tracking);

  std::printf("t=%.1fs connected=%d generation=%llu\n", elapsed, driver.connected() ? 1 : 0,
              static_cast<unsigned long long>(driver.generation()));
  std::printf("  system: \"%s\" flags=0x%x eye=%ux%u refresh=%.1fHz ipd=%.4fm\n", system.system_name, system.flags,
              system.eye_width, system.eye_height, system.refresh_hz, system.ipd);
  for (int eye = 0; eye < 2; eye++)
    std::printf("  fov[%d]: left=%.3f right=%.3f up=%.3f down=%.3f rad\n", eye, system.fov[eye].left,
                system.fov[eye].right, system.fov[eye].up, system.fov[eye].down);
  std::printf("  video: %s %ux%u %.1ffps %.1fMbps\n", video.codec == metalvr::Codec::H264 ? "H.264" : "HEVC",
              video.frame_width, video.frame_height, video.fps, static_cast<double>(video.bitrate_bps) / 1e6);
  std::printf("  tracking: result=0x%x flags=0x%x sample_id=%llu head=(%.3f %.3f %.3f) q=(%.3f %.3f %.3f %.3f)\n",
              tracking.result, tracking.flags, static_cast<unsigned long long>(tracking.sample_id),
              tracking.head.position.x, tracking.head.position.y, tracking.head.position.z,
              tracking.head.orientation.x, tracking.head.orientation.y, tracking.head.orientation.z,
              tracking.head.orientation.w);
  std::printf("  views: left x=%.4f right x=%.4f\n", tracking.views[0].pose.position.x,
              tracking.views[1].pose.position.x);
  for (int hand = 0; hand < 2; hand++) {
    const mvr_controller_state &controller = tracking.controllers[hand];
    std::printf("  controller[%d]: flags=0x%x buttons=0x%x touches=0x%x trigger=%.2f squeeze=%.2f stick=(%.2f %.2f) "
                "grip=(%.3f %.3f %.3f)\n",
                hand, controller.flags, controller.buttons, controller.touches, controller.trigger, controller.squeeze,
                controller.thumbstick_x, controller.thumbstick_y, controller.grip.position.x,
                controller.grip.position.y, controller.grip.position.z);
  }
}

} // namespace

int main(int argc, char **argv) {
  std::setvbuf(stdout, nullptr, _IOLBF, 0); // keep this program's lines in order with the driver's on stderr
  metalvr::AlvrOptions options;
  options.config_dir = argc > 1 ? argv[1] : "build/alvr/smoke-config";
  const double seconds = argc > 2 ? std::atof(argv[2]) : 3.0;
  for (int i = 3; i < argc; i++) options.trusted_clients.push_back(argv[i]);

  std::unique_ptr<metalvr::Driver> driver = metalvr::makeAlvrDriver(options);
  if (!driver) {
    std::printf("FAIL: makeAlvrDriver returned null\n");
    return 1;
  }
  std::printf("driver \"%s\", config in %s\n", driver->name(), options.config_dir.c_str());

  int failures = 0;
  const auto expect = [&](bool ok, const char *what) {
    if (!ok) {
      std::printf("FAIL: %s\n", what);
      failures++;
    }
  };

  // usable before start(): defaults, no headset
  print(*driver, 0.0);
  expect(!driver->connected(), "not connected before start");

  if (!driver->start()) {
    std::printf("FAIL: start() returned false\n");
    return 1;
  }
  std::printf("started\n");

  // every headset asked for is trusted in ALVR's session, under its plain name however it was typed; nobody else
  {
    std::ifstream file(options.config_dir + "/session.json");
    const std::string session((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    for (const std::string &hostname : options.trusted_clients) {
      std::string bare = hostname;
      if (!bare.empty() && bare.back() == '.') bare.pop_back();
      if (bare.size() > 6 && bare.compare(bare.size() - 6, 6, ".local") == 0) bare.resize(bare.size() - 6);
      const size_t at = session.find('"' + bare + "\": {");
      const size_t end = at == std::string::npos ? at : session.find('}', at);
      const bool trusted = end != std::string::npos &&
                           session.substr(at, end - at).find("\"trusted\": true") != std::string::npos;
      std::printf("session.json trusts \"%s\" (given as \"%s\"): %d\n", bare.c_str(), hostname.c_str(),
                  trusted ? 1 : 0);
      expect(trusted, "a headset asked for is trusted in session.json");
    }
    expect(session.find("\"trusted\": false") == std::string::npos, "no untrusted client is listed without a headset");
    expect(session.find("\"auto_trust_clients\": false") != std::string::npos, "ALVR does not trust clients by itself");
  }

  // none of these may do anything, or crash, without a headset
  const uint8_t parameter_sets[] = {0, 0, 0, 1, 0x40, 1, 0, 0, 0, 1, 0x42, 1, 0, 0, 0, 1, 0x44, 1};
  const uint8_t frame[] = {0, 0, 0, 1, 0x26, 1, 0xAF};
  metalvr::FrameInfo info;
  info.sample_id = 1234;
  info.codec = metalvr::Codec::HEVC;
  info.keyframe = true;
  info.encoded_ns = static_cast<int64_t>(clock_gettime_nsec_np(CLOCK_UPTIME_RAW));
  info.composed_ns = info.encoded_ns - 10'000'000;
  info.submitted_ns = info.composed_ns - 2'000'000;
  driver->sendVideoConfig(metalvr::Codec::HEVC, parameter_sets, sizeof(parameter_sets));
  driver->sendFrame(info, frame, sizeof(frame));
  driver->haptic(0, 0.1f, 0.0f, 0.5f);
  std::printf("keyframe requested before a connection: %d\n", driver->consumeKeyframeRequest() ? 1 : 0);

  // once a second, and whenever the driver says its answers changed
  const auto begin = std::chrono::steady_clock::now();
  bool was_connected = false;
  uint64_t generation = 0;
  double printed = -1.0;
  for (;;) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    if (driver->generation() != generation || elapsed - printed >= 1.0 || elapsed >= seconds) {
      generation = driver->generation();
      printed = elapsed;
      print(*driver, elapsed);
    }
    if (driver->connected() && !was_connected)
      std::printf("  keyframe requested on connection: %d\n", driver->consumeKeyframeRequest() ? 1 : 0);
    was_connected = driver->connected();
    if (elapsed >= seconds) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  const mvr_system_info system = driver->systemInfo();
  const metalvr::VideoParams video = driver->videoParams();
  mvr_tracking_response tracking{};
  tracking.result = 7;
  driver->tracking(0, tracking);
  expect(system.eye_width >= 256 && system.eye_height >= 256, "a plausible eye size");
  expect(system.refresh_hz >= 30.0f && system.refresh_hz <= 144.0f, "a plausible refresh rate");
  expect(system.ipd > 0.04f && system.ipd < 0.09f, "a plausible IPD");
  expect(system.fov[0].left < 0 && system.fov[0].right > 0 && system.fov[0].down < 0 && system.fov[0].up > 0,
         "an OpenXR-signed field of view");
  expect(video.frame_width >= 512 && video.frame_width % 2 == 0 && video.frame_height >= 256, "a plausible frame");
  expect(video.bitrate_bps >= 1'000'000, "a plausible bitrate");
  expect(tracking.result == 7, "tracking() leaves result alone");
  expect(tracking.time_ns > 0, "tracking() fills time_ns");
  expect(tracking.views[1].pose.position.x - tracking.views[0].pose.position.x > 0.04f, "views are an IPD apart");
  if (!driver->connected()) {
    expect((tracking.flags & (MVR_TRACKING_HMD_CONNECTED | MVR_TRACKING_PRESENTING)) == 0,
           "no headset flags without a headset");
    expect(tracking.sample_id == 0, "no sample without a headset");
  }

  auto mark = std::chrono::steady_clock::now();
  driver->stop();
  std::printf("stopped in %.2fs\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - mark).count());
  expect(!driver->connected(), "not connected after stop");

  // ALVR's environment is one per process; a second start must reuse it
  mark = std::chrono::steady_clock::now();
  if (!driver->start()) {
    std::printf("FAIL: second start() returned false\n");
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  print(*driver, 0.5);
  driver->stop();
  std::printf("restarted and stopped in %.2fs\n",
              std::chrono::duration<double>(std::chrono::steady_clock::now() - mark).count());

  std::printf(failures == 0 ? "PASS\n" : "FAILED (%d)\n", failures);
  return failures == 0 ? 0 : 1;
}
