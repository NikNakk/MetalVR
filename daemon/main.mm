// metalvrd: the MetalVR server. OpenXR runtimes connect to it; it composes their frames, encodes them, and
// streams them to a headset through a driver.
#import <Foundation/Foundation.h>

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <string>

#include "common.h"
#include "driver.h"
#include "server.h"

namespace metalvr {
bool g_verbose = false;
}

using namespace metalvr;

namespace {

Server *g_server = nullptr;

void onSignal(int) {
  if (g_server)
    g_server->requestStop();
}

void usage() {
  std::fprintf(
      stderr,
      "usage: metalvrd [options]\n"
      "  --driver sim|alvr        headset driver (default: alvr if built in, else sim)\n"
      "  --port N                 loopback port runtimes connect to (default %d, or $%s)\n"
      "  --config-dir PATH        where the ALVR driver keeps its session and logs\n"
      "  --trust HOSTNAME         let this headset connect (as its ALVR client shows it, e.g. 1234.client.local.);\n"
      "                           repeatable. A headset that is not trusted is logged with its name and refused.\n"
      "  --verbose                log per-swapchain and per-interval detail\n"
      "simulated headset:\n"
      "  --eye WxH                per-eye size (default 1440x1584)\n"
      "  --fps N                  refresh rate (default 72)\n"
      "  --codec h264|hevc        (default h264)\n"
      "  --bitrate MBPS           (default 30)\n"
      "  --dump FILE              write the encoded stream as Annex B\n"
      "  --no-animate             hold the head and controllers still\n"
      "testing:\n"
      "  --snapshot FILE.png      write one composed frame as a PNG\n"
      "  --snapshot-frame N       which composed frame (default 30)\n"
      "  --exit-after-session     quit when the client that ran a session disconnects\n"
      "  --software-encoder       encode on the CPU; for machines with no hardware encoder, not for a headset\n",
      MVR_DEFAULT_PORT, MVR_PORT_ENV);
}

} // namespace

int main(int argc, char **argv) {
  @autoreleasepool {
    std::string driver_name;
    ServerOptions server_options;
    SimOptions sim;
    AlvrOptions alvr;
    if (const char *port = std::getenv(MVR_PORT_ENV))
      server_options.port = static_cast<uint16_t>(std::atoi(port));
    NSString *support =
        [NSSearchPathForDirectoriesInDomains(NSApplicationSupportDirectory, NSUserDomainMask, YES) firstObject];
    alvr.config_dir = std::string(support.UTF8String) + "/MetalVR";

    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      const auto value = [&]() -> const char * {
        if (i + 1 >= argc) {
          usage();
          std::exit(2);
        }
        return argv[++i];
      };
      if (arg == "--driver")
        driver_name = value();
      else if (arg == "--port")
        server_options.port = static_cast<uint16_t>(std::atoi(value()));
      else if (arg == "--config-dir")
        alvr.config_dir = value();
      else if (arg == "--trust")
        alvr.trusted_clients.push_back(value());
      else if (arg == "--verbose")
        g_verbose = true;
      else if (arg == "--eye") {
        unsigned w = 0, h = 0;
        if (std::sscanf(value(), "%ux%u", &w, &h) != 2 || !w || !h || (w & 1) || (h & 1)) {
          std::fprintf(stderr, "--eye wants WxH with even numbers\n");
          return 2;
        }
        sim.eye_width = w;
        sim.eye_height = h;
      } else if (arg == "--fps")
        sim.refresh_hz = static_cast<float>(std::atof(value()));
      else if (arg == "--codec") {
        const std::string codec = value();
        if (codec == "hevc")
          sim.codec = Codec::HEVC;
        else if (codec == "h264")
          sim.codec = Codec::H264;
        else {
          usage();
          return 2;
        }
      } else if (arg == "--bitrate")
        sim.bitrate_bps = static_cast<uint64_t>(std::atof(value()) * 1e6);
      else if (arg == "--dump")
        sim.dump_path = value();
      else if (arg == "--no-animate")
        sim.animate = false;
      else if (arg == "--snapshot")
        server_options.snapshot_path = value();
      else if (arg == "--snapshot-frame")
        server_options.snapshot_frame = static_cast<uint32_t>(std::atoi(value()));
      else if (arg == "--exit-after-session")
        server_options.exit_after_session = true;
      else if (arg == "--software-encoder")
        server_options.software_encoder = true;
      else {
        usage();
        return arg == "--help" || arg == "-h" ? 0 : 2;
      }
    }

    std::unique_ptr<Driver> driver;
    if (driver_name.empty() || driver_name == "alvr") {
      [[NSFileManager defaultManager] createDirectoryAtPath:[NSString stringWithUTF8String:alvr.config_dir.c_str()]
                                withIntermediateDirectories:YES
                                                 attributes:nil
                                                      error:nil];
      driver = makeAlvrDriver(alvr);
      if (!driver && driver_name == "alvr") {
        std::fprintf(stderr, "metalvrd was built without the ALVR driver (see docs/ALVR.md)\n");
        return 1;
      }
    }
    if (!driver) {
      if (!driver_name.empty() && driver_name != "sim") {
        usage();
        return 2;
      }
      driver = makeSimDriver(sim);
    }

    Server server(std::move(driver), server_options);
    g_server = &server;
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    if (!server.start())
      return 1;
    server.run();
    g_server = nullptr;
    logInfo("stopped");
  }
  return 0;
}
