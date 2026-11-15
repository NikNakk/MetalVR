// The runtime-facing side of metalvrd: accepts OpenXR runtimes on the loopback interface and serves the protocol.
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "driver.h"

namespace metalvr {

struct ServerOptions {
  uint16_t port = MVR_DEFAULT_PORT;
  // writes the Nth composed frame of the process as a PNG, to check composition without a headset
  std::string snapshot_path;
  uint32_t snapshot_frame = 30;
  // stop serving once the client that held a session disconnects
  bool exit_after_session = false;
  // encode on the CPU: for machines without a hardware encoder (the virtual machines tests run in)
  bool software_encoder = false;
};

class Server {
public:
  Server(std::unique_ptr<Driver> driver, const ServerOptions &options);
  ~Server();

  bool start();
  // serves until stop() or, with exit_after_session, until the session's client goes
  void run();
  // safe to call from a signal handler's thread of control only through requestStop()
  void requestStop();

  struct Impl;

private:
  std::unique_ptr<Impl> impl_;
};

} // namespace metalvr
