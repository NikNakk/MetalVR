// The runtime's connection to metalvrd. Compiles for Windows (Winsock, under Wine) and macOS (BSD sockets).
#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>

#include "metalvr_protocol.h"

namespace metalvr {

// this process's monotonic clock, nanoseconds
int64_t localNowNs();
void sleepUntilLocalNs(int64_t deadline_ns);
uint32_t currentProcessId();

class Connection {
public:
  Connection() = default;
  ~Connection();
  Connection(const Connection &) = delete;
  Connection &operator=(const Connection &) = delete;

  // connects to metalvrd on the loopback interface; the port is $METALVR_PORT or the default
  bool connect();
  void close();
  bool alive() const { return alive_; }

  // One request and its response. `tail` is appended to the request payload. False when the server is gone or
  // answered with something else than `response_size` bytes; the connection is then dead for good.
  bool call(uint16_t type, const void *request, size_t request_size, void *response, size_t response_size,
            const void *tail = nullptr, size_t tail_size = 0);

  template <typename Request, typename Response> bool call(uint16_t type, const Request &request, Response &response) {
    return call(type, &request, sizeof(Request), &response, sizeof(Response));
  }
  template <typename Response> bool call(uint16_t type, Response &response) {
    return call(type, nullptr, 0, &response, sizeof(Response));
  }

  // server time = local time + offset
  int64_t clockOffsetNs() const { return clock_offset_ns_; }
  bool syncClock();

private:
  std::mutex mutex_;
  intptr_t socket_ = -1;
  bool alive_ = false;
  int64_t clock_offset_ns_ = 0;
};

} // namespace metalvr
