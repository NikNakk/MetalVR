#include "ipc.h"

#include <cstdlib>
#include <cstring>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#endif

namespace metalvr {

#ifdef _WIN32

int64_t localNowNs() {
  static LARGE_INTEGER frequency = [] {
    LARGE_INTEGER f;
    QueryPerformanceFrequency(&f);
    return f;
  }();
  LARGE_INTEGER counter;
  QueryPerformanceCounter(&counter);
  const int64_t seconds = counter.QuadPart / frequency.QuadPart;
  const int64_t rest = counter.QuadPart % frequency.QuadPart;
  return seconds * 1'000'000'000 + rest * 1'000'000'000 / frequency.QuadPart;
}

void sleepUntilLocalNs(int64_t deadline_ns) {
  for (;;) {
    const int64_t remaining = deadline_ns - localNowNs();
    if (remaining <= 0)
      return;
    // Sleep is coarse; the last stretch is yielded away
    if (remaining > 2'000'000)
      Sleep(static_cast<DWORD>((remaining - 1'000'000) / 1'000'000));
    else
      SwitchToThread();
  }
}

uint32_t currentProcessId() { return GetCurrentProcessId(); }

namespace {
bool socketsReady() {
  static bool ready = [] {
    WSADATA data;
    return WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }();
  return ready;
}
void closeSocket(intptr_t s) { closesocket(static_cast<SOCKET>(s)); }
int sendSome(intptr_t s, const void *data, size_t size) {
  return ::send(static_cast<SOCKET>(s), static_cast<const char *>(data), static_cast<int>(size), 0);
}
int recvSome(intptr_t s, void *data, size_t size) {
  return ::recv(static_cast<SOCKET>(s), static_cast<char *>(data), static_cast<int>(size), 0);
}
bool interrupted() { return WSAGetLastError() == WSAEINTR; }
} // namespace

#else

int64_t localNowNs() { return static_cast<int64_t>(clock_gettime_nsec_np(CLOCK_UPTIME_RAW)); }

void sleepUntilLocalNs(int64_t deadline_ns) {
  for (;;) {
    const int64_t remaining = deadline_ns - localNowNs();
    if (remaining <= 0)
      return;
    timespec ts{static_cast<time_t>(remaining / 1'000'000'000), static_cast<long>(remaining % 1'000'000'000)};
    nanosleep(&ts, nullptr);
  }
}

uint32_t currentProcessId() { return static_cast<uint32_t>(getpid()); }

namespace {
bool socketsReady() { return true; }
void closeSocket(intptr_t s) { ::close(static_cast<int>(s)); }
int sendSome(intptr_t s, const void *data, size_t size) {
  return static_cast<int>(::send(static_cast<int>(s), data, size, 0));
}
int recvSome(intptr_t s, void *data, size_t size) {
  return static_cast<int>(::recv(static_cast<int>(s), data, size, 0));
}
bool interrupted() { return errno == EINTR; }
} // namespace

#endif

namespace {

bool sendAll(intptr_t s, const void *data, size_t size) {
  auto *p = static_cast<const uint8_t *>(data);
  while (size) {
    int n = sendSome(s, p, size);
    if (n < 0 && interrupted())
      continue;
    if (n <= 0)
      return false;
    p += n;
    size -= static_cast<size_t>(n);
  }
  return true;
}

bool recvAll(intptr_t s, void *data, size_t size) {
  auto *p = static_cast<uint8_t *>(data);
  while (size) {
    int n = recvSome(s, p, size);
    if (n < 0 && interrupted())
      continue;
    if (n <= 0)
      return false;
    p += n;
    size -= static_cast<size_t>(n);
  }
  return true;
}

} // namespace

Connection::~Connection() { close(); }

bool Connection::connect() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (alive_)
    return true;
  if (!socketsReady())
    return false;
  int port = MVR_DEFAULT_PORT;
  if (const char *env = std::getenv(MVR_PORT_ENV)) {
    const int parsed = std::atoi(env);
    if (parsed > 0 && parsed < 65536)
      port = parsed;
  }
  auto s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifdef _WIN32
  if (s == INVALID_SOCKET)
    return false;
#else
  if (s < 0)
    return false;
#endif
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(port));
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::connect(s, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0) {
    closeSocket(static_cast<intptr_t>(s));
    return false;
  }
  int one = 1;
  ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char *>(&one), sizeof(one));
#ifndef _WIN32
  ::setsockopt(s, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
  socket_ = static_cast<intptr_t>(s);
  alive_ = true;
  return true;
}

void Connection::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ != -1) {
    closeSocket(socket_);
    socket_ = -1;
  }
  alive_ = false;
}

bool Connection::call(uint16_t type, const void *request, size_t request_size, void *response, size_t response_size,
                      const void *tail, size_t tail_size) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!alive_)
    return false;
  mvr_header header{MVR_MAGIC, type, 0, static_cast<uint32_t>(request_size + tail_size)};
  bool ok = sendAll(socket_, &header, sizeof(header));
  if (ok && request_size)
    ok = sendAll(socket_, request, request_size);
  if (ok && tail_size)
    ok = sendAll(socket_, tail, tail_size);
  mvr_header answer{};
  if (ok)
    ok = recvAll(socket_, &answer, sizeof(answer));
  if (ok)
    ok = answer.magic == MVR_MAGIC && answer.type == (type | MVR_RESPONSE_BIT) && answer.size == response_size;
  if (ok && response_size)
    ok = recvAll(socket_, response, response_size);
  if (!ok) {
    // a half-read stream cannot be resynchronized
    closeSocket(socket_);
    socket_ = -1;
    alive_ = false;
  }
  return ok;
}

bool Connection::syncClock() {
  int64_t best_rtt = INT64_MAX;
  int64_t offset = 0;
  for (int i = 0; i < 8; ++i) {
    mvr_clock_response response{};
    const int64_t before = localNowNs();
    if (!call(MVR_MSG_CLOCK, response))
      return false;
    const int64_t after = localNowNs();
    if (after - before < best_rtt) {
      best_rtt = after - before;
      offset = response.server_time_ns - (before + (after - before) / 2);
    }
  }
  clock_offset_ns_ = offset;
  return true;
}

} // namespace metalvr
