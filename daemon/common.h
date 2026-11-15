#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <ctime>

namespace metalvr {

// the server clock every protocol time is in
inline int64_t nowNs() { return static_cast<int64_t>(clock_gettime_nsec_np(CLOCK_UPTIME_RAW)); }

extern bool g_verbose;

__attribute__((format(printf, 1, 2))) inline void logInfo(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  std::fputs("[metalvrd] ", stderr);
  std::vfprintf(stderr, fmt, args);
  std::fputc('\n', stderr);
  va_end(args);
}

__attribute__((format(printf, 1, 2))) inline void logDebug(const char *fmt, ...) {
  if (!g_verbose)
    return;
  va_list args;
  va_start(args, fmt);
  std::fputs("[metalvrd] ", stderr);
  std::vfprintf(stderr, fmt, args);
  std::fputc('\n', stderr);
  va_end(args);
}

} // namespace metalvr
