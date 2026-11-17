#include "backend_win32.h"

namespace metalvr {

bool mullionShareNameFromHandle(HANDLE handle, char out[MVR_SHARE_NAME_LEN]) {
  static const char prefix[] = "MULLION_shared_";
  if (!handle)
    return false;
  bool found = false;
  if (void *view = MapViewOfFile(handle, FILE_MAP_READ, 0, 0, 0)) {
    MEMORY_BASIC_INFORMATION region{};
    if (VirtualQuery(view, &region, sizeof(region)) && region.RegionSize > sizeof(prefix)) {
      const char *bytes = static_cast<const char *>(view);
      const size_t size = region.RegionSize;
      for (size_t i = 0; i + sizeof(prefix) < size && !found; ++i) {
        if (std::memcmp(bytes + i, prefix, sizeof(prefix) - 1))
          continue;
        const size_t digits = 32, length = sizeof(prefix) - 1 + digits;
        if (i + length >= size || bytes[i + length])
          continue;
        bool hex = true;
        for (size_t d = sizeof(prefix) - 1; d < length && hex; ++d) {
          const char c = bytes[i + d];
          hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
        }
        if (!hex)
          continue;
        std::memset(out, 0, MVR_SHARE_NAME_LEN);
        std::memcpy(out, bytes + i, length);
        found = true;
      }
    }
    UnmapViewOfFile(view);
  }
  CloseHandle(handle);
  return found;
}

} // namespace metalvr
