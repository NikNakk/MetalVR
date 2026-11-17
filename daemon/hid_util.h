// IOHIDManager access shared by the direct-display backends (backend_rift_s.mm, backend_lighthouse.mm): devices by
// vendor and product, their USB interface, and feature and input reports always with their report id first, which
// is how the protocol parsers take them (macOS hands numbered reports over with or without it).
#pragma once

#import <IOKit/hid/IOHIDManager.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace metalvr::hid {

inline int intProperty(IOHIDDeviceRef device, CFStringRef key) {
  int value = 0;
  CFTypeRef p = IOHIDDeviceGetProperty(device, key);
  if (p && CFGetTypeID(p) == CFNumberGetTypeID())
    CFNumberGetValue(static_cast<CFNumberRef>(p), kCFNumberIntType, &value);
  return value;
}

inline uint16_t vendor(IOHIDDeviceRef d) { return static_cast<uint16_t>(intProperty(d, CFSTR(kIOHIDVendorIDKey))); }
inline uint16_t product(IOHIDDeviceRef d) { return static_cast<uint16_t>(intProperty(d, CFSTR(kIOHIDProductIDKey))); }

// the USB interface a HID device is, from the registry above it; 99 when unknown
inline int interfaceNumber(IOHIDDeviceRef dev) {
  int number = 99;
  io_service_t service = IOHIDDeviceGetService(dev);
  CFTypeRef value = service ? IORegistryEntrySearchCFProperty(service, kIOServicePlane, CFSTR("bInterfaceNumber"),
                                                              kCFAllocatorDefault,
                                                              kIORegistryIterateRecursively | kIORegistryIterateParents)
                            : nullptr;
  if (value) {
    if (CFGetTypeID(value) == CFNumberGetTypeID())
      CFNumberGetValue(static_cast<CFNumberRef>(value), kCFNumberIntType, &number);
    CFRelease(value);
  }
  return number;
}

// the devices a manager matches now, retained (release each with CFRelease)
inline std::vector<IOHIDDeviceRef> devices(IOHIDManagerRef manager) {
  std::vector<IOHIDDeviceRef> out;
  CFSetRef set = IOHIDManagerCopyDevices(manager);
  if (!set)
    return out;
  std::vector<const void *> values(CFSetGetCount(set));
  CFSetGetValues(set, values.data());
  for (const void *v : values) {
    IOHIDDeviceRef d = static_cast<IOHIDDeviceRef>(const_cast<void *>(v));
    CFRetain(d);
    out.push_back(d);
  }
  CFRelease(set);
  return out;
}

inline void release(std::vector<IOHIDDeviceRef> &list) {
  for (IOHIDDeviceRef d : list)
    CFRelease(d);
  list.clear();
}

// a feature report whose first byte is its id
inline bool setFeature(IOHIDDeviceRef device, const uint8_t *report, size_t size) {
  if (!device || size == 0)
    return false;
  return IOHIDDeviceSetReport(device, kIOHIDReportTypeFeature, report[0], report, static_cast<CFIndex>(size)) ==
         kIOReturnSuccess;
}

// feature report `id` into `out`, its id first; how many bytes, 0 if none came
inline size_t getFeature(IOHIDDeviceRef device, uint8_t id, uint8_t *out, size_t size) {
  if (!device || size < 2)
    return 0;
  std::memset(out, 0, size);
  out[0] = id;
  CFIndex length = static_cast<CFIndex>(size);
  if (IOHIDDeviceGetReport(device, kIOHIDReportTypeFeature, id, out, &length) != kIOReturnSuccess || length < 1)
    return 0;
  size_t n = static_cast<size_t>(length);
  if (out[0] != id) {
    // handed over without its id: put it in front
    n = std::min(n + 1, size);
    std::memmove(out + 1, out, n - 1);
    out[0] = id;
  }
  return n;
}

// an input report as the callback gives it, with its id first; `buffer` must hold length + 1 bytes
inline size_t withId(uint32_t id, const uint8_t *report, CFIndex length, uint8_t *buffer, size_t capacity) {
  if (length <= 0)
    return 0;
  size_t n = static_cast<size_t>(length);
  if (report[0] == id) {
    n = std::min(n, capacity);
    std::memcpy(buffer, report, n);
    return n;
  }
  n = std::min(n + 1, capacity);
  buffer[0] = static_cast<uint8_t>(id);
  std::memcpy(buffer + 1, report, n - 1);
  return n;
}

} // namespace metalvr::hid
