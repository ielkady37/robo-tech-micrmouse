#pragma once
#include <cstddef>
inline int testFlashWrites = 0;
class Preferences {
public:
  bool begin(const char*, bool) { return true; }
  void end() {}
  void clear() {}
  size_t putBytes(const char*, const void*, size_t size) { ++testFlashWrites; return size; }
  size_t getBytes(const char*, void*, size_t size) { return size; }
};
