#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <functional>
#include <string>
#include <vector>
#include <sstream>

#define IRAM_ATTR
#define PI 3.14159265358979323846
#define constrain(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))
using SemaphoreHandle_t = void*;
constexpr int pdTRUE = 1, pdPASS = 1, portMAX_DELAY = -1;
inline uint32_t testNowMs = 0;
inline std::function<void()> testTick;
inline unsigned long millis() { return testNowMs; }
inline unsigned long micros() { return testNowMs * 1000UL; }
inline void delay(unsigned long ms) {
  while (ms--) { ++testNowMs; if (testTick) testTick(); }
}
inline void vTaskDelay(int ticks) { delay(ticks); }
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return reinterpret_cast<void*>(1); }
inline int testMutexDepth = 0;
inline int xSemaphoreTake(SemaphoreHandle_t, int) { ++testMutexDepth; return pdTRUE; }
inline void xSemaphoreGive(SemaphoreHandle_t) { --testMutexDepth; }
inline void (*testTask)(void*) = nullptr;
inline int xTaskCreatePinnedToCore(void (*task)(void*), const char*, int, void*, int, void*, int) {
  testTask = task;
  return pdPASS;
}
struct TestSerial {
  std::vector<std::string> lines;
  std::string pending;
  void begin(int) {}
  template <typename T> void print(const T& value) {
    std::ostringstream text;
    text << value;
    pending += text.str();
  }
  void print(uint8_t value) { print(static_cast<unsigned int>(value)); }
  template <typename T> void println(const T& value) { print(value); println(); }
  void println() { lines.push_back(pending); pending.clear(); }
};
inline TestSerial Serial;
