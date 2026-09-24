#pragma once
#include <cstdint>
#include <deque>
#include <functional>
#include <vector>
constexpr int SH2_ARVR_STABILIZED_RV = 40;
struct sh2_SensorValue_t {
  int sensorId = SH2_ARVR_STABILIZED_RV;
  uint64_t timestamp = 0;
  uint8_t sequence = 0;
  struct {
    struct { float real = 1, i = 0, j = 0, k = 0; } arvrStabilizedRV;
  } un;
};
inline std::deque<sh2_SensorValue_t> testImuEvents;
inline uint32_t testReportIntervalUs = 0;
inline bool testImuReset = false;
inline bool testReportsEnabled = false;
inline bool testEnableReportSucceeds = true;
inline int testBnoBeginCalls = 0;
inline sh2_SensorValue_t* testCallbackBuffer = nullptr;
inline std::vector<uint32_t> testReportRequests;
inline std::function<void(sh2_SensorValue_t*)> testOnEnableReport;
class Adafruit_BNO08x {
public:
  bool begin_I2C(int) {
    ++testBnoBeginCalls;
    testImuReset = true;
    testReportsEnabled = false;
    return true;
  }
  bool wasReset() { bool reset = testImuReset; testImuReset = false; return reset; }
  bool enableReport(int, uint32_t interval) {
    testReportIntervalUs = interval;
    testReportRequests.push_back(millis());
    if (testOnEnableReport) testOnEnableReport(testCallbackBuffer);
    if (testEnableReportSucceeds) testReportsEnabled = true;
    return testEnableReportSucceeds;
  }
  bool getSensorEvent(sh2_SensorValue_t* value) {
    testCallbackBuffer = value;
    if (!testReportsEnabled || testImuEvents.empty()) return false;
    *value = testImuEvents.front();
    testImuEvents.pop_front();
    return true;
  }
};
