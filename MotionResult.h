#ifndef MOTION_RESULT_H
#define MOTION_RESULT_H

enum class MotionResult {
  Completed,
  Stalled,
  ImuTimeout,
  TurnTimeout,
  Blocked
};

inline const char* motionResultName(MotionResult result) {
  switch (result) {
    case MotionResult::Completed: return "Completed";
    case MotionResult::Stalled: return "Stalled";
    case MotionResult::ImuTimeout: return "ImuTimeout";
    case MotionResult::TurnTimeout: return "TurnTimeout";
    case MotionResult::Blocked: return "Blocked";
  }
  return "Unknown";
}

#endif
