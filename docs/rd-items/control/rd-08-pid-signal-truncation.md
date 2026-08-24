[Home](../../index.md) › [R&D Items](../../index.md) › [Control & Motion](index.md) › **RD-08**

# RD-08 — Integer `pidSignal` Truncates Float PID Output

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Control & Motion · **Source:** `Robot.cpp:199`, `Robot.cpp:212`

## Problem

The turn PID stores its output in an `int`:

```cpp
int pidSignal;
...
pidSignal = kp * error + kd * derv;
```

The assignment truncates the fractional part of the float expression *before* it is scaled into wheel speeds. At small errors — exactly where fine settling matters — quantization causes chatter or premature deadband entry, worsening the settling issues tracked in [RD-02](rd-02-turn-feedforward-prevents-settling.md).

## Proposed Approach

Change the declaration to `float pidSignal;` and keep everything downstream in float until the final `setMotors()` call, which already converts.

## Acceptance Criteria

- [ ] No integer cast between PID computation and motor command scaling.
- [ ] Turn settle behavior measurably equal or better than baseline (log settle counts).

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-07 Full-speed drift calibration](rd-07-full-speed-drift-calibration.md) | [Control & Motion](index.md) | [RD-16 Busy-wait in sensor task](rd-16-busy-wait-sensor-task.md) |
