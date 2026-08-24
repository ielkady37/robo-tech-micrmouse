[Home](../../index.md) › [R&D Items](../../index.md) › [Control & Motion](index.md) › **RD-07**

# RD-07 — Drift Calibration Drives Both Motors at Full Speed Unattended

**Priority:** P1 (High) · **Status:** Open · **Area:** Control & Motion · **Source:** `Robot.cpp:292`

## Problem

`calibrateDriftFactor()` measures left/right odometry mismatch by running both motors open-loop:

```cpp
motor_driver.setMotors(255, 255);
delay(5000);
motor_driver.setMotors(0, 0);
```

That is maximum 8-bit PWM — roughly four times the normal driving cap (`MAX_SPEED_FORWARD ≈ 65`) — held for five seconds with no obstacle check, no encoder-based stop, and no way to abort. Placed on the ground, the robot simply accelerates away for five meters.

## Proposed Approach

```cpp
motor_driver.setMotors(60, 60);                    // moderate speed
uint32_t t0 = millis();
while (millis() - t0 < 5000 && tof.getTofCenter() > 60) {
  // optional: also bail on encoder delta > expected max travel
}
motor_driver.setMotors(0, 0);
```

Also consider bench mode: calibration only makes sense with wheels off the ground unless measuring under load deliberately.

## Acceptance Criteria

- [ ] Routine cannot exceed a bounded distance regardless of duration.
- [ ] Center ToF obstruction aborts within one control period.
- [ ] Resulting drift factor is persisted or surfaced (see related [RD-24](../code-health/rd-24-stubbed-drift-calibration.md)).

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-02 Turn feed-forward prevents settling](rd-02-turn-feedforward-prevents-settling.md) | [Control & Motion](index.md) | [RD-08 PID signal truncation](rd-08-pid-signal-truncation.md) |
