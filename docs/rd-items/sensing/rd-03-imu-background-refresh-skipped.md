[Home](../../index.md) › [R&D Items](../../index.md) › [Sensing](index.md) › **RD-03**

# RD-03 — Background Task Never Refreshes the IMU

**Priority:** P1 (High) · **Status:** Open · **Area:** Sensing · **Source:** `Robot.cpp:253-260`

## Problem

The FreeRTOS sensor task is meant to keep both sensor caches fresh between control iterations, but it calls a plain getter instead of the update routine:

```cpp
void Robot::update(void *parameters) {
  while (true) {
    tof.updateReadings();
    imu.getRoll();          // reads cached value; does NOT pump the sensor queue
    vTaskDelay(0);
  }
}
```

`imu.getRoll()` returns stored state; only `imu.update()` drains the BNO08x event queue and refreshes yaw/pitch/roll. Consequently the IMU is only ever updated inside `move()`/`turn()`/`snapToCardinal()` loops, and the "background freshness" design silently does nothing. It also starves the calibration window — see [RD-04](rd-04-imu-calibration-window-broken.md).

## Proposed Approach

```cpp
tof.updateReadings();
imu.update();
vTaskDelay(pdMS_TO_TICKS(5));
```

Pair with [RD-16](../control/rd-16-busy-wait-sensor-task.md) for the delay fix.

## Acceptance Criteria

- [ ] Yaw readings advance while the robot sits still (log proves continuous updates).
- [ ] `move()` sees fresh yaw on its first iteration rather than a stale sample.
- [ ] No mutex contention regressions with concurrent getters ([RD-09](rd-09-pitch-roll-getter-race.md)).

---

| ← Previous | Up | Next → |
|---|---|---|
| [Sensing](index.md) | [Sensing](index.md) | [RD-04 IMU calibration window broken](rd-04-imu-calibration-window-broken.md) |
