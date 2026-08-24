[Home](../../index.md) › [R&D Items](../../index.md) › [Sensing](index.md) › **RD-04**

# RD-04 — IMU Calibration Window Never Starts

**Priority:** P1 (High) · **Status:** Open · **Area:** Sensing · **Source:** `IMU.cpp:60` (with `IMU.cpp:15`, `Robot.cpp:44`)

## Problem

The IMU averages yaw over a five-second window after boot to compute `yawOffset`. But the window's start timestamp is initialized once in the constructor and never set again in the active code path:

```cpp
// constructor
calibStart = 0;
...
// update()
if (millis() - calibStart < 5000) {   // measures time since boot, not since begin()
  sum += yawNow; count++;
} else {
  if (count > 0) { yawOffset = sum / count; }
  else           { yawOffset = yawNow; }   // fallback always taken today
```

Two compounding failures:

1. `calibStart` stays `0`, so the gate compares against uptime — the window is already "over" before sampling begins.
2. Even inside the window, nothing calls `imu.update()` during it (the background task bug, [RD-03](rd-03-imu-background-refresh-skipped.md)), so `count == 0` regardless.

Net effect: the intended averaged calibration never happens; the fallback single-sample offset is always used.

## Proposed Approach

In `IMU::begin()`, after the sensor is confirmed ready: `calibStart = millis(); sum = 0; count = 0;` — and guarantee `update()` is actually called during the window (fix RD-03 first, or loop `update()` inside `begin()`'s existing 5 s settle delay).

## Acceptance Criteria

- [ ] After boot, logs show `count > 100` samples contributing to the average.
- [ ] Yaw reads ~0° at rest post-calibration across several boots.
- [ ] Mounting orientation changes produce correct re-zeroing without code edits.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-03 IMU background refresh skipped](rd-03-imu-background-refresh-skipped.md) | [Sensing](index.md) | [RD-09 Pitch/Roll getter race](rd-09-pitch-roll-getter-race.md) |
