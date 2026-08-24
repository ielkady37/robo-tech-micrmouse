[Home](../../index.md) › [R&D Items](../../index.md) › [Sensing](index.md) › **RD-09**

# RD-09 — `getPitch()`/`getRoll()` Read Shared State Without the Mutex

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Sensing · **Source:** `IMU.cpp:109-114`

## Problem

The writer side (`update()`) stores yaw/pitch/roll under `imu_Mutex`, but two of the three getters bypass it:

```cpp
float IMU::getYaw()  { /* takes imu_Mutex */ }
float IMU::getPitch() { return pitch; }   // no mutex
float IMU::getRoll()  { return roll;  }   // no mutex
```

Once the background task actually refreshes the IMU ([RD-03](rd-03-imu-background-refresh-skipped.md)), concurrent readers can observe torn or stale `long double` values. Today the race is masked by RD-03 (the background task never writes), so fixing one without the other would *introduce* the race.

## Proposed Approach

Mirror `getYaw()`'s locking in both getters — or refactor to a single locked snapshot getter returning all three axes, which also halves mutex traffic for callers needing multiple axes.

## Acceptance Criteria

- [ ] All shared IMU state is read under `imu_Mutex`.
- [ ] Stress test: background updates + getter loop show no torn values over 10⁵ reads.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-04 IMU calibration window broken](rd-04-imu-calibration-window-broken.md) | [Sensing](index.md) | [RD-12 Zero distance read as wall](rd-12-zero-distance-read-as-wall.md) |
