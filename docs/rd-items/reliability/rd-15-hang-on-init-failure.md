[Home](../../index.md) › [R&D Items](../../index.md) › [Reliability](index.md) › **RD-15**

# RD-15 — Sensor Init Failures Hang Forever with No Recovery

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Reliability · **Source:** `Robot.cpp:26-30`, `IMU.cpp:26`, `IMU.cpp:33`, `Tof.cpp:61`, `Tof.cpp:73`, `Tof.cpp:85`

## Problem

Every sensor bring-up path handles failure identically:

```cpp
if (!imu.begin()) {
  Serial.println("IMU failed to initialize");
  while (1);   // permanent hang
}
```

One flaky I2C transaction at boot — a marginal connector, brownout glitch, bus contention during ToF re-addressing — strands the robot in a silent loop until someone power-cycles it. There is no retry, no watchdog reset, no fault indication.

## Proposed Approach

Layered fix:

1. Retry each `begin()` N times with a short backoff before declaring failure.
2. Enable the hardware watchdog (`esp_task_wdt`) so any residual hang self-resets.
3. Signal failure externally (blink LED on GPIO2 / log) instead of dying quietly.

```cpp
for (int attempt = 0; attempt < 3; ++attempt) {
  if (imu.begin()) break;
  delay(50);
}
```

## Acceptance Criteria

- [ ] Injected init failure recovers via retry or watchdog reset without manual power-cycle.
- [ ] Failure mode is observable from outside the enclosure.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-06 Flash wear from per-cell saves](rd-06-flash-wear-per-cell-save.md) | [Reliability](index.md) | [RD-19 Mode switch polarity](rd-19-mode-switch-polarity.md) |
