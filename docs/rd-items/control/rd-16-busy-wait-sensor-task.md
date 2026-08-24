[Home](../../index.md) › [R&D Items](../../index.md) › [Control & Motion](index.md) › **RD-16**

# RD-16 — Sensor Task Busy-Waits with `vTaskDelay(0)`

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Control & Motion · **Source:** `Robot.cpp:258`

## Problem

The background sensor task ends each iteration with:

```cpp
vTaskDelay(0);
```

`vTaskDelay(0)` only yields to tasks of *equal* priority — it does not block. As the sole priority-1 task on its core, the task effectively spins between I2C transactions, burning CPU that the idle task (and anything else sharing that core) needs for housekeeping.

## Proposed Approach

Block for a period matched to the sensors' useful update rate:

```cpp
vTaskDelay(pdMS_TO_TICKS(5));   // ~200 Hz polling ceiling
```

Coordinate with the fix for [RD-03](../sensing/rd-03-imu-background-refresh-skipped.md) since the loop body changes there too.

## Acceptance Criteria

- [ ] Idle CPU on the sensor core returns to the idle task between samples (observable via `uxTaskGetSystemState` or CPU-load print).
- [ ] Sensor cache latency stays acceptable for `move()`'s emergency stop check.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-08 PID signal truncation](rd-08-pid-signal-truncation.md) | [Control & Motion](index.md) | [RD-29 Hardcoded motor trim](rd-29-hardcoded-motor-trim.md) |
