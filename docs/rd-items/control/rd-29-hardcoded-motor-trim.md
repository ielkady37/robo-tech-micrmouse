[Home](../../index.md) › [R&D Items](../../index.md) › [Control & Motion](index.md) › **RD-29**

# RD-29 — Hardcoded ×0.98 Left-Motor Trim

**Priority:** P3 (Low) · **Status:** Open · **Area:** Control & Motion · **Source:** `motor.cpp:109`

## Problem

The left PWM channel carries an undocumented fixed trim:

```cpp
analogWrite(speedL, abs(leftSpeed)*0.98);
analogWrite(speedR, abs(rightSpeed));
```

A 2% hand-tuned reduction compensates motor imbalance today, but nothing records what it was tuned for, and it is disconnected from `calibrateDriftFactor()`, which exists precisely to measure this imbalance ([RD-24](../code-health/rd-24-stubbed-drift-calibration.md)). Any hardware swap invalidates it silently.

## Proposed Approach

1. Name it: `constexpr float LEFT_PWM_TRIM = 0.98f;` with a comment on provenance.
2. Preferably load it from NVS after a successful drift calibration, defaulting to 1.0.

## Acceptance Criteria

- [ ] Constant is named and documented.
- [ ] Value source (NVS-persisted calibration or compile-time constant) is explicit at the call site.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-16 Busy-wait in sensor task](rd-16-busy-wait-sensor-task.md) | [Control & Motion](index.md) | [Sensing](../sensing/index.md) |
