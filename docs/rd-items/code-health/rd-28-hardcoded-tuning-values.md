[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-28**

# RD-28 — PID Gains and Limits Live as Scattered Magic Numbers

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `Robot.cpp:5-15`, `Robot.cpp:88-93`, `Robot.cpp:191-192`

## Problem

Every control constant is a bare literal or local define: distance/heading gains (`kp_dist`, `kd_dist`, `kp_heading`, `kd_heading`), turn gains (`kp`, `kd`), speed caps, deadbands, settle counters, tolerances. None carry units, tuning dates, or rationale. Consequences:

- Tuning on the robot means re-flashing blind guesses; there is no persistence or runtime adjustment.
- Two separate gain sets exist without documented relationships.
- Constants with units (PWM counts, cm, degrees, ms) are indistinguishable from dimensionless ratios.

## Proposed Approach

1. Collect into one named configuration struct with units in comments:

```cpp
struct PidConfig {
  float kpDist;        // PWM per cm of remaining distance
  float kdDist;        // PWM per cm/s
  float kpHeading;     // PWM per degree yaw error
  // ...
};
```

2. Optionally persist tunables to NVS so field tuning survives power cycles.

## Acceptance Criteria

- [ ] Single location defines every control constant, each with units.
- [ ] No unnamed numeric literals remain in `move()`/`turn()` bodies.

Related: [RD-29](../control/rd-29-hardcoded-motor-trim.md), [RD-34](rd-34-threshold-units-undocumented.md).

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-25 Dead cell helpers](rd-25-dead-cell-helpers.md) | [Code Health](index.md) | [RD-31 Dead code in sketch](rd-31-dead-code-in-sketch.md) |
