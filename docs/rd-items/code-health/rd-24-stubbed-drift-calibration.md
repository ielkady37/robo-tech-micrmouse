[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-24**

# RD-24 — Drift-Factor Feature Is Stubbed Out End-to-End

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `Robot.h:25-26`, `Robot.cpp:286-328`

## Problem

A drift-calibration feature exists in fragments that never connect:

- `calibrateDriftFactor()` measures left/right odometry asymmetry but never persists the result — the NVS write is commented out.
- Its return value has no caller that uses it.
- `getDriftFactor()`'s body is entirely commented out.
- Private member `float heading;` (`Robot.h:34`) is never read or written by anything.

Meanwhile the drivetrain applies a hardcoded ×0.98 trim ([RD-29](../control/rd-29-hardcoded-motor-trim.md)) that this feature was presumably meant to replace, and the measurement routine itself is unsafe to run ([RD-07](../control/rd-07-full-speed-drift-calibration.md)).

## Proposed Approach

Decide the feature's fate explicitly:

1. **Complete it**: persist the factor via `Preferences`, load at boot, feed it into `setMotors()` as the trim source; delete the unused `heading` member.
2. **Remove it**: delete the stubbed methods and member; keep the named constant trim until a replacement exists.

Half-present features cost more than absent ones.

## Acceptance Criteria

- [ ] Either fully wired (measure → persist → apply) or fully removed; no stubs remain.
- [ ] Trim provenance traceable to calibration output or documented constant.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-23 Unused SparkFun include](rd-23-unused-sparkfun-include.md) | [Code Health](index.md) | [RD-25 Dead cell helpers](rd-25-dead-cell-helpers.md) |
