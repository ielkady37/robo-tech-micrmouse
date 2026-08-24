[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-22**

# RD-22 — Unused, Inconsistent IMU Report Members

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `IMU.h:33-35`

## Problem

The IMU class declares report configuration that never participates in the active path:

```cpp
void setReports();                                    // declared, never defined or called
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;   // never read; wrong report anyway
long reportIntervalUs = 1000;                         // never read
```

Meanwhile `begin()` actually enables `SH2_ARVR_STABILIZED_RV` and `update()` filters for the same. So the members document a *different* sensor configuration than the one running, and `setReports()` is a declaration with no definition waiting to break a future caller.

## Proposed Approach

Either delete all three members, or implement `setReports()` properly and call it from `begin()` so report selection has exactly one definition of truth.

## Acceptance Criteria

- [ ] No declared-but-undefined functions in `IMU`.
- [ ] Configured report type appears in exactly one place and matches what `update()` consumes.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-21 Dead position state in API](rd-21-dead-position-state-in-api.md) | [Code Health](index.md) | [RD-23 Unused SparkFun include](rd-23-unused-sparkfun-include.md) |
