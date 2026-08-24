[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-34**

# RD-34 — Detection Thresholds Lack Units

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `Robot.h:9-10`

## Problem

```cpp
#define THRESHOLD_SIDE 170
#define THRESHOLD_FRONT 70
```

These are millimeters (the VL53L0X reports mm), but nothing says so. A future reader tuning against centimeter assumptions — or a sensor swap reporting a different unit — misconfigures wall detection by an order of magnitude with no compile-time hint. Object-like macros also leak into every translation unit.

## Proposed Approach

```cpp
// Wall-detection thresholds for the VL53L0X array, in millimeters.
constexpr int kThresholdSideMm  = 170;
constexpr int kThresholdFrontMm = 70;
```

Typed constants add scope and unit context at zero runtime cost.

## Acceptance Criteria

- [ ] Every physical-quantity constant names its unit.
- [ ] Thresholds scoped to their consumer rather than global macros.

Related: [RD-28](rd-28-hardcoded-tuning-values.md) covers the same discipline for control gains.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-33 Commented-out legacy implementations](rd-33-commented-legacy-code.md) | [Code Health](index.md) | [Design Evolution](../design-evolution/index.md) |
