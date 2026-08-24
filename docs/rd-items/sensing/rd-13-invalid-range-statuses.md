[Home](../../index.md) › [R&D Items](../../index.md) › [Sensing](index.md) › **RD-13**

# RD-13 — Invalid Range Statuses Are Accepted as Valid Distances

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Sensing · **Source:** `Tof.cpp:96`, `Tof.cpp:104`, `Tof.cpp:112`

## Problem

Each ranging pass keeps the measured value unless the status equals 4 (out of range):

```cpp
if (measure3.RangeStatus != 4)
  leftDistance = measure3.RangeMilliMeter;
else
  leftDistance = 65535;
```

The VL53L0X defines other failure statuses — signal fail, phase-bounds error, hardware fault — whose `RangeMilliMeter` payloads are meaningless. Today those garbage values flow into the wall-detection thresholds, producing phantom walls or missed walls depending on what the failing sensor returns.

## Proposed Approach

Invert the test to accept only known-good data:

```cpp
if (measure3.RangeStatus == 0)            // VL53L0X_RANGE_VALID
  leftDistance = measure3.RangeMilliMeter;
else
  leftDistance = 65535;                   // treat as open until proven otherwise
```

Coordinate with [RD-12](rd-12-zero-distance-read-as-wall.md) so consumers share one validity predicate instead of each re-implementing it.

## Acceptance Criteria

- [ ] Unit test or bench log demonstrates error statuses map to the sentinel.
- [ ] Wall detection unchanged for normal valid readings.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-12 Zero distance read as wall](rd-12-zero-distance-read-as-wall.md) | [Sensing](index.md) | [RD-30 ToF sensor naming contradicts semantics](rd-30-tof-sensor-naming.md) |
