[Home](../../index.md) › [R&D Items](../../index.md) › [Sensing](index.md) › **RD-30**

# RD-30 — ToF Sensor Numbering Contradicts Semantic Order

**Priority:** P3 (Low) · **Status:** Open · **Area:** Sensing · **Source:** `Tof.cpp:91-97`

## Problem

The three rangefinders are named by init order but map to positions out of order:

```text
lox1 -> RIGHT    lox2 -> CENTER    lox3 -> LEFT
```

Anyone wiring hardware or reading the update loop must memorize that 1=right and 3=left — the opposite of natural reading order. This is exactly the kind of latent trap that produces mirrored sensor installs after a hardware revision.

## Proposed Approach

Rename objects and measurement structs to match semantics:

```cpp
Adafruit_VL53L0X loxLeft, loxCenter, loxRight;
```

and order the XSHUT bring-up in `setID()` to assign addresses accordingly. Pure refactor; no behavior change intended.

## Acceptance Criteria

- [ ] Names match physical semantics everywhere; no numeric suffixes remain.
- [ ] Post-refactor bench check: covering each physical sensor moves only its named distance.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-13 Invalid range statuses accepted](rd-13-invalid-range-statuses.md) | [Sensing](index.md) | [Navigation](../navigation/index.md) |
