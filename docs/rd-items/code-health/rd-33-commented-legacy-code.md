[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-33**

# RD-33 — ~180 Lines of Commented-Out Legacy Implementations

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `Robot.cpp:332-414` (old `move()`), `IMU.cpp:117-218` (SparkFun BNO080 variant)

## Problem

Two large commented-out blocks preserve entire alternative implementations: the previous motion routine and a complete second vendor's IMU driver integration. Costs:

- Readers cannot tell which code is real without mentally diffing both versions.
- The blocks rot silently as active code evolves; "restoring" them later would break.
- They feed dead includes ([RD-23](rd-23-unused-sparkfun-include.md)) and confuse search/grep across the repo.

Git history already preserves every one of these lines forever.

## Proposed Approach

Delete both blocks. If either variant is plausibly wanted again (e.g., the SparkFun driver for its onboard fusion features), note that in the commit message where it's removed.

## Acceptance Criteria

- [ ] No commented-out function bodies remain in `Robot.cpp` or `IMU.cpp`.
- [ ] File line counts drop by roughly 180 lines combined.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-32 Debug string quality](rd-32-debug-string-quality.md) | [Code Health](index.md) | [RD-34 Threshold units undocumented](rd-34-threshold-units-undocumented.md) |
