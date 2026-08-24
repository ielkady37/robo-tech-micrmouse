[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-23**

# RD-23 — Unused SparkFun Driver Include Alongside Active Adafruit Driver

**Priority:** P3 (Low) · **Status:** Open · **Area:** Code Health · **Source:** `IMU.h:6`

## Problem

The IMU pulls in two competing drivers:

```cpp
#include <Adafruit_BNO08x.h>                          // active
#include "SparkFun_BNO080_Arduino_Library.h"         // only referenced by dead code
```

The active implementation is Adafruit's; the SparkFun library appears solely inside a commented-out legacy block (`IMU.cpp:117-218`, see [RD-33](rd-33-commented-legacy-code.md)). The stray include slows compilation, risks symbol/macro collisions between two vendors' BNO0x stacks, and confuses readers about which driver owns the hardware.

## Proposed Approach

Delete the include. If the SparkFun variant is ever wanted again, git history preserves it better than a live include feeding dead code.

## Acceptance Criteria

- [ ] Exactly one IMU driver include remains.
- [ ] Clean build verified on the target toolchain.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-22 Unused IMU report members](rd-22-unused-imu-report-members.md) | [Code Health](index.md) | [RD-24 Stubbed drift calibration](rd-24-stubbed-drift-calibration.md) |
