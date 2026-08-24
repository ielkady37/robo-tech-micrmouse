[Home](../../index.md) › [R&D Items](../../index.md) › [Sensing](index.md) › **RD-12**

# RD-12 — Zero ToF Reading Is Interpreted as "Wall Ahead"

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Sensing · **Source:** `Robot.cpp:47-52`, `Robot.cpp:174`

## Problem

ToF distances live in members that start at zero and remain zero until the first successful ranging pass. Both consumers interpret zero as an obstruction:

```cpp
bool Robot::isWallFront() {
  if (tof.getTofCenter() <= THRESHOLD_FRONT) return true;   // 0 <= 70 -> wall!
```

```cpp
if (tof.getTofCenter() < 40) break;   // move(): e-stops immediately on stale 0
```

So before the sensors ever report, the robot believes a wall is directly ahead — and `move()` aborts every cell transition instantly if the cache happens to be cold.

## Proposed Approach

Define "no data" explicitly and filter it:

```cpp
uint16_t d = tof.getTofCenter();
if (d != 0 && d != 65535 && d <= THRESHOLD_FRONT) return true;
```

Apply the same guard in the emergency-stop check, optionally falling back to odometry-only completion when the sensor has never produced a valid sample.

## Acceptance Criteria

- [ ] Cold boot with sensors delayed: no phantom walls reported, moves proceed.
- [ ] Genuine close obstruction (< 40 mm valid reading) still e-stops.
- [ ] Out-of-range sentinel (65535) also handled consistently.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-09 Pitch/Roll getter race](rd-09-pitch-roll-getter-race.md) | [Sensing](index.md) | [RD-13 Invalid range statuses accepted](rd-13-invalid-range-statuses.md) |
