[Home](../../index.md) › [R&D Items](../../index.md) › [Control & Motion](index.md) › **RD-02**

# RD-02 — Unconditional Feed-Forward Prevents Turn Settling

**Priority:** P1 (High) · **Status:** Open · **Area:** Control & Motion · **Source:** `Robot.cpp:226-227`

## Problem

Inside the `turn()` loop, whenever `target > 0` (right turn) a constant `±15` is added to the motor command every iteration regardless of the PID state. When the controller output has already settled to zero inside the deadband, this bias keeps the wheels spinning at speed 15 — so the robot overshoots, re-enters the deadband from the other side, and limit-cycles. It also makes the `stableCount` exit condition unreliable. The bias applies asymmetrically (right turns only), so left turns behave differently.

## Evidence

```cpp
if (target > 0)
  speed += speed < 0 ? -15 : 15;
```

## Proposed Approach

Gate the feed-forward by remaining error and apply it symmetrically:

```cpp
if (fabs(error) > 10.0f) {          // kick only far from target
  int ff = (target > 0) ? 15 : -15;
  speed += (speed >= 0) ? ff : -ff;
}
```

Alternatively remove it entirely and let PD handle the full profile.

## Acceptance Criteria

- [ ] 90° turns settle without visible hunting at the end of the move.
- [ ] Left and right 90° turns show symmetric settle time and overshoot.
- [ ] 180° about-face completes reliably using the same code path.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-01 Heading correction disabled](rd-01-heading-correction-disabled.md) | [Control & Motion](index.md) | [RD-07 Full-speed drift calibration](rd-07-full-speed-drift-calibration.md) |
