[Home](../../index.md) › [R&D Items](../../index.md) › [Code Health](index.md) › **RD-01**

# RD-01 — Heading Correction Is Multiplied by Zero

**Priority:** P0 (Critical) · **Status:** Open · **Area:** Control & Motion · **Source:** `Robot.cpp:132-133`

## Problem

The straight-line controller computes a heading-hold PID term and then multiplies it by zero before mixing it into the wheel speeds:

```cpp
float rightSpeed = baseSpeed - pid_heading*0;
float leftSpeed  = baseSpeed + pid_heading*0;
```

Both wheels always receive `baseSpeed`. The robot cannot correct yaw drift while driving forward, so every cell transition accumulates heading error that later steps must undo. This is almost certainly a debugging leftover that was never restored — it silently disables an entire control loop.

## Proposed Approach

```cpp
float rightSpeed = baseSpeed - pid_heading;
float leftSpeed  = baseSpeed + pid_heading;
```

Verify sign conventions against motor wiring and the yaw axis before committing (a flipped sign turns correction into positive feedback).

## Acceptance Criteria

- [ ] Straight run of N cells ends within a small yaw tolerance of start heading.
- [ ] Deliberately skewing the robot mid-run converges back to line without oscillation.
- [ ] No interaction regression with the distance PID ([RD-08](rd-08-pid-signal-truncation.md)).

---

| ← Previous | Up | Next → |
|---|---|---|
| [Control & Motion](index.md) | [Control & Motion](index.md) | [RD-02 Turn feed-forward prevents settling](rd-02-turn-feedforward-prevents-settling.md) |
