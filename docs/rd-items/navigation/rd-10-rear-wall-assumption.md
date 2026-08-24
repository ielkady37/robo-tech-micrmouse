[Home](../../index.md) › [R&D Items](../../index.md) › [Navigation](index.md) › **RD-10**

# RD-10 — Rear-Direction Wall Query Silently Answers "No Wall"

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Navigation · **Source:** `algorithm.cpp:149-154`

## Problem

Wall queries exist only for relative front/left/right. When the solver asks for an absolute direction that happens to be behind the mouse, control falls through:

```cpp
if (direction == static_cast<Direction>((currentDirection + 1) % 4)) return api.wallRight();
return false;   // "Should never reach here"
```

The comment is wrong — this line *is* reached whenever the queried absolute direction equals `currentDirection + 2`. The flood-fill loop probes all four absolutes per cell, so every step asks exactly one rear-direction question and gets an unconditional "open" answer. A wall behind (known or unknown) is recorded as absent.

## Proposed Approach

Resolve rear queries from stored maze knowledge instead of sensors: compute the cell behind and return its facing-wall flag from the North/East arrays ([Wall Encoding](../../design-decisions/wall-encoding.md)). If that neighbor is outside the maze, return true (border). Update the comment regardless.

## Acceptance Criteria

- [ ] All four absolute directions return meaningful answers at every in-maze pose.
- [ ] Sim regression: walls behind the mouse are never marked as absent after being seen.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-05 Fallback move NORTH](rd-05-fallback-move-north-oob.md) | [Navigation](index.md) | [RD-11 Return phase unimplemented](rd-11-return-phase-unimplemented.md) |
