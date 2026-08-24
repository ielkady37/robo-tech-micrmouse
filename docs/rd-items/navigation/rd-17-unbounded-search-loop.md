[Home](../../index.md) › [R&D Items](../../index.md) › [Navigation](index.md) › **RD-17**

# RD-17 — Exploration Loop Has No Termination Guard

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Navigation · **Source:** `algorithm.cpp:294`

## Problem

The main solver loop runs until the mouse stands on a zero-distance cell:

```cpp
while (distance[currentRow][currentCol] != 0) { ... }
```

There is no iteration cap and no stuck detection. Combined with the NORTH fallback ([RD-05](rd-05-fallback-move-north-oob.md)) or any future state desync ([RD-14](rd-14-duplicate-direction-state.md)), a failure mode that stops making progress also never stops *moving* — the robot wanders until its battery dies or it hits a wall at speed. On the bench, a sim desync manifests as an infinite loop pegging a core.

## Proposed Approach

Bound the work by maze capacity:

```cpp
const uint32_t kMaxSteps = MAZE_WIDTH * MAZE_LENGTH * 4;  // generous ceiling
uint32_t steps = 0;
while (distance[currentRow][currentCol] != 0) {
  if (++steps > kMaxSteps) { /* halt safely, log diagnostics */ break; }
  ...
}
```

Pair with RD-05's explicit STUCK signal for immediate detection of no-progress states.

## Acceptance Criteria

- [ ] Any non-progressing scenario terminates within bounded steps with a diagnostic log.
- [ ] Normal 16×16 exploration completes far below the cap (assert in sim).

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-14 Duplicate direction state](rd-14-duplicate-direction-state.md) | [Navigation](index.md) | [RD-26 Redundant wall readings](rd-26-redundant-wall-readings.md) |
