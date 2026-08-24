[Home](../../index.md) › [R&D Items](../../index.md) › [Reliability](index.md) › **RD-06**

# RD-06 — Full Maze Written to Flash After Every Cell Move

**Priority:** P1 (High) · **Status:** Open · **Area:** Reliability · **Source:** `algorithm.cpp:346`, `algorithm.cpp:36-42`

## Problem

The exploration loop ends every iteration with:

```cpp
moveInDirection(nextDirection, api);
...
currentDirection = nextDirection;
save();          // every single cell
```

`save()` serializes all three 16×16 matrices (`hasNorthWall`, `hasEastWall`, `distance`) into NVS. Consequences:

1. **Flash wear**: NVS entries have limited erase cycles; hundreds of full-array commits per run, times every practice run, accumulates toward endurance limits.
2. **Latency**: each commit stalls the control loop for flash erase/program time — dead time in the hottest path of exploration.
3. Most writes are *no-ops*: typically nothing new was learned while traversing an already-explored corridor.

## Proposed Approach

Only persist when knowledge changed: set a dirty flag when a genuinely new wall is recorded (composes naturally with [RD-26](../navigation/rd-26-redundant-wall-readings.md)), and call `save()` only when it is set — or simply save once at end-of-exploration. A mid-exploration crash then costs at most the current run's discoveries.

## Acceptance Criteria

- [ ] NVS commit count for a full 16×16 exploration drops by ≥ 10× (log counter).
- [ ] Power-cycle after any point still restores at least last-committed consistent state.
- [ ] No measurable per-cell latency regression.

---

| ← Previous | Up | Next → |
|---|---|---|
| [Reliability](index.md) | [Reliability](index.md) | [RD-15 Hang on init failure](rd-15-hang-on-init-failure.md) |
