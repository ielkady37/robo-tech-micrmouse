[Home](../../index.md) › [R&D Items](../../index.md) › [Navigation](index.md) › **RD-11**

# RD-11 — Return-to-Start Phase Is Never Activated

**Priority:** P2 (Medium) · **Status:** Open · **Area:** Navigation · **Source:** `algorithm.cpp:291`

## Problem

A `returning` flag threads through the heuristic, the distance-map builder, and neighbor selection precisely to support navigating back to the start after reaching the goal. But it is hardcoded and never flipped:

```cpp
void floodFill(API api) {
  bool returning = false;
  updateDistancesAStar(returning, api);
  while (distance[currentRow][currentCol] != 0) { ... }
  // goal reached -> function exits; mouse stays put
}
```

So the machinery for the return leg exists but is dead: after solving, the mouse simply stops at the goal. In competition format this wastes the run's remaining value — you want the mouse back at start (or sprinting home) under firmware control.

## Proposed Approach

After the goal loop completes:

```cpp
returning = true;
updateDistancesAStar(returning, api);
while (!(currentRow == startingRow && currentCol == startingCol)) {
  // same sense -> choose -> move cycle as the outbound leg
}
```

Verify the heuristic's returning branch targets the start coordinates correctly before enabling.

## Acceptance Criteria

- [ ] Sim: after goal, robot returns to start cell using only known walls (no new sensing required).
- [ ] Total path length outbound + return is consistent with the final distance map.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-10 Rear wall assumption](rd-10-rear-wall-assumption.md) | [Navigation](index.md) | [RD-14 Duplicate direction state](rd-14-duplicate-direction-state.md) |
