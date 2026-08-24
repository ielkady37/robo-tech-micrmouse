[Home](../../index.md) › [R&D Items](../../index.md) › [Navigation](index.md) › **RD-27**

# RD-27 — Distance Array Type Collides with Infinity Sentinel

**Priority:** P3 (Low) · **Status:** Open · **Area:** Navigation · **Source:** `algorithm.cpp:29`, `algorithm.cpp:165`

## Problem

Distances are stored as `uint8_t` while unreachable cells are marked with `UINT8_MAX` (255):

```cpp
uint8_t distance[MAZE_WIDTH][MAZE_LENGTH];
...
distance[r][c] = UINT8_MAX;   // infinity
```

Any real path length of exactly 255 would be indistinguishable from "unreachable". For a 16×16 maze the theoretical maximum simple-path distance is 30, so this cannot trigger today — but the encoding is fragile: adding maze size, weighted edges, or a different metric silently breaks the sentinel.

## Proposed Approach

```cpp
static constexpr uint16_t UNREACHABLE = 0xFFFF;
uint16_t distance[MAZE_WIDTH][MAZE_LENGTH];
```

Cheap (2 KB → still trivial), removes the collision class entirely, and makes intent explicit.

## Acceptance Criteria

- [ ] Named `UNREACHABLE` constant replaces raw `UINT8_MAX`.
- [ ] Sim regression: identical exploration traces after widening the type.

---

| ← Previous | Up | Next → |
|---|---|---|
| [RD-26 Redundant wall readings](rd-26-redundant-wall-readings.md) | [Navigation](index.md) | [Reliability](../reliability/index.md) |
