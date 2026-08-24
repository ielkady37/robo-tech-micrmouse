[Home](../index.md) › [Design Decisions](index.md) › **Maze Persistence (NVS)**

# Maze Persistence: NVS via Preferences

## The Feature

The explored maze survives power cycles. `algorithm.cpp` implements:

| Function | Behavior |
|---|---|
| `save()` | Writes `hasNorthWall`, `hasEastWall`, and `distance` matrices to NVS using the ESP32 `Preferences` library |
| `loadMatrix()` | Restores the matrices at boot |
| `eraseMatrix()` | Clears stored state |

Boot-time selection is physical: GPIO23 (`INPUT_PULLUP`, `micromouse.ino:78-86`) decides whether `setup()` calls `loadMatrix()`.

## NVS Round-Trip

```mermaid
flowchart LR
    RAM["RAM: hasNorthWall, hasEastWall, distance"] -->|"save()"| NVS["NVS Flash: keys north, east, distance"]
    NVS -->|"loadMatrix()"| RAM
    NVS -->|"eraseMatrix()"| CLR["Cleared"]
```

Boot-time selection: GPIO23 HIGH triggers `loadMatrix()` in `setup()`, restoring the maze from a previous run.

## Why It Matters Strategically

Micromouse competition rewards *speed runs*: explore once slowly, learn the maze, then sprint the shortest path. Persistence is what separates runs — the robot reboots between exploration and sprint with the map intact, skipping the wall arrays' rebuild.

## The Trade-Off

NVS is flash-backed with limited write endurance per entry. `save()` currently fires once per cell traversal inside the `floodFill` loop (`algorithm.cpp:346`) — three full 256-cell matrices rewritten every step. That both slows each step and accumulates wear over a season of practice ([RD-06](../rd-items/reliability/rd-06-flash-wear-per-cell-save.md)).

## Sensible Evolution

- Write only when a **new wall** is discovered (dirty flag), not on every move.
- Or save once at end-of-exploration; mid-run crashes lose at most one run.
- Consider a RAM mirror + explicit commit, matching how `Preferences` commits work anyway.

---

| ← Previous | Up | Next → |
|---|---|---|
| [State Management](state-management.md) | [Design Decisions](index.md) | [Dual-Target Workflow](dual-target-workflow.md) |
