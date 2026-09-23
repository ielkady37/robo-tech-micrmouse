# Maze Robot — Flood Fill / A* Bug Report

Trace analyzed: manual run from cell (row 8, col 1) to cell (row 8, col 5), where the robot fails to turn and defaults south even though a turn should have been possible.

## Bug 1: Out-of-bounds writes in `updateDistancesAStar()`

The maze is 8×8 (valid indices 0–7), but the goal initialization writes to indices one past the array bounds:

```cpp
distance[targetRow][targetCol + 1] = 0;      // distance[7][8]  -- out of bounds
distance[targetRow + 1][targetCol] = 0;      // distance[8][7]  -- out of bounds
distance[targetRow + 1][targetCol + 1] = 0;  // distance[8][8]  -- out of bounds
```

With no memory protection on the embedded target, these writes silently corrupt adjacent memory — by declaration order, `hasNorthWall[][]`. This runs on every call to `updateDistancesAStar(false, ...)`, i.e. constantly during the trace, and can make wall data appear to change between iterations for no physical reason.

**Fix:** bounds-check before writing/pushing each of the four goal-adjacent cells — only touch the ones that are actually `< MAZE_LENGTH` and `< MAZE_WIDTH`.

## Bug 2: Position and heading update even when a move/turn fails

```cpp
moveInDirection(nextDirection, api);
if (nextDirection == NORTH) currentRow++;   // unconditional
...
currentDirection = nextDirection;           // unconditional
```

and in `turn()`:

```cpp
currentDirection = targetDirection;         // unconditional
```

Neither checks whether the API call actually succeeded, despite the API clearly reporting `STALL` / `BLOCKED -- aborting move` / `aborting turn`. When a move is aborted, the robot may still be physically in the same cell while the algorithm believes it advanced — causing it to log real wall detections against the wrong internal cell.

**Fix:** only commit `currentRow`/`currentCol`/`currentDirection` updates when the API confirms the move/turn completed. Otherwise, re-sense from the actual (unchanged) position and recompute.

## Bug 3: `getNextMovement()` defaults to NORTH when boxed in

```cpp
uint8_t minDistance = distance[currentRow][currentCol];
Direction bestDirection = NORTH;   // default, unconditional
```

If a cell is walled on all sides currently known (with the rest out of bounds), none of the four `if` branches fire, and the function silently falls back to `NORTH` — potentially straight into a wall it just detected. This matches the behavior at (0,0): north and east walls just flagged, south/west out of bounds, so it defaults to NORTH and stalls against the wall.

**Fix:** return an explicit "no valid move" sentinel/error instead of defaulting to NORTH, so `floodFill()` can trigger a recovery/re-plan step rather than ramming a wall.

## Bug 4 (or physical limitation): failed 180° turn at the boundary

At (6,0), "Going South" is a legitimate decision (north and east walled, west out of bounds by definition of column 0). This is the first turn in the trace requiring more than 0° of rotation — every prior step was a straight-through move.

`turn()` resolves the tie for a 180° turn (`leftTurns == rightTurns == 2`) by always turning left:

```cpp
if (leftTurns <= rightTurns) { ... turnLeft() * leftTurns ... }
```

There's no fallback to try right if left stalls, and no check that each individual `turnLeft()` call succeeded before issuing the next one.

Also worth considering: column 0 is the maze's outer boundary, so "west" at any col-0 cell is *always* a wall, not an open path. Combined with the east wall just detected at (6,0), this cell is boxed on both sides — an in-place 180° turn there may be outside the chassis's physical turning clearance, independent of any software bug.

**Fix:**
- Give `turn()` a fallback: if a `turnLeft()` in the sequence stalls, try `turnRight()` instead.
- Track heading incrementally as each sub-turn succeeds, rather than jumping straight to the target direction.
- If turning radius in tight corridors is the actual constraint, add a dedicated maneuver (e.g. back up slightly, then turn) for in-place 180s instead of relying on the open-area turn routine everywhere.

## Priority order for fixes

1. Fix the out-of-bounds writes (Bug 1) — this corrupts data used by everything else.
2. Gate position/heading updates on confirmed API success (Bug 2).
3. Handle the "boxed in" case explicitly instead of defaulting to NORTH (Bug 3).
4. Add a turn-fallback and incremental heading tracking, and consider a tight-corridor turn maneuver (Bug 4).

## Open question

Confirm what `API.h`'s `moveForward()` / `turnLeft()` / `turnRight()` actually return (bool, status enum, or nothing) — this determines how straightforward the Bug 2 fix is to implement.
