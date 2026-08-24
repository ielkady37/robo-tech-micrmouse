[Home](../index.md) › **Algorithms**

# Algorithms

What the firmware computes and how it moves.

## Pages

| Page | Contents |
|---|---|
| [Maze Exploration Loop](maze-exploration.md) | `floodFill`: sense → mark walls → recompute → step → save |
| [A\*-Style Distance Map](astar-distance-map.md) | Heuristic, relaxation to fixpoint, unreachable sentinel |
| [Movement Primitives](movement-primitives.md) | Neighbor selection, relative/absolute direction mapping |
| [PID Motion Control](pid-motion-control.md) | Distance + heading control in `move()`, yaw loop in `turn()` |

## The Big Picture

The solver is a **gradient follower**: a distance map assigns every cell its heuristic cost-to-goal; the robot repeatedly walks downhill (to the lowest-distance open neighbor) while updating the map with newly seen walls. Because the map is recomputed globally after discoveries ([A\*-Style Distance Map](astar-distance-map.md)), the robot can backtrack out of dead ends without explicit path memory — the gradient encodes everything.

Motion is layered beneath: [Movement Primitives](movement-primitives.md) decide *where* to go one cell at a time; [PID Motion Control](pid-motion-control.md) makes each cell transition and turn physically happen.

Open gaps in this pipeline are tracked as R&D items: out-of-bounds fallback ([RD-05](../rd-items/navigation/rd-05-fallback-move-north-oob.md)), missing return phase ([RD-11](../rd-items/navigation/rd-11-return-phase-unimplemented.md)), unbounded search ([RD-17](../rd-items/navigation/rd-17-unbounded-search-loop.md)).

---

| ← Previous | Up | Next → |
|---|---|---|
| [Dual-Target Workflow](../design-decisions/dual-target-workflow.md) | **Algorithms** | [Maze Exploration Loop](maze-exploration.md) |
