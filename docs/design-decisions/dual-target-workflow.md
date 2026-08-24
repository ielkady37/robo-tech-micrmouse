[Home](../index.md) › [Design Decisions](index.md) › **Dual-Target Workflow**

# Dual-Target Workflow: Simulator First, Hardware Second

## The Idea

`algorithm.cpp` contains no hardware code. It talks exclusively to the abstract vocabulary inherited from the Micromouse Simulator:

```text
wallFront() / wallLeft() / wallRight()
moveForward() / turnRight() / turnLeft()
setColor() / setText()
```

Two adapters implement that vocabulary:

| Adapter | Status | Notes |
|---|---|---|
| `API` (simulator) | **Active** — wired into `micromouse.ino` | Fast iteration on solver logic with zero risk to hardware |
| `Robot` (hardware) | Written, currently commented out of the sketch | `isWallFront()` etc. mirror the same semantics over real sensors |

This lets solver development proceed in simulation while motion control (`Robot::move/turn`) matures separately against the real drivetrain — historically visible in the sketch's commented-out choreography of `robot.move(1)` / `robot.turn(90)` calls (`micromouse.ino:1-73`).

## How Switching Works Today

Switching targets is done by editing the sketch: comment out the `API`/`algorithm` block, uncomment the `Robot` block. There is no compile-time abstraction (interface class, template parameter, or build flag) behind the choice.

## Known Friction

- The two adapters are only informally equivalent — e.g., simulator `turnLeft()` rotates exactly 90°, while hardware `turn(90)` is a closed-loop approximation with settling behavior ([PID Motion Control](../algorithms/pid-motion-control.md)).
- `Robot`'s drift calibration and heading features are stubbed ([RD-24](../rd-items/code-health/rd-24-stubbed-drift-calibration.md)).
- Dead legacy code from both worlds litters the sources ([RD-31](../rd-items/code-health/rd-31-dead-code-in-sketch.md), [RD-33](../rd-items/code-health/rd-33-commented-legacy-code.md)).

A small `IEnvironment` interface (or build flag selecting the adapter) would make target selection explicit and keep both adapters honest.

---

| ← Previous | Up | Next → |
|---|---|---|
| [Maze Persistence](maze-persistence.md) | [Design Decisions](index.md) | [Algorithms](../algorithms/index.md) |
