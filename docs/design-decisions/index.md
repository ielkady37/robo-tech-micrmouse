[Home](../index.md) › [Design Decisions](index.md) › **Design Decisions**

# Design Decisions

The "why" behind the firmware's structure — constraints chosen deliberately and trade-offs accepted.

## Pages

| Page | Decision |
|---|---|
| [Wall Encoding: North/East Only](wall-encoding.md) | Store N/E walls once per shared edge; derive S/W |
| [State Management](state-management.md) | Two parallel pose records and what it costs |
| [Maze Persistence (NVS)](maze-persistence.md) | Surviving power cycles to enable speed runs |
| [Dual-Target Workflow](dual-target-workflow.md) | Simulator-first development against an abstract API |

## Reading Order

Each page is self-contained, but they build on each other: the wall encoding shapes persistence, persistence enables the competition strategy, and the dual-target workflow explains why solver and hardware code look so different.

---

| ← Previous | Up | Next → |
|---|---|---|
| [Simulator Interface](../architecture/simulator-interface.md) | **Design Decisions** | [Wall Encoding](wall-encoding.md) |
