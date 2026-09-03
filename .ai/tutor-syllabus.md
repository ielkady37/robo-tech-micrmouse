# Tutor Syllabus (Crash)

## Scenario
Learn the five foundational disciplines behind embedded robotics — C/C++,
hardware interfacing, data structures, layered architecture, and graph search —
as *general, transferable knowledge*. Each unit teaches the universal concepts
first; the micromouse codebase serves as a single running illustration, not the
definition of the topic. By the end you can design your own data structures,
write your own A*, architect your own layered system, and read any datasheet.

**Audience**: developers with general software experience (Python/JavaScript
etc.) but no C/C++ or static-typing background. Unit 0 bridges the mental
model; every later unit builds on it.

## Authoritative Sources
- **C/C++**: cppreference.com (definitive language + standard library reference);
  Stroustrup *A Tour of C++* (concise modern overview); K&R *The C Programming
  Language* (classic foundations)
- **Embedded**: Elecia White *Making Embedded Systems* (practical hardware +
  software co-design); ESP32 Technical Reference Manual (bus/peripheral details)
- **Data structures & algorithms**: CLRS *Introduction to Algorithms* (Ch. 6,
  10, 22–24 — heaps, graphs, shortest paths); Weiss *Data Structures and
  Algorithm Analysis in C++*
- **Architecture**: Robert C. Martin *Clean Architecture* (layering, dependency
  rule, SOLID); *Head First Design Patterns* (Adapter, Facade chapters)
- **Graph search**: Russell & Norvig *Artificial Intelligence: A Modern
  Approach* (Ch. 3 — uninformed & informed search); Hart, Nilsson & Raphael
  original A* paper (1968)
- **This codebase** (secondary — illustrations only): `micromouse.ino`,
  `algorithm.*`, `API.*`, `Robot.*`, `motor.*`, `IMU.*`, `Tof.*`

## Critical Path (8 units, time-boxed)

### Unit 0 — C/C++ Orientation for General Programmers — 120 min
**Problem**: you already know how to program — loops, functions, objects, all
of it — but C/C++ will surprise you. There is no garbage collector, no dynamic
typing, no runtime safety net. Every variable has a fixed type you must
declare, every allocation has an owner who must free it, and a pointer is just
a memory address you can dereference (or crash on). This unit bridges from your
mental model in Python/JavaScript into the C/C++ mental model — the foundation
every subsequent unit assumes.

**Learning objectives**:
- **Static, manual typing**: every variable has a type declared upfront —
  `int`, `float`, `char`, `bool`, sized types (`uint8_t`, `int32_t`);
  signed vs unsigned; there is no runtime type inference or dynamic type
  switching to fall back on
- **Manual memory & ownership**: stack (automatic, scoped) vs heap (explicit,
  persistent); `new`/`delete` in C++, `malloc`/`free` in C; "who owns this
  memory and when does it get freed"; leaks (never freed) and dangling pointers
  (freed too early or twice)
- **Value vs reference semantics**: pass-by-value makes a copy; pass-by-reference
  (`T&`) aliases the original; pass-by-pointer (`T*`) also aliases but can be
  null; `const` correctness basics — `const T&` for read-only access,
  `const T*` for read-only pointed-to data
- **Pointers**: a pointer holds a memory address; dereferencing (`*p`) reads/
  writes the pointed-to value; address-of (`&x`) gets the address; `nullptr`;
  pointer vs reference — a pointer can be reassigned and be null, a reference
  cannot
- **Arrays & strings**: arrays are contiguous blocks of raw memory; array name
  decays to a pointer; C-strings are null-terminated `char` arrays (no length,
  no bounds checking); `std::string` is safer; out-of-bounds access is
  undefined behavior — it may crash, corrupt data, or appear to work
- **The build & type-checking loop**: source → preprocess → compile → link →
  binary; the compiler is your first reviewer and catches type errors before
  runtime; reading compiler errors (they mention types constantly); contrast
  with interpreted "just run it and see"
- **Structs**: a plain data aggregate — fields grouped together, no methods
  yet; the stepping stone before classes — `struct Point { int x; int y; };`
  then `Point p = {3, 4};`
- **First working program**: a minimal annotated example showing type
  declarations, a function, a pointer, stack allocation, and output:
  ```cpp
  #include <iostream>

  int add(int a, int b) {       // typed parameters, typed return
      return a + b;
  }

  int main() {
      int x = 5;                // stack-allocated integer
      int* ptr = &x;            // pointer holds address of x
      std::cout << *ptr << "\n"; // dereference: prints 5
      return 0;                 // 0 means success
  }
  ```
- **Foundational vocabulary**: declaration (announces a name) vs definition
  (allocates storage); scope (where a name is visible); translation unit (a
  single .cpp after preprocessing); linkage (how names connect across units) —
  just the words now; depth comes in Unit 1

**Where used here**: `algorithm.cpp::getCell()` takes
`uint8_t row, uint8_t col` and returns `(col << 4) | (row & 0x0F)` — an early
example of the explicit sized types and raw-memory bit manipulation this unit
introduces; the 16×16 maze fits because each coordinate fits in 4 bits.

**Canonical reference**: Stroustrup *Programming: Principles and Practice
Using C++* (2nd ed, Ch. 1–7 — types, operations, functions, arrays, pointers,
structs) + cppreference.com

---

### Unit 1 — C++ Idioms, Build Toolchain & Standard Library — 90 min
**Problem**: master the language features any non-trivial C/C++ project uses,
so you can read, write, and reason about code in any codebase.

> Assumes Unit 0 vocabulary: pointer, reference, const, struct, stack vs heap.

**Learning objectives**:
- `const`, `constexpr`, `#define` — why macros are dangerous and when
  `constexpr` replaces them
- `static`: three meanings (local persistence, internal linkage, class member)
- Translation units, headers, `#pragma once` vs include guards, ODR
  (One Definition Rule)
- STL essentials: `std::vector`, `std::priority_queue`, `std::map`,
  `std::pair`, iterators, range-for
- Classes (not just structs): access specifiers, constructors/destructors,
  operator overloading
- Build model depth: toolchain basics (GCC/clang, CMake, Make); Arduino/ESP32
  specifics (`setup`/`loop`, `IRAM_ATTR`, `portMUX_TYPE`)

**Where used here**: `algorithm.cpp` defines a `Node` struct with
`operator>` so `std::priority_queue<Node, vector<Node>, greater<Node>>`
orders by `f`-cost — a standard pattern for any min-heap of custom types.

**Canonical reference**: Stroustrup *A Tour of C++* (Ch. 1–5) +
cppreference.com

---

### Unit 2 — Embedded Hardware Interfacing — 120 min
**Problem**: understand how microcontrollers talk to the physical world —
buses, GPIO, PWM, interrupts, sensors, and real-time concurrency — so you
can integrate *any* sensor or actuator from a datasheet.

**Learning objectives**:
- Serial buses: I2C (topology, addressing, pull-ups, clock stretching),
  SPI (MOSI/MISO/SCLK/CS, full-duplex), UART (TX/RX, baud) — when to
  use which
- GPIO modes: input, output, input-pullup, open-drain; debouncing
- PWM: duty cycle, frequency, `analogWrite` / LEDC on ESP32; driving
  motors via H-bridge
- Quadrature encoders: A/B channels, direction decoding, ISR constraints
  (keep short, use `volatile`, `IRAM_ATTR` on ESP32)
- Interrupts: latency, nesting, critical sections (`portMUX_TYPE`,
  `portENTER_CRITICAL`)
- Real-time concurrency: FreeRTOS tasks, priorities, `xTaskCreatePinnedToCore`;
  mutexes vs semaphores vs critical sections — when each is appropriate
- Reading a datasheet: register maps, timing diagrams, initialization
  sequences, error states

**Where used here**: `Tof.cpp::setID()` demonstrates the classic I2C
address-conflict workaround — three VL53L0X sensors share one bus by
sequencing their XSHUT shutdown pins to reassign addresses at boot.

**Canonical reference**: Elecia White *Making Embedded Systems* (Ch. 5–8)
+ ESP32 Technical Reference Manual (I2C, LEDC, GPIO chapters)

---

### Unit 3 — Data Structures & When to Choose Them — 90 min
**Problem**: pick the right structure for the job — not by memorizing a
list, but by understanding trade-offs (time, space, cache behavior,
complexity) on constrained hardware.

**Learning objectives**:
- Arrays: 1D, 2D (row-major layout), fixed-size vs dynamic; when a flat
  array beats a linked list (cache locality, no allocation)
- Enums + modular arithmetic for cyclic state machines (directions, phases)
- Graphs: adjacency matrix vs adjacency list vs *implicit* graph (grid
  neighbors computed on the fly) — space/time trade-offs
- Heaps / priority queues: binary heap properties, `std::priority_queue`
  (max-heap by default, `std::greater` for min-heap), O(log n) insert/extract
- Stacks (LIFO) and queues (FIFO) — BFS uses queue, DFS uses stack,
  Dijkstra/A* uses priority queue
- Bit-packing: squeezing two 4-bit values into one `uint8_t` for
  memory-constrained grids
- Sentinel values vs optional types: `UINT8_MAX` as "unreachable" —
  the risk of arithmetic overflow with unsigned sentinels

**Where used here**: `algorithm.cpp` packs `(row, col)` into a single
`uint8_t` via `(col << 4) | (row & 0x0F)` — a space optimization that
works because the grid is exactly 16×16 (each index fits in 4 bits).

**Canonical reference**: CLRS *Introduction to Algorithms* (Ch. 6 Heaps,
Ch. 10 Elementary Structures, Ch. 22 Elementary Graph Algorithms)

---

### Unit 4 — OOD, Layered Architecture & Abstraction — 120 min
**Problem**: design software that can change — swap hardware for a
simulator, add tests, refactor one layer without breaking another —
using universal principles, not project-specific hacks.

**Learning objectives**:
- SOLID: Single Responsibility, Open/Closed, Liskov Substitution,
  Interface Segregation, Dependency Inversion — what each actually means
- Interfaces / abstract classes: defining a *contract* (pure virtual
  functions in C++), why they enable polymorphism and testability
- Dependency Injection: constructor injection, setter injection —
  receiving dependencies rather than reaching for globals
- Layered architecture: presentation → application → domain →
  infrastructure; the Dependency Rule (inner layers know nothing of
  outer layers)
- Adapter pattern: wrapping a concrete implementation behind an
  interface (e.g., a `RealRobot` adapter and a `SimRobot` adapter both
  implementing `IMazeEnvironment`)
- Facade pattern: simplifying a complex subsystem behind a clean API
- The singleton anti-pattern: hidden global state, duplicate state,
  untestability — and when a singleton *is* acceptable (rare)
- When NOT to over-engineer: YAGNI, small embedded projects with tight
  memory/flash — pragmatism over purity

**Where used here**: the current `API` class is a static singleton that
maintains its own `currentX`/`currentY`/`currentDirection` — duplicating
the pose state already tracked in `algorithm.cpp`'s globals. This is a
textbook case of why hidden global state causes divergence bugs.

**Canonical reference**: Robert C. Martin *Clean Architecture* (Part 2:
Paradigms, Part 4: Principles, Part 5: Architecture) + *Head First
Design Patterns* (Adapter & Facade chapters)

---

### Unit 5 — Graph Search & Pathfinding — 120 min
**Problem**: find shortest paths in any graph — grid, road network,
state space — using the right algorithm for the job, with understanding
of *why* it works.

**Learning objectives**:
- Graph representations revisited: grid as implicit graph, neighbors
  through open edges
- Uninformed search: BFS (shortest path on unweighted graphs), DFS
  (exploration, not optimal)
- Dijkstra's algorithm: weighted graphs, priority queue, greedy
  expansion by minimum cost — the foundation A* builds on
- A*: `f = g + h`, open set (priority queue), closed set (or
  re-expansion check), heuristic design
- Heuristics: admissibility (never overestimates), consistency
  (triangle inequality), why Manhattan distance is admissible on grids,
  what happens when h is inadmissible
- Priority queues in search: why `std::priority_queue` with lazy
  deletion (skip stale entries) is simpler than decrease-key
- Grid pathfinding specifics: 4-connected vs 8-connected, diagonal
  cost (√2), tie-breaking
- Partial knowledge & replanning: when the map changes mid-search
  (new walls discovered), recompute vs incremental (D* Lite, LPA*)
- Distance map as gradient field: once A* fills `distance[][]`, greedy
  descent (always move to lowest neighbor) yields the optimal path
  without re-running search — the "flood fill" interpretation

**Where used here**: `updateDistancesAStar()` runs A* from the goal
cells outward with Manhattan heuristic; the resulting `distance[][]`
array is then used by `getNextMovement()` as a gradient field — greedy
descent on the precomputed costs.

**Canonical reference**: Russell & Norvig *AI: A Modern Approach*
(Ch. 3 — Solving Problems by Searching) + Hart, Nilsson & Raphael
"A Formal Basis for the Heuristic Determination of Minimum Cost Paths"
(1968)

---

### Unit 6 — Integration: Building a Navigator Layer — 90 min
**Problem**: assemble Units 1–5 into a clean application layer that
depends on an interface, not on concrete hardware — the same pattern
used in game AI, robotics frameworks, and simulation systems.

> Assumes Unit 0 vocabulary (pointer, reference, const, struct) and Unit 1
> class/interface syntax.

**Learning objectives**:
- Define the application seam: an `IMazeEnvironment` interface
  (`senseWalls()`, `move()`, `turn()`) that the navigator depends on
- Wire adapters: `RealRobotEnvironment` (wraps actual hardware) and
  `SimEnvironment` (wraps a simulator) behind the same interface
- Single source of truth: the navigator owns pose; no duplicate state
  in the environment or algorithm layer
- Host-side testing: run the navigator on a PC with the sim backend —
  no hardware required, fast iteration
- Incremental development: build the interface first, then one adapter,
  then the solver, then the second adapter
- Avoiding the dual-pose bug: one class owns state, everyone else
  receives it

**Where used here**: the target architecture — a `Navigator` class
receiving `IMazeEnvironment&` via constructor injection, calling
`updateDistancesAStar()` and `getNextMovement()` without ever
`#include`-ing `Robot.h`, `IMU.h`, or `Tof.h`.

**Canonical reference**: Robert Nystrom *Game Programming Patterns*
(State, Command, Observer chapters — free online at
gameprogrammingpatterns.com)

---

### Unit 7 — Quiz & Review — 30 min
- ▶ READ `tutor/crash/quick-quiz.md`
- Confirm Units 0–6 with the quick-quiz format; add gaps to review list
- Optional deeper dive: switch to Deep mode per-topic if more depth is
  wanted

## Prerequisite Chain
0 → 1 → 2 → 3 and 0 → 4 → 5 → 6
(Unit 0 is a universal prerequisite. Units 3 and 4 are independent; both feed
Units 5 and 6.)
