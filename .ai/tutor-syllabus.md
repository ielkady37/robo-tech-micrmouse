# Tutor Syllabus (Crash)

## Scenario
Design and build a proper **navigation unit** (an application layer) for this
ESP32 micromouse so that maze logic talks to an interface instead of reaching
down into low-level hardware. Along the way we master C/C++, hardware
interfacing, data structures, OOD/layered architecture, and an A* maze solver —
each taught against the actual files in this repo.

## Authoritative Sources
- Primary: the codebase itself (`micromouse.ino`, `algorithm.*`, `API.*`,
  `Robot.*`, `motor.*`, `IMU.*`, `Tof.*`)
- Secondary: `docs/` in this repo (architecture, algorithms, design-decisions)
- Reference: C++ cppreference, Arduino-Espressif / ESP32 API docs

## Critical Path (7 units, time-boxed)

### Unit 1 — C/C++ Foundations for this Project — 90 min
Problem: understand every language feature the existing code already uses so
you can read before you write.
Cover, against real files:
- Pointers, references, `static` (class + file scope) — see `Robot` singleton
- `const`, `constexpr`, magic numbers vs `#define` — thresholds in `Robot.cpp`
- Namespaces, includes/headers, one-definition-rule
- STL containers used here: `std::priority_queue`, `std::vector` in `algorithm.cpp`
- Structs vs classes; access specifiers
- Arduino/ESP32 quirks: `setup/loop`, `IRAM_ATTR`, `portMUX_TYPE`

### Unit 2 — Hardware Interfacing for This Robot — 120 min
Problem: how do I2C sensors, GPIO, PWM, encoders, ISRs, and FreeRTOS tasks work
together to feed the navigation layer reliable data without it knowing a wire
ever moved.
Cover, against real files:
- I2C bus + device addressing: `Tof.setID()` XSHUT bring-up, IMU at 0x4B
- GPIO, PWM (`analogWrite`), quadrature encoder ISRs + critical sections (`motor.*`)
- FreeRTOS: sensor refresh task on Core 0 (mutexes/semaphores) vs `loop()` on Core 1
- Thread-safety: why IMU/ToF caches use semaphores
- Sensor → data: mm ranges → wall booleans (thresholds)

### Unit 3 — Data Structures for This Project — 90 min
Problem: represent a 16x16 maze, discoveries, and a distance field compactly and
safely on a microcontroller.
Cover, against real files:
- 2D arrays `[16][16]`, packing row/col into a byte; adjacency/north-east wall encoding
- `Direction` enum + modular arithmetic for relative turns
- Priority queue / binary heap for A* open set
- Sentinel sentinels: `UINT8_MAX` for unreachable, and why it's risky (RD-27)
- NVS persistence of the 3 matrices via `Preferences`

### Unit 4 — OOD & Layered Architecture (Problem-Oriented) — 120 min
Problem: today the solver reaches into the world through a hard-coded static
singleton (`API`) that duplicates pose state, and `Robot` is a monolithic
static class. We will refactor so the navigation unit depends on **interfaces**,
not on concrete hardware, enabling sim/real swap and host-side unit tests.
Cover:
- Layered architecture: application/navigation layer → interface seam → driver layer
- Interfaces / abstract classes: define `IMazeEnvironment` (sense walls, move,
  turn) as the *problem's contract* — not a mechanical copy of the API
- Dependency inversion: navigation unit receives the interface (constructor/DI),
  never `#include`s `Robot`/`IMU`/`Tof` directly
- Single Responsibility: split `Robot`'s sensing vs motion vs odometry concerns
- The trouble with singletons & hidden global pose (API vs algorithm dual state)
- Exercises: extract interfaces, wire sim + real implementations behind one unit

### Unit 5 — A* Maze Solving for This Robot — 120 min
Problem: from a partially explored maze, produce a shortest path to the goal
cells (7,7..8,8), updating as new walls are discovered.
Cover, against real files:
- Grid as graph; neighbors through open walls
- A*: `g` (moves so far), `h` (Manhattan heuristic), `f = g+h`; min-heap open set
- Distance map as gradient field; greedy descent = follow falling gradient
- Recompute-on-discovery vs incremental updates (RD-26)
- Adapting to a 3-sensor mouse: only front/left/right walls known per cell
- Return-to-start phase (RD-11) and speed-run phase as extensions

### Unit 6 — Integration: The Navigator Application Layer — 90 min
Problem: assemble Units 1–5 into a `Navigator` that sits on `IMazeEnvironment`
and drives exploration without touching hardware.
Cover:
- Build a clean navigation unit + interface + adapter wiring
- Verify on host with the simulator backend; keep domain code hardware-free
- Test seam: `updateDistancesAStar` + `getNextMovement` unit-testable on host
- Avoid the old dual-pose bug by keeping one source of truth in the navigator

### Unit 7 — Quiz + Review List — 30 min
- ▶ READ `tutor/crash/quick-quiz.md`
- Confirm Units 1–6 with the quick-quiz format; add gaps to review list
- Optional deeper dive: switch to Deep mode per-topic if more depth is wanted

## Prerequisite Chain
1 → 2 → 3 and 4 → 5 → 6 (Units 3 and 4 can precede 5; both feed Unit 6).
