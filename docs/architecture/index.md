[Home](../index.md) › **Architecture**

# Architecture

How the firmware is structured: layers, hardware, and the interfaces between solver and environment.

## Pages

| Page | Contents |
|---|---|
| [System Overview](system-overview.md) | Execution model, component map, data flow, concurrency |
| [Hardware Platform](hardware-platform.md) | ESP32, full pin map, I2C bus, odometry constants |
| [Sensor Suite](sensor-suite.md) | VL53L0X ranging array and BNO08x inertial sensing |
| [Motor Drive](motor-drive.md) | H-bridge outputs, quadrature odometry, distance math |
| [Simulator Interface](simulator-interface.md) | The `API` facade and its Micromouse Simulator heritage |

## Layering at a Glance

```
micromouse.ino            entry point / mode select
        │
   algorithm.*            maze logic + persistence (pure, no hardware)
        │
     API.*                environment adapter (simulator today)
  ── or ──
    Robot.*               environment adapter (hardware)
     │      │      │
   Tof.*   IMU.*   motor.*
```

The solver never touches registers; everything below it is swappable — that is the core architectural idea of this codebase.

---

| ← Previous | Up | Next → |
|---|---|---|
| [Overview](../overview/index.md) | **Architecture** | [System Overview](system-overview.md) |
