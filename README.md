# Steering Turret Control

Low-level embedded Linux control for a military steering turret, with real-time scheduling, CAN motor control, and fault handling for critical defense applications.

**C11 / C++17 · Linux / ARM64 · SocketCAN · CANopen / CiA 402 · TCP / UDP · Qt6**

[Engineering report](manuals/steering-turret-report.pdf) · [Build and setup](manuals/SETUP.md) · [Software architecture](manuals/ARCHITECTURE.md) · [Protocol reference](manuals/communication_protocol.md)

## What it does

STC connects a control application to a multi-axis motor system. It receives movement commands, checks whether they are allowed, drives the motors, and sends back position, speed, and health information. A software simulator implements the motor-backend interface for functional tests.

![STC diagnostic console showing motor status, live telemetry, and command controls](docs/images/diagnostics-console.png)

*Diagnostic console during a CAN bench session. This view shows one connected actuator; it is not evidence of a full 18-axis hardware test.*

## My engineering work

I developed the control software and motor integration as the engineer responsible for this software stack. My work covered:

- **Low-level motor control:** Nanotec PD4-C CANopen / CiA 402 integration and a separate MYACTUATOR CAN driver, including command encoding, feedback, and unit conversion.
- **Concurrent runtime:** a dedicated motion worker, shared-state synchronization, bounded command queues, and periodic telemetry threads.
- **Fault handling:** state-based motion checks, position and speed limits, stale-feedback detection, speed-stream watchdogs, and stop requests after critical faults.
- **Communication:** a binary TCP command/response codec and UDP telemetry at configurable rates, with defaults of 100 Hz for motion and 10 Hz for health.
- **Integration tools:** a Qt diagnostic console, command-line clients, actuator bench utilities, a simulator, and regression tests.
- **Embedded deployment:** configuration-driven startup, systemd services, Linux real-time scheduling controls, and CPU-affinity guidance for ARM64 targets.

## System at a glance

![Software architecture: control client, runtime, telemetry, and selectable motor backends](docs/images/software-architecture.svg)

The network and telemetry threads stay separate from motor transactions. A single worker owns the motion backend, so hardware access has a clear ordering and commands are checked against current state and axis availability.

## Technology stack

| Area | Technologies |
|---|---|
| Runtime | C11, C++17, POSIX threads, mutexes, monotonic clocks |
| Platform | Embedded Linux, ARM64, RockPro64 / RK3399 |
| Motor interfaces | Linux SocketCAN, CANopen SDO/PDO, CiA 402, MYACTUATOR CAN |
| Network | TCP commands and responses, UDP unicast telemetry, explicit little-endian encoding |
| Diagnostics | Qt6 Widgets / Network, command-line tools, structured logs |
| Build and verification | CMake, GCC/G++, CTest, protocol and concurrency regression tests |
| Deployment | systemd, Linux capabilities, FIFO/RR scheduling controls, CPU affinity |

<img src="docs/images/embedded-target.jpg" width="420" alt="RockPro64 embedded Linux target board">

*RockPro64 embedded target, shown in the engineering report.*

## Explore the implementation

| Start here | What it shows |
|---|---|
| [`stc_server.c`](dev/src/stc_server.c) | Runtime threads, command processing, telemetry, and fault coordination |
| [`stc_icd.c`](dev/src/stc_icd.c) | Binary parsing and packing without relying on compiler struct layout |
| [`motion_backend_router.c`](dev/src/motion_backend_router.c) | Shared backend contract for simulation and hardware |
| [`pd4c.c`](dev/src/pd4c.c) / [`x8_25.c`](dev/src/x8_25.c) | Two different low-level motor protocols |
| [`stc_state.c`](dev/src/stc_state.c) / [`stc_limits.c`](dev/src/stc_limits.c) | State transitions and motion checks |
| [`diag_gui_qt.cpp`](dev/tools/diag_gui_qt.cpp) | Operator diagnostics and live hardware inventory |
| [`dev/tests/`](dev/tests/) | Existing codec, configuration, routing, state, and concurrency tests |

## Build and run software checks

On Linux with CMake and a C/C++ compiler:

```bash
git clone https://github.com/mitsosasim/STC.git
cd STC
cmake -S . -B build -DUSE_SIMULATOR=ON -DBUILD_DIAG_GUI=OFF
cmake --build build -j2
ctest --test-dir build --output-on-failure -LE '^hardware$'
./build/stc_server --backend simulator --rt-enabled 0
```

This no-axis smoke run reports FAIL because the startup motor-inventory check has no configured motors. Status queries remain available. In another terminal:

```bash
./build/icd_tx_tcp 127.0.0.1 7000 req_hw
```

See [Build and setup](manuals/SETUP.md) for GUI setup, telemetry, CAN bench work, and service deployment.

## Scope and verification

The software represents up to 18 axes and supports simulation and individual-actuator bench workflows. Those capabilities do not establish that every axis has been tested together on hardware. The simulator checks functional behavior; hardware timing and mechanical behavior require separate bench validation.

Real-time scheduling is configurable. This repository does not establish certified hard real-time deadlines or a certified safety level. Laser fields currently represent commanded software state; a physical laser backend is not integrated. Autonomous visual tracking and payload operation are outside the demonstrated control-software scope.

**Software engineering: Dimitrios Asimakopoulos**
