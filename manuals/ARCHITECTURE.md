# Software architecture

STC separates network communication, command validation, motor transactions, and telemetry. The main runtime is in [`dev/src/stc_server.c`](../dev/src/stc_server.c); configuration and shared types are in [`dev/include/`](../dev/include/).

## Command path

1. The TCP server receives a frame and passes it to the binary codec.
2. The runtime checks the command, payload, state, limits, and axis availability.
3. Accepted work enters the action queue. Streaming speed updates use a keep-latest policy.
4. The motion worker dispatches the action through the selected backend.
5. The client receives an ACK or NACK. An ACK is not a guarantee that a physical movement has finished.
6. Telemetry reports the resulting motion and health state.

## Ownership and synchronization

The motion worker owns the backend. Network and telemetry threads use shared snapshots rather than issuing motor transactions themselves. Mutexes protect shared state and queue operations. This makes the ordering of motion, stop, and reset work explicit.

The server checks stale feedback and expired speed streams. Critical faults enter the FAIL state and schedule a stop through the same worker. Software checks depend on valid configuration and functioning hardware; they do not replace independent physical protections.

## Backend boundary

| Layer | Implementation |
|---|---|
| Runtime selection | [`motion_backend_router.c`](../dev/src/motion_backend_router.c) |
| Hardware orchestration | [`can_backend.c`](../dev/src/can_backend.c) |
| Nanotec protocol | [`pd4c.c`](../dev/src/pd4c.c), CANopen / CiA 402 |
| MYACTUATOR protocol | [`myact_backend.c`](../dev/src/myact_backend.c), [`x8_25.c`](../dev/src/x8_25.c), vendor CAN frames |
| Simulation | [`dev/src/simulation/`](../dev/src/simulation/), C++ kinematic model |

`--backend can` selects real CAN access. `[AxisN].driver` selects the motor protocol for each configured axis. These are separate decisions. The simulator uses the same runtime-facing interface but cannot validate bus traffic, physical stopping, or real motor timing.

## Telemetry and diagnostics

Two UDP unicast streams report motion and health. The default periods are 10 ms and 100 ms. The server selects the active TCP client's address as the destination; a client must connect before normal telemetry delivery begins.

The codec explicitly packs little-endian values into buffers. It checks lengths and identifiers without casting wire data to C structs. The CLI and Qt tools use the same codec.

## Repository layout

| Path | Purpose |
|---|---|
| `dev/src/`, `dev/include/` | Active runtime, motor drivers, codec, and configuration |
| `dev/tools/` | TCP client, UDP receivers, Qt diagnostic console |
| `dev/tests/` | Regression tests and hardware bench utilities |
| `dev/stub_ST/` | Reference simulator components; the current backend uses the motor model |
| `dev/config/` | Runtime and bench configuration examples |
| `deploy/` | Service unit, preflight checks, operator runbook, CPU affinity |
| `scripts/` | Bench and virtual-CAN helpers |
| `manuals/` | Setup, engineering report, and implementation references |
| `docs/images/` | Portfolio visuals |

See [SETUP.md](SETUP.md) for runnable commands and the [engineering report](steering-turret-report.pdf) for detailed discussion.
