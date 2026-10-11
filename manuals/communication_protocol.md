# Communication protocol

This is an implementation reference for the active C codec and server. The wire layouts below come from [`stc_icd.h`](../dev/include/stc_icd.h) and [`stc_icd.c`](../dev/src/stc_icd.c). Command behavior is implemented in [`stc_server.c`](../dev/src/stc_server.c).

## Transport

| Channel | Transport | Default endpoint | Purpose |
|---|---|---|---|
| Commands / responses | TCP | Port 7000 | Requests and ACK/NACK responses |
| TLMH | UDP unicast | Port 6010, 100 Hz | Axis motion feedback |
| TLML | UDP unicast | Port 6020, 10 Hz | State and health |

UDP telemetry is directed to the active TCP client's address. Rates and ports are configurable. Sequence numbers help clients detect ordering or missing telemetry; UDP does not guarantee delivery.

Multi-byte integers and floating-point values are encoded little-endian. The version byte defaults to `0x01`. Magic values are literal ASCII bytes, not integers sent in host byte order. The codec packs fields explicitly rather than serializing native structs.

## Command frame

The command header is 20 bytes. Total size is `20 + payload_size`; the codec supports at most 2048 payload bytes.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Magic `TCMD` |
| 4 | 1 | Version |
| 5 | 1 | Controller ID |
| 6 | 2 | Command ID |
| 8 | 2 | Sequence |
| 10 | 8 | Timestamp in milliseconds |
| 18 | 2 | Payload size |
| 20 | Variable | Payload |

## Response frame

The response header is 14 bytes. Total size is `14 + payload_size`. Command ID and sequence identify the corresponding request.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Magic `TRSP` |
| 4 | 1 | Version |
| 5 | 2 | Command ID |
| 7 | 2 | Sequence |
| 9 | 1 | Result: ACK `0`, NACK `1` |
| 10 | 2 | Error code |
| 12 | 2 | Payload size |
| 14 | Variable | Payload |

An ACK reports acceptance under the command's rules. It does not establish that asynchronous motor work has finished. Use telemetry and diagnostics to check completion and faults. Error values and names are defined in [`stc_icd.h`](../dev/include/stc_icd.h).

## Command payloads

The codec recognizes the following payload shapes. Recognition by the codec does not mean a physical subsystem is implemented.

| ID | Command | Bytes | Payload |
|---|---|---:|---|
| `0x0001` | CU_STATE | 1 | State `u8` |
| `0x0002` | SELFLEV | 1 | Mode / action `u8` |
| `0x0003` | IBIT | 1 | Mode / action `u8` |
| `0x0004` | STORAGE | 0 | None |
| `0x0005` | LASER | 2 | ID `u8`, on/off `u8` |
| `0x0006` | TURRET | 9 | Motion type `u8`, azimuth `f32`, elevation `f32` |
| `0x0007` | MIRROR | 6 | ID `u8`, motion type `u8`, elevation `f32` |
| `0x0008` | SUT | 10 | ID `u8`, motion type `u8`, azimuth `f32`, elevation `f32` |
| `0x000A` | RESET | 0 | None |
| `0x000B` | DELLOG | 0 | None |
| `0x000C` | FACTORY | 0 | None |
| `0x000D` | SELFTEST | 0 | None |
| `0x000E` | SHUTDOWN | 0 | None |
| `0x000F` | CONFIG | Variable | Raw payload; consult runtime support |
| `0x0010` | MANUALPOS | 32 | Latitude, longitude, altitude, heading: four `f64` values |
| `0x0011` | POSMODE | 1 | Mode `u8` |
| `0x0012` | REQ_SW_VERSION | 0 | None |
| `0x0013` | REQ_HW_INFO | 0 | None |
| `0x0014` | REQ_WORK_TIME | 0 | None |

Motion types are relative `0`, absolute `1`, and speed `2`. Position values refer to the external load in degrees; speed values use degrees per second. SUT is the protocol's name for a paired payload-axis group. Logical objects are mapped to axes through configuration.

Movement and speed streaming require OPER and an eligible axis. State transitions, limits, and driver availability can reject otherwise well-formed frames. See [`stc_state.c`](../dev/src/stc_state.c) for the active transition rules.

## High-rate telemetry: TLMH

The fixed size is 231 bytes: a 15-byte header and three arrays of 18 float32 values.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Magic `TLMH` |
| 4 | 1 | Version |
| 5 | 2 | Sequence |
| 7 | 8 | Timestamp in milliseconds |
| 15 | 72 | `position[18]`, degrees |
| 87 | 72 | `rates[18]`, degrees/second |
| 159 | 72 | `acceleration[18]`, degrees/second² |

## Low-rate telemetry: TLML

The fixed portion is 100 bytes. Total size is `100 + 4 * error_count`. The packer limits the diagnostic list to 64 entries.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 4 | Magic `TLML` |
| 4 | 1 | Version |
| 5 | 1 | Controller ID |
| 6 | 2 | Sequence |
| 8 | 8 | Timestamp in milliseconds |
| 16 | 1 | State |
| 17 | 4 | Uptime in seconds |
| 21 | 12 | Temperature, humidity, pressure: three `f32` values |
| 33 | 32 | Latitude, longitude, altitude, heading: four `f64` values |
| 65 | 18 | Motor status array |
| 83 | 8 | Laser status array |
| 91 | 8 | Level X/Y: two `f32` values |
| 99 | 1 | Error count |
| 100 | Variable | Diagnostic codes, `u32` each |

Fields that lack a hardware backend must not be interpreted as measured physical evidence. Laser fields currently describe commanded software state. Diagnostic encoding is implemented in [`stc_diag.c`](../dev/src/stc_diag.c).

## Tools and verification

[`icd_tx_tcp`](../dev/tools/icd_tx_tcp.c), [`tlm_rx_udp`](../dev/tools/tlm_rx_udp.c), and the [Qt console](../dev/tools/diag_gui_qt.cpp) share the codec. [`test_icd_codec.c`](../dev/tests/test_icd_codec.c) covers frame parsing and packing. See [SETUP.md](SETUP.md) for commands.
