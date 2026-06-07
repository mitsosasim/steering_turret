# Steering Turret Communication ICD — Merged Final Version

**Document purpose**

This document is the merged, clarified, implementation-oriented communication ICD for the Steering Turret (ST) interface.
It consolidates:

1. the original PDF ICD (the most detailed source for command/state/diagnostic semantics),
2. the more recent `ICD_ST_COM.md` (the protocol currently followed by the stub transport), and
3. direct inspection of the uploaded stub source code.

The goal is to define one final protocol description that is accurate, explicit, and usable for further implementation.

---

## 1. Source reconciliation and final decisions

### 1.1 What was confirmed from the stub

The stub source **does follow the recent `.md` protocol in the main transport-level aspects**, specifically:

- TCP command magic is `TCMD`
- TCP response magic is `TRSP`
- high telemetry magic is `TLMH`
- low telemetry magic is `TLML`
- telemetry is sent by **UDP unicast** to the IP address of the currently connected TCP client
- high telemetry includes `acceleration[18]`
- `DelLog (0x000B)` and `SelfTest (0x000D)` exist in the stub command enum and are ACKed
- `ManualPos` is accepted as **32 bytes = 4 x double**
- `ReqSwVersion`, `ReqHwInfo`, `ReqWorkTime` return payloads

### 1.2 Stub gaps / mismatches found during code inspection

The stub does **not** fully match the original PDF and does **not** yet implement every intended command/semantic:

- `MIRROR (0x0007)` exists in the original PDF section text, but the stub does **not** currently implement it.
- `controller_id` values from the PDF are **not** fully used by the stub; the stub defaults to `0x01`.
- `CU_STATE` handling in the stub is partially mapped to its internal FSM, rather than enforcing the full final state table exactly.
- `CONFIG (0x000F)` is currently ACK-only in the stub and has no implemented payload semantics.
- `LASER` validation in the stub checks the laser ID range but does not fully enforce `on_off` semantics.

### 1.3 Final merged protocol decisions in this document

This merged ICD adopts the following as the **final canonical protocol**:

- **Protocol version byte on the wire remains `0x01`** for compatibility with current implementations.
- **Command magic** = `TCMD`
- **Response magic** = `TRSP`
- **Telemetry magics** = `TLMH` and `TLML`
- **Little-endian binary encoding** everywhere
- **`MIRROR (0x0007)` is part of the final protocol**
- **`DelLog (0x000B)` and `SelfTest (0x000D)` are retained** because they exist in the newer `.md` and in the stub
- **`ManualPos` uses 4 x `double` (32 bytes)** as final/canonical encoding
- **High-rate telemetry includes `acceleration[18]`**
- **Low-rate telemetry is 100 ms periodicity (10 Hz)** as the final intended behavior
- **Telemetry transport mode**: UDP **unicast** to the active TCP client IP so we remove any previous multicast functionality


This lets the final ICD preserve the rich semantics of the original PDF while matching the current stub transport behavior.

---

## 2. Overview

The protocol defines communication between the **Control Unit (CU)** and the **Steering Turret (ST)**.

It consists of:

- a **TCP command/response channel**
- a **UDP high-frequency telemetry channel**
- a **UDP low-frequency telemetry channel**

The protocol is binary, deterministic, versioned, and intended for real-time control.

---

## 3. Physical and transport layer

### 3.1 Command / response channel

| Parameter | Value |
|---|---|
| Medium | Ethernet |
| Transport | TCP/IP |
| Default TCP port | `7000` |
| Direction | Commands: CU → ST, Responses: ST → CU |
| Transmission mode | Event-driven |
| Encoding | Binary, Little-Endian |
| Character set | Not applicable |

### 3.2 Telemetry channel

Two UDP streams are used:

- **TLMH** = high-frequency telemetry
- **TLML** = low-frequency telemetry

#### Final current implementation profile

| Stream | Default Port | Destination | Period |
|---|---:|---|---|
| TLMH | `6010` | UDP unicast to the IP of the currently connected TCP client | 10 ms (100 Hz) |
| TLML | `6020` | UDP unicast to the IP of the currently connected TCP client | 100 ms (10 Hz) |


---

## 4. Protocol versioning and magic constants

- `protocol_version = 0x01`

### 4.1 Magic constants

| Frame Type | Value (ASCII) | Hex |
|---|---|---|
| Command | `TCMD` | `0x54434D44` |
| Response | `TRSP` | `0x54525350` |
| Telemetry High | `TLMH` | `0x544C4D48` |
| Telemetry Low | `TLML` | `0x544C4D4C` |

---

## 5. Common enums

### 5.1 Response result

| Name | Value |
|---|---:|
| ACK | `0x00` |
| NACK | `0x01` |

### 5.2 Error codes

The original PDF defined the base error codes `0x0000`..`0x0006`.
The recent `.md` / stub extends them.
The final merged set is:

| Error code | Name | Meaning |
|---|---|---|
| `0x0000` | NoError | No error |
| `0x0001` | ClientUnknown | Unknown client/controller |
| `0x0002` | ProtocolMismatch | Protocol version or framing mismatch |
| `0x0003` | SyntaxError | Malformed command or wrong payload size |
| `0x0004` | ValueOutOfRange | Invalid value or invalid ID |
| `0x0005` | NotAllowedInState | Command rejected in the current state |
| `0x0006` | InsufficientResources | Resource shortage |
| `0x0007` | Timeout | Timeout condition |
| `0x0008` | CommandNotImplemented | Recognized but not implemented |
| `0x00FF` | Generic | Generic failure |

### 5.3 Motion type

Used by `TURRET`, `MIRROR`, and `SUT` commands.

| Value | Name | Meaning |
|---|---|---|
| `0` | Relative | Value is a relative offset |
| `1` | Absolute | Value is an absolute target |
| `2` | Speed | Value is a speed command |

### 5.4 Position mode

| Value | Meaning |
|---|---|
| `0` | GPS |
| `1` | Manual |

---

## 6. Controller ID values

These come from the original PDF. However, we are not using different controller id anymore.

| controller_id | Meaning |
|---:|---|
| `11` | Control Unit |
| `13` | Logger application |
| `15` | Diagnostic application |
| `21` | Simulator or other test application |

**Implementation note:** the current stub still defaults `controller_id` to `0x01` rather than using these final values consistently so we follow the same logic.

---

## 7. TCP command message format

### 7.1 Header layout

All fields are little-endian.

| Field | Size (bytes) | Type | Description |
|---|---:|---|---|
| `magic` | 4 | `uint32_t` / 4 ASCII bytes | Constant `TCMD` |
| `protocol_version` | 1 | `uint8_t` | Current value `0x01` |
| `controller_id` | 1 | `uint8_t` | Sender identifier |
| `cmd_id` | 2 | `uint16_t` | Command identifier |
| `seq` | 2 | `uint16_t` | Sequence number |
| `timestamp` | 8 | `uint64_t` | Epoch / UTC-like timestamp |
| `payload_size` | 2 | `uint16_t` | Payload length in bytes |
| `payload` | variable | binary | Command payload |

### 7.2 Packed C-like layout

```cpp
#pragma pack(push, 1)
struct CommandHeader {
    uint32_t magic;            // "TCMD"
    uint8_t  protocolVersion;  // 0x01
    uint8_t  controllerId;     // 0x01
    uint16_t cmdId;
    uint16_t seq;
    uint64_t timestamp;
    uint16_t payloadSize;
};
#pragma pack(pop)
```

Header size = **20 bytes**.

---

## 8. Command set

### 8.1 Final command ID table

| ID | Name | Response payload |
|---|---|---|
| `0x0001` | `CU_STATE` | none |
| `0x0002` | `SELFLEV` | none |
| `0x0003` | `IBIT` | none |
| `0x0004` | `STORAGE` | none |
| `0x0005` | `LASER` | none |
| `0x0006` | `TURRET` | none |
| `0x0007` | `MIRROR` | none |
| `0x0008` | `SUT` | none |
| `0x000A` | `RESET` | none |
| `0x000B` | `DELLOG` | none |
| `0x000C` | `FACTORY` | none |
| `0x000D` | `SELFTEST` | none |
| `0x000E` | `SHUTDOWN` | none |
| `0x000F` | `CONFIG` | currently none |
| `0x0010` | `MANUALPOS` | none |
| `0x0011` | `POSMODE` | none |
| `0x0012` | `REQ_SW_VERSION` | ASCII string |
| `0x0013` | `REQ_HW_INFO` | ASCII string / structured info |
| `0x0014` | `REQ_WORK_TIME` | `uint32_t uptime_sec` |

---

## 9. Command payload definitions

### 9.1 `CU_STATE` (`0x0001`)

Payload size: **1 byte**

| Field | Size | Type | Description |
|---|---:|---|---|
| `cu_state` | 1 | `uint8_t` | State ID |

#### Final state IDs

| Value | Name | Meaning |
|---:|---|---|
| `0` | `INIT` | Initialization |
| `1` | `INIT_DONE` | Initialization completed |
| `2` | `SAFE` | Safe state |
| `3` | `OPER` | Operational state |
| `4` | `SELF_LEVELLING` | Performing self-levelling |
| `5` | `IBIT_RUNNING` | IBIT running |
| `6` | `STORE` | Motors in storage position |
| `0xFF` | `FAIL` | Failure state |

**Implementation note:** the current stub uses an internal mapping from its CU-side state machine and does not fully enforce the final state table exactly. However, we should still follow the correct semantics and state transitions. 

---

### 9.2 `SELFLEV` (`0x0002`)

Payload size: **1 byte**

| Field | Size | Type | Description |
|---|---:|---|---|
| `start_stop` | 1 | `uint8_t` | `1 = Start`, `0 = Stop` |

---

### 9.3 `IBIT` (`0x0003`)

Payload size: **1 byte**

| Field | Size | Type | Description |
|---|---:|---|---|
| `start_stop` | 1 | `uint8_t` | `1 = Start`, `0 = Stop` |

---

### 9.4 `STORAGE` (`0x0004`)

Payload size: **0 bytes**

Moves the turret to the storage state / storage position.

---

### 9.5 `LASER` (`0x0005`)

Payload size: **2 bytes**

| Field | Size | Type | Description |
|---|---:|---|---|
| `id` | 1 | `uint8_t` | Laser ID `1..8` |
| `on_off` | 1 | `uint8_t` | `0 = OFF`, `1 = ON` |

**Implementation note:** the current stub validates the ID range but does not fully enforce the `on_off` value. We implement the command logic but leave its behavior as a to-do until further clarification. 

---

### 9.6 `TURRET` (`0x0006`)

Payload size: **9 bytes**

| Field | Size | Type | Description |
|---|---:|---|---|
| `motion_type` | 1 | `uint8_t` | `0=relative`, `1=absolute`, `2=speed` |
| `az_data` | 4 | `float` | turret azimuth value |
| `el_data` | 4 | `float` | camera elevation value |

Units depend on `motion_type`:

- relative: delta degrees
- absolute: degrees
- speed: degrees/second

**Important note:** The degrees for every command refer to external load movement and not internal motor shaft rotation.

---

### 9.7 `MIRROR` (`0x0007`)

Payload size: **6 bytes**

| Field | Size | Type | Description |
|---|---:|---|---|
| `mirror_id` | 1 | `uint8_t` | `1..8`, or `0xFF = all mirrors` |
| `motion_type` | 1 | `uint8_t` | `0=relative`, `1=absolute`, `2=speed` |
| `elevation_data` | 4 | `float` | mirror elevation value |

Units depend on `motion_type`:

- relative: delta degrees
- absolute: degrees
- speed: degrees/second

**Important note:** this command exists in the original PDF and in the newer C implementation, but the current uploaded stub does **not** yet implement it. It remains part of the final merged protocol.

---

### 9.8 `SUT` (`0x0008`)

Payload size: **10 bytes**

| Field | Size | Type | Description |
|---|---:|---|---|
| `sut_id` | 1 | `uint8_t` | `1..8`, or `0xFF = all SUTs` |
| `motion_type` | 1 | `uint8_t` | `0=relative`, `1=absolute`, `2=speed` |
| `az_data` | 4 | `float` | SUT azimuth value |
| `el_data` | 4 | `float` | paired mirror elevation value |

Units depend on `motion_type`:

- relative: delta degrees
- absolute: degrees
- speed: degrees/second

Final semantic meaning:

- `az_data` controls the **SUT azimuth motor** of the addressed SUT
- `el_data` controls the **paired mirror elevation motor** of the addressed SUT

For `sut_id = 0xFF`, the command applies to all SUT pairs.

---

### 9.9 `RESET` (`0x000A`)

Payload size: **0 bytes**

Administrative command.

---

### 9.10 `DELLOG` (`0x000B`)

Payload size: **0 bytes**

Administrative command introduced by the newer `.md` / stub profile.

Current stub behavior: ACK only so we follow that and later enhance it. 

---

### 9.11 `FACTORY` (`0x000C`)

Payload size: **0 bytes**

Administrative command.

---

### 9.12 `SELFTEST` (`0x000D`)

Payload size: **0 bytes**

Administrative command introduced by the newer `.md` / stub profile.

Current stub behavior: ACK only so we follow that and later enhance it.

---

### 9.13 `SHUTDOWN` (`0x000E`)

Payload size: **0 bytes**

Administrative command.

---

### 9.14 `CONFIG` (`0x000F`)

Payload: **reserved / implementation-defined**

The original PDF lists `CONFIG` but does not define a payload schema.
The current stub ACKs it with no semantic effect.

**Final merged status:** reserved until a dedicated configuration payload is formally defined.

---

### 9.15 `MANUALPOS` (`0x0010`)

Payload size: **32 bytes**

| Field | Size | Type | Description |
|---|---:|---|---|
| `lat` | 8 | `double` | Degrees |
| `lon` | 8 | `double` | Degrees |
| `alt` | 8 | `double` | Meters |
| `heading` | 8 | `double` | Degrees |

**Final decision:** `MANUALPOS` is encoded as **4 x double = 32 bytes**.

This resolves the inconsistency in the original PDF command list table, whose summary line used `float`, while the dedicated detailed section used `double`. The stub also accepts only the 32-byte double form.

---

### 9.16 `POSMODE` (`0x0011`)

Payload size: **1 byte**

| Field | Size | Type | Description |
|---|---:|---|---|
| `mode` | 1 | `uint8_t` | `0=GPS`, `1=Manual` |

---

### 9.17 `REQ_SW_VERSION` (`0x0012`)

Payload size: **0 bytes**

Response payload: ASCII string, no terminator required.

Current stub behavior: returns the `version_info::detailed()` string.

---

### 9.18 `REQ_HW_INFO` (`0x0013`)

Payload size: **0 bytes**

Response payload: ASCII string or structured hardware info.

Current stub behavior: returns ASCII string:

```text
STEERING_TURRET_SIM
```

---

### 9.19 `REQ_WORK_TIME` (`0x0014`)

Payload size: **0 bytes**

Response payload: `uint32_t uptime_sec`

Current stub behavior: elapsed simulator runtime in seconds.

---

## 10. Response message format

### 10.1 Header layout

| Field | Size (bytes) | Type | Description |
|---|---:|---|---|
| `magic` | 4 | `uint32_t` / 4 ASCII bytes | Constant `TRSP` |
| `protocol_version` | 1 | `uint8_t` | Current value `0x01` |
| `cmd_id` | 2 | `uint16_t` | Echo of the received command ID |
| `seq` | 2 | `uint16_t` | Echo of the command sequence |
| `result` | 1 | `uint8_t` | `0x00 = ACK`, `0x01 = NACK` |
| `err_code` | 2 | `uint16_t` | Detailed status/error code |
| `payload_size` | 2 | `uint16_t` | Additional payload size |
| `payload` | variable | binary | Optional return data |

### 10.2 Packed C-like layout

```cpp
#pragma pack(push, 1)
struct ResponseHeader {
    uint32_t magic;            // "TRSP"
    uint8_t  protocolVersion;  // 0x01
    uint16_t cmdId;
    uint16_t seq;
    uint8_t  result;           // 0=ACK, 1=NACK
    uint16_t errCode;
    uint16_t payloadSize;
};
#pragma pack(pop)
```

Header size = **14 bytes**.

---

## 11. Sequence handling and error management

- Each command shall include a sequence number incremented by the sender.
- The receiver shall echo the same sequence number in the response.
- Sequence numbers wrap at `65535`.
- Invalid command IDs, malformed frames, or invalid parameters shall generate `NACK` with an appropriate `err_code`.
- Query commands may return a payload in the response.
- Administrative commands may validly return an empty ACK payload.

---

## 12. Command validation / state rules (merged behavioral rules)

These reflect the intended final behavior and also capture the most relevant current stub rules.

### 12.1 Motion commands

The following commands are motion commands:

- `TURRET`
- `MIRROR`
- `SUT`
- `LASER`

Final intended rule:

- motion commands shall be accepted only in `OPER`

Current stub behavior:

- `TURRET`, `SUT`, `LASER` are accepted only in `OPER`
- `MIRROR` is not yet implemented in the stub

### 12.2 `MANUALPOS` and `POSMODE`

Final intended rule:

- allowed in `SAFE` and `OPER`

Current stub behavior matches this rule.

### 12.3 `SELFLEV` and `IBIT`

Final intended rule:

- start allowed in `OPER`
- stop allowed from the corresponding running state

Current stub behavior matches this rule approximately through its internal FSM.

### 12.4 `STORAGE`

Final intended rule:

- allowed unless blocked by system failure policy

Current stub behavior:

- allowed unless in `FAIL`

### 12.5 Administrative commands

- `RESET`
- `DELLOG`
- `FACTORY`
- `SELFTEST`
- `SHUTDOWN`
- `CONFIG`

Current stub behavior:

- ACK only, no detailed implementation

---

## 13. High-frequency telemetry (`TLMH`)

### 13.1 Purpose

High-rate telemetry provides fast motion state updates from ST to CU.

### 13.2 Final transport behavior

- default rate: **100 Hz** (every 10 ms) configurable through the config file and CLI
- default port: **6010** configurable through the config file and CLI
- current implementation profile: **UDP unicast** to the connected TCP client IP
- legacy/original profile: multicast `239.255.10.2:6010`

### 13.3 Final payload format

The merged final high-rate telemetry includes **position**, **rates**, and **acceleration**.

| Field | Type | Size | Description |
|---|---|---:|---|
| `magic` | `uint32_t` | 4 | `TLMH` |
| `protocol_version` | `uint8_t` | 1 | `0x01` |
| `seq` | `uint16_t` | 2 | telemetry sequence |
| `timestamp` | `uint64_t` | 8 | UNIX time in milliseconds |
| `position[18]` | `float[18]` | 72 | angular positions |
| `rates[18]` | `float[18]` | 72 | angular rates |
| `acceleration[18]` | `float[18]` | 72 | angular accelerations |

### 13.4 Packed layout

```cpp
#pragma pack(push, 1)
struct TelemetryHHeader {
    uint32_t magic;           // "TLMH"
    uint8_t  protocolVersion; // 0x01
    uint16_t seq;
};

struct TelemetryHPayload {
    TelemetryHHeader header;
    uint64_t timestamp;
    float position[18];
    float rates[18];
    float acceleration[18];
};
#pragma pack(pop)
```

Total size = **231 bytes**.

### 13.5 Array index mapping

The final canonical array order is:

| Index | Meaning |
|---:|---|
| 0 | ST azimuth (turret mast) |
| 1 | ST elevation (camera)|
| 2 | SUT1 azimuth |
| 3 | SUT1 elevation / paired mirror elevation |
| 4 | SUT2 azimuth |
| 5 | SUT2 elevation / paired mirror elevation |
| 6 | SUT3 azimuth |
| 7 | SUT3 elevation / paired mirror elevation |
| 8 | SUT4 azimuth |
| 9 | SUT4 elevation / paired mirror elevation |
| 10 | SUT5 azimuth |
| 11 | SUT5 elevation / paired mirror elevation |
| 12 | SUT6 azimuth |
| 13 | SUT6 elevation / paired mirror elevation |
| 14 | SUT7 azimuth |
| 15 | SUT7 elevation / paired mirror elevation |
| 16 | SUT8 azimuth |
| 17 | SUT8 elevation / paired mirror elevation |

**Compatibility note:** the original PDF defined only `position[18]` and `rates[18]`; the newer `.md` / stub adds `acceleration[18]`. This merged ICD adopts the newer extended form as final.

---

## 14. Low-frequency telemetry (`TLML`)

### 14.1 Purpose

Low-rate telemetry provides state, environment, positioning, health, and diagnostics.

### 14.2 Final transport behavior

- final rate: **100 ms periodicity = 10 Hz** configurable through the config file and CLI
- default port: **6020** configurable through the config file and CLI
- current implementation profile: UDP **unicast** to connected TCP client IP
- legacy/original profile: multicast `239.255.10.3:6020`


### 14.3 Final payload format

| Field | Type | Size | Description |
|---|---|---:|---|
| `magic` | `uint32_t` | 4 | `TLML` |
| `protocol_version` | `uint8_t` | 1 | `0x01` |
| `controller_id` | `uint8_t` | 1 | originating controller ID |
| `seq` | `uint16_t` | 2 | telemetry sequence |
| `timestamp` | `uint64_t` | 8 | UNIX time in milliseconds |
| `state` | `uint8_t` | 1 | turret state |
| `uptime` | `uint32_t` | 4 | seconds since boot |
| `temperature` | `float` | 4 | internal temperature (°C) |
| `humidity` | `float` | 4 | relative humidity (%) |
| `pressure` | `float` | 4 | atmospheric pressure (hPa) |
| `position_lat` | `double` | 8 | latitude (degrees) |
| `position_lon` | `double` | 8 | longitude (degrees) |
| `position_alt` | `double` | 8 | altitude (meters) |
| `turret_heading` | `double` | 8 | heading (degrees) |
| `motor_status[8]` | `uint8_t[8]` | 8 | motorized subassembly status flags |
| `laser_status[8]` | `uint8_t[8]` | 8 | laser status flags |
| `level_x` | `float` | 4 | platform inclination X |
| `level_y` | `float` | 4 | platform inclination Y |
| `num_errors` | `uint8_t` | 1 | number of diagnostic codes |
| `error_codes[]` | `uint32_t[]` | variable | compact diagnostic list |

### 14.4 Packed layout

```cpp
#pragma pack(push, 1)
struct TelemetryLHeader {
    uint32_t magic;           // "TLML"
    uint8_t  protocolVersion; // 0x01
    uint8_t  controllerId;
    uint16_t seq;
};

struct TelemetryLPayloadFixed {
    TelemetryLHeader header;
    uint64_t timestamp;
    uint8_t state;
    uint32_t uptime;
    float temperature;
    float humidity;
    float pressure;
    double geoPos[3];         // lat, lon, alt
    double orientationAz;     // heading
    uint8_t motorStatus[8];
    uint8_t laserStatus[8];
    float levelX;
    float levelY;
    uint8_t numErrors;
    // followed by numErrors * uint32_t errorCodes
};
#pragma pack(pop)
```

Effective packet size = fixed part + `numErrors * 4` bytes.

---

## 15. Diagnostic error / warning code structure in TLML

The original PDF diagnostic encoding is retained as final.

Each entry in `error_codes[]` is a 32-bit unsigned integer:

| Byte | Meaning |
|---|---|
| Byte 3 | Category |
| Byte 2 | Subcategory |
| Byte 1..0 | Error / Warning Code |

### 15.1 Ordering

The list is ordered as:

1. errors first
2. warnings next

### 15.2 Category / subcategory examples

| Category | Meaning | Example subcategories |
|---:|---|---|
| 0 | General system | 0 = General subsystem |
| 1 | Pointing motors | 0 = Azimuth motor, 1 = Elevation motor |
| 2 | Mirror elevation motors | 0 = Mirror 1, 1 = Mirror 2, ... |
| 3 | SUT azimuth motors | 0 = SUT motor 1, 1 = SUT motor 2, ... |
| 4 | GPS / Orientation | 0 = GPS 1, ... |
| 5 | Self-Leveling | 0 = Self-Leveling actuator, ... |
| 6 | Environmental sensors | 0 = Sensor 1, ... |
| ... | Extendable | additional subsystems may be added |

### 15.3 Meaning of errors vs warnings

**Errors**

Critical malfunction or violation of operational constraints.
Examples:

- motor over-current
- encoder failure
- communication timeout
- self-levelling failure

**Warnings**

Non-critical degradations that may reduce performance but do not necessarily require leaving `OPER`.
Examples:

- GPS not fixed
- low GPS accuracy
- environmental sensor out of tolerance
- motor temperature approaching limit

---