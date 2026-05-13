# STC Development Platform (Control Unit Stub)

This document defines the **development workstation prerequisites** for building and testing the STC Control Unit (CU) software stub in this `dev/` folder.

> Scope: this is the PC/laptop environment used to send commands, run the CU stub, compile code, and debug behavior. The RockPro MCU runtime platform is documented in the parent `README.md`.

---

## 1) Recommended Host Operating Systems

Use one of the following host OS options:

- **Linux (recommended for full compatibility)**
  - Ubuntu 22.04 LTS or newer (or equivalent modern distro)
  - Best choice for native build + SocketCAN + low-friction tooling
- **Windows 10/11 (supported for development/editor workflow)**
  - Use either:
    - Native Windows + SSH into Linux/RockPro for build/run, or
    - WSL2 Ubuntu for local Linux-like build/test workflow

### Why Linux is preferred

The STC stack uses Linux-oriented tools (CMake + GCC + SocketCAN workflows). Windows is still practical as an IDE/front-end machine, but Linux execution remains the reference behavior.

---

## 2) Core Toolchain Requirements

Install these components on your development platform:

1. **Git**
   - Clone, branch, and version control.
2. **CMake >= 3.16**
   - Configure/build project targets from `CMakeLists.txt`.
3. **C/C++ compiler toolchain**
   - Linux: `gcc`/`g++` (C11/C++17 capable)
   - Windows: MSVC is fine for editing, but Linux GCC is the reference for runtime parity.
4. **Build backend**
   - `ninja` (preferred) or `make`.
5. **Terminal + shell**
   - Linux shell (`bash`) for scripts and tooling.

---

## 3) IDE / Editor Options

Use one of these setups:

- **Visual Studio Code (recommended)**
  - Extensions:
    - C/C++
    - CMake Tools
    - (Optional) Remote - SSH
  - Good for local Linux, WSL2, or remote RockPro editing/debug.

- **Visual Studio (Windows)**
  - Useful as an editor and debugger for C/C++ source browsing.
  - If target behavior must match deployed Linux runtime, perform final build/run on Linux or via SSH to RockPro.

---

## 4) Remote Development and Access

For a Windows-hosted developer workflow, configure:

1. **OpenSSH client** (Windows built-in is fine)
2. **SSH key-based login** to RockPro/Linux target
3. **X11 forwarding (optional, for Qt GUI tools)**
   - X server on Windows (e.g., VcXsrv/Xming)
   - `ssh -Y <user>@<host>` when launching remote GUI apps

This gives a practical split:
- Windows handles IDE + source management
- RockPro/Linux handles runtime-accurate execution

---

## 5) Networking and Interface Prerequisites

Development host should provide:

- **TCP connectivity** to the STC server command port (for CU command flow)
- **UDP receive path** for telemetry monitoring
- **CAN tooling support** for bench simulation or integration testing
  - Linux packages: `iproute2`
  - Optional diagnostics: `can-utils`
  - Virtual CAN for local tests: `vcan0`

---

## 6) Optional but Highly Recommended Packages

- **Qt6** (`Core`, `Gui`, `Widgets`, `Network`) for `diag_gui_qt`
- **libcap / setcap** utilities when testing capability-based runtime behavior
- **Python 3** for helper scripts/utilities (if needed during bench automation)

---

---

## Task Iota Acceptance (PD4-C + X8-25)

Use one high-level path for both backends: `GUI -> stc_server TCP ICD -> backend router -> device`.

### Preconditions

1. Build once:

```bash
cmake -S . -B build -DBUILD_DIAG_GUI=ON
cmake --build build -j"$(nproc)"
```

2. Bring up transport for the backend under test:
   - PD4-C backend: configure CAN link/device for PD4-C.
   - X8-25 backend: configure CAN link/device for X8-25.

3. Start server:

```bash
./build/stc_server
```

### Scripted server-path acceptance

Run the same scripted high-level commands for each backend:

```bash
./dev/tests/task_iota_acceptance.sh 127.0.0.1 7000 all
```

Scenarios covered:
- native speed move
- position move
- quick-stop
- hold/release
- active telemetry
- server roundtrip

### GUI roundtrip acceptance

In a second terminal, run:

```bash
./build/diag_gui_qt
```

Execute this sequence in GUI for **both** backends:
1. Connect TCP to server.
2. Hold/Enable toggle: HOLD -> ENABLED -> HOLD.
3. Send speed command and then STOP / E-STOP.
4. Send target absolute position command twice with same setpoint to confirm deterministic response.
5. Confirm telemetry panels keep updating (age remains live and values refresh).

### Determinism + native/fallback reporting checks

For each backend run:

```bash
./build/icd_tx_tcp 127.0.0.1 7000 turret abs 5.0 0.0
./build/icd_tx_tcp 127.0.0.1 7000 turret abs 5.0 0.0
```

Acceptance expectations:
- Identical high-level `turret abs` command path for PD4-C and X8-25.
- Position command behavior is deterministic for repeated same-command inputs.
- Backend capability/mode indication is visible in GUI status labels (native capability vs translated/fallback path).
- A2 speed command behavior remains independent from A8 position fallback behavior (validate by issuing A2 speed sequence and A8/absolute sequence separately and confirming no cross-coupled behavior).
