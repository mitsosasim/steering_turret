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
