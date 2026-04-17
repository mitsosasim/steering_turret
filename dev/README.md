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

## 7) Baseline Environment Checklist

A developer machine is considered ready when all items below are true:

- [ ] Git available from terminal
- [ ] CMake 3.16+ available
- [ ] GCC/G++ toolchain available (or Linux-equivalent in WSL2/remote)
- [ ] Ninja or Make available
- [ ] SSH login to RockPro works without password prompts (keys configured)
- [ ] (If GUI needed) X11 forwarding verified
- [ ] (If CAN testing needed) `vcan` or physical CAN interface is operational

---

## 8) Suggested Operating Modes

### Mode A: Linux-native development (best)
- Edit, build, test, and run directly on Linux workstation.

### Mode B: Windows + Remote Linux/RockPro (common)
- Edit on Windows (VS Code / Visual Studio), build/run over SSH on Linux target.

### Mode C: Windows + WSL2 Ubuntu
- Edit in Windows tools, compile/test in WSL2 shell, use SSH to RockPro for final integration.

---

## 9) Intent of this `dev/` Platform

The `dev/` folder is the software-side control-unit stub and validation workspace. It should let engineers:

- Build and iterate command/control logic quickly
- Run stubbed CU behavior before/alongside MCU integration
- Validate command paths (TCP/UDP/CAN) with repeatable tooling
- Debug issues early on a developer workstation before deployment
