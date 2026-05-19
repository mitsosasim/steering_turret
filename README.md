# STC

This repository builds an STC server, CLI tooling, optional Qt diagnostics GUI, and multiple test/demo programs.

## 1) Prerequisites

### Required
- Linux (SocketCAN-based workflows assume Linux).
- CMake >= 3.16.
- GCC/G++ (or another C11/C++17 toolchain).
- `make`/Ninja (any CMake generator is fine).

### Optional (depending on what you want to run)
- **Qt6** (`Core`, `Gui`, `Widgets`, `Network`) for `diag_gui_qt`.
- **SocketCAN tools** (`iproute2`, optional `can-utils`) for real CAN interfaces.
- `setcap` (from `libcap`) when granting runtime capabilities.

---

## 2) Build from scratch

From the repo root:

```bash
cmake -S . -B build
cmake --build build -j
```

This builds:
- Main server: `build/stc_server`
- CLI helpers: `build/icd_tx_tcp`, `build/tlm_rx_udp`
- Unit/integration tests in `build/` (see [Section 7](#7-run-tests-and-demos))
- Scripted demo copied to `build/turret_motion_demo`

### Build variants you will likely use

#### A) Server + CLI tools + tests (default)
```bash
cmake -S . -B build
cmake --build build -j
```

#### B) Server **without GUI** (explicit)
If you want to force-disable GUI target generation:

```bash
cmake -S . -B build_nogui -DBUILD_DIAG_GUI=OFF
cmake --build build_nogui -j
```

#### C) Server with hardware CAN backend (no simulator)
By default, `USE_SIMULATOR=ON`. To build against `can_backend.c` instead:

```bash
cmake -S . -B build_hwcan -DUSE_SIMULATOR=OFF
cmake --build build_hwcan -j
```

#### D) Attempt GUI build (Qt required)
```bash
cmake -S . -B build_gui -DBUILD_DIAG_GUI=ON
cmake --build build_gui -j
```

> If Qt6 is not found, CMake logs: `Qt6 not found: skipping diag_gui_qt target`.

---

## 3) Run modes

### Mode 1: Server only (no GUI)

```bash
./build/stc_server
```

Show usage/options:

```bash
./build/stc_server --help
```

Key runtime flags include TCP/CAN settings, telemetry endpoints, and RT scheduling controls.

Note: `[diagnostics].motor_status_map` is deprecated and ignored; TLML uses canonical `motor_status[18]`.

Protocol compatibility note: protocol version byte remains `0x01` for current CU compatibility.
TLML now follows the canonical **100-byte base layout**.
Legacy 83-byte payloads are no longer supported.

### Mode 2: Server + GUI

1. Start server in one terminal:

```bash
./build/stc_server
```

2. Start GUI in another terminal (if built):

```bash
./build/diag_gui_qt
```

The GUI provides command controls and UDP telemetry listeners.

### Mode 3: Server + CLI-only control (no GUI)

1. Start server:

```bash
./build/stc_server
```

2. Use CLI command sender:

```bash
./build/icd_tx_tcp <host> <port> <cmd> [args...]
```

Examples:

```bash
./build/icd_tx_tcp 127.0.0.1 7001 cu_state 2
./build/icd_tx_tcp 127.0.0.1 7001 laser 1 1
./build/icd_tx_tcp 127.0.0.1 7001 turret speed 10.0 0.0
./build/icd_tx_tcp 127.0.0.1 7001 manualpos 37.5665 126.9780 45.0 90.0
```

3. (Optional) Monitor telemetry via UDP:

```bash
./build/tlm_rx_udp
./build/tlm_rx_udp --ifname eth0
./build/tlm_rx_udp --raw --verbose
```

---

## 4) Real-time scheduling + capability setup

`stc_server` supports RT policy options (`fifo`, `rr`, `other`) and strict/non-strict behavior:

- `--rt-enabled 0|1`
- `--rt-strict 0|1`
- `--rt-policy fifo|rr|other`
- `--can-worker-prio <int> --tlmh-prio <int> --tlml-prio <int>`

Grant RT privilege to installed binary:

```bash
sudo setcap cap_sys_nice=ep /opt/stc/bin/stc_server
getcap /opt/stc/bin/stc_server
```

For GUI raw-socket needs (if deployed to `/opt/stc/bin`):

```bash
sudo setcap cap_net_admin+ep /opt/stc/bin/diag_gui_qt
getcap /opt/stc/bin/diag_gui_qt
```

---

## 5) Systemd deployment (optional)

```bash
sudo install -d /opt/stc/deploy
sudo install -m 0755 deploy/stc_preflight.sh /opt/stc/deploy/stc_preflight.sh
sudo install -m 0644 deploy/stc_server.service /etc/systemd/system/stc_server.service
sudo systemctl daemon-reload
sudo systemctl enable --now stc_server
sudo systemctl status stc_server --no-pager
```

Also see:
- `deploy/CPU_AFFINITY.md`
- `deploy/OPERATOR_RUNBOOK.md`

---

## 6) CAN setup quick start (for local testing)

If you do not have physical CAN hardware, you can use the included helper:

```bash
sudo ./scripts/vcan_setup.sh
```

Or manually:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
ip -details link show vcan0
```

Then run server/tests using `vcan0` where applicable.

---


### Common RockPro + Windows workflow (SSH X11 forwarding)

If you run the server on RockPro and display the Qt GUI on Windows, this flow is known to work.

On **Windows**:

1. Launch **XLaunch** (Xming/VcXsrv).
2. In Command Prompt:

```bat
set DISPLAY=localhost:0.0
ssh -Y STSide
```

Then on the SSH shell (RockPro):

```bash
./build/diag_gui_qt
```

> Note: `ssh -Y` enables trusted X11 forwarding. If needed, `ssh -X` can also be used.


### `diag_gui_qt` over SSH quick notes

1. Start **VcXsrv/XLaunch** on Windows before opening SSH.
2. Connect with trusted X11 forwarding (`ssh -Y <user>@<host>`).
3. Install required X11/Qt packages on Linux (`xorg-xauth`, `xorg-xclock`, `xcb-util-cursor`, and Qt xcb dependencies).
4. Verify X forwarding with `xclock` before launching `./build/diag_gui_qt`.

### VS Code build/run command examples

Simulator-enabled GUI build:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_DIAG_GUI=ON -DUSE_SIMULATOR=ON
cmake --build build -j"$(nproc)"
./build/stc_server
```

Hardware-CAN GUI build:

```bash
cmake -S . -B build -DUSE_SIMULATOR=OFF -DBUILD_DIAG_GUI=ON
cmake --build build -j"$(nproc)"
```

### CAN bus detailed monitoring (`candump`)

For detailed live CAN traffic insight while the server is running, use `candump` from `can-utils`:

```bash
candump -tz -cae can0
```

Useful variants:

```bash
candump -L can0            # log-format output
candump any,0:0,#FFFFFFFF  # capture from any interface with mask
candump can0,123:7FF       # filter specific CAN ID range
```

### `canup` alias note

If you already use a local `canup` alias/function to bring CAN up quickly, keep using it before starting `stc_server`/GUI. Example:

```bash
canup
ip -details link show can0
```

(Exact `canup` definition is environment-specific, so keep your local alias/script as the source of truth.)


---

## 7) Run tests and demos

### 7.1 CMake-built tests

After `cmake --build build -j`, run:

```bash
./build/test_actions
./build/stc_state_test
./build/stc_diag_test
./build/test_diag_state_integration
./build/test_icd_codec
./build/stc_config_test
./build/test_log_concurrency
./build/test_can_worker_concurrency
# Optional TSAN build (slower):
./build/test_can_worker_concurrency_tsan
```

### 7.2 Scripted turret demo (CMake-copied)

CMake copies `dev/tests/turret_motion_demo.sh` to:

```bash
./build/turret_motion_demo [host] [port]
```

This drives turret state/motion via `icd_tx_tcp` against a running `stc_server`.

### 7.3 Standalone C test files not built by default CMake targets

Some demo/test C files in `dev/tests/` are intentionally standalone and can be compiled directly.

### `pd4c_test.c`

```bash
gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
  -Idev/include \
  dev/tests/pd4c_test.c dev/src/pd4c.c -lm -o build/pd4c_test

./build/pd4c_test <can_iface> <node_id>
# example:
./build/pd4c_test can0 1
```

### `turret_motion_demo.c`

```bash
gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
  -Idev/include \
  dev/tests/turret_motion_demo.c dev/src/pd4c.c -lm -o build/turret_motion_demo_c

./build/turret_motion_demo_c <can_iface> <node_id>
# example:
./build/turret_motion_demo_c can0 1
```

> Note: this file includes additional optional CLI parameters; run it without enough arguments to print usage guidance.

### `x8_25_test.c` (MyActuator X8-25 / X8-PRO over SocketCAN)

Compile:

```bash
gcc -O2 -Wall -Wextra -std=c11 -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L \
  dev/src/x8_25.c dev/tests/x8_25_test.c \
  -Idev/include -lm -o build/x8_25_test_can
```

Run:

```bash
./build/x8_25_test_can <can_iface> <node_id>
# example:
./build/x8_25_test_can can0 1
```


Commissioning-only persistent configuration APIs (zeroing, PID/accel persistent writes, CAN-ID and vendor/communication protection writes) are intentionally **not** declared in `dev/include/x8_25.h`.

For bench/service tooling, include `dev/include/x8_25_commissioning.h` and compile with `-DSTC_ENABLE_COMMISSIONING` to export those entry points from `dev/src/x8_25.c`.

Monitor live CAN traffic with `candump` (separate terminal):

```bash
candump -tz -cae can0
```

---

## 8) Typical end-to-end workflows

### Workflow A: Fast local (simulator, CLI only)
```bash
cmake -S . -B build
cmake --build build -j
./build/stc_server
# second terminal:
./build/icd_tx_tcp 127.0.0.1 7001 cu_state 2
./build/tlm_rx_udp --raw
```

### Workflow B: GUI operator station
```bash
cmake -S . -B build_gui -DBUILD_DIAG_GUI=ON
cmake --build build_gui -j
./build_gui/stc_server
./build_gui/diag_gui_qt
```

### Workflow C: Hardware CAN validation
```bash
cmake -S . -B build_hwcan -DUSE_SIMULATOR=OFF -DBUILD_DIAG_GUI=OFF
cmake --build build_hwcan -j
./build_hwcan/stc_server --can can0 --node 1
./build_hwcan/icd_tx_tcp 127.0.0.1 7001 suite
```

---

## 9) Troubleshooting

- **`diag_gui_qt` missing**: Qt6 dependencies were not found at configure time.
- **Permission errors on RT/network features**: apply `setcap` commands above or run with appropriate service policy.
- **No CAN interface**: create `vcan0` first (Section 6).
- **Server option confusion**: run `./build/stc_server --help` and use long-form flags shown there.
