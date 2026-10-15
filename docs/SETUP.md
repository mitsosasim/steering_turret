# Build and setup

Run the commands below from the repository root. Start with the simulator. Hardware commands require a configured CAN interface and a reviewed motor configuration.

## Requirements

- Linux, CMake 3.16 or later, and a C11/C++17 compiler such as GCC/G++.
- Make or Ninja.
- Optional: Qt6 Core, Gui, Widgets, and Network for the diagnostic console.
- For CAN bench work: `iproute2` and, for frame capture, `can-utils`.
- For deployment: systemd and capability tools (`setcap`, `getcap`).

On Debian/Ubuntu:

```bash
sudo apt install build-essential cmake qt6-base-dev iproute2 can-utils libcap2-bin
```

On Arch/Manjaro:

```bash
sudo pacman -S --needed base-devel cmake qt6-base iproute2 can-utils libcap
```

## Build variants

Use separate build directories so configuration changes do not overwrite another build.

```bash
# Simulator, no GUI
cmake -S . -B build -DUSE_SIMULATOR=ON -DBUILD_DIAG_GUI=OFF \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2

# Hardware backend, with optional Qt GUI
cmake -S . -B build-can -DUSE_SIMULATOR=OFF -DBUILD_DIAG_GUI=ON \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build-can -j2
```

If Qt6 is not found, CMake skips `diag_gui_qt`. The server and CLI tools can still build. To build a simulator with the GUI, use `-DBUILD_DIAG_GUI=ON` in the first command.

## Tests

```bash
ctest --test-dir build --output-on-failure -LE '^hardware$'
ctest --test-dir build-can --output-on-failure -LE '^hardware$'
./build/test_diag_gui_regressions
```

The hardware label contains a test that connects to `can0` and actuator node 1. It is excluded from the commands above. Run it only during a prepared actuator bench session:

```bash
ctest --test-dir build-can -R '^x8_25_test_can_hw$' --output-on-failure
```

## Run the simulator

```bash
./build/stc_server --backend simulator --rt-enabled 0
```

This is a no-axis smoke run. It starts the network server without motors or RT privileges, but the startup inventory check reports FAIL because the expected motor count is zero. Status requests and telemetry remain available. The current configuration guard also rejects simulator startup with enabled hardware-driver axes. Use the backend regression tests for simulated motion. This command does not demonstrate a complete simulated turret session.

In another terminal:

```bash
./build/icd_tx_tcp 127.0.0.1 7000 req_hw
./build/icd_tx_tcp 127.0.0.1 7000 req_sw_version
./build/icd_tx_tcp 127.0.0.1 7000 req_work_time
```

Use the client's help output to inspect supported commands. Command-line options override loaded configuration values.

## Receive telemetry

Start one receiver per stream, then connect the TCP client. The server sends UDP unicast telemetry to the address of the connected TCP client.

```bash
./build/tlm_rx_udp 6010 --verbose
./build/tlm_rx_udp 6020 --verbose
```

| Stream | Default port | Default rate | Content |
|---|---:|---:|---|
| TLMH | 6010 | 100 Hz | Position, velocity, acceleration |
| TLML | 6020 | 10 Hz | State, uptime, hardware status, diagnostics |

For packet layouts, see [communication_protocol.md](communication_protocol.md). Laser status is commanded software state, not measured physical output.

## Qt diagnostic console

Run on a Linux desktop with Qt6 installed:

```bash
./build-can/diag_gui_qt
```

For a headless Linux target, use SSH X11 forwarding with your own target identity:

```bash
ssh -Y developer@your-target
./build-can/diag_gui_qt
```

The workstation needs a running X server. No company-specific hosts, addresses, or accounts are required.

## CAN setup

Match the bitrate to your bench hardware. This example uses 1 Mbit/s:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
ip -details -statistics link show can0
```

Virtual CAN is useful for transport experiments. It does not simulate an actuator responding to motor commands:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

To capture traffic:

```bash
mkdir -p logs/can
candump -tz can0 | tee logs/can/session.log
```

## Configure the hardware backend

`--backend can` selects real CAN access. Set the motor protocol separately in each enabled `[AxisN]` section:

| Driver | Protocol |
|---|---|
| `pd4c` | Nanotec PD4-C CANopen / CiA 402 |
| `myact_x8_25` | MYACTUATOR vendor CAN |
| `disabled` | Unmapped axis |

Examples are under [`dev/config/`](../dev/config/). Both bench profiles currently select MYACTUATOR. For a Nanotec PD4-C bench, select `driver=pd4c` in the enabled axis section. Confirm node IDs, bus settings, encoder scaling, gear ratio, limits, and logical mappings for your bench. CAN discovery is disabled by default. Missing axis sections do not automatically enable motors.

```bash
./build-can/stc_server --config dev/config/stc_bench_one_motor.ini --backend can
# Or use the dedicated X8-25 bench profile:
./build-can/stc_server --config dev/config/stc_bench_x8_25.ini --backend can
```

Motion is gated by operational state and axis availability. An ACK means the command was accepted under the protocol rules. Check feedback to establish physical completion.

## Low-level motor bench tools

MYACTUATOR test executable:

```bash
./build-can/x8_25_test_can can0 1
```

Nanotec test program:

```bash
cc -std=c11 -O2 -Idev/include dev/tests/pd4c_test.c dev/src/pd4c.c \
  -lm -o /tmp/pd4c_test
/tmp/pd4c_test can0 1
```

These tools can command real motion. Read the tool options and prepare the actuator bench before running them.

## Service deployment

The supplied service expects a `stc` service account, a working `can0`, and files under `/opt/stc`. Create the account if it does not already exist, and install the hardware build:

```bash
sudo useradd --system --user-group --no-create-home stc
sudo install -d /opt/stc/bin /opt/stc/config /opt/stc/deploy
sudo install -m 0755 build-can/stc_server /opt/stc/bin/stc_server
sudo install -m 0644 dev/config/stc_default.ini /opt/stc/config/stc_default.ini
sudo install -m 0755 deploy/stc_preflight.sh /opt/stc/deploy/stc_preflight.sh
sudo install -m 0644 deploy/stc_server.service /etc/systemd/system/stc_server.service
```

Review `/opt/stc/config/stc_default.ini` for your hardware before starting the service. The service enables strict FIFO scheduling and supplies capabilities and RT resource limits.

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now stc_server
sudo systemctl status stc_server --no-pager
sudo journalctl -u stc_server -b --no-pager
```

See [`deploy/OPERATOR_RUNBOOK.md`](../deploy/OPERATOR_RUNBOOK.md) for the privilege model and preflight checks, and [`deploy/CPU_AFFINITY.md`](../deploy/CPU_AFFINITY.md) for affinity guidance.

## Troubleshooting

| Symptom | Check |
|---|---|
| GUI target missing | Qt6 packages and `BUILD_DIAG_GUI=ON` |
| RT startup fails | Capabilities, resource limits, scheduler policy. Use `--rt-enabled 0` for simulation |
| CAN backend fails | Interface state, bitrate, permissions, wiring, enabled axes, node IDs |
| No telemetry | TCP client connection, receiver binding, UDP ports, firewall |
| Motion rejected | Operational state, live axis availability, payload range, configured limits |
| Unexpected scaling | Feedback units, encoder counts, gear ratio, command and telemetry signs |

[Back to the project overview](../README.md)
