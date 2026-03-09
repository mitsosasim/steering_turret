# STC

## Build

```bash
cmake -S . -B build
cmake --build build -j
```

## Run

```bash
./build/stc_server --help
```

If `--help` is not implemented in your build, run the server directly:

```bash
./build/stc_server
```

### Real-time scheduling privileges

`stc_server` can request RT scheduler policies (`SCHED_FIFO`/`SCHED_RR`).
At startup it probes RT privilege once and behaves deterministically:

- `--rt-strict 1`: fail startup if RT policy/priority cannot be set.
- `--rt-strict 0`: log warning + remediation and continue with RT disabled.

Runtime flags:

```text
--rt-enabled 0|1
--rt-strict 0|1
--rt-policy fifo|rr|other
--can-worker-prio <int> --tlmh-prio <int> --tlml-prio <int>
```

Grant `CAP_SYS_NICE` directly on the binary:

```bash
sudo setcap cap_sys_nice=ep /opt/stc/bin/stc_server
getcap /opt/stc/bin/stc_server
```

Install the Linux deployment artifacts (`systemd` unit + preflight checks):

```bash
sudo install -d /opt/stc/deploy
sudo install -m 0755 deploy/stc_preflight.sh /opt/stc/deploy/stc_preflight.sh
sudo install -m 0644 deploy/stc_server.service /etc/systemd/system/stc_server.service
sudo systemctl daemon-reload
sudo systemctl enable --now stc_server
sudo systemctl status stc_server --no-pager
```

See `deploy/CPU_AFFINITY.md` for CPU isolation guidance and `deploy/OPERATOR_RUNBOOK.md` for privilege model (root vs capability-based) operations.

### ICD command transmitter (`icd_tx_tcp`)

`icd_tx_tcp` uses:

```text
./build/icd_tx_tcp <host> <port> <cmd> [args...]
```

Examples:

```bash
./build/icd_tx_tcp 127.0.0.1 7001 cu_state 2
./build/icd_tx_tcp 127.0.0.1 7001 laser 1 1
./build/icd_tx_tcp 127.0.0.1 7001 turret speed 10.0 0.0
./build/icd_tx_tcp 127.0.0.1 7001 manualpos 37.5665 126.9780 45.0 90.0 --float
```

### Telemetry receiver (`tlm_rx_udp`)

`tlm_rx_udp` uses:

```text
./build/tlm_rx_udp [--ifname <iface>] [--tlmh-ip <ip> --tlmh-port <port>] [--tlml-ip <ip> --tlml-port <port>] [--raw]
```

Examples:

```bash
./build/tlm_rx_udp
./build/tlm_rx_udp --ifname eth0
./build/tlm_rx_udp --tlmh-ip 239.255.10.2 --tlmh-port 6010 --tlml-ip 239.255.10.3 --tlml-port 6020 --raw
```


### Qt diagnostics GUI (`diag_gui_qt`)

If Qt6 (Core/Gui/Widgets/Network) is available, CMake builds `diag_gui_qt`.
The GUI provides:

- ICD command send controls (`CU_STATE`, `SELFLEV`, `IBIT`, turret motion).
- Dual UDP telemetry listeners (high/low rate) with asynchronous `readyRead` handling.
- Dynamic 18-axis panel (`Position`, `Velocity`, `Target`, `Type`).
- Simulation mode toggle with a 10ms update timer.

Run:

```bash
./build/diag_gui_qt
```
