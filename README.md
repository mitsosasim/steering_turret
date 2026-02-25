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
