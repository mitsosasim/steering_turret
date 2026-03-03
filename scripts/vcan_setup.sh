#!/usr/bin/env bash
set -euo pipefail

modprobe vcan || true
ip link show vcan0 >/dev/null 2>&1 || ip link add dev vcan0 type vcan
ip link set up vcan0
ip -details link show vcan0
