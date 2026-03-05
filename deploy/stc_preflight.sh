#!/usr/bin/env bash
set -euo pipefail

CAN_IFACE="${1:-can0}"
REQ_RTPRIO="${STC_REQ_RTPRIO:-90}"

fail() {
  echo "[preflight][ERROR] $*" >&2
  exit 1
}

info() {
  echo "[preflight][INFO] $*"
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || fail "Required command not found: $1"
}

check_can_interface() {
  if ! ip link show "$CAN_IFACE" >/dev/null 2>&1; then
    fail "Interface '$CAN_IFACE' does not exist. Configure SocketCAN before starting STC."
  fi

  local kind
  kind="$(ip -d -o link show "$CAN_IFACE" | awk '{for (i=1;i<=NF;i++) if ($i=="can") {print $i; exit}}')"
  [[ "$kind" == "can" ]] || fail "Interface '$CAN_IFACE' exists but is not a CAN interface."

  if ! ip -o link show "$CAN_IFACE" | grep -q "UP"; then
    fail "Interface '$CAN_IFACE' is down. Bring it up (e.g., 'ip link set $CAN_IFACE up type can bitrate <rate>')."
  fi

  info "CAN interface '$CAN_IFACE' is present and UP."
}

check_can_permissions() {
  python3 - "$CAN_IFACE" <<'PY'
import socket
import sys

iface = sys.argv[1]
try:
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.close()
except PermissionError as exc:
    print(f"[preflight][ERROR] Missing permission for raw CAN socket on {iface}: {exc}", file=sys.stderr)
    sys.exit(1)
except OSError as exc:
    print(f"[preflight][ERROR] Cannot bind raw CAN socket on {iface}: {exc}", file=sys.stderr)
    sys.exit(1)
else:
    print(f"[preflight][INFO] Raw SocketCAN access check passed on {iface}.")
PY
}

check_rt_limits() {
  local rtprio memlock

  rtprio="$(ulimit -r)"
  memlock="$(ulimit -l)"

  if [[ "$rtprio" != "unlimited" ]] && (( rtprio < REQ_RTPRIO )); then
    fail "RLIMIT_RTPRIO is $rtprio, below required $REQ_RTPRIO."
  fi

  if [[ "$memlock" != "unlimited" ]] && (( memlock == 0 )); then
    fail "RLIMIT_MEMLOCK is 0. Set LimitMEMLOCK=infinity (or a non-zero value) in service configuration."
  fi

  info "RT limits check passed (RTPRIO=$rtprio, MEMLOCK=$memlock)."
}

check_capabilities_hint() {
  if [[ -r /proc/self/status ]]; then
    local capeff
    capeff="$(awk '/^CapEff:/ {print $2}' /proc/self/status)"
    info "Effective capabilities mask: 0x$capeff"
  fi
}

main() {
  require_cmd ip
  require_cmd python3

  check_can_interface
  check_can_permissions
  check_rt_limits
  check_capabilities_hint

  info "Preflight checks completed successfully."
}

main "$@"
