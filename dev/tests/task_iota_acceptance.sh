#!/usr/bin/env bash
# ALMA SISTEMI SRL - Steering Turret Control (STC)
#
# Property of ALMA SISTEMI SRL.
# This script is part of the STC development codebase.
# Checks integration-level command responses to catch regressions in control-path interoperability.

set -euo pipefail

HOST="${1:-127.0.0.1}"
PORT="${2:-7000}"
TX="${TX:-./build/icd_tx_tcp}"

log(){ printf '[%s] %s\n' "$(date +'%H:%M:%S')" "$*"; }
run(){ log "CMD: $*"; "$TX" "$HOST" "$PORT" "$@"; }

require_bin(){
  if [[ ! -x "$TX" ]]; then
    echo "missing tx binary: $TX" >&2
    exit 2
  fi
}

# Phase 1: verify speed-mode command path and explicit zero-speed stop behavior.
native_speed_move(){
  log "native speed move"
  run cu_state 3
  run turret speed 12.0 0.0
  sleep 1
  run turret speed 0.0 0.0
}

# Phase 2: verify deterministic absolute-position command path and return-to-origin behavior.
position_move(){
  log "position move deterministic path"
  run turret abs 8.0 0.0
  sleep 1
  run turret abs 0.0 0.0
}

# Phase 3: verify quick-stop safety gate by dropping to SAFE during active speed command.
quick_stop(){
  log "quick-stop"
  run turret speed 20.0 0.0
  sleep 0.5
  run cu_state 2
  run turret speed 0.0 0.0
}

# Phase 4: verify SAFE->OPER hold/release interoperability with controller state command.
hold_release(){
  log "hold/release"
  run cu_state 2
  sleep 0.2
  run cu_state 3
}

# Phase 5: verify stream enable path; observable is live telemetry on UDP listener when attached.
active_telemetry(){
  log "active telemetry (enable stream + optional listener)"
  run cu_state 3
  echo "INFO: run ./build/tlm_rx_udp in parallel to confirm live telemetry updates"
}

# Phase 6: verify representative request/response roundtrip commands through full STC server path.
server_roundtrip(){
  log "server roundtrip through STC command path"
  run req_sw
  run posmode gps
  run posmode manual
}

all(){
  native_speed_move
  position_move
  quick_stop
  hold_release
  active_telemetry
  server_roundtrip
}

require_bin
case "${3:-all}" in
  native_speed_move) native_speed_move ;;
  position_move) position_move ;;
  quick_stop) quick_stop ;;
  hold_release) hold_release ;;
  active_telemetry) active_telemetry ;;
  server_roundtrip) server_roundtrip ;;
  all) all ;;
  *) echo "unknown scenario: ${3:-}" >&2; exit 2 ;;
esac
