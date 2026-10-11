#!/usr/bin/env bash
# Steering Turret Control (STC)
#
# This script is part of the STC development codebase.
# Exercises command sequencing used in field demos while keeping bounded motion/safety-oriented flow.

set -euo pipefail

HOST="${1:-127.0.0.1}"
PORT="${2:-7000}"
TX="${TX:-./build/icd_tx_tcp}"

MIN_DEG=-140.0
MAX_DEG=140.0
HOME_DEG=0.0
MAX_LOAD_SPEED=25.0
STEP_DEG=0.1
STEP_INTERVAL_S=2

HOLD_CENTER_S=3
HOLD_LIMIT_S=4
HOME_SETTLE_S=8

MOVE_140_S=5.8
MOVE_280_S=11.4

log() {
  printf '[%s] %s\n' "$(date +'%H:%M:%S')" "$*"
}

send() {
  log "CMD: $*"
  "$TX" "$HOST" "$PORT" "$@"
}

# Safety phase: always quiesce motion and return home, including interrupt path cleanup.
safe_stop_and_home() {
  log "Safety stop + home"
  set +e
  send turret speed 0 0
  sleep 1
  send turret abs "$HOME_DEG" 0
  sleep "$HOME_SETTLE_S"
  set -e
}

on_interrupt() {
  log "Interrupted by user (SIGINT/SIGTERM)"
  safe_stop_and_home
  exit 130
}
trap on_interrupt INT TERM

# Phase 0: bring runtime to OPER before sending motion commands.
log "Starting turret motion demo host=$HOST port=$PORT"

# Enter OPER state; expected observable is command acceptance for turret commands.
send cu_state 3
sleep 1

# Phase 1: normalize initial pose to home so subsequent dwell timings are reproducible.
log "Normalize start position to HOME=$HOME_DEG"
send turret abs "$HOME_DEG" 0
sleep "$HOME_SETTLE_S"

log "Hold center for ${HOLD_CENTER_S}s"
sleep "$HOLD_CENTER_S"

# Phase 2: bounded sweep to minimum; expected stop at limit hold interval without faulting.
log "Move to MIN=$MIN_DEG at ${MAX_LOAD_SPEED} deg/s"
send turret speed -"$MAX_LOAD_SPEED" 0
sleep "$MOVE_140_S"
send turret speed 0 0
log "Hold at MIN for ${HOLD_LIMIT_S}s"
sleep "$HOLD_LIMIT_S"

# Phase 3: bounded sweep to maximum; validates opposite-direction command path symmetry.
log "Move to MAX=$MAX_DEG at ${MAX_LOAD_SPEED} deg/s"
send turret speed "$MAX_LOAD_SPEED" 0
sleep "$MOVE_280_S"
send turret speed 0 0
log "Hold at MAX for ${HOLD_LIMIT_S}s"
sleep "$HOLD_LIMIT_S"

# Phase 4: controlled return to home and settle before script exit.
log "Return HOME at ${MAX_LOAD_SPEED} deg/s"
send turret speed -"$MAX_LOAD_SPEED" 0
sleep "$MOVE_140_S"
send turret speed 0 0
sleep 2

log "Granularity demo: abs steps ${STEP_DEG} deg every ${STEP_INTERVAL_S}s"
pos="$HOME_DEG"
while awk "BEGIN {exit !($pos <= $MAX_DEG + 1e-9)}"; do
  send turret abs "$pos" 0
  sleep "$STEP_INTERVAL_S"
  pos="$(awk "BEGIN {printf \"%.1f\", $pos + $STEP_DEG}")"
done

log "Granularity demo done; returning HOME"
send turret abs "$HOME_DEG" 0
sleep "$HOME_SETTLE_S"

log "Turret motion demo complete"
