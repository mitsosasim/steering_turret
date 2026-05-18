#!/usr/bin/env bash
set -euo pipefail

cd ~/STC

RUN="$(cat /tmp/stc_x8_server_run_id)"

grep -E "ACK" logs/icd/*_${RUN}.log
grep -E " A8 " logs/can/stc_x8_server_${RUN}.candump.log
grep -E " A2 " logs/can/stc_x8_server_${RUN}.candump.log
grep -E " 81 " logs/can/stc_x8_server_${RUN}.candump.log
grep -E " 80 " logs/can/stc_x8_server_${RUN}.candump.log

! grep -iE "segmentation|assert|unknown driver|config error|FAIL_NO_ACK|ETIMEDOUT" \
  logs/server/stc_x8_server_${RUN}.log
