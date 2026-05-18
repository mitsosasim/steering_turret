#!/usr/bin/env bash
set -euo pipefail

cd ~/STC
mkdir -p logs/can logs/server logs/icd

sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100

RUN="$(date +%F_%H%M%S)"

candump -tz -cae can0,141:7FF,241:7FF \
  | tee "logs/can/stc_x8_server_${RUN}.candump.log" &
CANDUMP_PID=$!

./build/stc_server \
  --config dev/config/stc_bench_x8_25.ini \
  --can can0 \
  --tcp-port 7000 \
  2>&1 | tee "logs/server/stc_x8_server_${RUN}.log" &
SERVER_PID=$!

sleep 2

./build/icd_tx_tcp 127.0.0.1 7000 cu_state 3 | tee "logs/icd/cu_oper_${RUN}.log"
./build/icd_tx_tcp 127.0.0.1 7000 mirror 1 rel 5 50 | tee "logs/icd/mirror_rel_${RUN}.log"
sleep 2
./build/icd_tx_tcp 127.0.0.1 7000 mirror 1 rel -5 50 | tee "logs/icd/mirror_rel_back_${RUN}.log"
sleep 2
./build/icd_tx_tcp 127.0.0.1 7000 mirror 1 speed 5 | tee "logs/icd/mirror_speed_${RUN}.log"
sleep 1
./build/icd_tx_tcp 127.0.0.1 7000 mirror 1 speed 0 | tee "logs/icd/mirror_speed_stop_${RUN}.log"
./build/icd_tx_tcp 127.0.0.1 7000 cu_state 2 | tee "logs/icd/cu_safe_${RUN}.log"

kill "$SERVER_PID" || true
kill "$CANDUMP_PID" || true

echo "$RUN" > /tmp/stc_x8_server_run_id
