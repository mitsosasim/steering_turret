# CPU isolation and affinity recommendations (CAN + telemetry)

For stable real-time behavior with SocketCAN (`can0`) and telemetry threads:

1. Reserve at least one isolated core for the STC process.
   - Typical kernel arguments: `isolcpus=2 nohz_full=2 rcu_nocbs=2`
   - Keep IRQ-heavy housekeeping on non-isolated cores.
2. Pin the service to isolated CPU(s):
   - In `systemd`: `CPUAffinity=2`
   - If two dedicated cores are available, use one for CAN and one for telemetry (`CPUAffinity=2 3`).
3. Keep CAN IRQ handling aligned with the CAN worker core:
   - Pin CAN controller IRQs to the same core as CAN worker where possible.
4. Prefer strict RT priority ordering:
   - CAN worker thread highest application RT priority.
   - High-rate telemetry next.
   - Low-rate telemetry below high-rate telemetry.
   - Leave headroom beneath kernel RT threads.
5. Validate scheduler and placement at runtime:
   - `ps -eLo pid,tid,cls,rtprio,psr,comm | grep stc_server`
   - `/proc/interrupts` and IRQ affinity masks for CAN controller.

> Note: exact CPU IDs depend on the target. Update examples to match platform topology.
