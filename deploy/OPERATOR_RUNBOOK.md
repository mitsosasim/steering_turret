# STC operator runbook (RT + SocketCAN)

## Privilege model choices

Two deployment models are supported:

1. **Capability-based service user (recommended)**
   - Service runs as non-root user (`stc`).
   - Grants only required privileges:
     - `CAP_SYS_NICE` for RT scheduler policy/priority.
     - `CAP_NET_RAW` for SocketCAN raw access on `can0`.
   - Enforced through `CapabilityBoundingSet=` + `AmbientCapabilities=` in `deploy/stc_server.service`.

2. **Root service**
   - Service runs as `root`.
   - Works without explicit capability declarations but increases blast radius.
   - Use only when policy or platform constraints prevent capability-based operation.

## Install/upgrade checklist

1. Install artifacts:
   - `stc_server` binary under `/opt/stc/bin/`
   - config under `/opt/stc/config/`
   - preflight script under `/opt/stc/deploy/stc_preflight.sh` (executable)
   - unit file to `/etc/systemd/system/stc_server.service`
2. Reload + start:
   - `systemctl daemon-reload`
   - `systemctl enable --now stc_server`
3. Verify:
   - `systemctl status stc_server --no-pager`
   - `journalctl -u stc_server -b --no-pager`

## Preflight behavior

The service executes `ExecStartPre=/opt/stc/deploy/stc_preflight.sh`.
It blocks startup if any of the following fail:

- `can0` does not exist.
- `can0` is not UP or not a CAN interface.
- Raw CAN socket bind test fails (missing `CAP_NET_RAW`/permissions).
- RT limits are below required floor (`RTPRIO < 90`, `MEMLOCK == 0`).

## Troubleshooting by privilege model

### Capability-based mode

- Symptom: startup error about RT privileges.
  - Confirm service has `CAP_SYS_NICE` in both capability directives.
  - Confirm `LimitRTPRIO=90` (or higher than configured thread priorities).
- Symptom: cannot open `can0` raw socket.
  - Confirm `CAP_NET_RAW` is present in both capability directives.
  - Confirm `can0` exists and is UP.

### Root mode

- Symptom: still failing RT or CAN checks.
  - Confirm `can0` setup and bitrate are correct.
  - Confirm `LimitRTPRIO`/`LimitMEMLOCK` are not constrained by local overrides.

## Operational notes

- Keep `NoNewPrivileges=true` in capability-based mode.
- For deterministic timing, apply CPU isolation and affinity guidance in `deploy/CPU_AFFINITY.md`.
- Re-run preflight manually during maintenance windows:
  - `/opt/stc/deploy/stc_preflight.sh can0`
