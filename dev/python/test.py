#!/usr/bin/env python3
"""
PD4-C CANopen full basic test on SocketCAN (can0).

What this script does, in sequence:

1) Opens can0 and sends a dummy CAN frame (ID 0x123) to prove TX works.
2) Reads identity info via SDO (1000h, 1018h).
3) Reads initial mode, statusword, error register via SDO.
4) Runs VELOCITY MODE via SDO:
   - NMT Start
   - Clear fault, vendor power ON
   - 6040 sequence (0x0006, 0x0007, 0x000F)
   - 6060=2 (Velocity)
   - 6042 = +200 for ~3s, then 0
5) Runs PROFILE POSITION via SDO (simple test, may not move if scaling isn’t commissioned yet).
6) Runs VELOCITY MODE via PDO:
   - Still 6060=2 via SDO
   - Configures TxPDO1 event timer (1800:05) to 100 ms
   - Sends RxPDO3 (COB-ID 0x400+Node) with 6042 target velocity
   - Listens for TxPDO1 (0x180+Node) and decodes statusword/position/velocity actual
7) Performs SAFE STOP:
   - 6040 quick stop + disable voltage
   - vendor power OFF (2400:07)

All along, it prints debug logs with timestamps.
"""

import time
import struct
import can

# -------------------------------------------------------------------
# Configuration
# -------------------------------------------------------------------

NODE_ID = 1
CAN_CHANNEL = "can0"
CAN_BUSTYPE = "socketcan"  # SocketCAN on Linux

SDO_TX_ID = 0x600 + NODE_ID   # Client → drive
SDO_RX_ID = 0x580 + NODE_ID   # Drive → client
NMT_ID    = 0x000

# PDO COB-IDs (factory preset, per PD4-C CANopen manual)
RXPDO1_ID = 0x200 + NODE_ID   # Controlword + mode (example mapping)
RXPDO3_ID = 0x400 + NODE_ID   # 6042h:00h VL target velocity (velocity control) 
TXPDO1_ID = 0x180 + NODE_ID   # 6041h, 6064h, 6044h (statusword, pos, vl velocity actual) 

ABORT = {
    0x05030000: "Toggle bit not altered",
    0x05040000: "SDO timeout",
    0x05040001: "Command specifier invalid",
    0x06010000: "Unsupported access",
    0x06010001: "Read/Write only mismatch",
    0x06010002: "Write to read-only",
    0x06020000: "Object does not exist",
    0x06090011: "Subindex does not exist",
    0x06090030: "Value range exceeded",
    0x06070010: "Data type mismatch",
    0x06070012: "Data type mismatch",
    0x06070013: "Data length mismatch",
    0x08000000: "General error",
    0x08000020: "Cannot transfer/store",
}

def abtxt(code: int) -> str:
    return ABORT.get(code, f"SDO abort 0x{code:08X}")


# -------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------

def log(msg: str) -> None:
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] {msg}")


def sw_tags(sw: int) -> str:
    """Decode 6041 statusword bits into human-readable tags (CiA 402 style)."""
    tags = []
    if sw & 0x0001: tags.append("Ready")
    if sw & 0x0002: tags.append("SwOn")
    if sw & 0x0004: tags.append("OpEn")
    if sw & 0x0008: tags.append("Fault")
    if sw & 0x0010: tags.append("VoltEn")
    if sw & 0x0020: tags.append("QStopOK")
    if sw & 0x0040: tags.append("SwOnDis")
    if sw & 0x0080: tags.append("Warn")
    if sw & 0x0100: tags.append("Rem2")
    if sw & 0x0200: tags.append("Remote")
    if sw & 0x0400: tags.append("TargetReached")
    if sw & 0x0800: tags.append("InternalLimit")
    if sw & 0x1000: tags.append("SetptAck")
    return ",".join(tags)


def sdo_xfer(bus: can.Bus, req: bytes) -> can.Message | None:
    """Low-level SDO transfer: send request, wait for response on SDO_RX_ID."""
    try:
        bus.send(can.Message(arbitration_id=SDO_TX_ID, data=req, is_extended_id=False))
    except Exception as e:
        log(f"⚠️ SDO TX error: {e}")
        return None

    t0 = time.time()
    while True:
        msg = bus.recv(0.25)
        if msg is None:
            return None
        if msg.arbitration_id == SDO_RX_ID:
            return msg
        if time.time() - t0 > 0.7:
            return None


def sdo_read(bus: can.Bus, idx: int, sub: int, label: str = "") -> int | None:
    if label:
        log(f"SDO R {idx:04X}:{sub:02X} ({label})")
    else:
        log(f"SDO R {idx:04X}:{sub:02X}")

    req = bytes([0x40, idx & 0xFF, (idx >> 8) & 0xFF, sub, 0, 0, 0, 0])
    msg = sdo_xfer(bus, req)
    if msg is None:
        log("  → timeout")
        return None
    if msg.data[0] == 0x80:
        ac = struct.unpack("<I", msg.data[4:8])[0]
        log(f"  → ABORT: {abtxt(ac)}")
        return None

    n = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}.get(msg.data[0], 0)
    if n == 0:
        log(f"  → unexpected cmd specifier 0x{msg.data[0]:02X}")
        return None

    val = int.from_bytes(msg.data[4:4 + n], "little", signed=False)
    log(f"  → 0x{val:X} ({val})")
    return val


def sdo_write(
    bus: can.Bus,
    idx: int,
    sub: int,
    value: int,
    size: int,
    label: str = "",
    retry_4b_on_len_mismatch: bool = False,
) -> bool:
    if label:
        log(f"SDO W {idx:04X}:{sub:02X} ({label}) <- 0x{value:X}")
    else:
        log(f"SDO W {idx:04X}:{sub:02X} <- 0x{value:X}")

    cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[size]
    if size == 1:
        payload = bytes([value & 0xFF, 0, 0, 0])
    elif size == 2:
        payload = struct.pack("<H", value & 0xFFFF) + b"\x00\x00"
    else:
        payload = struct.pack("<I", value & 0xFFFFFFFF)

    req = bytes([cmd, idx & 0xFF, (idx >> 8) & 0xFF, sub]) + payload
    msg = sdo_xfer(bus, req)
    if msg is None:
        log("  → timeout")
        return False
    if msg.data[0] == 0x80:
        ac = struct.unpack("<I", msg.data[4:8])[0]
        log(f"  → ABORT: {abtxt(ac)}")
        if retry_4b_on_len_mismatch and ac == 0x06070013 and size in (1, 2):
            log("  → retrying as 4-byte write")
            return sdo_write(bus, idx, sub, value, 4, label, False)
        return False

    log("  → OK")
    return True


# -------------------------------------------------------------------
# CANopen high-level helpers
# -------------------------------------------------------------------

def nmt_start(bus: can.Bus) -> None:
    log("Sending NMT Start…")
    data = bytes([0x01, NODE_ID & 0x7F])
    try:
        bus.send(can.Message(arbitration_id=NMT_ID, data=data, is_extended_id=False))
        log(f"NMT sent: cs=0x01 to node {NODE_ID}")
    except Exception as e:
        log(f"⚠️ NMT send error: {e}")


def vendor_power_on(bus: can.Bus) -> None:
    log("Vendor power ON (2400:07 = 0x10)…")
    sdo_write(bus, 0x2400, 0x07, 0x10, 1, "Vendor power ON", retry_4b_on_len_mismatch=True)


def vendor_power_off(bus: can.Bus) -> None:
    log("Vendor power OFF (2400:07 = 0x00)…")
    sdo_write(bus, 0x2400, 0x07, 0x00, 1, "Vendor power OFF", retry_4b_on_len_mismatch=True)


def read_statusword(bus: can.Bus, label: str = "") -> int | None:
    v = sdo_read(bus, 0x6041, 0x00, label or "Statusword")
    if v is not None:
        sw = v & 0xFFFF
        log(f"Statusword = 0x{sw:04X}  [{sw_tags(sw)}]")
    return v


def enable_operation(bus: can.Bus) -> None:
    log("=== ENABLE OPERATION SEQUENCE ===")
    nmt_start(bus)
    log("Clearing fault (6040 = 0x0080)…")
    sdo_write(bus, 0x6040, 0x00, 0x0080, 2, "Controlword clear fault")

    vendor_power_on(bus)
    time.sleep(0.05)

    for value, name in [
        (0x0006, "Shutdown"),
        (0x0007, "Switch on"),
        (0x000F, "Enable operation"),
    ]:
        log(f"Controlword 6040 = 0x{value:04X} ({name})")
        sdo_write(bus, 0x6040, 0x00, value, 2, "Controlword")
        time.sleep(0.05)
        read_statusword(bus, f"Statusword after 0x{value:04X}")

    v = read_statusword(bus, "Statusword final")
    if v is not None:
        log(f"Statusword final = 0x{v & 0xFFFF:04X}")


def set_mode(bus: can.Bus, mode: int, name: str) -> bool:
    log(f"Setting mode of operation 6060 = {mode} ({name})…")
    if not sdo_write(bus, 0x6060, 0x00, mode, 1, "Mode of operation"):
        return False
    md = sdo_read(bus, 0x6061, 0x00, "Mode of operation display")
    if md is None:
        log("⚠️ Could not read 6061 back")
        return False
    log(f"Actual mode (6061) = {md} (expected {mode})")
    return md == mode


def safe_stop(bus: can.Bus) -> None:
    log("=== SAFE STOP ===")
    sdo_write(bus, 0x6040, 0x00, 0x0002, 2, "Quick stop")
    sdo_write(bus, 0x6040, 0x00, 0x0000, 2, "Disable voltage")
    vendor_power_off(bus)
    log("Safe stop sequence sent.")


# -------------------------------------------------------------------
# Tests
# -------------------------------------------------------------------

def test_identity_and_snapshot(bus: can.Bus) -> None:
    log("=== READING IDENTITY OBJECTS ===")
    sdo_read(bus, 0x1000, 0x00, "Device type")
    sdo_read(bus, 0x1018, 0x01, "Identity: Vendor ID")
    sdo_read(bus, 0x1018, 0x02, "Identity: Product code")
    sdo_read(bus, 0x1018, 0x03, "Identity: Revision")
    sdo_read(bus, 0x1018, 0x04, "Identity: Serial")

    log("=== INITIAL STATUS SNAPSHOT ===")
    sdo_read(bus, 0x6060, 0x00, "Mode of operation (commanded)")
    sdo_read(bus, 0x6061, 0x00, "Mode of operation display")
    read_statusword(bus, "Statusword initial")
    sdo_read(bus, 0x603F, 0x00, "Error register")


def test_velocity_mode_sdo(bus: can.Bus) -> None:
    log("=== VELOCITY MODE TEST VIA SDO (6060=2, 6042 speed) ===")
    enable_operation(bus)

    if not set_mode(bus, 2, "Velocity"):
        log("❌ Could not enter velocity mode, aborting this test.")
        return

    read_statusword(bus, "Statusword final before velocity")

    speed = 200  # units as PD4-C is currently configured (not normalized here)
    log(f"Writing target speed 6042 = {speed}…")
    sdo_write(bus, 0x6042, 0x00, speed, 2, "Velocity demand (6042)")
    log("Letting motor run for 3 seconds…")
    t_end = time.time() + 3.0
    while time.time() < t_end:
        read_statusword(bus, "Statusword while running")
        time.sleep(1.0)

    log("Stopping: 6042 = 0, then 6040 = 0x0006 (Shutdown)…")
    sdo_write(bus, 0x6042, 0x00, 0, 2, "Velocity demand (6042 stop)")
    sdo_write(bus, 0x6040, 0x00, 0x0006, 2, "Controlword shutdown")
    read_statusword(bus, "Statusword after stop")


def test_profile_position_sdo(bus: can.Bus) -> None:
    """
    Simple Profile Position test using whatever scaling the drive currently has.

    NOTE:
    - If the axis is not commissioned for user-friendly units (0.1° etc.), this
      may result in a very small motion or none at all. We still exercise:
      6060=1, 6081/6083/6084, 607A, 'new setpoint' via 6040.
    """
    log("=== PROFILE POSITION TEST (+90° equivalent in current units) ===")
    enable_operation(bus)

    read_statusword(bus, "Statusword before PP")
    v = sdo_read(bus, 0x6041, 0x00, "Statusword before PP")
    if v is not None:
        log(f"Statusword before PP = 0x{v & 0xFFFF:04X}")

    if not set_mode(bus, 1, "Profile Position"):
        log("❌ Could not enter Profile Position mode, aborting this test.")
        return

    log("Setting profile velocity/acc/dec in user units…")
    sdo_write(bus, 0x6081, 0x00, 0x14, 2, "Profile velocity")
    sdo_write(bus, 0x6083, 0x00, 0x64, 2, "Profile acceleration")
    sdo_write(bus, 0x6084, 0x00, 0x64, 2, "Profile deceleration")

    start = sdo_read(bus, 0x6064, 0x00, "Actual position before move (raw)")
    if start is None:
        log("⚠️ Could not read initial position; still issuing target.")
        start = 0
    else:
        log(f"Using start position = {start} (raw)")

    # Command something like +90° in '0.1°-like' increments; if scale is different,
    # this is just "some offset".
    target = start + 900
    log(f"Commanding target position = {target} (raw units)")
    sdo_write(bus, 0x607A, 0x00, target & 0xFFFFFFFF, 4, "Target position")

    log("New setpoint + start: 6040 = 0x001F then 0x003F")
    sdo_write(bus, 0x6040, 0x00, 0x001F, 2, "Controlword new setpoint")
    sdo_write(bus, 0x6040, 0x00, 0x003F, 2, "Controlword start movement")

    log("Waiting 1.5s for motion…")
    for _ in range(3):
        read_statusword(bus, "Statusword during PP")
        time.sleep(0.5)

    end = sdo_read(bus, 0x6064, 0x00, "Actual position after move (raw)")
    if end is not None:
        delta = (end - start)
        log(f"Move complete. Δpos = {delta} (raw units)")


def test_velocity_mode_pdo(bus: can.Bus) -> None:
    """
    Velocity mode driven via PDOs:

    - Still uses SDO for NMT/enable and setting 6060=2
    - Uses RxPDO3 (0x400+Node) mapped to 6042h:00h to send velocity demand
    - Configures TxPDO1 (0x180+Node) event timer so we get periodic status/position
    - Decodes TxPDO1 as: 6041 (statusword), 6064 (position), 6044 (velocity actual)
    """
    log("=== VELOCITY MODE TEST VIA PDO (RxPDO3 + TxPDO1) ===")
    enable_operation(bus)

    if not set_mode(bus, 2, "Velocity"):
        log("❌ Could not enter velocity mode, aborting PDO test.")
        return

    # Set TxPDO1 event timer to 100 ms so it spits out frames periodically 
    sdo_write(bus, 0x1800, 0x05, 100, 2, "TxPDO1 event timer (100 ms)")

    target_speed = 150  # same units as 6042
    log(f"Sending velocity {target_speed} via RxPDO3 (COB-ID 0x{RXPDO3_ID:03X})…")

    data = struct.pack("<i", target_speed)  # 6042h is INT32 in mapping
    msg = can.Message(arbitration_id=RXPDO3_ID, data=data, is_extended_id=False)
    try:
        bus.send(msg)
        log("RxPDO3 sent.")
    except Exception as e:
        log(f"⚠️ Error sending RxPDO3: {e}")
        return

    log("Listening for TxPDO1 (status/position/velocity) for ~3 seconds…")
    t_end = time.time() + 3.0
    while time.time() < t_end:
        msg = bus.recv(0.5)
        if msg is None:
            continue
        if msg.arbitration_id != TXPDO1_ID:
            continue

        # Decode TxPDO1 = 6041:00 (16b), 6064:00 (32b), 6044:00 (16b) 
        d = msg.data + b"\x00" * (8 - len(msg.data))  # pad defensively
        sw = int.from_bytes(d[0:2], "little", signed=False)
        pos = int.from_bytes(d[2:6], "little", signed=True)
        vel_act = int.from_bytes(d[6:8], "little", signed=True)
        log(
            f"TPDO1 0x{msg.arbitration_id:03X}: "
            f"SW=0x{sw:04X} [{sw_tags(sw)}], Pos={pos}, VelAct={vel_act}"
        )

    # Stop via PDO (set 6042=0)
    log("Stopping via PDO: sending RxPDO3 with 0 velocity…")
    data0 = struct.pack("<i", 0)
    try:
        bus.send(can.Message(arbitration_id=RXPDO3_ID, data=data0, is_extended_id=False))
        log("RxPDO3 stop sent.")
    except Exception as e:
        log(f"⚠️ Error sending RxPDO3 stop: {e}")


# -------------------------------------------------------------------
# Main
# -------------------------------------------------------------------

def main() -> None:
    log("Opening SocketCAN interface can0…")
    bus = None
    try:
        bus = can.interface.Bus(
            channel=CAN_CHANNEL,
            bustype=CAN_BUSTYPE,
            receive_own_messages=False,
        )
        log("Bus opened OK.")

        # Quick raw CAN sanity check
        log("Sending test CAN frame on can0 (ID 0x123)…")
        bus.send(can.Message(arbitration_id=0x123, data=b"\x11\x22\x33\x44", is_extended_id=False))
        log("Test frame sent (motor will ignore ID 0x123; just proves TX works).")

        # 1) Identity + initial status
        test_identity_and_snapshot(bus)

        # 2) Velocity mode via SDO
        test_velocity_mode_sdo(bus)

        # 3) Profile Position via SDO (simple sanity)
        test_profile_position_sdo(bus)

        # 4) Velocity mode via PDO (RxPDO3 / TxPDO1)
        test_velocity_mode_pdo(bus)

        # 5) Safe stop at the end
        safe_stop(bus)

        log("All tests done. If something did not move, check:")
        log("  - Node ID (NODE_ID in this script vs DIP/software)")
        log("  - Bitrate / termination (but SDOs already prove basic comms)")
        log("  - Enable / safety inputs on the drive wiring")
        log("  - Axis commissioning & scaling (60A8/60A9/gear, etc.)")

    finally:
        if bus is not None:
            try:
                bus.shutdown()
                log("CAN bus shut down cleanly.")
            except Exception as e:
                log(f"⚠️ Error shutting down CAN bus: {e}")


if __name__ == "__main__":
    main()
