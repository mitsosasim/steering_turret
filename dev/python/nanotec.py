# nanotec.py — Nanotec PD4-C over PCAN-USB (python-can)
# Minimal, robust GUI: Connect → Enable → PP / Velocity (VL or PV) → Safe Stop
# Log is always visible at bottom; Advanced commissioning hidden behind a toggle.
#
# Requirements (Windows):
#   pip install python-can
#   Install PEAK drivers (PCAN-USB). Choose channel "PCAN_USBBUS1".
#
# Notes:
#   - Units: Position = 0.1°, Velocity = deg/s (commissioning).
#   - Gear handled via 60E8/60ED. Speed requests are capped by 6080h (max motor rpm).
#   - Safe mode switching waits for 6061 to confirm.
#   - SDO writes auto-retry with 4B on “Data length mismatch”.

import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
import can, struct, threading, time, math, queue

# ------------------------ Helpers / Constants ------------------------

def sdo_tx(nid): return 0x600 + (nid & 0x7F)
def sdo_rx(nid): return 0x580 + (nid & 0x7F)
NMT_ID = 0x000

ABORT = {
    0x05030000:"Toggle bit not altered",
    0x05040000:"SDO timeout",
    0x05040001:"Cmd specifier invalid",
    0x06010000:"Unsupported access",
    0x06010001:"Read/Write only mismatch",
    0x06010002:"Write to read-only",
    0x06020000:"Object does not exist",
    0x06090011:"Subindex does not exist",
    0x06090030:"Value range exceeded",
    0x06070010:"Data type mismatch",
    0x06070012:"Data type mismatch",
    0x06070013:"Data length mismatch",
    0x08000000:"General error",
    0x08000020:"Cannot transfer/store",
}
def abtxt(code): return ABORT.get(code, f"SDO abort 0x{code:08X}")

def sw_tags(sw):
    tags=[]
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

# ----------------------------- App ----------------------------------

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Nanotec PD4-C • PCAN")
        self.geometry("980x700")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # State
        self.bus=None
        self.sdo_lock=threading.Lock()
        self.nid=tk.IntVar(value=1)
        self.iface=tk.StringVar(value="pcan")
        self.channel=tk.StringVar(value="PCAN_USBBUS1")
        self.bitrate=tk.StringVar(value="1M")
        self.verbose=tk.BooleanVar(value=False)
        self.adv=tk.BooleanVar(value=False)   # show Advanced section
        self.poll=True
        self.msgq=queue.Queue()
        self._last=""; self._dupcount=0

        # Motion parameters
        self.pp_t=tk.DoubleVar(value=90.0)
        self.pp_v=tk.DoubleVar(value=20.0)
        self.pp_a=tk.DoubleVar(value=100.0)
        self.pp_d=tk.DoubleVar(value=100.0)

        self.vl_v=tk.DoubleVar(value=5.0)   # Velocity Mode (6060=2), deg/s
        self.vl_a=tk.DoubleVar(value=50.0)
        self.vl_d=tk.DoubleVar(value=50.0)

        self.pv_v=tk.DoubleVar(value=5.0)   # Profile Velocity (6060=3), deg/s
        self.pv_a=tk.DoubleVar(value=50.0)
        self.pv_d=tk.DoubleVar(value=50.0)

        # Layout: top notebook + bottom log (persistent)
        self._build_ui()
        self.after(120, self._drain)
        threading.Thread(target=self._poll_task, daemon=True).start()

    # ---------------------------- UI --------------------------------

    def _build_ui(self):
        root_pane = ttk.PanedWindow(self, orient="vertical")
        root_pane.pack(fill="both", expand=True)

        top = ttk.Frame(root_pane); root_pane.add(top, weight=3)
        bottom = ttk.Frame(root_pane); root_pane.add(bottom, weight=1)

        # Notebook at top
        nb=ttk.Notebook(top); nb.pack(fill="both", expand=True)
        self.pg_setup=ttk.Frame(nb); nb.add(self.pg_setup, text="Setup")
        self.pg_motion=ttk.Frame(nb); nb.add(self.pg_motion, text="Motion")
        self.pg_diag=ttk.Frame(nb); nb.add(self.pg_diag, text="Diagnostics")

        # Global log at bottom
        self.log=ScrolledText(bottom, height=10); self.log.pack(fill="both", expand=True, padx=8, pady=(4,4))
        bar=ttk.Frame(bottom); bar.pack(fill="x", padx=8, pady=(0,8))
        ttk.Button(bar, text="Copy Log", command=self._copy).pack(side="left")
        ttk.Label(bar, text="Log stays visible while switching tabs").pack(side="left", padx=12)

        # Build pages
        self._ui_setup()
        self._ui_motion()
        self._ui_diag()

    def _ui_setup(self):
        f=self.pg_setup

        lf=ttk.LabelFrame(f, text="Connection", padding=10); lf.pack(fill="x", padx=8, pady=(8,6))
        ttk.Label(lf, text="Interface").grid(row=0, column=0, sticky="w")
        ttk.Entry(lf, textvariable=self.iface, width=10).grid(row=0, column=1)
        ttk.Label(lf, text="Channel").grid(row=0, column=2, sticky="w", padx=(8,0))
        ttk.Entry(lf, textvariable=self.channel, width=14).grid(row=0, column=3)
        ttk.Label(lf, text="Bitrate").grid(row=0, column=4, sticky="w", padx=(8,0))
        ttk.Combobox(lf, textvariable=self.bitrate, values=["1M","500k","250k","125k","50k"], width=10).grid(row=0, column=5)
        ttk.Label(lf, text="Node-ID").grid(row=1, column=0, sticky="w", pady=(8,0))
        ttk.Spinbox(lf, from_=1, to=127, textvariable=self.nid, width=6).grid(row=1, column=1, sticky="w", pady=(8,0))

        row=ttk.Frame(lf); row.grid(row=2, column=0, columnspan=6, sticky="w", pady=10)
        ttk.Button(row, text="Connect", command=self.connect).pack(side="left", padx=4)
        ttk.Button(row, text="Disconnect", command=self.disconnect).pack(side="left", padx=4)
        ttk.Button(row, text="NMT Start", command=lambda:self.nmt(0x01)).pack(side="left", padx=12)
        ttk.Button(row, text="NMT Stop", command=lambda:self.nmt(0x02)).pack(side="left", padx=4)
        ttk.Button(row, text="Reset Node", command=lambda:self.nmt(0x81)).pack(side="left", padx=4)
        ttk.Button(row, text="Reset Comm", command=lambda:self.nmt(0x82)).pack(side="left", padx=4)

        ef=ttk.LabelFrame(f, text="Enable / Power / Safety", padding=10); ef.pack(fill="x", padx=8, pady=(6,8))
        ttk.Button(ef, text="Enable Operation", command=self.enable).pack(side="left", padx=4)
        ttk.Button(ef, text="Quick Stop", command=self.quick_stop).pack(side="left", padx=4)
        ttk.Button(ef, text="Disable Voltage", command=self.disable_voltage).pack(side="left", padx=4)
        ttk.Button(ef, text="Vendor Power OFF", command=self.vendor_power_off).pack(side="left", padx=12)
        ttk.Button(ef, text="Safe Stop All", command=self.safe_stop_all).pack(side="left", padx=12)

        advf=ttk.LabelFrame(f, text="Advanced (commissioning)", padding=10); advf.pack(fill="x", padx=8, pady=(0,8))
        hdr=ttk.Frame(advf); hdr.pack(anchor="w", fill="x")
        ttk.Checkbutton(hdr, text="Show Advanced", variable=self.adv, command=lambda:self._toggle_adv(av)).pack(side="left")
        ttk.Checkbutton(hdr, text="Verbose SDO", variable=self.verbose).pack(side="left", padx=12)

        av=ttk.Frame(advf); av.pack(fill="x", padx=2, pady=(6,0))
        ttk.Button(av, text="Commission axis (0.1°, deg/s, Gear=100:1, wide limits)",
                   command=self.commission).pack(side="left", padx=4)
        ttk.Button(av, text="Save params to NVM", command=self.save_params).pack(side="left", padx=8)
        ttk.Button(av, text="Read key OD values", command=self.read_key_od).pack(side="left", padx=8)
        self._adv_frame = av
        self._toggle_adv(av, force_hide=True)

        self.conn_lbl=ttk.Label(f, text="—"); self.conn_lbl.pack(anchor="w", padx=10, pady=4)

    def _toggle_adv(self, frame, force_hide=False):
        if force_hide or not self.adv.get():
            try: frame.pack_forget()
            except Exception: pass
        else:
            frame.pack(fill="x", padx=2, pady=(6,0))

    def _ui_motion(self):
        f=self.pg_motion

        lf=ttk.LabelFrame(f, text="Live Feedback", padding=10); lf.pack(fill="x", padx=8, pady=(8,6))
        self.lbl_pos=ttk.Label(lf, text="Pos: —"); self.lbl_pos.pack(side="left", padx=10)
        self.lbl_sw =ttk.Label(lf, text="Status: —"); self.lbl_sw.pack(side="left", padx=10)
        self.lbl_err=ttk.Label(lf, text="Error: —"); self.lbl_err.pack(side="left", padx=10)
        self.lbl_md =ttk.Label(lf, text="Mode: —"); self.lbl_md.pack(side="left", padx=10)

        pf=ttk.LabelFrame(f, text="Profile Position (6060=1; Position in 0.1°)", padding=10); pf.pack(fill="x", padx=8, pady=6)
        ttk.Label(pf, text="Target (deg)").grid(row=0, column=0); ttk.Entry(pf, textvariable=self.pp_t, width=10).grid(row=0, column=1)
        ttk.Label(pf, text="Vel (deg/s)").grid(row=0, column=2); ttk.Entry(pf, textvariable=self.pp_v, width=8).grid(row=0, column=3)
        ttk.Label(pf, text="Acc/Dec (deg/s²)").grid(row=0, column=4)
        ttk.Entry(pf, textvariable=self.pp_a, width=8).grid(row=0, column=5)
        ttk.Entry(pf, textvariable=self.pp_d, width=8).grid(row=0, column=6)
        ttk.Button(pf, text="Move ABS", command=self.do_pp).grid(row=0, column=7, padx=10)

        vlf=ttk.LabelFrame(f, text="Velocity Mode (6060=2; 604C makes input = deg/s)", padding=10); vlf.pack(fill="x", padx=8, pady=6)
        ttk.Label(vlf, text="Velocity (deg/s)").grid(row=0, column=0); ttk.Entry(vlf, textvariable=self.vl_v, width=10).grid(row=0, column=1)
        ttk.Label(vlf, text="Slope Acc/Dec (deg/s per s)").grid(row=0, column=2)
        ttk.Entry(vlf, textvariable=self.vl_a, width=8).grid(row=0, column=3)
        ttk.Entry(vlf, textvariable=self.vl_d, width=8).grid(row=0, column=4)
        ttk.Button(vlf, text="Set Velocity (V-mode)", command=self.do_vl).grid(row=0, column=5, padx=10)
        ttk.Button(vlf, text="Stop (V-mode)", command=self.vl_stop).grid(row=0, column=6, padx=10)

        vf=ttk.LabelFrame(f, text="Profile Velocity (6060=3; Velocity in deg/s)", padding=10); vf.pack(fill="x", padx=8, pady=(6,8))
        ttk.Label(vf, text="Velocity (deg/s)").grid(row=0, column=0); ttk.Entry(vf, textvariable=self.pv_v, width=10).grid(row=0, column=1)
        ttk.Label(vf, text="Acc/Dec (deg/s²)").grid(row=0, column=2)
        ttk.Entry(vf, textvariable=self.pv_a, width=8).grid(row=0, column=3)
        ttk.Entry(vf, textvariable=self.pv_d, width=8).grid(row=0, column=4)
        ttk.Button(vf, text="Set Velocity", command=self.do_pv).grid(row=0, column=5, padx=10)

    def _ui_diag(self):
        f=self.pg_diag
        io=ttk.LabelFrame(f, text="SDO Read / Write", padding=10); io.pack(fill="x", padx=8, pady=(8,8))
        self.idx=tk.StringVar(value="6041"); self.sub=tk.StringVar(value="00")
        self.sz =tk.StringVar(value="2"); self.wv=tk.StringVar(value="")
        ttk.Label(io, text="Index (hex)").grid(row=0,column=0); ttk.Entry(io, textvariable=self.idx, width=8).grid(row=0,column=1)
        ttk.Label(io, text="Sub").grid(row=0,column=2); ttk.Entry(io, textvariable=self.sub, width=4).grid(row=0,column=3)
        ttk.Button(io, text="Read", command=self.ui_rd).grid(row=0,column=4, padx=6)
        ttk.Label(io, text="Write (hex/dec)").grid(row=1,column=0); ttk.Entry(io, textvariable=self.wv, width=16).grid(row=1,column=1, sticky="w")
        ttk.Label(io, text="Size").grid(row=1,column=2); ttk.Combobox(io, textvariable=self.sz, values=["1","2","4"], width=4).grid(row=1,column=3)
        ttk.Button(io, text="Write", command=self.ui_wr).grid(row=1,column=4, padx=6)

        rf=ttk.Frame(f); rf.pack(fill="x", padx=8, pady=(0,8))
        ttk.Button(rf, text="Read Statusword", command=lambda:self._log_sw()).pack(side="left", padx=4)
        ttk.Button(rf, text="Read Error Reg (603F)", command=lambda:self._log_err()).pack(side="left", padx=4)
        ttk.Button(rf, text="Clear Fault (6040=0x80)", command=lambda:self.cw(0x0080)).pack(side="left", padx=12)

    # --------------------------- Logging -----------------------------

    def _log(self, s):
        ts=time.strftime("%H:%M:%S") if self.bus else "--:--:--"
        self.msgq.put(f"[{ts}] {s}")

    def _drain(self):
        try:
            while True:
                line=self.msgq.get_nowait()
                tail=line.split("] ",1)[1]
                if tail == self._last:
                    self._dupcount += 1
                    try:
                        self.log.delete("end-2l linestart", "end-1c")
                    except Exception:
                        pass
                    self.log.insert("end", f"{line}  (x{self._dupcount})\n")
                else:
                    self._last = tail; self._dupcount = 1
                    self.log.insert("end", line+"\n")
                self.log.see("end"); self.conn_lbl.config(text=line)
        except queue.Empty:
            pass
        self.after(120, self._drain)

    def _copy(self):
        try:
            self.clipboard_clear(); self.clipboard_append(self.log.get("1.0","end"))
            self._log("Log copied to clipboard.")
        except Exception:
            pass

    # ---------------------------- CAN I/O ----------------------------

    def _bitrate(self):
        return {"1M":1_000_000,"500k":500_000,"250k":250_000,"125k":125_000,"50k":50_000}[self.bitrate.get()]

    def connect(self):
        if self.bus:
            self._log("Already connected."); return
        try:
            self.bus=can.interface.Bus(interface=self.iface.get(), channel=self.channel.get(),
                                       bitrate=self._bitrate(), receive_own_messages=False)
            self._log(f"✅ Connected: {self.iface.get()} {self.channel.get()} @ {self.bitrate.get()}")
        except Exception as e:
            self._log(f"❌ Connect error: {e}")

    def disconnect(self):
        try:
            self.safe_stop_all()
        except Exception:
            pass
        try:
            if self.bus:
                self.bus.shutdown()
        except Exception:
            pass
        self.bus=None
        self._log("Disconnected.")

    def nmt(self, cs):
        if not self.bus: return
        nid=self.nid.get()
        data=bytes([cs, nid & 0x7F])
        try:
            self.bus.send(can.Message(arbitration_id=NMT_ID, data=data, is_extended_id=False))
            desc={0x01:"start",0x02:"stop",0x81:"reset node",0x82:"reset comm"}.get(cs,hex(cs))
            self._log(f"NMT sent: cs=0x{cs:02X} to node {nid} ({desc})")
        except Exception as e:
            self._log(f"❌ NMT send error: {e}")

    # --------------------------- SDO core ----------------------------

    def _sdo_xfer(self, req):
        nid=self.nid.get()
        with self.sdo_lock:
            try:
                self.bus.send(can.Message(arbitration_id=sdo_tx(nid), data=req, is_extended_id=False))
                t0=time.time()
                while True:
                    msg=self.bus.recv(0.12)
                    if not msg:
                        return None
                    if msg.arbitration_id==sdo_rx(nid):
                        return msg
                    if time.time()-t0>0.5:
                        return None
            except Exception as e:
                self._log(f"⚠️ SDO tx error: {e}")
                return None

    def sdo_w(self, idx, sub, val, size, oklog=False):
        if not self.bus: self._log("Not connected."); return False
        cmd={1:0x2F,2:0x2B,4:0x23}[size]
        if size==1: payload=bytes([val & 0xFF, 0,0,0])
        elif size==2: payload=struct.pack("<H", val)+b"\0\0"
        else: payload=struct.pack("<I", val & 0xFFFFFFFF)
        req=bytes([cmd, idx & 0xFF, (idx>>8)&0xFF, sub]) + payload
        msg=self._sdo_xfer(req)
        if not msg:
            self._log(f"SDO w {idx:04X}:{sub:02X} — timeout"); return False
        if msg.data[0]==0x80:
            ac=struct.unpack("<I", msg.data[4:8])[0]
            if ac==0x06070013 and size in (1,2):
                # Retry as 4 bytes
                return self.sdo_w(idx, sub, val, 4, oklog)
            self._log(f"❌ SDO w {idx:04X}:{sub:02X}: {abtxt(ac)}"); return False
        if oklog or self.verbose.get():
            self._log(f"✔ SDO w {idx:04X}:{sub:02X} <- 0x{int.from_bytes(req[4:8],'little'):X}")
        return True

    def sdo_r(self, idx, sub, oklog=False):
        if not self.bus: self._log("Not connected."); return None
        req=bytes([0x40, idx & 0xFF, (idx>>8)&0xFF, sub, 0,0,0,0])
        msg=self._sdo_xfer(req)
        if not msg:
            self._log(f"SDO r {idx:04X}:{sub:02X} — timeout"); return None
        if msg.data[0]==0x80:
            ac=struct.unpack("<I", msg.data[4:8])[0]
            self._log(f"❌ SDO r {idx:04X}:{sub:02X}: {abtxt(ac)}"); return None
        n={0x4F:1,0x4B:2,0x47:3,0x43:4}.get(msg.data[0],0)
        if n==0:
            self._log(f"SDO r {idx:04X}:{sub:02X} — unexpected cmd 0x{msg.data[0]:02X}")
            return None
        val=int.from_bytes(msg.data[4:4+n], "little")
        if oklog or self.verbose.get():
            self._log(f"✔ SDO r {idx:04X}:{sub:02X} -> 0x{val:X}")
        return val

    # ----------------------- Commission / Reads ----------------------

    def commission(self):
        """Units = 0.1° (60A8) / deg/s (60A9), Gear=100:1, 6080=1500 rpm, wide limits."""
        if not self.bus: self._log("Not connected."); return
        self._log("Commissioning units & scaling…")
        self.sdo_w(0x60A8, 0x00, 0xFF410000, 4, oklog=True)     # Position 0.1°
        self.sdo_w(0x60A9, 0x00, 0x00410300, 4, oklog=True)     # Velocity deg/s
        self.sdo_w(0x60E8, 0x02, 100, 4, oklog=True)            # Gear motor
        self.sdo_w(0x60ED, 0x02,   1, 4, oklog=True)            # Gear output
        self._write_u16_or_u32(0x6080, 0x00, 1500)              # Max motor rpm
        self.sdo_w(0x607D, 0x01, (-2_000_000) & 0xFFFFFFFF, 4, oklog=True)
        self.sdo_w(0x607D, 0x02, ( 2_000_000) & 0xFFFFFFFF, 4, oklog=True)
        self._log("✓ Commissioned. PP in 0.1°, PV/V in deg/s, Gear=100:1, 6080h=1500 rpm.")

    def save_params(self):
        self._log("Saving parameters to NVM…")
        self.sdo_w(0x1010, 0x06, 0x65766173, 4, oklog=True)  # "save"
        self._log("✓ Save command issued. Power-cycle to confirm.")

    def read_key_od(self):
        keys = [
            (0x60A8,0x00,"60A8: pos unit"),
            (0x60A9,0x00,"60A9: vel unit"),
            (0x60E8,0x02,"60E8: gear motor"),
            (0x60ED,0x02,"60ED: gear output"),
            (0x607D,0x01,"607D: min pos"),
            (0x607D,0x02,"607D: max pos"),
            (0x607F,0x00,"607F: max prof vel"),
            (0x6080,0x00,"6080: max motor rpm"),
            (0x6065,0x00,"6065: follow err win"),
            (0x6066,0x00,"6066: follow err time"),
        ]
        for idx,sub,name in keys:
            val=self.sdo_r(idx,sub,oklog=True)
            if val is not None: self._log(f"{name} = 0x{val:X}")

    # --------------------- Enable / Power Control --------------------

    def vendor_power_on(self):
        if not self.sdo_w(0x2400,0x07,0x10,1,oklog=True):
            self.sdo_w(0x2400,0x07,0x10,4,oklog=True)

    def vendor_power_off(self):
        if not self.sdo_w(0x2400,0x07,0x00,1,oklog=True):
            self.sdo_w(0x2400,0x07,0x00,4,oklog=True)

    def cw(self, value):
        if self.sdo_w(0x6040,0x00,value,2,oklog=True):
            self._log(f"6040h <- 0x{value:04X}")

    def sw(self):
        v=self.sdo_r(0x6041,0x00,oklog=True)
        return (v & 0xFFFF) if v is not None else None

    def enable(self):
        self.nmt(0x01); time.sleep(0.05)
        self.cw(0x0080); time.sleep(0.02)         # clear fault
        self.vendor_power_on(); time.sleep(0.05)  # internal power path
        for cw, expect in [(0x0006,0x0021),(0x0007,0x0023),(0x000F,0x0027)]:
            self.cw(cw); time.sleep(0.06)
            v=self.sw()
            if v is not None and (v & 0x006F) != expect:
                self._log(f"ℹ️ After 0x{cw:04X} SW=0x{v:04X} [{sw_tags(v)}] (expected mask 0x{expect:04X})")
        v=self.sw()
        if v and (v & 0x006F)==0x0027 and (v & 0x0010):
            self._log("✅ Operation Enabled")
        else:
            self._log(f"❗ Not enabled: 6041=0x{(v or 0):04X} [{sw_tags(v or 0)}]")

    def quick_stop(self):
        self.cw(0x0002)

    def disable_voltage(self):
        self.cw(0x0000)

    def safe_stop_all(self):
        """Stop motion, disable torque, power off vendor stage (kills hum)."""
        try:
            self.cw(0x0002); time.sleep(0.03)  # quick stop
            self.cw(0x0000); time.sleep(0.03)  # disable voltage
            self.vendor_power_off()
            self._log("🛑 Safe stop & power off.")
        except Exception as e:
            self._log(f"⚠️ Safe stop error: {e}")

    # --------------------------- Utilities ---------------------------

    def _read_ratio_and_rpm_limit(self):
        gm = self.sdo_r(0x60E8, 0x02, oklog=False) or 1
        go = self.sdo_r(0x60ED, 0x02, oklog=False) or 1
        if go == 0: go = 1
        rpm_max = self.sdo_r(0x6080, 0x00, oklog=False) or 200
        return gm, go, rpm_max

    def _cap_axis_vel_by_6080(self, vel_deg_s):
        gm, go, rpm_max = self._read_ratio_and_rpm_limit()
        ratio = gm / go
        if ratio <= 0: return vel_deg_s
        max_axis_deg_s = (rpm_max * 6.0) / ratio  # motor_rpm = axis_deg_s * ratio / 6
        if vel_deg_s > max_axis_deg_s:
            self._log(f"⚠️ Requested {vel_deg_s:.1f}°/s exceeds 6080h limit ({rpm_max} rpm with gear {gm}:{go}). Capping to {max_axis_deg_s:.1f}°/s.")
            return max_axis_deg_s
        return vel_deg_s

    def _write_u16_or_u32(self, idx, sub, value):
        if not self.sdo_w(idx, sub, int(value), 2, oklog=True):
            self.sdo_w(idx, sub, int(value), 4, oklog=True)

    def _set_mode(self, mode):
        """Write 6060h with retries and wait for 6061 to confirm."""
        for _ in range(3):
            if not self.sdo_w(0x6060,0x00,mode,1,oklog=True):
                time.sleep(0.02); continue
            t0=time.time()
            while time.time()-t0 < 0.4:
                md=self.sdo_r(0x6061,0x00,oklog=False)
                if md == mode:
                    return True
                time.sleep(0.02)
        self._log(f"❌ Mode change to {mode} failed (6061 didn’t confirm).")
        return False

    def _ensure_enabled(self):
        v=self.sw()
        if not (v and (v & 0x006F)==0x0027 and (v & 0x0010)):
            self._log("Info: enabling …")
            self.enable()

    # --------------------------- Motion ------------------------------

    def do_pp(self):
        """Profile Position in axis degrees (0.1° units)."""
        self._ensure_enabled()
        if not self._set_mode(1): return
        tgt=int(round(self.pp_t.get()*10))       # 0.1°
        vel_req=float(self.pp_v.get())           # deg/s
        vel_cap=self._cap_axis_vel_by_6080(vel_req)
        vel=int(round(max(0.1, vel_cap)))
        acc=int(round(max(0.1, self.pp_a.get())))
        dec=int(round(max(0.1, self.pp_d.get())))
        self.sdo_w(0x6081,0x00,vel,4,oklog=True) # vel in deg/s
        self.sdo_w(0x6083,0x00,acc,4,oklog=True)
        self.sdo_w(0x6084,0x00,dec,4,oklog=True)
        p0=self.sdo_r(0x6064,0x00,oklog=False)
        self.sdo_w(0x607A,0x00,tgt,4,oklog=True)
        self.cw(0x001F); time.sleep(0.01); self.cw(0x003F)  # latch & start
        time.sleep(0.05)
        sv=self.sw()
        if sv and (sv & 0x1000): self._log("Set-point accepted (6041 bit12).")
        else: self._log("⚠️ No set-point ack → check limits/following error/scaling.")
        self._log(f"→ Move ABS {self.pp_t.get():.3f}° @ {vel:.1f}°/s (req {vel_req:.1f})")
        # Safety: stop if it jumps way too much too fast
        time.sleep(0.2)
        p1=self.sdo_r(0x6064,0x00,oklog=False)
        if p0 is not None and p1 is not None and abs(p1-p0) > 7200:  # >720°
            self._log("⚠️ Large position jump detected → Quick Stop for safety.")
            self.quick_stop()

    # Velocity Mode (6060=2) using 604C so 6042 expects deg/s directly
    def _config_vl_dimension(self):
        gm, go, _ = self._read_ratio_and_rpm_limit()
        denom = gm
        numer = 6 * max(1, go)
        self._log(f"VL dim (604C): denom={denom}, numer={numer}  → rpm = deg/s * {denom}/{numer}")
        self._write_u16_or_u32(0x604C,0x01,denom)   # denominator
        self._write_u16_or_u32(0x604C,0x02,numer)   # numerator

    def do_vl(self):
        """Velocity Mode jog with 6042 in deg/s (thanks to 604C)."""
        self._ensure_enabled()
        if not self._set_mode(2): return
        self._config_vl_dimension()
        vmax=int(round(self._cap_axis_vel_by_6080(abs(self.vl_v.get())))) or 1
        acc=int(round(max(1, self.vl_a.get())))
        dec=int(round(max(1, self.vl_d.get())))
        # Limits/ramps
        self._write_u16_or_u32(0x6046,0x01,0)       # min
        self._write_u16_or_u32(0x6046,0x02,vmax)    # max
        self._write_u16_or_u32(0x6048,0x01,acc); self._write_u16_or_u32(0x6048,0x02,1)
        self._write_u16_or_u32(0x6049,0x01,dec); self._write_u16_or_u32(0x6049,0x02,1)
        vel=int(round(self.vl_v.get()))
        self._write_u16_or_u32(0x6042,0x00,vel)
        self._log(f"→ V-mode Velocity {vel}°/s (VL via 6042)")

    def vl_stop(self):
        self._write_u16_or_u32(0x6042,0x00,0)
        self._log("V-mode STOP (6042=0).")

    def do_pv(self):
        """Profile Velocity (6060=3) in deg/s: 60FF target, 6083/6084 ramps."""
        self._ensure_enabled()
        if not self._set_mode(3): return
        acc=int(round(max(0.1,self.pv_a.get())))
        dec=int(round(max(0.1,self.pv_d.get())))
        vel_req=float(self.pv_v.get())
        vel_cap=self._cap_axis_vel_by_6080(vel_req)
        vel=int(round(vel_cap))
        self.sdo_w(0x6083,0x00,acc,4,oklog=True)
        self.sdo_w(0x6084,0x00,dec,4,oklog=True)
        self.sdo_w(0x60FF,0x00,vel,4,oklog=True)
        self._log(f"→ PV Velocity {vel:.2f}°/s (req {vel_req:.2f})")

    # --------------------------- Diagnostics -------------------------

    def _log_err(self):
        v=self.sdo_r(0x603F,0x00,oklog=True)
        if v is not None: self._log(f"ErrorReg 603F=0x{v:04X}")

    def _log_sw(self):
        v=self.sw()
        if v is not None: self._log(f"Statusword 6041=0x{v:04X} [{sw_tags(v)}]")

    # ------------------------- Polling task --------------------------

    def _poll_task(self):
        while self.poll:
            try:
                if self.bus:
                    pos=self.sdo_r(0x6064,0x00,oklog=False)  # actual pos (0.1°)
                    if pos is not None: self.lbl_pos.config(text=f"Pos: {pos/10.0:.3f}°")
                    sw =self.sdo_r(0x6041,0x00,oklog=False)
                    if sw  is not None: self.lbl_sw.config(text=f"Status: 0x{sw:04X} [{sw_tags(sw)}]")
                    er =self.sdo_r(0x603F,0x00,oklog=False)
                    if er  is not None: self.lbl_err.config(text=f"Error: 0x{er:04X}")
                    md =self.sdo_r(0x6061,0x00,oklog=False)
                    if md  is not None:
                        m={1:"PP",2:"V",3:"PV",6:"HM",7:"IP",8:"CSP"}.get(md,str(md))
                        self.lbl_md.config(text=f"Mode: {m}")
            except Exception:
                pass
            time.sleep(0.25)

    # -------------------------- UI SDO I/O ---------------------------

    def ui_rd(self):
        try:
            idx=int(self.idx.get(),16); sub=int(self.sub.get(),16)
        except ValueError:
            messagebox.showerror("Input", "Index/Subindex must be hex.")
            return
        v=self.sdo_r(idx,sub,oklog=True)
        if v is not None: self._log(f"READ {idx:04X}:{sub:02X} = 0x{v:X}")

    def ui_wr(self):
        try:
            idx=int(self.idx.get(),16); sub=int(self.sub.get(),16)
            sv=self.wv.get().strip()
            if sv.lower().startswith("0x"): val=int(sv,16)
            else: val=int(sv)
            sz=int(self.sz.get())
        except ValueError:
            messagebox.showerror("Input", "Bad write value/size.")
            return
        self.sdo_w(idx,sub,val,sz,oklog=True)

    def on_close(self):
        self.poll=False
        try: self.safe_stop_all()
        except Exception: pass
        try:
            if self.bus: self.bus.shutdown()
        except Exception: pass
        self.destroy()

if __name__ == "__main__":
    App().mainloop()
