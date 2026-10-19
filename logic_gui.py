#!/usr/bin/env python3
"""logic_gui.py — viewer/controller for logic_nano33iot.ino (5-channel logic analyzer).

Usage:
    python logic_gui.py                 # GUI, pick the Nano's COM port, Capture
    python logic_gui.py capture.json    # GUI, open a saved capture offline
    python logic_gui.py --selftest      # run the decoders on synthetic data, no hardware

Features: stacked digital traces (zoom/pan via the matplotlib toolbar, mouse wheel
zooms around the pointer), two cursors (left click = A, right click = B, delta in
the status line), per-channel pulse statistics, DShot300 frame decoder (throttle,
telemetry bit, CRC check, measured bit period / T0H / T1H; per-channel "inv" for
bidirectional DShot: idle high, low pulses, inverted CRC) and a UART 8N1 decoder
with KISS-telemetry packet check (CRC-8, temperature/U/I/mAh/eRPM). Captures
save/load as JSON.

Development aid only — no verification evidence (QuadSE PLAN-001).
Requires: pyserial, matplotlib, numpy (all present in the project's Python).
"""
import bisect
import json
import queue
import statistics
import sys
import threading
import time

N_CH = 5
DEFAULT_NAMES = ["M1 P22.1", "M2 P22.0", "M3 P22.2", "M4 P22.3", "ESC T"]
DEFAULT_DECODERS = ["DShot", "DShot", "DShot", "DShot", "UART"]
DSHOT_BIT_US = 1e6 / 300e3          # 3.333 us
GCR_BIT_US = DSHOT_BIT_US * 4 / 5    # 2.667 us: bidir reply runs at 5/4 of the command rate
POLE_PAIRS = 7                       # X2216 12N14P (QuadSE dispatch SYS1-012 §6)
UART_BAUD = 115200


# ----------------------------------------------------------------------------
# capture model
# ----------------------------------------------------------------------------
class Capture:
    """Run-length encoded multi-channel capture with per-channel transition lists."""

    def __init__(self, ns_per_sample, runs, bits, names=None, meta=None):
        self.ns = float(ns_per_sample)
        self.runs = [(int(v), int(n)) for v, n in runs]
        self.bits = list(bits)
        self.names = list(names) if names else [f"CH{i}" for i in range(len(bits))]
        self.meta = dict(meta or {})
        self.n = sum(n for _, n in self.runs)
        self.t_end_us = self.n * self.ns / 1000.0
        self._trans = None

    # --- (de)serialisation -------------------------------------------------
    @classmethod
    def from_lines(cls, header, lines):
        kv = {}
        for tok in header.split()[1:]:
            if "=" in tok:
                k, v = tok.split("=", 1)
                kv[k] = v
        runs = []
        for ln in lines:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            v, n = ln.split(",")
            runs.append((int(v), int(n)))
        bits = [int(b) for b in kv.get("bits", "2,4,5,0,3").split(",")]
        cap = cls(float(kv["ns"]), runs, bits, meta=kv)
        return cap

    def to_json(self):
        return {"version": 1, "ns_per_sample": self.ns, "runs": self.runs,
                "bits": self.bits, "names": self.names, "meta": self.meta,
                "saved": time.strftime("%Y-%m-%d %H:%M:%S")}

    @classmethod
    def from_json(cls, d):
        return cls(d["ns_per_sample"], d["runs"], d["bits"], d.get("names"), d.get("meta"))

    # --- transitions -------------------------------------------------------
    def transitions(self, ch):
        """[(t_us, level), ...] starting at t=0 with the initial level."""
        if self._trans is None:
            self._trans = [None] * len(self.bits)
        if self._trans[ch] is None:
            bit = self.bits[ch]
            out = []
            t = 0
            last = None
            for v, n in self.runs:
                lvl = (v >> bit) & 1
                if lvl != last:
                    out.append((t * self.ns / 1000.0, lvl))
                    last = lvl
                t += n
            self._trans[ch] = out
        return self._trans[ch]

    def level_at(self, ch, t_us):
        tr = self.transitions(ch)
        times = [t for t, _ in tr]
        i = bisect.bisect_right(times, t_us) - 1
        return tr[max(i, 0)][1]


# ----------------------------------------------------------------------------
# decoders
# ----------------------------------------------------------------------------
def high_pulses(trans, t_end):
    """[(t_rise, t_fall), ...] complete high pulses."""
    out = []
    rise = None
    for t, lvl in trans:
        if lvl == 1:
            rise = t
        elif rise is not None:
            if rise > 0.0:                  # a pulse already high at t=0 is truncated by the trigger: ignore
                out.append((rise, t))
            rise = None
    return out


def pulse_stats(trans, t_end):
    highs = [f - r for r, f in high_pulses(trans, t_end)]
    lows = []
    fall = None
    for t, lvl in trans:
        if lvl == 0:
            fall = t
        elif fall is not None:
            if fall > 0.0:                  # same for a low phase that started before the capture
                lows.append(t - fall)
            fall = None
    st = {"edges": max(len(trans) - 1, 0)}
    if highs:
        st["high_min"], st["high_max"] = min(highs), max(highs)
    if lows:
        st["low_min"], st["low_max"] = min(lows), max(lows)
    return st


def dshot_crc(value12):
    return (value12 ^ (value12 >> 4) ^ (value12 >> 8)) & 0x0F


def invert_trans(trans):
    """Same edges, levels swapped: a low pulse on the wire becomes a high pulse for the decoders."""
    return [(t, 1 - lvl) for t, lvl in trans]


def decode_dshot(trans, t_end, nominal_bit_us=DSHOT_BIT_US, inverted=False):
    """Group high pulses into frames (rise-to-rise gap < 2 bit periods), decode 16-pulse groups.

    inverted=True: bidirectional DShot — the line idles high, the bits are low pulses and
    the 4-bit CRC is sent inverted (AM32 dshot.c: checkCRC = ~checkCRC + 16)."""
    if inverted:
        trans = invert_trans(trans)
    pulses = high_pulses(trans, t_end)
    groups, cur = [], []
    for p in pulses:
        if cur and (p[0] - cur[-1][0]) > 2.0 * nominal_bit_us:
            groups.append(cur)
            cur = []
        cur.append(p)
    if cur:
        groups.append(cur)

    frames = []
    for g in groups:
        f = {"t0": g[0][0], "t1": g[-1][1], "n": len(g)}
        if len(g) != 16:
            f["error"] = f"{len(g)} pulses (16 expected)"
            frames.append(f)
            continue
        periods = [g[i + 1][0] - g[i][0] for i in range(15)]
        period = statistics.median(periods)
        highs = [fall - rise for rise, fall in g]
        bits = [1 if h > period / 2.0 else 0 for h in highs]
        raw = 0
        for b in bits:
            raw = (raw << 1) | b
        value12 = raw >> 4
        crc = raw & 0xF
        f.update({
            "period_us": period,
            "throttle": value12 >> 1,
            "telem": value12 & 1,
            "crc": crc,
            "crc_ok": (dshot_crc(value12) ^ (0xF if inverted else 0)) == crc,
            "raw": raw,
            "t0h_us": statistics.mean([h for h, b in zip(highs, bits) if b == 0]) if 0 in bits else None,
            "t1h_us": statistics.mean([h for h, b in zip(highs, bits) if b == 1]) if 1 in bits else None,
        })
        frames.append(f)
    return frames


def decode_uart(cap, ch, baud=UART_BAUD):
    """8N1, idle high, LSB first. Returns [(t_start_us, byte, framing_ok), ...]."""
    bit = 1e6 / baud
    trans = cap.transitions(ch)
    # A capture triggered on this line's falling edge starts a hair after the
    # start bit's edge, so the line is already low at t=0: keep that as a start
    # bit candidate (the trigger latency is a small fraction of a bit).
    falls = [t for t, lvl in trans if lvl == 0]
    out = []
    t_next = 0.0
    for t_start in falls:
        if t_start < t_next:
            continue
        if t_start + 10 * bit > cap.t_end_us:
            break
        byte = 0
        for k in range(8):
            if cap.level_at(ch, t_start + bit * (1.5 + k)):
                byte |= 1 << k
        stop_ok = cap.level_at(ch, t_start + bit * 9.5) == 1
        out.append((t_start, byte, stop_ok))
        # the next start bit may come right after the stop bit's midpoint: a transmitter
        # running a little fast (the ESC does) starts the next byte before 10 nominal bit times
        t_next = t_start + 9.5 * bit
    return out


def crc8_kiss(data):
    crc = 0
    for d in data:
        crc ^= d
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def group_packets(bytes_list, baud=UART_BAUD):
    """Split a byte stream into packets separated by idle gaps > 2 byte times."""
    gap = 2 * 10 * 1e6 / baud
    packets, cur = [], []
    for item in bytes_list:
        if cur and item[0] - cur[-1][0] > gap:
            packets.append(cur)
            cur = []
        cur.append(item)
    if cur:
        packets.append(cur)
    return packets


def describe_kiss(pkt):
    bad = sum(1 for x in pkt if not x[2])
    b = [x[1] for x in pkt if x[2]]          # bytes with a framing error (stop bit low) are not data
    note = f"  (+{bad} framing error)" if bad else ""
    if len(b) != 10:
        return f"{len(b)} bytes: " + " ".join(f"{x:02X}" for x in b) + note
    ok = crc8_kiss(b[:9]) == b[9]
    temp = b[0] - 256 if b[0] > 127 else b[0]
    u = ((b[1] << 8) | b[2]) / 100.0
    i = ((b[3] << 8) | b[4]) / 100.0
    mah = (b[5] << 8) | b[6]
    erpm = ((b[7] << 8) | b[8]) * 100
    return (f"KISS {' '.join(f'{x:02X}' for x in b)}  CRC {'ok' if ok else 'FAIL'}  "
            f"temp={temp} C U={u:.2f} V I={i:.2f} A mAh={mah} eRPM={erpm}  (U/I invalid on the G55M)" + note)


# ----------------------------------------------------------------------------
# synthetic capture for --selftest
# ----------------------------------------------------------------------------
# --- bidirectional DShot reply (AM32 Src/dshot.c make_dshot_package, AT32/F421 branch) ---
GCR_ENCODE = [0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
              0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111]
GCR_DECODE = {code: nib for nib, code in enumerate(GCR_ENCODE)}
EDT_TYPES = {0x2: "temp", 0x4: "voltage", 0x6: "current", 0x8: "debug1", 0xA: "debug2",
             0xC: "stress", 0xE: "status"}


def gcr_reply_encode(value12):
    """Wire levels (1 = released/high) of one reply, 21 slots — the reference encoder
    for the selftest, written as AM32 does it: 12 bit + inverted nibble-XOR CRC,
    4b/5b GCR, then NRZI (every GCR 1 toggles the line), after a leading low start slot."""
    csum = (~(value12 ^ (value12 >> 4) ^ (value12 >> 8))) & 0xF
    w16 = (value12 << 4) | csum
    g20 = 0
    for sh in (12, 8, 4, 0):
        g20 = (g20 << 5) | GCR_ENCODE[(w16 >> sh) & 0xF]
    lvl = [0]                                          # start slot: line pulled low
    for i in range(19, -1, -1):
        lvl.append(lvl[-1] ^ ((g20 >> i) & 1))
    return lvl


def decode_gcr_reply(trans, t_start, t_limit, bit_us=GCR_BIT_US):
    """Decode one reply starting at the falling edge t_start (wire levels, line idles high).

    NRZI: an edge at slot boundary k (1..20) is a GCR 1, no edge a GCR 0. The ESC's clock is
    its own (bench 2026-09-26: ~3.5 % faster than 5/4 x ours), so every interval between two
    edges is rounded on its own (GCR keeps it at 1..3 slots, the error cannot pile up), the
    bit time is then measured over the whole reply and the rounding repeated with it."""
    edges = [t for t, _ in trans if t_start < t < t_limit]
    r = {"t0": t_start}
    bit = bit_us
    for _ in range(2):
        inside, k, t_prev = [], 0, t_start
        for t in edges:
            k += max(1, round((t - t_prev) / bit))
            if k > 20:
                break
            inside.append((t, k))
            t_prev = t
        if inside and inside[-1][1] >= 10:
            bit = (inside[-1][0] - t_start) / inside[-1][1]
    r["bit_us"] = bit
    r["t1"] = t_start + 21 * bit
    g20 = 0
    hit = {k for _, k in inside}
    for k in range(1, 21):
        g20 = (g20 << 1) | (1 if k in hit else 0)
    w16 = 0
    for sh in (15, 10, 5, 0):
        code = (g20 >> sh) & 0x1F
        if code not in GCR_DECODE:
            r["error"] = f"invalid GCR code {code:05b}"
            r["gcr"] = g20
            return r
        w16 = (w16 << 4) | GCR_DECODE[code]
    value12 = w16 >> 4
    r.update({"gcr": g20, "raw": w16, "value": value12,
              "crc_ok": ((value12 ^ (value12 >> 4) ^ (value12 >> 8) ^ w16) & 0xF) == 0xF})
    if (value12 & 0x100) == 0 and (value12 >> 8) != 0:
        r["edt"] = EDT_TYPES.get(value12 >> 8, f"type {value12 >> 8:X}")
        r["edt_val"] = value12 & 0xFF
    else:
        period = (value12 & 0x1FF) << (value12 >> 9)
        r["period_us"] = period
        r["erpm"] = 0 if value12 == 0xFFF or period == 0 else round(60e6 / period)   # 0xFFF = AM32 "not running"
        r["rpm"] = round(r["erpm"] / POLE_PAIRS)
    return r


def decode_bidir(trans, t_end, frames):
    """For every CRC-ok command frame: the first falling edge 5..150 us after its end is the reply."""
    out = []
    for f in frames:
        if not f.get("crc_ok"):
            continue
        falls = [t for t, lvl in trans if lvl == 0 and f["t1"] + 5.0 < t < f["t1"] + 150.0]
        if not falls:
            out.append({"t0": f["t1"], "error": "no reply", "cmd_t1": f["t1"]})
            continue
        r = decode_gcr_reply(trans, falls[0], min(falls[0] + 21 * GCR_BIT_US * 1.15, t_end))
        r["turnaround_us"] = falls[0] - f["t1"]
        out.append(r)
    return out


def describe_reply(r):
    if "error" in r:
        return r["error"]
    head = f"raw=0x{r['raw']:04X} CRC {'ok' if r['crc_ok'] else 'FAIL'}"
    if "edt" in r:
        body = f"EDT {r['edt']}={r['edt_val']}"
    elif r["erpm"] == 0:
        body = "stopped (eRPM 0)"
    else:
        body = f"period={r['period_us']} us eRPM={r['erpm']} rpm={r['rpm']}"
    return f"{head}  {body}  turnaround={r['turnaround_us']:.1f} us bit={r['bit_us']:.3f} us"


def synth_capture(ns=100.0):
    """Build a capture: DShot frames on CH0..CH3, one KISS packet on CH4."""
    bits = [2, 4, 5, 0, 3]
    dt = ns / 1000.0
    total_us = 1500.0
    n = int(total_us / dt)
    samples = [0] * n

    def set_bit(ch, t0, t1, level):
        i0, i1 = int(t0 / dt), int(t1 / dt)
        m = 1 << bits[ch]
        for i in range(max(i0, 0), min(i1, n)):
            samples[i] = (samples[i] | m) if level else (samples[i] & ~m)

    def dshot_frame(ch, t0, throttle, telem):
        v = ((throttle & 0x7FF) << 1) | (telem & 1)
        raw = (v << 4) | dshot_crc(v)
        for k in range(16):
            b = (raw >> (15 - k)) & 1
            ts = t0 + k * DSHOT_BIT_US
            set_bit(ch, ts, ts + (2.5 if b else 1.25), 1)

    dshot_frame(0, 10.0, 0, 0)
    dshot_frame(1, 12.0, 6, 0)
    dshot_frame(2, 14.0, 1046, 1)
    dshot_frame(3, 16.0, 2047, 1)
    dshot_frame(0, 1010.0, 48, 1)

    # UART on CH4: idle high, KISS packet
    set_bit(4, 0, total_us, 1)
    pkt = [0x16, 0x05, 0xC7, 0x1C, 0x1F, 0x00, 0x4F, 0x00, 0x00]
    pkt.append(crc8_kiss(pkt))
    bit = 1e6 / UART_BAUD
    t = 200.0
    for byte in pkt:
        set_bit(4, t, t + bit, 0)                      # start bit
        for k in range(8):
            lvl = (byte >> k) & 1
            set_bit(4, t + bit * (1 + k), t + bit * (2 + k), lvl)
        set_bit(4, t + 9 * bit, t + 10 * bit, 1)       # stop bit
        t += 10 * bit + 3.0

    runs = []
    v, cnt = samples[0], 0
    for s in samples:
        if s == v:
            cnt += 1
        else:
            runs.append((v, cnt))
            v, cnt = s, 1
    runs.append((v, cnt))
    return Capture(ns, runs, bits, DEFAULT_NAMES, {"synthetic": "1"})


def selftest():
    cap = synth_capture()
    ok = True
    for ch in range(4):
        frames = decode_dshot(cap.transitions(ch), cap.t_end_us)
        print(f"CH{ch} {cap.names[ch]}: {len(frames)} frame(s)")
        for f in frames:
            print("   ", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in f.items()})
            ok &= f.get("crc_ok", False)
    ub = decode_uart(cap, 4)
    for pkt in group_packets(ub):
        d = describe_kiss(pkt)
        print("CH4:", d)
        ok &= "CRC ok" in d
    expect = [(0, 0), (6, 0), (1046, 1), (2047, 1)]
    got = [(decode_dshot(cap.transitions(c), cap.t_end_us)[0]["throttle"],
            decode_dshot(cap.transitions(c), cap.t_end_us)[0]["telem"]) for c in range(4)]
    ok &= got == expect

    # bidirectional (inverted) frame: idle high, low pulses, inverted CRC
    v = (1046 << 1) | 1
    raw = (v << 4) | (dshot_crc(v) ^ 0xF)
    trans = [(0.0, 1)]
    for k in range(16):
        ts = 10.0 + k * DSHOT_BIT_US
        trans += [(ts, 0), (ts + (2.5 if (raw >> (15 - k)) & 1 else 1.25), 1)]
    fi = decode_dshot(trans, 100.0, inverted=True)
    fn = decode_dshot(trans, 100.0)
    print("inverted:", {k: (round(x, 3) if isinstance(x, float) else x) for k, x in fi[0].items()})
    ok &= len(fi) == 1 and fi[0].get("crc_ok") and fi[0]["throttle"] == 1046 and fi[0]["telem"] == 1
    ok &= not (len(fn) == 1 and fn[0].get("crc_ok", False))      # must not pass as a normal frame

    # bidir reply after that frame: stopped, a real period, an EDT temperature frame;
    # ESC clock 5 % fast / 5 % slow (bench 2026-09-26 measured ~3.5 % fast)
    cases = [(v, w, c) for v, w in ((0xFFF, 0), ((1 << 9) | 300, round(60e6 / 600)), ((0x2 << 8) | 35, None))
             for c in (0.95, 1.05)]
    for value12, want, clk in cases:
        t, tr2 = fi[0]["t1"] + 27.0, list(trans)
        lv = gcr_reply_encode(value12)
        prev = 1
        for k, l in enumerate(lv):
            if l != prev:
                tr2.append((t + k * GCR_BIT_US * clk, l))
                prev = l
        if prev == 0:
            tr2.append((t + 21 * GCR_BIT_US * clk, 1))        # ESC releases the line
        rr = decode_bidir(tr2, 400.0, decode_dshot(tr2, 400.0, inverted=True))
        print(f"reply 0x{value12:03X} clk x{clk}:", describe_reply(rr[0]))
        r = rr[0]
        ok &= r.get("crc_ok", False) and r.get("value") == value12
        ok &= (r.get("erpm") == want) if want is not None else (r.get("edt") == "temp" and r.get("edt_val") == 35)
    print("SELFTEST", "PASS" if ok else "FAIL")
    return 0 if ok else 1


# ----------------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------------
def run_gui(path=None):
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
    from matplotlib.figure import Figure
    import serial
    import serial.tools.list_ports

    class App:
        def __init__(self, root):
            self.root = root
            root.title("Nano 33 IoT logic analyzer — DShot / KISS")
            self.ser = None
            self.rx_q = queue.Queue()
            self.rx_thread = None
            self.cap = None
            self.cursor = [None, None]
            self.collect = None
            self.pending_capture = False

            top = ttk.Frame(root, padding=4)
            top.pack(fill="x")
            ttk.Label(top, text="Port").pack(side="left")
            self.port_cb = ttk.Combobox(top, width=10, values=self.ports())
            self.port_cb.pack(side="left", padx=2)
            nano = [p.device for p in serial.tools.list_ports.comports() if "2341:8057" in (p.hwid or "")]
            if nano:
                self.port_cb.set(nano[0])            # prefer the Nano 33 IoT by USB id
            elif self.port_cb["values"]:
                self.port_cb.current(0)
            ttk.Button(top, text="⟳", width=3, command=lambda: self.port_cb.config(values=self.ports())).pack(side="left")
            self.conn_btn = ttk.Button(top, text="Connect", command=self.toggle_connect)
            self.conn_btn.pack(side="left", padx=4)

            ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=6)
            ttk.Label(top, text="Trigger").pack(side="left")
            self.trig_ch = ttk.Combobox(top, width=4, values=[f"CH{i}" for i in range(N_CH)], state="readonly")
            self.trig_ch.current(0)
            self.trig_ch.pack(side="left")
            self.trig_edge = ttk.Combobox(top, width=14, values=["frame start", "frame start inv", "rising", "falling", "any", "none"], state="readonly")
            self.trig_edge.current(0)
            self.trig_edge.pack(side="left", padx=2)
            ttk.Label(top, text="Samples").pack(side="left", padx=(6, 0))
            self.len_var = tk.IntVar(value=20000)
            ttk.Spinbox(top, from_=16, to=20000, increment=1000, width=7, textvariable=self.len_var).pack(side="left")
            ttk.Label(top, text="Divider").pack(side="left", padx=(6, 0))
            self.div_var = tk.IntVar(value=0)
            ttk.Spinbox(top, from_=0, to=1000, width=5, textvariable=self.div_var).pack(side="left")
            ttk.Label(top, text="Timeout ms").pack(side="left", padx=(6, 0))
            self.to_var = tk.IntVar(value=2000)
            ttk.Spinbox(top, from_=10, to=60000, increment=500, width=6, textvariable=self.to_var).pack(side="left")

            ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=6)
            self.cap_btn = ttk.Button(top, text="Capture", command=self.capture, state="disabled")
            self.cap_btn.pack(side="left")
            self.st_btn = ttk.Button(top, text="Self-test PWM D2", command=lambda: self.send("G"), state="disabled")
            self.st_btn.pack(side="left", padx=4)
            self.scan_btn = ttk.Button(top, text="Pin scan", command=lambda: self.send("s"), state="disabled")
            self.scan_btn.pack(side="left")
            ttk.Separator(top, orient="vertical").pack(side="left", fill="y", padx=6)
            self.snap_var = tk.BooleanVar(value=True)
            ttk.Checkbutton(top, text="Snap to edges", variable=self.snap_var).pack(side="left")
            ttk.Button(top, text="Clear cursors", command=self.clear_cursors).pack(side="left", padx=4)
            ttk.Button(top, text="Save", command=self.save).pack(side="left", padx=(12, 2))
            ttk.Button(top, text="Load", command=self.load).pack(side="left")

            chf = ttk.Frame(root, padding=(4, 0))
            chf.pack(fill="x")
            self.name_vars, self.dec_cbs, self.inv_vars = [], [], []
            for i in range(N_CH):
                ttk.Label(chf, text=f"CH{i}").pack(side="left", padx=(8, 2))
                nv = tk.StringVar(value=DEFAULT_NAMES[i])
                ttk.Entry(chf, width=10, textvariable=nv).pack(side="left")
                cb = ttk.Combobox(chf, width=6, values=["none", "DShot", "UART"], state="readonly")
                cb.set(DEFAULT_DECODERS[i])
                cb.pack(side="left")
                cb.bind("<<ComboboxSelected>>", lambda e: self.render())
                iv = tk.BooleanVar(value=False)
                ttk.Checkbutton(chf, text="inv", variable=iv, command=self.render).pack(side="left", padx=(2, 0))
                self.inv_vars.append(iv)
                nv.trace_add("write", lambda *a: self.render())
                self.name_vars.append(nv)
                self.dec_cbs.append(cb)

            body = ttk.Panedwindow(root, orient="vertical")
            body.pack(fill="both", expand=True)

            plotf = ttk.Frame(body)
            body.add(plotf, weight=4)
            self.fig = Figure(figsize=(11, 5), dpi=100)
            self.ax = self.fig.add_subplot(111)
            self.canvas = FigureCanvasTkAgg(self.fig, master=plotf)
            self.toolbar = NavigationToolbar2Tk(self.canvas, plotf)
            self.toolbar.update()
            self.canvas.get_tk_widget().pack(fill="both", expand=True)
            self.canvas.mpl_connect("button_press_event", self.on_click)
            self.canvas.mpl_connect("button_release_event", self.on_release)
            self.canvas.mpl_connect("motion_notify_event", self.on_move)
            self.canvas.mpl_connect("scroll_event", self.on_scroll)
            self.cur_lines = [None, None]      # cursor artists (axvline)
            self.cur_text = None               # on-plot readout
            self.drag = None                   # index of the cursor being dragged

            self.text = tk.Text(body, height=12, font=("Consolas", 9))
            body.add(self.text, weight=1)

            self.status = tk.StringVar(value="not connected")
            ttk.Label(root, textvariable=self.status, anchor="w", padding=(4, 2)).pack(fill="x")

            self.root.after(50, self.poll)
            if path:
                self.load(path)

        # --- serial -------------------------------------------------------
        @staticmethod
        def ports():
            return [p.device for p in serial.tools.list_ports.comports()]

        def toggle_connect(self):
            if self.ser:
                self.ser.close()
                self.ser = None
                self.conn_btn.config(text="Connect")
                self.cap_btn.config(state="disabled")
                self.st_btn.config(state="disabled")
                self.scan_btn.config(state="disabled")
                self.status.set("disconnected")
                return
            try:
                self.ser = serial.Serial(self.port_cb.get(), 115200, timeout=0.1)
            except Exception as e:
                messagebox.showerror("Serial", str(e))
                return
            self.rx_thread = threading.Thread(target=self.reader, daemon=True)
            self.rx_thread.start()
            self.conn_btn.config(text="Disconnect")
            self.cap_btn.config(state="normal")
            self.st_btn.config(state="normal")
            self.scan_btn.config(state="normal")
            self.status.set(f"connected {self.ser.port}")
            self.send("i")

        def reader(self):
            ser = self.ser
            buf = b""
            while ser and ser.is_open:
                try:
                    chunk = ser.read(4096)
                except Exception:
                    break
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.rx_q.put(line.decode("ascii", "replace").strip())

        def send(self, cmd):
            if self.ser:
                self.ser.write((cmd + "\n").encode("ascii"))

        def capture(self):
            edge = {"frame start": "F", "frame start inv": "I", "rising": "r", "falling": "f", "any": "a", "none": "n"}[self.trig_edge.get()]
            self.send(f"t{self.trig_ch.current()}{edge}")
            self.send(f"l{self.len_var.get()}")
            self.send(f"d{self.div_var.get()}")
            self.send(f"w{self.to_var.get()}")
            self.send("a")
            self.pending_capture = True
            self.status.set("armed, waiting for trigger … (timeout %d ms)" % self.to_var.get())
            self.cap_btn.config(state="disabled")

        def poll(self):
            try:
                while True:
                    line = self.rx_q.get_nowait()
                    self.handle_line(line)
            except queue.Empty:
                pass
            self.root.after(50, self.poll)

        def handle_line(self, line):
            if line.startswith("#CAP"):
                self.collect = (line, [])
            elif line.startswith("#END") and self.collect:
                header, lines = self.collect
                self.collect = None
                self.pending_capture = False
                self.cap_btn.config(state="normal")
                try:
                    self.cap = Capture.from_lines(header, lines)
                    self.cap.names = [v.get() for v in self.name_vars]
                    self.cursor = [None, None]
                    self.render()
                except Exception as e:
                    import traceback
                    self.text.delete("1.0", "end")
                    self.text.insert("end", traceback.format_exc())
                    self.status.set(f"capture error: {e}")
                    return
                trig = "trigger ok" if self.cap.meta.get("trigok") == "1" else "TRIGGER TIMEOUT, showing whatever was on the lines"
                self.status.set(f"capture received: {self.cap.n} samples, {self.cap.ns:.1f} ns/sample, {trig}")
            elif self.collect is not None:
                self.collect[1].append(line)
            elif line.startswith("#INFO") or line.startswith("#HELP") or line.startswith("#SCAN"):
                if not self.pending_capture:
                    self.status.set(line)

        # --- plot ---------------------------------------------------------
        def render(self):
            cap = self.cap
            self.ax.clear()
            self.text.delete("1.0", "end")
            if cap is None:
                self.canvas.draw_idle()
                return
            cap.names = [v.get() for v in self.name_vars]
            pitch = self.PITCH
            report = [f"ns/sample = {cap.ns:.1f}   samples = {cap.n}   window = {cap.t_end_us:.1f} us   "
                      f"trigger ok = {cap.meta.get('trigok', '?')}"]
            for ch in range(N_CH):
                tr = cap.transitions(ch)
                base = (N_CH - 1 - ch) * pitch
                xs = [t for t, _ in tr] + [cap.t_end_us]
                ys = [lvl for _, lvl in tr] + [tr[-1][1]]
                self.ax.step(xs, [base + y for y in ys], where="post", lw=1.0)
                self.ax.text(-0.01, base + 0.5, f"CH{ch} {cap.names[ch]}", transform=self.ax.get_yaxis_transform(),
                             ha="right", va="center", fontsize=8)
                st = pulse_stats(tr, cap.t_end_us)
                line = f"CH{ch} {cap.names[ch]:<10} edges={st['edges']}"
                if "high_min" in st:
                    line += f"  high {st['high_min']:.3f}..{st['high_max']:.3f} us"
                if "low_min" in st:
                    line += f"  low {st['low_min']:.3f}..{st['low_max']:.3f} us"
                report.append(line)
                dec = self.dec_cbs[ch].get()
                if dec == "DShot":
                    inv = self.inv_vars[ch].get()
                    frames = decode_dshot(tr, cap.t_end_us, inverted=inv)
                    replies = decode_bidir(tr, cap.t_end_us, frames) if inv else []
                    spans = [(r["t0"], r["t1"]) for r in replies if "t1" in r]
                    frames = [f for f in frames if "error" not in f or
                              not any(a - 1.0 <= f["t0"] <= b for a, b in spans)]
                    reply_lines = [f"    reply @ {r['t0']:.2f} us: {describe_reply(r)}" for r in replies]
                    for r in replies:
                        if "t1" not in r:
                            continue
                        rc = "purple" if r.get("crc_ok") else "red"
                        lab = (r["error"] if "error" in r
                               else describe_reply(r).split("  turnaround")[0].split("  ", 1)[1])
                        self.ax.axvspan(r["t0"], r["t1"], ymin=base / (N_CH * pitch),
                                        ymax=(base + 1.2) / (N_CH * pitch), color=rc, alpha=0.12)
                        self.ax.text(r["t0"], base + 1.12, lab, fontsize=7, color=rc, va="bottom")
                    for f in frames:
                        if "error" in f:
                            label = f"? {f['error']}"
                            color = "orange"
                            report.append(f"    DShot{' inv' if inv else ''} @ {f['t0']:.2f} us: {f['error']}")
                        else:
                            label = f"thr={f['throttle']} tlm={f['telem']} CRC {'ok' if f['crc_ok'] else 'FAIL'}"
                            color = "green" if f["crc_ok"] else "red"
                            t0h = f"{f['t0h_us']:.3f}" if f["t0h_us"] is not None else "-"
                            t1h = f"{f['t1h_us']:.3f}" if f["t1h_us"] is not None else "-"
                            report.append(f"    DShot{' inv' if inv else ''} @ {f['t0']:.2f} us: raw=0x{f['raw']:04X} throttle={f['throttle']} "
                                          f"telem={f['telem']} crc={f['crc']:X} {'ok' if f['crc_ok'] else 'FAIL'}  "
                                          f"bit={f['period_us']:.3f} us T0H={t0h} T1H={t1h} us")
                        self.ax.axvspan(f["t0"], f["t1"], ymin=(base) / (N_CH * pitch), ymax=(base + 1.2) / (N_CH * pitch),
                                        color=color, alpha=0.12)
                        self.ax.text(f["t0"], base + 1.12, label, fontsize=7, color=color, va="bottom")
                    report.extend(reply_lines)
                elif dec == "UART":
                    ub = decode_uart(cap, ch)
                    for t, b, ok in ub:
                        self.ax.text(t, base + 1.12, f"{b:02X}", fontsize=7, color="blue" if ok else "red", va="bottom")
                    for pkt in group_packets(ub):
                        report.append(f"    UART @ {pkt[0][0]:.2f} us: {describe_kiss(pkt)}")
            self.ax.set_ylim(-0.3, N_CH * pitch)
            self.ax.set_xlim(0, cap.t_end_us)
            self.ax.set_yticks([])
            self.ax.set_xlabel("t [us]")
            self.ax.grid(axis="x", alpha=0.3)
            self.cur_lines = [None, None]
            self.cur_text = None
            self.draw_cursors()
            self.text.insert("end", "\n".join(report))
            self.canvas.draw_idle()
            self.update_status()

        # --- cursors --------------------------------------------------------
        PITCH = 1.6

        def draw_cursors(self):
            """Create/update the two cursor lines and the on-plot readout without re-rendering."""
            for i, col in enumerate(("magenta", "cyan")):
                c = self.cursor[i]
                if c is None:
                    if self.cur_lines[i] is not None:
                        self.cur_lines[i].remove()
                        self.cur_lines[i] = None
                    continue
                if self.cur_lines[i] is None:
                    self.cur_lines[i] = self.ax.axvline(c, color=col, lw=1.2)
                else:
                    self.cur_lines[i].set_xdata([c, c])
            a, b = self.cursor
            lines = []
            if a is not None:
                lines.append(f"A = {a:.3f} us")
            if b is not None:
                lines.append(f"B = {b:.3f} us")
            if a is not None and b is not None:
                d = abs(b - a)
                lines.append(f"B-A = {d:.3f} us" + (f"  ({1e3 / d:.2f} kHz)" if d > 0 else ""))
            txt = "\n".join(lines)
            if self.cur_text is None:
                self.cur_text = self.ax.text(0.99, 0.98, txt, transform=self.ax.transAxes, ha="right", va="top",
                                             fontsize=9, family="monospace",
                                             bbox=dict(boxstyle="round", fc="white", ec="gray", alpha=0.85))
            else:
                self.cur_text.set_text(txt)
            self.cur_text.set_visible(bool(txt))
            self.canvas.draw_idle()

        def clear_cursors(self):
            self.cursor = [None, None]
            self.draw_cursors()
            self.update_status()

        def channel_at(self, y):
            ch = N_CH - 1 - int(y // self.PITCH)
            return min(max(ch, 0), N_CH - 1)

        def snap(self, x, y):
            """Snap x to the nearest transition of the channel under the pointer (within 2 % of the view)."""
            if not self.snap_var.get() or self.cap is None or y is None:
                return x
            ch = self.channel_at(y)
            times = [t for t, _ in self.cap.transitions(ch)]
            if not times:
                return x
            i = bisect.bisect_left(times, x)
            cands = [times[k] for k in (i - 1, i) if 0 <= k < len(times)]
            best = min(cands, key=lambda t: abs(t - x))
            x0, x1 = self.ax.get_xlim()
            return best if abs(best - x) <= 0.02 * (x1 - x0) else x

        def near_cursor(self, x):
            x0, x1 = self.ax.get_xlim()
            tol = 0.01 * (x1 - x0)
            hits = [(abs(c - x), i) for i, c in enumerate(self.cursor) if c is not None and abs(c - x) <= tol]
            return min(hits)[1] if hits else None

        def update_status(self, t=None):
            parts = []
            if self.cap:
                parts.append(f"{self.cap.ns:.1f} ns/sample")
            if t is not None:
                parts.append(f"t = {t:.3f} us")
            a, b = self.cursor
            if a is not None:
                parts.append(f"A = {a:.3f} us")
            if b is not None:
                parts.append(f"B = {b:.3f} us")
            if a is not None and b is not None:
                d = abs(b - a)
                parts.append(f"|B-A| = {d:.3f} us ({1e6 / d / 1e3:.2f} kHz)" if d > 0 else "|B-A| = 0")
            self.status.set("   ".join(parts))

        def on_click(self, ev):
            if ev.inaxes != self.ax or ev.xdata is None or self.cap is None:
                return
            if self.toolbar.mode:               # pan/zoom tool active: leave the mouse to it
                return
            if ev.button == 1:
                hit = self.near_cursor(ev.xdata)
                self.drag = hit if hit is not None else 0
            elif ev.button == 3:
                self.drag = 1
            else:
                return
            self.cursor[self.drag] = self.snap(ev.xdata, ev.ydata)
            self.draw_cursors()
            self.update_status(ev.xdata)

        def on_release(self, ev):
            self.drag = None

        def on_move(self, ev):
            if ev.inaxes != self.ax or ev.xdata is None or self.cap is None:
                return
            if self.drag is not None and not self.toolbar.mode:
                self.cursor[self.drag] = self.snap(ev.xdata, ev.ydata)
                self.draw_cursors()
            self.update_status(ev.xdata)

        def on_scroll(self, ev):
            if ev.inaxes != self.ax or ev.xdata is None:
                return
            x0, x1 = self.ax.get_xlim()
            f = 0.7 if ev.button == "up" else 1 / 0.7
            self.ax.set_xlim(ev.xdata - (ev.xdata - x0) * f, ev.xdata + (x1 - ev.xdata) * f)
            self.canvas.draw_idle()

        # --- files --------------------------------------------------------
        def save(self):
            if not self.cap:
                return
            p = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("capture", "*.json")])
            if p:
                self.cap.meta["inv"] = "".join("1" if v.get() else "0" for v in self.inv_vars)
                with open(p, "w", encoding="utf-8") as fh:
                    json.dump(self.cap.to_json(), fh)
                self.status.set(f"saved {p}")

        def load(self, p=None):
            p = p or filedialog.askopenfilename(filetypes=[("capture", "*.json")])
            if not p:
                return
            with open(p, encoding="utf-8") as fh:
                self.cap = Capture.from_json(json.load(fh))
            for v, nm in zip(self.name_vars, self.cap.names):
                v.set(nm)
            for v, c in zip(self.inv_vars, self.cap.meta.get("inv", "")):
                v.set(c == "1")
            self.cursor = [None, None]
            self.render()

    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    run_gui(sys.argv[1] if len(sys.argv) > 1 else None)
