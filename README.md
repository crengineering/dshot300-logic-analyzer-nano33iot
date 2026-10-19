# NanoLogicAnalyzer — 5-channel logic analyzer on an Arduino Nano 33 IoT

Generic tool, its own project since 2026-09-25 (moved out of AurixTricore/tools).
Built for the quadrocopter's DShot bring-up, hence the DShot/KISS decoders and
the wiring table below; the sketch and the viewer do not depend on that project.
Used from `AurixTricore/tools/killchain_test.py` as a sibling folder.

Development aid, no verification evidence (QuadSE PLAN-001). Looks at the AURIX
DShot300 lines and the ESC telemetry wire with ~100 ns resolution, decodes the
frames, and shows whether the ESC answered. Two files:

| File | What |
|---|---|
| `logic_nano33iot.ino` | Nano 33 IoT sketch: samples 5 pins at ~10 MSa/s into RAM, streams run-length data over USB |
| `logic_gui.py` | Python/Tk GUI: traces, cursors, DShot + UART/KISS decoders, save/load |

## Wiring

All five inputs are 3.3 V logic, the Nano is **not 5 V tolerant**. The DShot
lines are 3.3 V only because of the pull-up to +3V3 with the AURIX pad in
open-drain — never probe a push-pull P22 pin or the ESC's V pad.

| Nano | Signal | TriBoard | ESC pad |
|---|---|---|---|
| D8  (CH0) | M1 DShot | P22.1 = X702·30 | 1 |
| D9  (CH1) | M2 DShot | P22.0 = X702·32 | 2 |
| D10 (CH2) | M3 DShot | P22.2 = X702·36 | 3 |
| D11 (CH3) | M4 DShot | P22.3 = X702·34 | 4 |
| D12 (CH4) | KISS telemetry | P23.3 = X702·24 | T |
| GND | common ground | X702·1–4 | G |

D13 is not used: it carries the on-board LED, which would load a line.

## First use: self-test

1. Flash the sketch (Arduino IDE, board "Arduino Nano 33 IoT").
2. Jumper **D2 → D8**. In the GUI: Connect, then **Self-test PWM D2**, then
   **Capture** with trigger CH0 rising.
3. Expected: CH0 shows a square wave, high ≈ 1.667 µs, period ≈ 3.333 µs
   (cursor A/B delta, or the CH0 line in the report). This is the same TCC0
   that drove the ESC on 2026-09-16, so the analyzer is calibrated against a
   known-good DShot300 clock. Press Self-test again to release D2.

## Using it on the AURIX

- Trigger on the motor channel you are sending on with **frame start** (a rising
  edge after at least ~100 µs of low), so the capture begins at bit 15 of a
  frame. The window at full rate is ~1.5 ms, so a 1 kHz stream gives one or two
  complete frames per capture.
- The report lists every decoded frame: raw word, throttle, telemetry bit, CRC
  check, measured bit period, mean T0H and mean T1H. A frame with the wrong
  pulse count or a failed CRC is marked orange/red on the trace.
- **Bidirectional DShot:** tick **inv** next to the channel's decoder. The
  line then idles high and the bits are low pulses; the decoder swaps the
  levels and checks the inverted CRC (AM32 `checkCRC = ~checkCRC + 16`).
  Trigger with **frame start inv** (a falling edge after at least ~100 µs of
  high). The inv ticks are saved with the capture.
- **Bidir reply decoder** (inv channels only): the first falling edge 5–150 µs
  after each CRC-ok frame starts the ESC's reply. Decoded as AM32 sends it:
  NRZI (an edge on a slot boundary = GCR 1), 4b/5b GCR, 12 bit + inverted
  nibble-XOR CRC. The bit time starts at 2.667 µs (5/4 × DShot300) and is
  re-estimated from the edges. Shown in purple (red = CRC fail): eRPM,
  mechanical rpm (7 pole pairs), "stopped" for AM32's 0xFFF, or the EDT type
  and value. The report also lists the turnaround and the measured bit time.
- CH4 with the UART decoder shows the KISS reply after a frame with the
  telemetry bit set: 10 bytes, CRC-8 check, decoded fields. Voltage and
  current are invalid on the G55M by design (`docs/ESC_BRINGUP_2026-09-16.md`).
- Divider > 0 slows sampling for a longer window (e.g. to see the ESC's
  ~30 µs turnaround plus the whole 870 µs KISS packet in one capture). The
  exact ns/sample is always measured by TC4 and shown in the status line.
- **Pin scan** reads D2..D12 a few thousand times and reports the high count per
  pin: a PWM line shows a stable intermediate count, floating pins random ones.
  Use it when a trace stays flat to see where the wire really is.
- Left click = cursor A, right click = cursor B; drag a cursor by grabbing it;
  "Snap to edges" pulls a cursor onto the nearest transition of the channel
  under the pointer. The A/B/delta readout sits in the plot corner. Mouse
  wheel zooms, toolbar pans. Save stores the capture as JSON; `python logic_gui.py file.json`
  reopens it offline.

## Limits

- No pre-trigger: the capture starts a few samples after the trigger edge.
- 20 000 samples of RAM. At full rate that is ~2 ms; use the divider for more.
- Interrupts are off during arm + capture, so the USB link stalls until the
  trigger fires or the timeout (default 2 s) expires.
- The decoders are timing-based; a capture with the divider set so coarse that
  a 1.25 µs pulse spans fewer than ~4 samples will misclassify bits.

`python logic_gui.py --selftest` runs the decoders on synthetic frames.
