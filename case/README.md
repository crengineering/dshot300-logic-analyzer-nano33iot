# Case for the logic analyzer (Nano 33 IoT)

Two-part printed case for the analyzer in `../README.md`: a base with a board
pocket and a snap-on lid with two slots over the header rows, "LOGIC /
ANALYZER" engraved 0.6 mm deep and exported as a separate body for a second
filament (AMS colour change). The USB end of the case is open: the Nano's
micro-USB face sits flush with the case end, so the cable plugs in from
outside without entering the case.

| File | Content |
|---|---|
| `fusion_case.py` | generates the whole model in Fusion 360 (the script is the source; parameters in the `P` dict at the top, also created as user parameters in the document) |
| `la_case.step` | Base + Lid + letters, mm |
| **`la_case_bambu.3mf`** | **Bambu Studio project, ready to slice:** object "Lid" = part "Lid" (filament 1) + part "Label" (filament 2), lid already lying top-down on the bed; object "Base" next to it. File > Open Project. Presets: X2D 0.4 nozzle, 0.20 mm Standard, PLA |
| `make_bambu_3mf.py` | builds `la_case_bambu.3mf` from the three STLs (`python make_bambu_3mf.py [template.3mf]`) |
| `la_case.3mf` | raw Fusion export, every body separate (13 letters individually), fallback only |
| `la_case_Base.stl`, `la_case_Lid.stl`, `la_case_Label.stl` | single bodies (Label = all 13 letters in one file). Fallback without the project file: import Lid + Label together, "load as one object with multiple parts", give the Label part the second filament |

## Board dimensions used

- PCB **45.5 × 18 × 1.6 mm**, measured on the board. The mechanical drawing in
  the official datasheet (ABX00027, section 6) says 43.16 × 17.77 mm; a base
  printed to that size was too short for the real board, so do not use it.
- 4 mounting holes Ø 1.65 mm on the standard Nano 40.64 × 15.24 mm grid,
  assumed centred on the board (2.43 mm from the ends). Not yet verified on a
  print; if the locating pins miss the holes, cut them off and adjust `holeX`.
- header rows 15.24 mm apart, 15 pins per row
- micro-USB at the +X end, overhanging the PCB edge by ~1.6 mm, ~7.7 mm wide

## How the board is held

- The PCB rests on two **end ledges** (3.8 mm wide, `ledgeD`) 3.5 mm above the
  floor (`standoff`), which clears untrimmed header tails on the underside.
- Four **locating pins** Ø 1.3 × 1.4 mm (`pinD`, `pinH`) on the ledges go into
  the PCB's mounting holes from below. Drop the board onto them; this aligns the
  USB connector with the notch before the lid goes on. If the pins are too
  tight, sand them or set `pinD` to 1.2.
- Four **bosses** on the lid underside (2.2 mm wide, `bossW`) sit over the hole
  positions, from 0.7 mm inboard of the hole centre (`bossIn`) to the PCB end,
  and stop 0.3 mm above the PCB top (`bossGap`). They only limit lift, they do
  not clamp. They run through the lid, so the header slots end 0.7 mm short of
  the holes; the usable slot is 39.2 mm, 15 Dupont housings need 38.1 mm.
- The board sits 0.5 mm off-centre towards the USB end (`ox` in the script,
  derived from `clr`, `wall` and `usbOver`) so that the connector face is flush
  with the outer face of the base wall. The lid has no skirt on that end and its
  plate ends flush with the base wall, so the port is in the case face. The
  opening in the wall is 13 mm wide (`usbW`) from 2.5 mm below the PCB top
  (`usbDown`) to the rim.
- Headers are assumed **pointing up**; Dupont housings go in from above through
  the lid slots (3.2 mm wide). With headers pointing down the lid does not fit;
  set `standoff` to ≥ 9 mm and drop the slots.

## Dimensions

Base 49.7 × 22.2 × 12.2 mm (pocket 46.5 × 19.0 mm), lid 51.5 × 25.8 × 5.6 mm
(top 1.6 + skirt 4.0, 1.6 thick, U-shaped: long sides and the far end), lid
clearance 0.2 mm per side (`lidClear`, 0.3 if it is too tight). Assembled
51.5 × 25.8 × 13.8 mm. Pocket clearance around the PCB 0.5 mm per side (`clr`).

## Snap fit

Two ridges 8 × 1.2 mm, 0.6 mm proud, with 45° lead-in chamfers on each long
outer wall of the base (at x = ±14 mm, 1.5 mm above the skirt's lower edge),
and grooves with 0.15 mm clearance inside the skirt. The lid snaps in with
0.4 mm overlap; the grooves also locate the lid lengthwise, since it has no
skirt at the USB end. Too stiff: `nubH` 0.5; too loose: `nubClr` 0.1. Parameters
`nubH nubL nubT nubX nubZ nubClr`.

## Printing

- Base: opening up, as exported. The locating pins print as small vertical
  cylinders; no supports.
- Lid: **top face on the bed** ("Place on face" in Bambu Studio), so the
  engraving and the colour change come out clean and the bosses point up.
- 0.2 mm layers, 2–3 walls, no supports.

## Regenerating

Run `fusion_case.py` as a script in Fusion 360 (Utilities > Scripts and
Add-Ins > + > pick the file > Run; with the Fusion MCP add-in it can also be run
straight from Claude Code). It creates a new, unsaved document each
time. Then export Base, Lid and the Label bodies as STL (mm) and the whole
design as STEP into this folder, and run `python make_bambu_3mf.py` (pass the
previous `la_case_bambu.3mf` as the template to keep the printer presets).
