# Gehäuse für den Logic Analyzer (Nano 33 IoT)

Zweiteiliges Druckgehäuse für den 5-Kanal-Logic-Analyzer aus `../README.md`:
Unterschale mit Board-Tasche und Kappe mit zwei Schlitzen über den Pin-Reihen,
Schriftzug „LOGIC / ANALYZER“ 0,6 mm graviert und als eigener Körper für einen
zweiten Filamentwechsel (AMS).

| Datei | Inhalt |
|---|---|
| `fusion_case.py` | erzeugt das komplette Modell in Fusion (Skript ist die Quelle; Parameter im `P`-Dict oben, zusätzlich als Benutzerparameter im Dokument) |
| `la_case.step` | Base + Lid + Buchstaben, mm |
| **`la_case_bambu.3mf`** | **Bambu-Studio-Projekt, fertig zum Slicen:** Objekt „Lid“ = Teil „Lid“ (Filament 1) + Teil „Label“ (Filament 2), Kappe liegt schon mit der Oberseite auf dem Bett; Objekt „Base“ daneben. Datei > Projekt öffnen. Presets (X2D 0,4, 0.20 mm Standard, PLA) stammen aus `aurix_breakout.3mf`. |
| `make_bambu_3mf.py` | erzeugt `la_case_bambu.3mf` aus den drei STLs (`python make_bambu_3mf.py [vorlage.3mf]`) |
| `la_case.3mf` | roher Fusion-Export, alle Körper getrennt (13 Buchstaben einzeln) — nur Fallback |
| `la_case_Base.stl`, `la_case_Lid.stl`, `la_case_Label.stl` | Einzelkörper (Label = alle 13 Buchstaben in einer Datei). Fallback ohne Projektdatei: Lid + Label zusammen importieren, „als ein Objekt mit mehreren Teilen laden“, dem Teil Label das zweite Filament geben |

## Annahmen (bitte am Board prüfen)

- Nano 33 IoT **mit nach oben stehenden Stiftleisten**, Dupont-Buchsen werden von
  oben durch die Kappenschlitze gesteckt (Schlitz 3,2 mm breit, Buchse 2,54 mm).
  Bei Pins nach unten passt die Kappe nicht — dann `standoff` auf ≥ 9 mm und die
  Schlitze entfallen.
- Board 45 × 18 × 1,6 mm, Pin-Reihen 15,24 mm auseinander, mittig über die Länge.
- Micro-USB am +X-Ende, Stecker-Umspritzung ≤ 12 × 7 mm.
- Unterseite an beiden Board-Enden 2 mm breit frei (Auflageleisten).
- Die vier Niederhalter der Kappe (1,6 × 2,2 mm) sitzen auf den Board-Ecken —
  Reset-Taster am USB-Ende auf Kollision prüfen.

## Maße

Base 49 × 22 × 11,2 mm, Kappe 52,6 × 25,6 × 5,6 mm (Deckel 1,6 + Schürze 4,0 ×
1,6 dick), Kappenspiel 0,2 mm je Seite (Parameter `lidClear`, bei zu strammem
Sitz auf 0,3).

## Rastung

Je zwei Wülste 8 × 1,2 mm, 0,6 mm hoch, mit 45°-Anlaufschrägen auf den langen
Außenwänden der Unterschale (bei x = ±14 mm, 1,5 mm über der Schürzenunterkante),
dazu Nuten mit 0,15 mm Spiel innen in der Schürze. Die Kappe rastet mit 0,4 mm
Überdeckung ein. Rastet sie zu schwer, `nubH` auf 0,5 setzen; zu leicht, `nubClr`
auf 0,1. Parameter `nubH nubL nubT nubX nubZ nubClr`.

## Drucken

- Base: Öffnung nach oben, wie exportiert.
- Kappe: **Oberseite auf das Druckbett** (in Bambu Studio „Auf Fläche legen“),
  dann sind Gravur und Farbwechsel sauber und die Niederhalter zeigen nach oben.
- 0,2 mm Schichten, 2–3 Wände, keine Stützen nötig.

## Neu erzeugen

Fusion + Fusion-MCP-Add-in laufen lassen, `P` im Skript anpassen, Skript über
den MCP (`fusion_mcp_execute`, `featureType: script`) ausführen. Es legt jedes
Mal ein neues, ungespeichertes Dokument an.
