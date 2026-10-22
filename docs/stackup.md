# Stackup and controlled impedance

Board: 4 layers, 1.6 mm, JLCPCB stackup JLC04161H-3313A (the calculator's default and lowest-cost
option). Layer use: L1 signal and parts, L2 solid GND, L3 power, L4 signal.

JLC04161H-3313A as JLC's calculator lists it (dielectric thickness, Er):

| Layer | Material | Thickness | Er |
|---|---|---|---|
| L1 | copper 1 oz | 0.035 mm | |
| | prepreg 3313 RC57% | 0.107 mm | 4.1 |
| | prepreg 3313 RC57% | 0.0994 mm | 4.1 |
| L2/L3 | core 1.1 mm H/HOZ | 1.065 mm, 0.0152 mm copper | 4.38 |
| | prepreg 3313 RC57% | 0.0994 mm | 4.1 |
| | prepreg 3313 RC57% | 0.107 mm | 4.1 |
| L4 | copper 1 oz | 0.035 mm | |

L1 to L2 is 0.2064 mm (8.126 mil). JLC's solver models it at Er 4.3.

## 90 ohm differential pairs (USB 3 SS TX/RX, both USB 2 D+/D-)

JLC's own solver (`W2_DiffEdgeCoupledCoatedMicrostrip1B`, L1 over L2, solder-mask coated):
spacing 6 mil (0.152 mm) gives a trace width of 9.74 mil (0.247 mm) at 89.67 ohm.

Regenerate with `tools/jlc_impedance.js 6 14`; it drives the calculator page and records each
stackup's request and result. Width and spacing used on the board: 0.25 mm / 0.15 mm.

## Rules from AN70707
- SS pair lengths matched within 5 mil (0.127 mm); HS D+/D- within 20 mil (0.508 mm).
- Keep USB traces under 3 inches. No 90-degree bends. Solid reference plane, no split under any pair.
- A ground via within 40 mil of every signal via on a pair.
- Keep other signals at least 3x the trace width from a pair.
