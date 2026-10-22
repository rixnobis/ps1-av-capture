# Design notes, schematic rev A

One channel: PS1 AV multi-out in, two USB ports out. The TVP7002 digitises RGB, the FX3
(CYUSB3014) streams it over USB 3 as UVC (Cypress AN75779 pattern), and the PCM2903C is a
USB Audio Class device on its own USB 2.0 port. `schematic.pdf` has five sheets. `tools/build.sh`
regenerates it and runs the checks.

## Video path
- The TVP7002 has no BT.656 mode. SLES206C says "10-bit 4:2:2 YCbCr output format (i.e., ITU-R
  BT.656) is not supported". Its outputs are 30-bit 4:4:4, or 20-bit 4:2:2 with SAV/EAV codes.
- The top 8 bits of each channel go to GPIF II through 22R series resistors at the TVP end
  (AN70707 checklist item 9): DQ[7:0]=B, DQ[15:8]=G, DQ[23:16]=R. PCLK=DATACLK, and
  CTL[0..3]=HSOUT/VSOUT/FIDOUT/SOGOUT.
  - 4:2:2 needs only DQ[15:0] (Y on G, CbCr on B), which is the 16-bit shape AN75779 uses.
  - The FX3 datasheet contradicts itself on 24-bit GPIF support, which is why the 4:2:2 fallback
    is wired.
- Sync: the multi-out carries no sync pin. Composite (pin 7) feeds SOGIN_2 and S-Video Y (pin 8)
  feeds SOGIN_3, each through 1 nF. The TVP7002's slicer supports sync-on-luminance, and the
  OSSC takes SCART csync into SOGIN the same way. The digital HSYNC/VSYNC inputs are grounded.
- CVBS and Y also go to spare ADC inputs (GIN_2, GIN_3), so the TVP7002 can digitise luma too.
- Every input has a 75R termination at the connector and a TPD4E05U06 for ESD.
- A 27 MHz oscillator on EXT_CLK serves mode detection. SLES206C recommends it because the
  internal reference is about 6.5 MHz and imprecise.
- TVP7002 RESETB is held low by 2.2k and released by FX3 GPIO45. The TVP7002 must see no I2C
  traffic before its reset completes, and the FX3 boots over the same I2C bus.

## FX3
- Boot is PMODE F1F: I2C EEPROM (24LC1025, 128 KB) first, USB if that fails. Every PMODE pin has
  10k up/down footprints, and only PMODE1's pull-up is fitted. Jumper JP1 forces USB boot.
- 19.2 MHz crystal, FSLC=000 through one 10k.
- Reset is an RC (10k/1u, about 10 ms), a button, and the JTAG pin 10.
- Decoupling follows AN70707 Table 3. AVDD, U3TXVDDQ, U3RXVDDQ and CVDDQ each sit behind a
  ferrite, and each USB 3 supply has 22 uF (datasheet). All VIO are 3.3 V.
- USB 3 Standard-B connector:
  - SS TX has 100 nF coupling at the connector. AN70707 says both "near the connector" and
    "near the FX3"; p.22 and p.40 describe exactly this host-to-device case, so I follow them.
  - ESD at the connector.
  - The shield goes to GND through 100k || 2.2 nF.
- VBUS and VBATT both go to the connector's VBUS. AN70707 never says whether VBATT may float,
  so this is my choice.

### FX3 GPIO budget and the status LED
Used: GPIO[0..15] and [33..40] (DQ[23:0]), [16] PCLK, [17..20] CTL[0..3] (HS/VS/FID/SOG),
[30..32] PMODE, [45] TVP7002 reset, [55]/[56] debug UART, [58]/[59] I2C. Free: [21..29] (CTL[4..12]
on VIO1), [43]/[44] (VIO2), [46..54] (VIO3/VIO4), and [57]. [41] and [42] stay unused because the
datasheet makes them address lines in a 24-bit GPIF configuration. The FX3's I2S is unused,
because audio has its own codec on its own USB port.

The status LED D3 is an XL-1615RGBC-RF (LCSC C965840, common anode, pinout from its datasheet
p.8). The anode is on +3V3 and the cathodes are sunk by GPIO[50]/[51]/[52] (balls D1/D2/D3, the
I2S balls) through 470R (red) and 100R (green, blue). That is about 2 mA each at typical forward
voltage. Green and blue have little headroom at 3.3 V, so the values get tuned at bring-up if
those two read dim. All FX3 I/Os are tristated during reset (datasheet), so the LED is dark then.
The documents I have don't say whether the boot ROM drives GPIO[50..52] before firmware runs, so
"off = no firmware" gets checked at bring-up. The RGB LED replaces the single debug LED that was on GPIO[57]. The power LED is unchanged.

Firmware state map:

| LED | State |
|---|---|
| off | no firmware running |
| red | booted, USB not enumerated |
| amber (red + green) | enumerated, no video sync locked (firmware polls the TVP7002 sync-detect status over I2C) |
| green | sync locked |
| blue pulse | streaming to the host |

## Power, from the FX3's USB 3 VBUS
- Three TPS563201 bucks, with values from SLVSD90B Table 7-2:
  - 3.3 V digital;
  - 2.5 V, which feeds two TLV75519 LDOs: 1.9 V for TVP7002 AVDD/PLL (through ferrites) and
    1.9 V for DVDD. This is the OSSC's pattern.
  - 1.2 V FX3 core.
- TVP7002 A33VDD has its own TLV75733 from VBUS, kept off the buck rail.
- Estimated input is about 420 mA, from datasheet typicals (TVP7002 at 78.75 MHz, FX3 ICC
  maximums) and an assumed 85% buck efficiency. It has not been measured. It is over the
  500 mA USB 2 budget if a host falls back to USB 2 and the TVP7002 runs hot.

## Audio
- TI documents the PCM2903C only as self-powered (SBFS038A Figure 41). A TLV75533 makes its
  3.3 V from the audio port's VBUS.
- The D+ 1.5k pull-up goes to that 3.3 V rather than through TI's AND gate. On this board VDD
  cannot exist without VBUS, so the gate has nothing left to do. That is my inference.
- Line input: 2.2 uF, then 10k into the ADC's 30k input (x0.75), then 330 pF to GND. 2.6 Vpp
  maps to full scale (1.98 Vpp at 3.3 V) and the pole is about 48 kHz. The PS1 line level is
  not measured yet, and the 10k is the knob if it clips.

## Checks (tools/build.sh)
- ERC: 0. The `lib_symbol_issues`/`lib_symbol_mismatch` rules are off because the symbols are
  embedded and generated.
- `check_netlist.py` traces every TVP7002 output bit through one 22R to its FX3 ball and checks
  72 supply pins against their rails, using a hand-written map independent of the generator.
  - Positive control: swapping two FX3 balls in the netlist plus removing one VDD ball gives
    three findings.
  - It found a real defect on its first run: the EEPROM had been given the TVP7002's reference,
    U1, and ERC passed it.
- `check_footprints.py` checks that every symbol pin has a pad and every pad a pin.
- `check_lcsc.py` reads each LCSC number back from JLC and matches value, tolerance and
  package. It caught 7 mistyped passive numbers, a nonexistent inductor number and an LED with
  one in stock.
  - TVP7002 stock is 2 (C3824085). That covers this board and nothing more.

## Footprints (tools/fix_footprints.py, applied to the LCSC patterns)
- TVP7002 exposed pad: SLES206C and its package addendum carry no land drawing. The OSSC builds
  this chip in production with a 6.2 x 6.2 mm pad and a 7 x 7 grid of 0.33 mm vias at 0.8 mm
  (marqs85/ossc_pcb, TQFP-100_thermal), so I used that pad and grid, with 0.30 mm drills. The
  LCSC pattern had 5.445 mm and nine unnumbered vias.
- FX3 BGA pads: 0.28 mm (11 mil, AN70707) instead of LCSC's 0.40 mm.
- USB3-B shell pads renumbered SH to match the symbol.

## Boot EEPROM
AN76405 lists sizes from 32 Kb to 2048 Kb, 2-byte addressing only. Its Table 17 straps a
24LC1025 as A2/A1/A0 = Vcc/0/0, which is how U3 is wired. A second 24LC1025 footprint (DNP,
Vcc/0/1) holds a second 128 KB if the UVC firmware outgrows the first.

## AV input: J1 and the pigtail
J1 is a JST PHD 2x7 header (B14B-PHDSS, 2.0 mm, LCSC C471471): through-hole, keyed shroud,
latching cable housing. It is DNP in the JLC assembly and hand-soldered. The 75R terminations,
the TPD4E05U06 clamps and the coupling caps sit beside it, so the pigtail is the only part of the
input that is not on the board. Each signal shares a column with a ground, so the pigtail is seven
twisted signal/ground pairs; keep it short (15-20 cm).

Pigtail, J1 pin -> PS1 multi-AV male plug pin (multi-out pinout per psx-spx, "AV Multi Out"):

| J1 | Signal | Multi-AV pin |
|---|---|---|
| 1 | Red | 2 |
| 3 | Green | 1 |
| 5 | Blue | 4 |
| 7 | S-Video Y (luma) | 8 |
| 9 | Composite | 7 |
| 11 | Audio left | 9 |
| 13 | Audio right | 11 |
| 2, 4, 6, 8, 10 | video grounds | 5, and the plug shell where the kit has a metal one (psx-spx: shield = video ground) |
| 12 | audio left ground | 10 |
| 14 | audio right ground | 12 |

Multi-out pin 3 (+5 V) and pin 6 (S-Video C) are left empty in the plug. psx-spx numbers the
console socket 12..1 left to right seen from the rear; the plug is its mirror, so check each
cavity against the socket before soldering.

Build notes:
1. JLC assembles everything except J1.
2. Hand-solder J1 from the top, shroud key matching the silkscreen outline.
3. Crimp the PHDR-14VS housing with SPHD contacts (24-28 AWG), solder the other ends into the
   multi-AV plug kit per the table, and continuity-check every pin plus pin-to-pin shorts
   before it goes near a console.

## Open before layout
- Stackup: AN70707 asks for at least 4 layers for 90 ohm pairs. JLC's 4-layer impedance
  stackups are the plan, with lengths matched to 5 mil (SS) and 20 mil (HS).

## Collaboration decision
Sync on the 15 kHz path was the one front-end call put to Pixel, because he has run PS1 RGB
through real gear. His answer was to keep the BoM small, so the board uses the TVP7002's own SOG
slicer on composite (SOGIN_2) or S-Video Y (SOGIN_3), selected by register. There is no
LM1881-class separator.
