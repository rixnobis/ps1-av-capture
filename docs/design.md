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

## Open before layout
- TVP7002 PZP thermal pad: the LCSC pattern has a 5.445 mm pad. The datasheet text carries no
  land drawing, so TI's PZP drawing is the reference.
- FX3 BGA pads are 0.40 mm in the LCSC pattern; AN70707 recommends 11 mil (0.28 mm).
- Which EEPROM sizes the FX3 I2C bootloader accepts (AN76405). The 24LC1025 assumes the UVC
  firmware fits in 128 KB.
- Stackup: AN70707 asks for at least 4 layers for 90 ohm pairs. JLC's 4-layer impedance
  stackups are the plan, with lengths matched to 5 mil (SS) and 20 mil (HS).

## Collaboration decision
I'm asking Pixel how he would take sync on the 15 kHz path. That is a question, not a task.
My default is the TVP7002's own SOG slicer on composite. The alternative is a sync separator
(LM1881 class) into HSYNC_A. He has run PS1 RGB through real gear, and that is the one
front-end call where his experience beats my reading of the datasheet. The question is on the
card with a default and a date.
