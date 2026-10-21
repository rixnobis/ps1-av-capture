Pin tables for the parts that have no stock KiCad symbol.

- `fx3.csv`: CYUSB3014 ball map, read off the 121-ball BGA figure in Infineon datasheet
  001-52136 Rev *Z. Cross-checked against the datasheet's pin list: they agree except that
  the pin list writes F4/G2 as `GPIO[41]/A0`, `GPIO[42]/A1` and D11 as `O[60]` (the ball map
  says NC). D11 is left unconnected.
- `tvp7002.csv`: TVP7002 pins as named in the OSSC schematic symbol (marqs85/ossc_pcb),
  cross-checked pin by pin against Table 1 of TI SLES206C: no disagreement.
  Bit order: B9..B0 on pins 29..38, G9..G0 on 43..52, R9..R5 on 55..59, R4..R0 on 61..65.

The PCM2903C uses the stock `Audio:PCM2902` symbol. The 28 pin numbers carry the same
functions as the PCM2903C pin table in TI SBFS038A.
