# PS1 AV capture hat

One-channel PS1 capture hat for the Maker go CH32H417QEU6 dev board. Schematic: `gen_hat_sch.py`
(`tools/`); board: `tools/build_hat_pcb.py` (outline, headers, holes, placement, no routing);
`./build.sh` runs everything (ERC, netlist checks, PCB, DRC, renders in `renders/`).

## Mechanics
- The hat plugs on the dev board through two 3x19 2.54 mm headers (J4 = board U12, J5 = board U13)
  and sits 8-11 mm above it. Geometry comes from the board DXF (`tools/mg_dxf_geometry.json`,
  made by `tools/extract_mg_dxf.py`). Which DXF header is U12 is still unresolved:
  `TOP_IS_U12` in `tools/build_hat_pcb.py` (default True), flip and rebuild.
- The outline keeps 1 mm clear of the dev board's tall parts (barrel jack, screw terminal, RJ45,
  USB-A). The headers run to x = -37.5 (DXF) but the RJ45 starts at -43.1, so the outline is
  notched between two ears; the ears carry the two right-hand mounting holes.
- All four M3 holes of the dev board fall inside the hat outline (H1-H4 match them, 3.2 mm).
  Use standoffs of the header-stack height; the headers are not the only support.
- Placement is for review; nothing is routed. The dev board's tact switches and LEDs near its
  lower-left corner are covered by the hat.
