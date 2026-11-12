#!/bin/bash
# Regenerate the hat schematic and run the checks; stops on the first failure.
# tools/check_netlist.py (FX3 net map) is not used: it hard-codes FX3 nets.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build
python3 tools/gen_hat_sch.py
kicad-cli sch erc --format report --exit-code-violations -o build/erc.rpt ps1-av-hat.kicad_sch >/dev/null
grep 'ERC messages' build/erc.rpt
kicad-cli sch export netlist --format kicadxml -o build/netlist.xml ps1-av-hat.kicad_sch >/dev/null
python3 tools/check_hat_nets.py build/netlist.xml
python3 tools/check_hat_footprints.py build/netlist.xml 2> >(grep -v "swig/python detected" >&2)
kicad-cli sch export pdf -o schematic.pdf ps1-av-hat.kicad_sch >/dev/null
python3 tools/build_hat_pcb.py 2> >(grep -v "swig/python detected" >&2)
kicad-cli pcb drc --format report --severity-all -o build/drc.rpt ps1-av-hat.kicad_pcb >/dev/null
grep -E "^\*\* Found" build/drc.rpt
mkdir -p renders
kicad-cli pcb export svg --layers "F.Cu,F.Silkscreen,F.Fab,F.Courtyard,Edge.Cuts" --exclude-drawing-sheet --page-size-mode 2 -o build/placement.svg ps1-av-hat.kicad_pcb >/dev/null
kicad-cli pcb export svg --layers "F.Cu,F.Silkscreen,F.Courtyard,Edge.Cuts" --exclude-drawing-sheet --page-size-mode 2 -o build/placement_cu_silk_courtyard.svg ps1-av-hat.kicad_pcb >/dev/null
if command -v rsvg-convert >/dev/null; then
  rsvg-convert -w 2400 -b white build/placement.svg -o renders/placement.png
  rsvg-convert -w 2400 -b white build/placement_cu_silk_courtyard.svg -o renders/placement_cu_silk_courtyard.png
fi
echo "hat build ok"
