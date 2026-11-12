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
echo "hat build ok"
