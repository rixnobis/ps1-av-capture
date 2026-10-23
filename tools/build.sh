#!/bin/bash
# Regenerate the schematic and run every check; stops on the first failure.
set -euo pipefail
cd "$(dirname "$0")/.."
python3 tools/gen_sch.py
kicad-cli sch erc --format report --exit-code-violations -o build/erc.rpt ps1-av-capture.kicad_sch >/dev/null
grep 'ERC messages' build/erc.rpt
kicad-cli sch export netlist --format kicadxml -o build/netlist.xml ps1-av-capture.kicad_sch >/dev/null
python3 tools/check_netlist.py build/netlist.xml
python3 tools/check_footprints.py build/netlist.xml
if [ "${LCSC:-1}" = 1 ]; then python3 tools/check_lcsc.py build/netlist.xml; fi
kicad-cli sch export pdf -o schematic.pdf ps1-av-capture.kicad_sch >/dev/null
python3 tools/build_pcb.py place
python3 tools/route_usb.py 2> >(grep -v "swig/python detected" >&2)
python3 tools/fanout.py 2> >(grep -v "swig/python detected" >&2)
python3 tools/escape.py 2> >(grep -v "swig/python detected" >&2)
python3 tools/bga_route.py 2> >(grep -v "swig/python detected" >&2)
echo "build ok"
