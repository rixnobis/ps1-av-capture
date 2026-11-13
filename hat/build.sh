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
mkdir -p renders
kicad-cli pcb export svg --layers "F.Cu,F.Silkscreen,F.Fab,F.Courtyard,Edge.Cuts" --exclude-drawing-sheet --page-size-mode 2 -o build/placement.svg ps1-av-hat.kicad_pcb >/dev/null
kicad-cli pcb export svg --layers "F.Cu,F.Silkscreen,F.Courtyard,Edge.Cuts" --exclude-drawing-sheet --page-size-mode 2 -o build/placement_cu_silk_courtyard.svg ps1-av-hat.kicad_pcb >/dev/null
if command -v rsvg-convert >/dev/null; then
  rsvg-convert -w 2400 -b white build/placement.svg -o renders/placement.png
  rsvg-convert -w 2400 -b white build/placement_cu_silk_courtyard.svg -o renders/placement_cu_silk_courtyard.png
fi
# ---- routing: planes, pours, stitching, Freerouting (DSN/SES like tools/autoroute.py) -------------------
# Freerouting is NOT deterministic (thread timing), so the routed board differs from run to run and the
# build gates on DRC (0 errors, 0 unconnected, 0 parity) + check_geometry.py + the In1 plane check,
# never on a fixed result. The jar is fetched once into FREEROUTING_JAR's directory if it is missing.
FR_JAR="${FREEROUTING_JAR:-build/freerouting-2.5.0.jar}"
FR_ROUNDS="${FREEROUTING_ROUNDS:-3}"     # round n+1 starts from the board round n left
if [ ! -f "$FR_JAR" ]; then
  mkdir -p "$(dirname "$FR_JAR")"
  curl -sL -o "$FR_JAR" https://github.com/freerouting/freerouting/releases/download/v2.5.0/freerouting-2.5.0.jar
fi
python3 tools/route_hat.py prep 2> >(grep -v "swig/python detected" >&2)
for round in $(seq 1 "$FR_ROUNDS"); do
  python3 tools/route_hat.py dsn 2> >(grep -v "swig/python detected" >&2)
  rm -f build/ps1-av-hat.ses
  nice timeout 1200 java -Djava.awt.headless=true -jar "$FR_JAR" -de build/ps1-av-hat.dsn \
       -do build/ps1-av-hat.ses -mp 20 > "build/freerouting-$round.log" 2>&1 || true
  [ -s build/ps1-av-hat.ses ] || { echo "freerouting round $round produced no SES"; exit 1; }
  python3 tools/route_hat.py import 2> >(grep -v "swig/python detected" >&2)
done
python3 tools/route_hat.py prune 2> >(grep -v "swig/python detected" >&2)   # drop router wires made redundant by the plane vias
python3 tools/route_hat.py tidy 2> >(grep -v "swig/python detected" >&2)    # merge collinear chains, contract jogs
python3 tools/route_hat.py finish 2> >(grep -v "swig/python detected" >&2)
python3 tools/join_hat.py ps1-av-hat.kicad_pcb --rounds 3 --cut 2> >(grep -v "swig/python detected" >&2)   # wire the pairs DRC still reports (45-degree A*, F/In2/B)
python3 tools/route_hat.py trim 2> >(grep -v "swig/python detected" >&2)     # dangling ends and duplicates the joins left
python3 tools/join_hat.py ps1-av-hat.kicad_pcb --rounds 2 --cut 2> >(grep -v "swig/python detected" >&2)   # what the trim exposed
python3 tools/route_hat.py trim 2> >(grep -v "swig/python detected" >&2)     # dangling ends and duplicates the joins left
python3 tools/route_hat.py refill 2> >(grep -v "swig/python detected" >&2)
python3 tools/tidy_geometry.py 2> >(grep -v "swig/python detected" >&2)   # geometry tidy post-pass: overlaps, jogs, 135 degree snaps, chamfers, necks (DRC-gated, bisected)
python3 tools/route_hat.py refill 2> >(grep -v "swig/python detected" >&2)   # tidy moves copper, so the committed pours are refilled after it
kicad-cli pcb drc --format report --severity-all --schematic-parity --refill-zones -o build/drc.rpt ps1-av-hat.kicad_pcb >/dev/null
grep -E "^\*\* Found" build/drc.rpt
kicad-cli pcb drc --format report --severity-error --schematic-parity --exit-code-violations \
  -o build/drc-errors.rpt ps1-av-hat.kicad_pcb >/dev/null || { echo "DRC errors or unconnected items: see build/drc.rpt"; DRC_FAIL=1; }
# check_geometry.py exits 1 on any finding; Freerouting output always has some (pin-exit necks, 90 degree
# corners at pads, jogs), so the build gates on the findings that break copper (dangling ends, zero-length
# wires) and prints the census of the rest.
python3 ../tools/check_geometry.py ps1-av-hat.kicad_pcb > build/geometry.txt 2> >(grep -v "swig/python detected" >&2) || true
tail -1 build/geometry.txt
awk '/^(ACUTE|SHORT|NECKED|OVERLAP|DANGLING|ZERO-LEN)/ {c[$1]++} END {for (k in c) printf "  %-9s %d\n", k, c[k]}' build/geometry.txt | sort
if grep -qE '^(DANGLING|ZERO-LEN)' build/geometry.txt; then GEO_FAIL=1; fi
python3 tools/check_plane.py ps1-av-hat.kicad_pcb 2> >(grep -v "swig/python detected" >&2) || PLANE_FAIL=1
python3 tools/render_route.py 2> >(grep -v "swig/python detected" >&2)
if [ -n "${DRC_FAIL:-}${GEO_FAIL:-}${PLANE_FAIL:-}" ]; then echo "hat build: routing gates FAILED"; exit 1; fi
echo "hat build ok"
