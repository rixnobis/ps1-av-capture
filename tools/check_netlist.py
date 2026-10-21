#!/usr/bin/env python3
"""Check the exported netlist against the intent, independently of gen_sch.py's tables.

ERC passes anything whose pins are all passive and connected by labels, so it cannot see a
mis-wired bus. This reads the KiCad XML netlist and checks:
  1. no net has a single node (a mistyped label makes one);
  2. each TVP7002 output bit reaches the intended FX3 ball through exactly one 22R;
  3. every supply ball/pin of U1 and U2 sits on the rail its datasheet name requires.
Usage: check_netlist.py netlist.xml   (exit 1 on any finding)
"""
import csv
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PINS = Path(__file__).resolve().parent / "pins"
root = ET.parse(sys.argv[1]).getroot()
val = {c.get("ref"): c.findtext("value") for c in root.find("components")}
node_net, nets = {}, {}
for n in root.find("nets"):
    name = n.get("name")
    nodes = [(x.get("ref"), x.get("pin")) for x in n.findall("node")]
    nets[name] = nodes
    for nd in nodes:
        node_net[nd] = name
bad = []

for name, nodes in nets.items():
    if len(nodes) < 2 and not name.startswith("unconnected-"):
        bad.append(f"single-node net {name}: {nodes}")

tvp = {r["name"]: r["pin"] for r in csv.DictReader(open(PINS / "tvp7002.csv"))}
fx3 = {r["name"]: r["ball"] for r in csv.DictReader(open(PINS / "fx3.csv"))}
# Intended GPIF map, written out by hand: DQ[7:0]=B[9:2], DQ[15:8]=G[9:2], DQ[23:16]=R[9:2];
# DQ0-15 are GPIO0-15, DQ16-23 are GPIO33-40; PCLK=GPIO16, CTL0-3=GPIO17-20.
want = {}
for i in range(24):
    ch, bit = "BGR"[i // 8], 2 + i % 8
    gpio = i if i < 16 else 33 + (i - 16)
    want[f"{ch}_{bit}"] = f"GPIO[{gpio}]"
want.update({"DATACLK": "GPIO[16]", "HSOUT": "GPIO[17]", "VSOUT": "GPIO[18]",
             "FIDOUT": "GPIO[19]", "SOGOUT": "GPIO[20]"})
for tname, gname in want.items():
    src = node_net.get(("U1", tvp[tname]))
    if src is None:
        bad.append(f"TVP {tname} unconnected")
        continue
    rs = [r for r, p in nets[src] if r.startswith("R")]
    if len(rs) != 1 or val[rs[0]] != "22" or len(nets[src]) != 2:
        bad.append(f"TVP {tname}: net {src} is {nets[src]}, want U1 + one 22R")
        continue
    other = [n for (r, p), n in node_net.items() if r == rs[0] and n != src]
    dst = other[0] if other else None
    if dst is None or ("U2", fx3[gname]) not in nets.get(dst, []):
        bad.append(f"TVP {tname} -> {dst}, want FX3 {gname} ({fx3[gname]})")

rail = [("U2", r"^VDD$", "+1V2"), ("U2", r"^AVDD$", "FX3_AVDD"), ("U2", r"^U3TXVDDQ$", "FX3_U3TX"),
        ("U2", r"^U3RXVDDQ$", "FX3_U3RX"), ("U2", r"^CVDDQ$", "FX3_CVDDQ"), ("U2", r"^VIO\d$", "+3V3"),
        ("U2", r"^(VSS|AVSS|U3VSSQ)$", "GND"), ("U2", r"^(VBUS|VBATT)$", "VBUS"),
        ("U1", r"^A33VDD$", "+3V3A"), ("U1", r"^AVDD$", "TVP_AVDD"), ("U1", r"^PLL_AVDD$", "TVP_PLLVDD"),
        ("U1", r"^DVDD$", "+1V9D"), ("U1", r"^IOVDD$", "+3V3"),
        ("U1", r"^(A33GND|AGND|PLL_AGND|DGND|GND|IOGND|NSUB)$", "GND")]
tables = {"U2": [(r["ball"], r["name"]) for r in csv.DictReader(open(PINS / "fx3.csv"))],
          "U1": [(r["pin"], r["name"]) for r in csv.DictReader(open(PINS / "tvp7002.csv"))]}
checked = 0
for ref, pat, net in rail:
    for pin, name in tables[ref]:
        if re.match(pat, name):
            checked += 1
            got = node_net.get((ref, pin))
            if got != net:
                bad.append(f"{ref} {name} ({pin}) on {got}, want {net}")
if node_net.get(("U1", "101")) != "GND":
    bad.append("TVP7002 exposed pad (101) not on GND")

print(f"nets {len(nets)}; GPIF bits checked {len(want)}; supply pins checked {checked}; findings {len(bad)}")
for b in bad:
    print("  " + b)
sys.exit(1 if bad else 0)
