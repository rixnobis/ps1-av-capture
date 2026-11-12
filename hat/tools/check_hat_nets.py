#!/usr/bin/env python3
"""Hat netlist sanity: no single-node nets. (tools/check_netlist.py hard-codes the FX3 map.)"""
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
bad = [(n.get("name"), len(n.findall("node"))) for n in root.find("nets")
       if len(n.findall("node")) < 2 and not n.get("name").startswith("unconnected-")]
print(f"nets {len(root.find('nets'))}; single-node nets {len(bad)}")
for b in bad:
    print("  ", b)
sys.exit(1 if bad else 0)
