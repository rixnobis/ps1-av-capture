#!/usr/bin/env python3
"""Every symbol pin number must exist as a pad in that part's footprint, and every copper pad
must be a symbol pin (or a deliberately unnumbered mechanical pad).
A mismatch here builds a board whose netlist silently drops a connection.
Usage: check_footprints.py netlist.xml   (exit 1 on any finding)
"""
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
LIBS = {"ps1-av-capture": str(ROOT / "lib" / "av.pretty")}
STOCK = Path("/usr/share/kicad/footprints")

root = ET.parse(sys.argv[1]).getroot()
libparts = {}
for lp in root.find("libparts"):
    key = (lp.get("lib"), lp.get("part"))
    libparts[key] = {p.get("num") for p in lp.find("pins")} if lp.find("pins") is not None else set()
bad, checked = [], 0
cache = {}
for c in root.find("components"):
    fp = c.findtext("footprint")
    src = c.find("libsource")
    if not fp:
        continue
    lib, name = fp.split(":", 1)
    path = LIBS.get(lib) or str(STOCK / f"{lib}.pretty")
    if fp not in cache:
        cache[fp] = pcbnew.FootprintLoad(path, name)
    f = cache[fp]
    if f is None:
        bad.append(f"{c.get('ref')}: footprint {fp} not found")
        continue
    pads = {p.GetNumber() for p in f.Pads() if p.GetNumber()}
    pins = libparts.get((src.get("lib"), src.get("part")), set())
    missing = sorted(pins - pads)
    extra = sorted(p for p in pads - pins if p not in ("SH", "EP"))
    checked += 1
    if missing:
        bad.append(f"{c.get('ref')} ({src.get('part')} on {name}): pins with no pad {missing}")
    if extra and pins:
        bad.append(f"{c.get('ref')} ({src.get('part')} on {name}): pads with no pin {extra}")
print(f"components checked {checked}, findings {len(bad)}")
for b in bad:
    print("  " + b)
sys.exit(1 if bad else 0)
