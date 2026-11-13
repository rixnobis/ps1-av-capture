#!/usr/bin/env python3
"""Routing renders: route-top/bottom/in2 at 2400 px and 600x600 crops from a 4x (9600 px) export.

kicad-cli pcb export svg + rsvg-convert, as for the placement renders. Crop centres are KiCad mm.
"""
import subprocess
import sys
from pathlib import Path

from PIL import Image

Image.MAX_IMAGE_PIXELS = None
HAT = Path(__file__).resolve().parent.parent
PCB = HAT / "ps1-av-hat.kicad_pcb"
OUT = HAT / "renders"
BUILD = HAT / "build"
VIEWS = {
    "route-top": "F.Cu,F.Silkscreen,Edge.Cuts",
    "route-bottom": "B.Cu,Edge.Cuts",
    "route-in2": "In2.Cu,Edge.Cuts",
}
# name -> (layers view, centre x mm, centre y mm)
CROPS = {
    "crop-tvp-22r": ("route-top", 114.0, 113.8),
    "crop-buck-ldo": ("route-top", 121.0, 84.0),
    "crop-j3-pcm-usb": ("route-top", 100.5, 80.5),
    "crop-j5-data": ("route-top", 116.0, 120.0),
    "crop-j4-ctrl": ("route-top", 135.0, 76.0),
}


def export(layers, svg):
    subprocess.run(["kicad-cli", "pcb", "export", "svg", "--layers", layers, "--exclude-drawing-sheet",
                    "--page-size-mode", "2", "-o", str(svg), str(PCB)], check=True, capture_output=True)


def main():
    OUT.mkdir(exist_ok=True)
    BUILD.mkdir(exist_ok=True)
    for name, layers in VIEWS.items():
        svg = BUILD / f"{name}.svg"
        export(layers, svg)
        subprocess.run(["rsvg-convert", "-w", "2400", "-b", "white", str(svg), "-o", str(OUT / f"{name}.png")], check=True)
    svg = BUILD / "route-top.svg"
    big = BUILD / "route-top-4x.png"
    subprocess.run(["rsvg-convert", "-w", "9600", "-b", "white", str(svg), "-o", str(big)], check=True)
    im = Image.open(big)
    # bbox of the board in mm from the svg viewBox (page-size-mode 2: page = board bbox, mm units)
    import re
    head = svg.read_text()[:1500]
    vb = [float(v) for v in re.search(r'viewBox="([^"]+)"', head).group(1).split()]
    scale = im.width / vb[2]
    for name, (_, cx, cy) in CROPS.items():
        # the svg origin is the board bbox corner; its mm origin is the page corner
        x0 = (cx - X0) * scale - 300
        y0 = (cy - Y0) * scale - 300
        im.crop((int(x0), int(y0), int(x0) + 600, int(y0) + 600)).save(OUT / f"{name}.png")
    big.unlink()


X0, Y0 = 88.475, 69.970125          # board bbox corner (KiCad mm), from the Edge.Cuts outline

if __name__ == "__main__":
    sys.exit(main())
