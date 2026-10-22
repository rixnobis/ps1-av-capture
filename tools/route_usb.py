#!/usr/bin/env python3
"""Route the USB pairs by hand geometry: SS TX/RX and the FX3's D+/D- between U2 and J2.

Run after build_pcb.py place. 45-degree bends only. SS pairs are matched to 0.05 mm and
D+/D- to 0.25 mm (AN70707 asks 5 mil and 20 mil) by a 45-degree bump on one straight
segment; the report prints every length. D+/D- drop to B.Cu after their ESD (TX crosses
over them) with a GND via within 1 mm of each signal via (AN70707 p.26).
"""
import math
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
PCB = ROOT / "ps1-av-capture.kicad_pcb"
OX, OY = 100.0, 100.0
W = 0.25                      # 90 ohm on L1/L2 and L4/L3, docs/stackup.md
F, B = pcbnew.F_Cu, pcbnew.B_Cu

# Each path: list of (x, y) on one layer; pads it passes over are part of the polyline.
PATHS = {
    "USB3_TXN_C": [(F, [(62.0, 54.0), (62.0, 55.0), (61.9, 55.1), (61.9, 56.12)])],
    "USB3_TXP_C": [(F, [(62.8, 54.0), (62.8, 55.0), (62.9, 55.1), (62.9, 56.12)])],
    "USB3_TXN": [(F, [(61.9, 57.08), (61.9, 58.5), (62.0, 58.6), (62.0, 60.3), (60.0, 62.3),
                      (58.65, 62.3), (57.75, 63.2), (57.75, 64.0)])],
    "USB3_TXP": [(F, [(62.9, 57.08), (62.9, 58.4), (62.5, 58.8), (62.5, 61.722), (61.522, 62.7),
                      (59.5, 62.7), (59.5, 64.0)])],
    "USB3_RXP": [(F, [(63.6, 54.0), (63.6, 58.7), (63.5, 58.8), (63.5, 60.4), (64.5, 61.4),
                      (64.5, 64.0)])],
    "USB3_RXN": [(F, [(64.4, 54.0), (64.4, 58.7), (64.0, 59.1), (64.0, 60.334), (66.25, 62.584),
                      (66.25, 64.0)])],
    "USB3_DM": [(F, [(58.8, 54.0), (58.8, 55.5), (57.75, 56.55), (53.6, 56.55)]),
                (B, [(53.6, 56.55), (53.6, 66.0), (60.75, 66.0)])],
    "USB3_DP": [(F, [(59.6, 54.0), (59.6, 57.4), (58.55, 58.45), (52.7, 58.45)]),
                (B, [(52.7, 58.45), (53.2, 58.95), (53.2, 66.4), (58.3, 66.4), (59.9, 68.0),
                     (60.75, 68.0)])],
    "GND": [(F, [(63.0, 59.115), (63.0, 60.65)])],     # D4 pads 8 and 3, down to a via
}
# Where the polyline meets a capacitor it stops at one pad and the next net starts at the other:
# the 0.96 mm across each SS TX cap counts towards the pair's length.
CAP = {"USB3_TXN": 0.96, "USB3_TXP": 0.96}
VIAS = [("USB3_DM", 53.6, 56.55, 0.6, 0.3), ("USB3_DP", 52.7, 58.45, 0.6, 0.3),
        ("GND", 63.0, 60.65, 0.45, 0.2),
        ("GND", 53.6, 55.6, 0.45, 0.2), ("GND", 51.75, 58.45, 0.45, 0.2)]
# Bump: (net, segment index in its first path, side +1 right/-1 left) used to stretch the
# shorter member of each pair.
PAIRS = [(("USB3_TXN_C", "USB3_TXN"), ("USB3_TXP_C", "USB3_TXP"), 0.05),
         (("USB3_RXN",), ("USB3_RXP",), 0.05),
         (("USB3_DM",), ("USB3_DP",), 0.25)]
# net -> (path index, segment index, side +1/-1 relative to travel, max bump height)
BUMP_AT = {"USB3_RXP": (0, 4, -1, 0.8), "USB3_RXN": (0, 1, +1, 0.3), "USB3_TXP": (0, 4, -1, 0.3),
           "USB3_TXN": (0, 5, +1, 0.8), "USB3_DM": (1, 0, -1, 2.1)}


def seglen(pts):
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def netlen(net):
    return sum(seglen(p) for _, p in PATHS.get(net, [])) + CAP.get(net, 0.0)


def bump(pts, i, side, extra, teeth=1, hmax=None):
    """Replace straight segment i with `teeth` 45-degree bumps bulging sideways by h. Each
    tooth (up h, flat 0.3, down h) adds 2h(sqrt2 - 1) mm."""
    a, b = pts[i], pts[i + 1]
    L = math.dist(a, b)
    k = 2 * (math.sqrt(2) - 1)
    if hmax:
        teeth = max(teeth, math.ceil(extra / (k * hmax)))
    h = extra / (k * teeth)
    gap = 0.5                                      # between teeth: 2W
    pitch = 2 * h + 0.3 + gap
    span = teeth * pitch - gap
    assert span < L - 0.2, ("segment too short for the bump", a, b, h, teeth)
    ux, uy = (b[0] - a[0]) / L, (b[1] - a[1]) / L
    nx, ny = -uy * side, ux * side
    out = []
    t = (L - span) / 2
    for _ in range(teeth):
        p1 = (a[0] + ux * t, a[1] + uy * t)
        p2 = (p1[0] + ux * h + nx * h, p1[1] + uy * h + ny * h)
        p3 = (p2[0] + ux * 0.3, p2[1] + uy * 0.3)
        p4 = (p3[0] + ux * h - nx * h, p3[1] + uy * h - ny * h)
        out += [p1, p2, p3, p4]
        t += pitch
    return pts[:i + 1] + out + pts[i + 1:]


def match():
    for n_nets, p_nets, tol in PAIRS:
        ln = sum(netlen(n) for n in n_nets)
        lp = sum(netlen(n) for n in p_nets)
        if abs(ln - lp) <= tol:
            continue
        short = p_nets[-1] if lp < ln else n_nets[-1]
        where = BUMP_AT.get(short)
        assert where, ("no bump site for", short, ln, lp)
        pi, si, side, hmax = where
        layer, pts = PATHS[short][pi]
        PATHS[short][pi] = (layer, bump(pts, si, side, abs(ln - lp), hmax=hmax))


def main():
    match()
    board = pcbnew.LoadBoard(str(PCB))
    nets = board.GetNetsByName()
    for t in [t for t in board.GetTracks()]:
        board.Remove(t)
    for z in [z for z in board.Zones()]:
        board.Remove(z)
    for net, paths in PATHS.items():
        for layer, pts in paths:
            for a, b in zip(pts, pts[1:]):
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(OX + a[0]), pcbnew.FromMM(OY + a[1])))
                t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(OX + b[0]), pcbnew.FromMM(OY + b[1])))
                t.SetWidth(pcbnew.FromMM(W if net != "GND" else 0.2))
                t.SetLayer(layer)
                t.SetNet(nets[net])
                board.Add(t)
    for net, x, y, d, drill in VIAS:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(OX + x), pcbnew.FromMM(OY + y)))
        v.SetWidth(pcbnew.FromMM(d))
        v.SetDrill(pcbnew.FromMM(drill))
        v.SetNet(nets[net])
        board.Add(v)
    zone = pcbnew.ZONE(board)
    zone.SetLayer(pcbnew.In1_Cu)
    zone.SetNet(nets["GND"])
    zone.SetLocalClearance(pcbnew.FromMM(0.2))
    zone.SetMinThickness(pcbnew.FromMM(0.2))
    ol = zone.Outline()
    ol.NewOutline()
    for x, y in ((0.5, 0.5), (99.5, 0.5), (99.5, 79.5), (0.5, 79.5)):
        ol.Append(pcbnew.FromMM(OX + x), pcbnew.FromMM(OY + y))
    board.Add(zone)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(PCB))
    for n_nets, p_nets, tol in PAIRS:
        ln = sum(netlen(n) for n in n_nets)
        lp = sum(netlen(n) for n in p_nets)
        print(f"{'+'.join(n_nets):22s} {ln:7.3f}  {'+'.join(p_nets):22s} {lp:7.3f}  "
              f"skew {abs(ln - lp):.3f} (limit {tol})")
        assert abs(ln - lp) <= tol + 1e-6


if __name__ == "__main__":
    main()
