#!/usr/bin/env python3
"""Routing hygiene checks DRC does not make.

  - any trace vertex where two segments of one net turn by more than 45 degrees
    (interior angle under 135; a 90-degree point is the classic acid trap)
  - any segment shorter than its own width (a router jog, renders as a blob)
  - any segment narrower than its net class width (necked by the router)
  - any segment endpoint that touches no pad, via or other segment of its net
  - any two segments of one net that overlap collinearly (redundant copper)

Prints one line per finding and a summary; exits 1 if anything was found.
usage: check_geometry.py [board.kicad_pcb]
"""
import math
import sys
from collections import defaultdict
from pathlib import Path

import pcbnew

TOL = pcbnew.FromMM(0.005)


def mm(v):
    return pcbnew.ToMM(v)


def on_segment(p, a, b, slack):
    dx, dy = b.x - a.x, b.y - a.y
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(p.x - a.x, p.y - a.y) <= slack
    t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2
    if t < -1e-9 or t > 1 + 1e-9:
        return False
    return math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy) <= slack


def class_width(path, net):
    """Track width of a net's class, read from the project file (the netclass
    object is not reachable through the KiCad 10 SWIG wrapper)."""
    import fnmatch
    import json
    d = json.loads(Path(path).with_suffix(".kicad_pro").read_text())["net_settings"]
    widths = {c["name"]: c["track_width"] for c in d["classes"]}
    for pat in d.get("netclass_patterns", []):
        if fnmatch.fnmatchcase(net, pat["pattern"]):
            return widths[pat["netclass"]]
    return widths["Default"]


def main(path):
    board = pcbnew.LoadBoard(str(path))
    tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    pads = [p for fp in board.GetFootprints() for p in fp.Pads()]
    findings = []

    def where(p):
        return f"({mm(p.x):.3f}, {mm(p.y):.3f})"

    # endpoint census
    ends = defaultdict(list)  # (net, layer, x, y) -> [(track, other_end)]
    for t in tracks:
        for a, b in ((t.GetStart(), t.GetEnd()), (t.GetEnd(), t.GetStart())):
            ends[(t.GetNetCode(), t.GetLayer(), a.x, a.y)].append((t, b))

    for t in tracks:
        net, layer = t.GetNetCode(), t.GetLayer()
        for p in (t.GetStart(), t.GetEnd()):
            if len(ends[(net, layer, p.x, p.y)]) > 1:
                continue
            ok = any(pd.GetNetCode() == net and pd.IsOnLayer(layer) and pd.HitTest(p) for pd in pads)
            ok = ok or any(v.GetNetCode() == net and math.hypot(v.GetPosition().x - p.x,
                           v.GetPosition().y - p.y) <= v.GetWidth(layer) / 2 for v in vias)
            ok = ok or any(o is not t and o.GetNetCode() == net and o.GetLayer() == layer
                           and on_segment(p, o.GetStart(), o.GetEnd(), o.GetWidth() / 2)
                           for o in tracks)
            # an end inside a filled pour of its own net is connected
            ok = ok or any(z.GetNetCode() == net and not z.GetIsRuleArea() and z.IsOnLayer(layer)
                           and z.HitTestFilledArea(layer, p) for z in board.Zones())
            if not ok:
                findings.append(f"DANGLING  {t.GetNetname()} {board.GetLayerName(layer)} end {where(p)}")

    # vertex angles
    for (net, layer, x, y), lst in ends.items():
        if len(lst) < 2:
            continue
        for i in range(len(lst)):
            for j in range(i + 1, len(lst)):
                (t1, b1), (t2, b2) = lst[i], lst[j]
                v1 = (b1.x - x, b1.y - y)
                v2 = (b2.x - x, b2.y - y)
                n1, n2 = math.hypot(*v1), math.hypot(*v2)
                if n1 == 0 or n2 == 0:
                    findings.append(f"ZERO-LEN  {t1.GetNetname()} {board.GetLayerName(layer)} at {where(pcbnew.VECTOR2I(x, y))}")
                    continue
                ang = math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))))
                if ang < 135 - 0.01:
                    findings.append(f"ACUTE {ang:5.1f}deg {t1.GetNetname()} {board.GetLayerName(layer)} at {where(pcbnew.VECTOR2I(x, y))}")

    # micro-segments and necking. A rule area named BGA_ESC is where the .kicad_dru grants a
    # 0.1 mm neckdown; tracks wholly inside it are held to that instead of their class width.
    neck_ok = [z for z in board.Zones() if z.GetIsRuleArea() and z.GetZoneName() == "BGA_ESC"]

    def inside(z, p):
        # an escape route stops ON the box edge, which Contains() calls outside
        bb = z.GetBoundingBox()
        e = pcbnew.FromMM(0.01)
        return bb.GetLeft() - e <= p.x <= bb.GetRight() + e and bb.GetTop() - e <= p.y <= bb.GetBottom() + e

    # micro-segments and necking
    ds = board.GetDesignSettings()
    for t in tracks:
        L = math.hypot(t.GetEnd().x - t.GetStart().x, t.GetEnd().y - t.GetStart().y)
        if L < t.GetWidth():
            findings.append(f"SHORT {mm(L):.3f}mm {t.GetNetname()} {board.GetLayerName(t.GetLayer())} at {where(t.GetStart())}")
        want = pcbnew.FromMM(class_width(path, t.GetNetname()))
        if any(inside(z, t.GetStart()) and inside(z, t.GetEnd()) for z in neck_ok):
            want = min(want, pcbnew.FromMM(0.1))     # escape-box neckdown, .kicad_dru BGA_ESC
        if t.GetWidth() < want - TOL:
            findings.append(f"NECKED {mm(t.GetWidth()):.2f}<{mm(want):.2f}mm {t.GetNetname()} "
                            f"{board.GetLayerName(t.GetLayer())} at {where(t.GetStart())}")

    # collinear overlap
    for i, a in enumerate(tracks):
        for b in tracks[i + 1:]:
            if a.GetNetCode() != b.GetNetCode() or a.GetLayer() != b.GetLayer():
                continue
            inside = [p for p in (b.GetStart(), b.GetEnd()) if on_segment(p, a.GetStart(), a.GetEnd(), TOL)]
            inside += [p for p in (a.GetStart(), a.GetEnd()) if on_segment(p, b.GetStart(), b.GetEnd(), TOL)]
            uniq = {(p.x, p.y) for p in inside}
            if len(uniq) >= 2:
                findings.append(f"OVERLAP   {a.GetNetname()} {board.GetLayerName(a.GetLayer())} near {where(a.GetStart())}")

    for f in findings:
        print(f)
    print(f"check_geometry: {len(tracks)} segments, {len(vias)} vias, {len(pads)} pads; "
          f"{len(findings)} finding(s)")
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else
                  Path(__file__).resolve().parent.parent / "ps1-av-capture.kicad_pcb"))
