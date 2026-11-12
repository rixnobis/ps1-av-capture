#!/usr/bin/env python3
"""Extract the dev-board geometry the hat PCB is built from, into tools/mg_dxf_geometry.json.

Run with a Python that has ezdxf:  python -I extract_mg_dxf.py board.dxf tools/mg_dxf_geometry.json
(build_hat_pcb.py reads the JSON, so the build itself does not need ezdxf or the DXF.)
Coordinates are DXF mm, y up. Output: outline bbox, header pad-1 square pads, round holes
(arc-drawn circles on Multi-Layer, diameter >= 3.2 mm) and silkscreen clusters.
"""
import json
import sys

import ezdxf

d = ezdxf.readfile(sys.argv[1])
m = d.modelspace()
out = {}
o = next(iter(m.query('LWPOLYLINE[layer=="Board-Outline-Layer"]')))
xs = [p[0] for p in o.get_points()]
ys = [p[1] for p in o.get_points()]
out["outline_bbox"] = [min(xs), min(ys), max(xs), max(ys)]
sq, holes = [], set()
for e in m.query('LWPOLYLINE[layer=="Multi-Layer"]'):
    pts = [q[:2] for q in e.get_points()]
    px = [p[0] for p in pts]
    py = [p[1] for p in pts]
    w, h = max(px) - min(px), max(py) - min(py)
    if len(pts) == 4 and abs(w - 1.65) < 0.02 and abs(h - 1.65) < 0.02:
        sq.append((round((min(px) + max(px)) / 2, 3), round((min(py) + max(py)) / 2, 3)))
    if min(w, h) < 0.01 and max(w, h) >= 3.2:     # a circle drawn as arcs: chord is a diameter
        holes.add((round((min(px) + max(px)) / 2, 3), round((min(py) + max(py)) / 2, 3), round(max(w, h), 2)))
out["header_square_pads"] = sorted(set(sq), key=lambda p: -p[1])
out["round_features"] = sorted(holes)


def bb(e):
    p = [q[:2] for q in e.get_points()]
    return [min(a for a, b in p), min(b for a, b in p), max(a for a, b in p), max(b for a, b in p)]


def ov(a, b, t=0.3):
    return a[0] < b[2] + t and b[0] < a[2] + t and a[1] < b[3] + t and b[1] < a[3] + t


cl = [bb(e) for e in m.query('LWPOLYLINE[layer=="Top-Silkscreen-Layer"]')]
ch = True
while ch:
    ch, res = False, []
    for b in cl:
        for r in res:
            if ov(r, b):
                r[:] = [min(r[0], b[0]), min(r[1], b[1]), max(r[2], b[2]), max(r[3], b[3])]
                ch = True
                break
        else:
            res.append(b[:])
    cl = res
out["silk_clusters"] = sorted([round(v, 2) for v in b] for b in cl if (b[2] - b[0]) * (b[3] - b[1]) > 12)
json.dump(out, open(sys.argv[2], "w"), indent=1)
print(json.dumps(out)[:1500])
