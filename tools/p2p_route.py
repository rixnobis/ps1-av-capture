#!/usr/bin/env python3
"""Point-to-point step: join what the earlier stages left apart, net by net, on F.Cu and B.Cu.

Runs after gpif_route.py on the same board. For each net in ORDER, the board's own
connectivity gives the islands of copper; the smallest island is routed to the nearest copper
of another one (or, on a plane net, to a via dropped into the main fill of its plane) until
the net is whole or a route fails. Supplies keep their class width and neck down only where a
pad needs it. A* on a 0.05 mm grid against exact distance fields (per layer: nearest and
nearest-other-net copper), so a cell is free for a net when its centre clears every other net
by clearance + half width + half a diagonal step. Turns are 45 degrees only and every run
after a turn, via or width change is at least one track width long. Every finished segment
and via is then re-checked against the exact copper geometry before it is added. Bare FX3
supply balls are first dogboned to a via in a free diagonal gap (ESCAPE).

usage: p2p_route.py [-v] [--board PCB] [--only NET[,NET..]] [--max-exp N]
Prints per net the islands before -> after and why each failed route failed (-v: every search).
"""
import fnmatch
import heapq
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import pcbnew
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_usb import PCB, OX, OY, V3_CHANNELS, footprints, tracks  # noqa: E402

CHANNELS = [(min(x for x, _ in r) + OX, min(y for _, y in r) + OY,
             max(x for x, _ in r) + OX, max(y for _, y in r) + OY) for r in V3_CHANNELS]

G = 0.05
X0, Y0, BW, BH, RAD = 100.0, 100.0, 100.0, 80.0, 1.5     # board outline (build_pcb.outline)
NX, NY = int(round(BW / G)) + 1, int(round(BH / G)) + 1
CLR, CLR_BOX, EDGE = 0.15, 0.1, 0.4      # netclass / BGA_ESC clearance, copper to edge
M = G * math.sqrt(0.5) + 0.002           # a step's centre line strays this far from cell centres
VIA_D, VIA_DRILL, HOLE2HOLE = 0.45, 0.2, 0.4
RC = 0.5                                 # distance fields are exact up to this far from copper
BIG = 9.0
LAYERS = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
ROUTE = [0, 3]                           # F.Cu, B.Cu: the inner layers are planes
PLANE_LAYER = {"GND": 1, "+3V3": 2, "+1V2": 2}
TURN, VIA_COST, SWITCH = 0.15, 1.5, 0.3
EXP_CAP = 1500000
VERBOSE = "-v" in sys.argv
DIRS = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1)]

# FX3 balls the escape stages leave bare (bga_route skips supply nets; the FSLC straps are
# tied ball to ball): dogboned before anything else takes the gaps
ESCAPE = ["FX3_AVDD", "FX3_CVDDQ", "FX3_U3TX", "FX3_U3RX", "VBUS", "FX3_FSLC"]
# Route order: plane repairs, supplies, then the point-to-point signals. Patterns (fnmatch).
ORDER = [
    # plane nets the gpif In2 routes cut off, and pads fanout could not serve
    "GND", "+3V3", "+1V2",
    # supplies the signal routers leave alone, the tightest first: the FX3 balls (with its
    # SuperSpeed PHY supplies; the SS pairs themselves are route_usb.py's), then the TVP7002's
    # pins, then the rails that have room. Run in the plain order, the long rails took the
    # FX3 row A exits and the TVP decap channels before those nets got there.
    "FX3_AVDD", "FX3_CVDDQ", "FX3_U3TX", "FX3_U3RX",
    "TVP_AVDD", "TVP_PLLVDD", "+1V9D", "+3V3A",
    "+1V9A", "OSC_VDD", "+2V5", "VBUS", "VBUS_AUD", "+3V3_AUD", "SW_*",
    # crystals, I2C, the FSLC straps, the TVP7002 side of every 22R, reset/LED/PMODE
    "FX3_XI", "FX3_XO", "AUD_XI", "AUD_XO", "I2C_SDA", "I2C_SCL", "FX3_FSLC",
    # The four west FID..SOG nets share one tight escape south of U1's pin 22..25 column; a net
    # routed first takes the lane and walls in the later ones (HSOUT's pin sealed otherwise). Route
    # them innermost-corner first -- SOG, HS, VS, FID -- so each reaches its 22R before the next lays
    # copper across the pin it needs.
    "TVP_DATACLK", "TVP_[RGB][2-9]", "TVP_SOGOUT", "TVP_HSOUT", "TVP_VSOUT", "TVP_FIDOUT",
    "TVP_RESETn", "FX3_RESETn", "LED_*", "PWR_LED_K", "FX3_PMODE*",
]


def mm(v):
    return pcbnew.ToMM(v)


def nm(v):
    return int(round(v * 1e6))


# ---- exact geometry -------------------------------------------------------------------------
def pt_seg(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def seg_seg(a, b, c, d):
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return 0.0
    return min(pt_seg(*a, *c, *d), pt_seg(*b, *c, *d), pt_seg(*c, *a, *b), pt_seg(*d, *a, *b))


def pt_in_poly(x, y, pts):
    c = False
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            c = not c
    return c


def ob_seg_dist(ob, a, b):
    """Distance from segment a-b (centre line) to obstacle copper."""
    k = ob[0]
    if k == "seg":
        return seg_seg(a, b, (ob[1], ob[2]), (ob[3], ob[4])) - ob[5]
    if k == "circ":
        return pt_seg(ob[1], ob[2], *a, *b) - ob[3]
    pts = ob[1]
    if pt_in_poly(*a, pts) or pt_in_poly(*b, pts):
        return 0.0
    return min(seg_seg(a, b, pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts)))


def edge_sdf(x, y):
    """Signed distance to the rounded board outline, positive inside (numpy or float)."""
    cx, cy = X0 + BW / 2, Y0 + BH / 2
    qx = np.abs(x - cx) - (BW / 2 - RAD)
    qy = np.abs(y - cy) - (BH / 2 - RAD)
    out = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - RAD
    return -out


# ---- vectorised distances over a grid window -------------------------------------------------
def grid_seg(X, Y, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return np.hypot(X - ax, Y - ay)
    t = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0, 1)
    return np.hypot(X - ax - t * dx, Y - ay - t * dy)


def grid_dist(ob, X, Y):
    k = ob[0]
    if k == "seg":
        return grid_seg(X, Y, *ob[1:5]) - ob[5]
    if k == "circ":
        return np.hypot(X - ob[1], Y - ob[2]) - ob[3]
    pts = ob[1]
    d = np.full(X.shape, BIG)
    inside = np.zeros(X.shape, bool)
    n = len(pts)
    for i in range(n):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        d = np.minimum(d, grid_seg(X, Y, x1, y1, x2, y2))
        if y1 != y2:
            cross = ((y1 > Y) != (y2 > Y)) & (X < x1 + (Y - y1) * (x2 - x1) / (y2 - y1))
            inside ^= cross
    return np.where(inside, -d, d)


def ob_bbox(ob):
    k = ob[0]
    if k == "seg":
        return (min(ob[1], ob[3]) - ob[5], min(ob[2], ob[4]) - ob[5],
                max(ob[1], ob[3]) + ob[5], max(ob[2], ob[4]) + ob[5])
    if k == "circ":
        return (ob[1] - ob[3], ob[2] - ob[3], ob[1] + ob[3], ob[2] + ob[3])
    xs = [p[0] for p in ob[1]]
    ys = [p[1] for p in ob[1]]
    return (min(xs), min(ys), max(xs), max(ys))


def window(bb, pad):
    i0 = max(0, int(math.floor((bb[0] - pad - X0) / G)))
    j0 = max(0, int(math.floor((bb[1] - pad - Y0) / G)))
    i1 = min(NX - 1, int(math.ceil((bb[2] + pad - X0) / G)))
    j1 = min(NY - 1, int(math.ceil((bb[3] + pad - Y0) / G)))
    return i0, j0, i1, j1


def grids(w):
    i0, j0, i1, j1 = w
    xs = X0 + np.arange(i0, i1 + 1) * G
    ys = Y0 + np.arange(j0, j1 + 1) * G
    return np.meshgrid(xs, ys)


class Fields:
    """Per copper layer: distance to the nearest copper (d1, its net n1) and to the nearest
    copper of any other net (d2). The distance to other-net copper for net k is d2 where
    n1 == k, else d1."""

    def __init__(self, box):
        self.d1 = [np.full((NY, NX), BIG, np.float32) for _ in LAYERS]
        self.d2 = [np.full((NY, NX), BIG, np.float32) for _ in LAYERS]
        self.n1 = [np.full((NY, NX), -1, np.int32) for _ in LAYERS]
        self.dpad = [np.full((NY, NX), BIG, np.float32) for _ in LAYERS]   # any pad, any net
        self.dhole = np.full((NY, NX), BIG, np.float32)
        self.box = box
        self.bw = window(box, 0)
        bi0, bj0, bi1, bj1 = self.bw
        shp = (bj1 - bj0 + 1, bi1 - bi0 + 1)
        # inside BGA_ESC the rule is 0.1 mm to copper that also touches the area: these hold
        # distances with that copper moved 0.05 mm further away, so one threshold serves
        self.b1 = [np.full(shp, BIG, np.float32) for _ in LAYERS]
        self.b2 = [np.full(shp, BIG, np.float32) for _ in LAYERS]
        self.bn = [np.full(shp, -1, np.int32) for _ in LAYERS]
        self.obs = [[] for _ in LAYERS]       # (net id, ob, bbox, touches box)
        self.holes = []                       # (x, y, r)
        Xa, Ya = grids((0, 0, NX - 1, NY - 1))
        self.edge = edge_sdf(Xa, Ya).astype(np.float32)

    @staticmethod
    def _merge(d1, n1, d2, d, k):
        same = n1 == k
        lt = (~same) & (d < d1)
        nd2 = np.where(same, d2, np.where(lt, d1, np.minimum(d2, d)))
        nd1 = np.where(same, np.minimum(d1, d), np.where(lt, d, d1))
        nn = np.where(lt, k, n1)
        d1[...] = nd1
        d2[...] = nd2
        n1[...] = nn

    def touches_box(self, ob):
        b = self.box
        rect = ("poly", [(b[0], b[1]), (b[2], b[1]), (b[2], b[3]), (b[0], b[3])])
        bb = ob_bbox(ob)
        if bb[2] < b[0] or bb[0] > b[2] or bb[3] < b[1] or bb[1] > b[3]:
            return False
        if ob[0] == "poly":
            if any(b[0] <= x <= b[2] and b[1] <= y <= b[3] for x, y in ob[1]):
                return True
            return any(pt_in_poly(x, y, ob[1]) for x, y in rect[1])
        if ob[0] == "circ":
            a = (ob[1], ob[2])
            return ob_seg_dist(rect, a, a) <= ob[3]
        return ob_seg_dist(rect, (ob[1], ob[2]), (ob[3], ob[4])) <= ob[5]

    def add(self, li, k, ob, pad=False):
        bb = ob_bbox(ob)
        tb = self.touches_box(ob)
        self.obs[li].append((k, ob, bb, tb))
        w = window(bb, RC)
        i0, j0, i1, j1 = w
        X, Y = grids(w)
        d = grid_dist(ob, X, Y).astype(np.float32)
        sl = (slice(j0, j1 + 1), slice(i0, i1 + 1))
        self._merge(self.d1[li][sl], self.n1[li][sl], self.d2[li][sl], d, k)
        if pad:
            self.dpad[li][sl] = np.minimum(self.dpad[li][sl], d)
        bi0, bj0, bi1, bj1 = self.bw
        ci0, cj0, ci1, cj1 = max(i0, bi0), max(j0, bj0), min(i1, bi1), min(j1, bj1)
        if ci0 <= ci1 and cj0 <= cj1:
            dd = d[cj0 - j0:cj1 - j0 + 1, ci0 - i0:ci1 - i0 + 1] + (CLR - CLR_BOX if tb else 0.0)
            bs = (slice(cj0 - bj0, cj1 - bj0 + 1), slice(ci0 - bi0, ci1 - bi0 + 1))
            self._merge(self.b1[li][bs], self.bn[li][bs], self.b2[li][bs], dd, k)

    def add_hole(self, x, y, r):
        self.holes.append((x, y, r))
        w = window((x - r, y - r, x + r, y + r), 1.0)
        i0, j0, i1, j1 = w
        X, Y = grids(w)
        sl = (slice(j0, j1 + 1), slice(i0, i1 + 1))
        self.dhole[sl] = np.minimum(self.dhole[sl], (np.hypot(X - x, Y - y) - r).astype(np.float32))

    def other(self, li, k, w):
        i0, j0, i1, j1 = w
        sl = (slice(j0, j1 + 1), slice(i0, i1 + 1))
        return np.where(self.n1[li][sl] == k, self.d2[li][sl], self.d1[li][sl])

    def other_box(self, li, k, w):
        """Box-rule distances over window w (BIG outside the box)."""
        i0, j0, i1, j1 = w
        out = np.full((j1 - j0 + 1, i1 - i0 + 1), -BIG, np.float32)
        bi0, bj0, bi1, bj1 = self.bw
        ci0, cj0, ci1, cj1 = max(i0, bi0), max(j0, bj0), min(i1, bi1), min(j1, bj1)
        if ci0 <= ci1 and cj0 <= cj1:
            bs = (slice(cj0 - bj0, cj1 - bj0 + 1), slice(ci0 - bi0, ci1 - bi0 + 1))
            v = np.where(self.bn[li][bs] == k, self.b2[li][bs], self.b1[li][bs])
            out[cj0 - j0:cj1 - j0 + 1, ci0 - i0:ci1 - i0 + 1] = v
        return out

    def exact_ok(self, li, k, a, b, hw, inbox):
        """Exact clearance of a new segment (or via when a == b) on layer li."""
        lo = (min(a[0], b[0]) - hw - 1, min(a[1], b[1]) - hw - 1,
              max(a[0], b[0]) + hw + 1, max(a[1], b[1]) + hw + 1)
        for n, ob, bb, tb in self.obs[li]:
            if n == k or bb[2] < lo[0] or bb[0] > lo[2] or bb[3] < lo[1] or bb[1] > lo[3]:
                continue
            need = (CLR_BOX if (tb and inbox) else CLR) + hw + 0.0005
            dd = ob_seg_dist(ob, a, b)
            if dd < need:
                self.why = f"{ob[0]} net {n} at {bb[0]:.3f},{bb[1]:.3f} gap {dd - hw:.4f}"
                return False
        for p in (a, b):
            if edge_sdf(p[0], p[1]) < EDGE + hw + 0.0005:
                self.why = "edge"
                return False
        return True


def pad_polys(p, layer):
    """Pad copper outline on a layer (outside error: never smaller than the pad)."""
    if p.GetShape(layer) == pcbnew.PAD_SHAPE_CIRCLE:
        q = p.GetPosition()
        return [("circ", mm(q.x), mm(q.y), mm(p.GetSize(layer).x) / 2)]
    ps = p.GetEffectivePolygon(layer, pcbnew.ERROR_OUTSIDE)
    out = []
    for i in range(ps.OutlineCount()):
        ol = ps.Outline(i)
        out.append(("poly", [(mm(ol.CPoint(j).x), mm(ol.CPoint(j).y)) for j in range(ol.PointCount())]))
    return out


def class_width(pro, net):
    d = pro["net_settings"]
    widths = {c["name"]: c["track_width"] for c in d["classes"]}
    for pat in d.get("netclass_patterns", []):
        if fnmatch.fnmatchcase(net, pat["pattern"]):
            return pat["netclass"], widths[pat["netclass"]]
    return "Default", widths["Default"]


def fill_mask(ps, i):
    """Raster of filled island i of a SHAPE_POLY_SET (outline and holes, even-odd)."""
    chains = [ps.Outline(i)] + [ps.Hole(i, h) for h in range(ps.HoleCount(i))]
    edges = []
    for ch in chains:
        n = ch.PointCount()
        for j in range(n):
            a, b = ch.CPoint(j), ch.CPoint((j + 1) % n)
            edges.append((mm(a.x), mm(a.y), mm(b.x), mm(b.y)))
    E = np.array(edges)
    mask = np.zeros((NY, NX), bool)
    xs = X0 + np.arange(NX) * G
    ylo, yhi = min(E[:, 1].min(), E[:, 3].min()), max(E[:, 1].max(), E[:, 3].max())
    for j in range(max(0, int((ylo - Y0) / G)), min(NY, int((yhi - Y0) / G) + 2)):
        y = Y0 + j * G
        e = E[(E[:, 1] > y) != (E[:, 3] > y)]
        if len(e) == 0:
            continue
        xc = np.sort(e[:, 0] + (y - e[:, 1]) * (e[:, 2] - e[:, 0]) / (e[:, 3] - e[:, 1]))
        for x1, x2 in zip(xc[0::2], xc[1::2]):
            mask[j, np.searchsorted(xs, x1):np.searchsorted(xs, x2)] = True
    return mask


def main():
    args = sys.argv[1:]
    only = None
    if "--only" in args:
        only = args[args.index("--only") + 1].split(",")
    cap = int(args[args.index("--max-exp") + 1]) if "--max-exp" in args else EXP_CAP
    path = Path(args[args.index("--board") + 1]) if "--board" in args else PCB
    t0 = time.time()
    board = pcbnew.LoadBoard(str(path))
    pro = json.loads(path.with_suffix(".kicad_pro").read_text())
    nets = board.GetNetsByName()
    esc = [z for z in board.Zones() if z.GetIsRuleArea() and z.GetZoneName() == "BGA_ESC"]
    bb = esc[0].GetBoundingBox()
    box = (mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom()))
    F = Fields(box)
    code = {}

    def nid(n):
        return code.setdefault(n, len(code) + 1)

    # ---- copper census ----
    own = {}           # net -> list of (layer index, ob, kind, item uuid)
    ends = set()       # (layer index, x nm, y nm) of every track end: no new segment starts there
    for fp in footprints(board):
        for p in fp.Pads():
            n = p.GetNetname()
            k = nid(n) if n else 0
            if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
                q = p.GetPosition()
                r = mm(p.GetDrillSize().x) / 2
                F.add_hole(mm(q.x), mm(q.y), r)
                for li in range(4):     # copper keeps min_hole_clearance (0.2) from the hole
                    F.add(li, -2, ("circ", mm(q.x), mm(q.y), r + 0.2 - CLR), pad=True)
                continue
            if p.HasHole():
                q = p.GetPosition()
                F.add_hole(mm(q.x), mm(q.y), mm(p.GetDrillSize().x) / 2)
            for li, L in enumerate(LAYERS):
                if not p.IsOnLayer(L):
                    continue
                for ob in pad_polys(p, L):
                    F.add(li, k, ob, pad=True)
                    if n:
                        own.setdefault(n, []).append((li, ob, "pad", p.m_Uuid.AsString()))
    for t in tracks(board):
        n = t.GetNetname()
        k = nid(n)
        if t.GetClass() == "PCB_VIA":
            q = t.GetPosition()
            F.add_hole(mm(q.x), mm(q.y), mm(t.GetDrillValue()) / 2)
            for li, L in enumerate(LAYERS):
                ob = ("circ", mm(q.x), mm(q.y), mm(t.GetWidth(L)) / 2)
                F.add(li, k, ob)
                own.setdefault(n, []).append((li, ob, "via", t.m_Uuid.AsString()))
        else:
            li = LAYERS.index(t.GetLayer())
            s, e = t.GetStart(), t.GetEnd()
            ob = ("seg", mm(s.x), mm(s.y), mm(e.x), mm(e.y), mm(t.GetWidth()) / 2)
            F.add(li, k, ob)
            own.setdefault(n, []).append((li, ob, "track", t.m_Uuid.AsString()))
            ends.add((li, s.x, s.y))
            ends.add((li, e.x, e.y))
    print(f"p2p: fields built in {time.time() - t0:.1f} s")

    board.BuildConnectivity()
    conn = board.GetConnectivity()

    def clusters(net):
        """Island index per item uuid, from the board's connectivity."""
        items = [p for fp in footprints(board) for p in fp.Pads() if p.GetNetname() == net]
        items += [t for t in tracks(board) if t.GetNetname() == net]
        lab = {}
        nxt = 0
        for it in items:
            u = it.m_Uuid.AsString()
            if u in lab:
                continue
            group = [u] + [c.m_Uuid.AsString() for c in conn.GetConnectedItems(it)
                           if c.GetClass() != "ZONE"]
            known = {lab[g] for g in group if g in lab}
            c = min(known) if known else nxt
            if not known:
                nxt += 1
            for g in group:
                lab[g] = c
            for o in known - {c}:
                for g, v in list(lab.items()):
                    if v == o:
                        lab[g] = c
        return lab

    # ---- per-net routing ----
    added = {"tracks": 0, "vias": 0}
    report = []
    newtracks = []

    def route(net):
        k = nid(net)
        cls, cw = class_width(pro, net)
        if cls == "Power":
            modes = [cw, 0.25]
        elif cls == "Default":
            modes = [cw, 0.127]
        else:
            modes = [cw]
        modes.append(0.1)                                   # inside BGA_ESC only
        cost_m = [1.0] + [3.0] * (len(modes) - 2) + [3.0]
        lmin = [int(math.ceil(w / G)) + 1 for w in modes]
        lab = clusters(net)
        for u, pu in attach.items():                # dogbones this run added (escape_balls)
            if pu in lab:
                lab[u] = lab[pu]
        items = own.get(net, [])
        # cluster per raster item: pads may be stacked (two uuids, one place)
        cl_items = {}
        for li, ob, kind, u in items:
            if u in lab:
                cl_items.setdefault(lab[u], []).append((li, ob, kind))
        plane = PLANE_LAYER.get(net)
        drop = None
        main_c = None
        if plane is not None:
            # every fill of the net on its plane layer; same-net fills that overlap are one
            # copper, so the main island is the largest connected part of their union
            mask = np.zeros((NY, NX), bool)
            for z in board.Zones():
                if z.GetIsRuleArea() or z.GetNetname() != net or not z.IsOnLayer(LAYERS[plane]):
                    continue
                ps = z.GetFilledPolysList(LAYERS[plane])
                for i in range(ps.OutlineCount()):
                    mask |= fill_mask(ps, i)
            lbl, nl = ndimage.label(mask, structure=np.ones((3, 3)))
            sizes = ndimage.sum(mask, lbl, range(1, nl + 1))
            mask = lbl == (int(np.argmax(sizes)) + 1)
            # a via there lands well inside the fill: eroded by the via radius and a margin
            r = int(math.ceil((VIA_D / 2 + 0.1) / G))
            yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
            drop = ndimage.binary_erosion(mask, structure=(xx * xx + yy * yy) <= r * r)
            # the main island: the cluster holding a via whose centre the fill covers
            for c, lst in cl_items.items():
                for li, ob, kind in lst:
                    if kind == "via" and li == plane:
                        j, i = int(round((ob[2] - Y0) / G)), int(round((ob[1] - X0) / G))
                        if mask[j, i]:
                            main_c = c
                            break
                if main_c is not None:
                    break
        n_cl = len(cl_items)
        start_cl = n_cl if plane is None else n_cl - (main_c is not None)
        fails = []
        done = 0
        tried = set()
        while True:
            groups = sorted(cl_items, key=lambda c: (len(cl_items[c]), c))
            if plane is None and len(groups) <= 1:
                break
            if plane is not None:
                groups = [c for c in groups if c != main_c]
                if not groups:
                    break
            src = next((c for c in groups if c not in tried), None)
            if src is None:
                break
            tried.add(src)
            ok, why = connect(net, k, modes, cost_m, lmin, src, cl_items, plane, drop, main_c)
            if ok is None:
                b = [ob_bbox(ob) for _, ob, _ in cl_items[src]]
                fails.append(f"from ({(min(q[0] for q in b) + max(q[2] for q in b)) / 2:.2f}, "
                             f"{(min(q[1] for q in b) + max(q[3] for q in b)) / 2:.2f}): {why}")
                if why.startswith("no path") and len(cl_items) == 2 and plane is None:
                    break           # walled in: the other end cannot reach it either
                continue
            tgt = ok
            if tgt == "plane":
                tgt = main_c
            cl_items[tgt].extend(cl_items.pop(src))
            if src == main_c:
                main_c = tgt
            done += 1
            tried.discard(tgt)
        left = len(cl_items) - (1 if plane is None or main_c is not None else 0)
        report.append(f"{net:12s} islands {start_cl} -> {left}  routed {done}"
                      + (f"  failed: {'; '.join(fails)}" if fails else ""))
        print(report[-1], f"({time.time() - t0:.0f} s)", flush=True)

    def connect(net, k, modes, cost_m, lmin, src, cl_items, plane, drop, main_c):
        # own-copper rasters: source cells (starts) and target cells (goals) per route layer
        src_items = cl_items[src]
        tgt_items = [(c, it) for c, lst in cl_items.items() if c != src for it in lst
                     if plane is None or c == main_c]
        foreign = [it for c, lst in cl_items.items() if c != src and not (plane is None or c == main_c)
                   for it in lst]
        sb = [ob_bbox(ob) for _, ob, _ in src_items]
        sbb = (min(b[0] for b in sb), min(b[1] for b in sb), max(b[2] for b in sb), max(b[3] for b in sb))
        # nearest target distance sets the window
        dmin = BIG * 100
        for c, (li, ob, kind) in tgt_items:
            b = ob_bbox(ob)
            dx = max(b[0] - sbb[2], sbb[0] - b[2], 0)
            dy = max(b[1] - sbb[3], sbb[1] - b[3], 0)
            dmin = min(dmin, math.hypot(dx, dy))
        if plane is not None:
            dmin = min(dmin, 3.0)
        why = "no target"
        # widen the window, then lean harder on the heuristic (long runs past the bundle)
        for margin, wt in ((dmin + 3.0, 1.2), (dmin + 12.0, 1.5), (dmin + 12.0, 3.0)):
            w = window(sbb, margin)
            t1 = time.time()
            res = astar(net, k, modes, cost_m, lmin, w, src_items, tgt_items, foreign, plane, drop, wt)
            if VERBOSE:
                print(f"    {net} from ({(sbb[0] + sbb[2]) / 2:.1f}, {(sbb[1] + sbb[3]) / 2:.1f}) "
                      f"window {w[2] - w[0]}x{w[3] - w[1]} w{wt}: {res[1]} [{time.time() - t1:.1f} s]", flush=True)
            if res[0] is not None:
                return res
            why = res[1]
            if not why.startswith("search cap") and wt > 1.2:
                break                                # exhausted, not capped: weight will not help
        return None, why

    def astar(net, k, modes, cost_m, lmin, w, src_items, tgt_items, foreign, plane, drop, wt):
        i0, j0, i1, j1 = w
        WX, WY = i1 - i0 + 1, j1 - j0 + 1
        NC = WX * WY
        X, Y = grids(w)
        edge = F.edge[j0:j1 + 1, i0:i1 + 1]

        def sub(ob, pad):
            """Distance to ob over the part of the window within pad of it: (slice, d) or None."""
            b = ob_bbox(ob)
            a0, b0, a1, b1 = window(b, pad)
            a0, b0, a1, b1 = max(a0, i0), max(b0, j0), min(a1, i1), min(b1, j1)
            if a0 > a1 or b0 > b1:
                return None
            sl = (slice(b0 - j0, b1 - j0 + 1), slice(a0 - i0, a1 - i0 + 1))
            return sl, grid_dist(ob, X[sl], Y[sl])

        nm_ = len(modes)
        free = {}
        for ri, li in enumerate(ROUTE):
            D = F.other(li, k, w)
            Db = F.other_box(li, k, w)
            inbox = ((X > F.box[0] + 0.06) & (X < F.box[2] - 0.06) & (Y > F.box[1] + 0.06) & (Y < F.box[3] - 0.06))
            for m, wd in enumerate(modes):
                hw = wd / 2
                ok = (edge >= EDGE + hw + M)
                if m == nm_ - 1:
                    ok &= inbox & (Db >= CLR + hw + M)        # box rule, held as CLR on b-fields
                else:
                    ok &= D >= CLR + hw + M
                free[(ri, m)] = ok
        # same-net copper of other islands (plane nets): not to be run over
        for li, ob, kind in foreign:
            r = sub(ob, RC) if li in ROUTE else None
            if r:
                ri = ROUTE.index(li)
                for m in range(nm_):
                    free[(ri, m)][r[0]] &= r[1] >= CLR + modes[m] / 2 + M
        start = [np.zeros((WY, WX), bool) for _ in ROUTE]
        goal = [np.full((WY, WX), -1, np.int32) for _ in ROUTE]
        def on(ob, kind):
            """Cells a new track may start or end on: inside the pad or via, or on the body of
            a track between its ends (check_geometry does not count a track's round end cap)."""
            r = sub(ob, 0.1)
            if not r:
                return None
            sl, d = r
            m = d <= -0.01
            if kind == "track":
                ax, ay, bx, by = ob[1:5]
                L2 = (bx - ax) ** 2 + (by - ay) ** 2
                if L2 == 0:
                    return None
                t = ((X[sl] - ax) * (bx - ax) + (Y[sl] - ay) * (by - ay)) / L2
                m &= (t >= 0.001) & (t <= 0.999)
            return sl, m

        for li, ob, kind in src_items:
            r = on(ob, kind) if li in ROUTE else None
            if r:
                start[ROUTE.index(li)][r[0]] |= r[1]
        for c, (li, ob, kind) in tgt_items:
            r = on(ob, kind) if li in ROUTE else None
            if r:
                g = goal[ROUTE.index(li)][r[0]]
                g[r[1] & (g < 0)] = c
        for ri, li in enumerate(ROUTE):         # never start or end on an existing track end
            for (L, x, y) in ends:
                if L == li:
                    i, j = (x / 1e6 - X0) / G - i0, (y / 1e6 - Y0) / G - j0
                    ii, jj = int(round(i)), int(round(j))
                    if abs(i - ii) < 1e-6 and abs(j - jj) < 1e-6 and 0 <= ii < WX and 0 <= jj < WY:
                        start[ri][jj, ii] = False
                        goal[ri][jj, ii] = -1
        # vias: copper on every layer, hole spacing, no pad underneath, outside the escape box
        vok = edge >= EDGE + VIA_D / 2 + 0.001
        for li in range(4):
            vok &= F.other(li, k, w) >= CLR + VIA_D / 2 + 0.002
        for li in (0, 3):
            vok &= F.dpad[li][j0:j1 + 1, i0:i1 + 1] >= VIA_D / 2 + 0.05
        vok &= F.dhole[j0:j1 + 1, i0:i1 + 1] >= HOLE2HOLE + VIA_DRILL / 2 + 0.002
        # the 1 mm +3V3 channels across the +1V2 island: one foreign via antipad cuts one
        for cx0, cy0, cx1, cy1 in (CHANNELS if net != "+3V3" else []):
            vok &= ~((X > cx0 - 0.45) & (X < cx1 + 0.45) & (Y > cy0 - 0.45) & (Y < cy1 + 0.45))
        for li, ob, kind in foreign:
            r = sub(ob, RC)
            if r:
                vok[r[0]] &= r[1] >= CLR + VIA_D / 2 + 0.002
        dropm = None
        if plane is not None:
            dropm = vok & drop[j0:j1 + 1, i0:i1 + 1]
        anyg = (goal[0] >= 0) | (goal[1] >= 0)
        if dropm is not None:
            anyg |= dropm
        if not anyg.any():
            return None, "no reachable target in window"
        if not (start[0].any() or start[1].any()):
            return None, "source has no free start cell"
        h = ndimage.distance_transform_edt(~anyg) * G
        hl = h.ravel().tolist()
        fr = {key: bytearray(v.ravel().astype(np.uint8).tobytes()) for key, v in free.items()}
        vl = bytearray(vok.ravel().astype(np.uint8).tobytes())
        dl = bytearray(dropm.ravel().astype(np.uint8).tobytes()) if dropm is not None else None
        gl = [g.ravel().tolist() for g in goal]
        steps = [(dx + dy * WX, G * math.hypot(dx, dy), dx, dy) for dx, dy in DIRS]
        NS = 9

        def key(ri, m, d, c):
            return ((ri * nm_ + m) * NS + d) * NC + c

        def unkey(s):
            c = s % NC
            s //= NC
            d = s % NS
            s //= NS
            return s // nm_, s % nm_, d, c

        def run_ok(ri, m, c, d, L):
            """Cells c+step .. c+L*step free; returns the end cell or -1."""
            off, _, dx, dy = steps[d]
            ci, cj = c % WX, c // WX
            ei, ej = ci + dx * L, cj + dy * L
            if not (0 <= ei < WX and 0 <= ej < WY):
                return -1
            f = fr[(ri, m)]
            x = c
            for _ in range(L):
                x += off
                if not f[x]:
                    return -1
            return x

        heap = []
        gsc = {}
        par = {}
        for ri in range(len(ROUTE)):
            for c in np.flatnonzero(start[ri].ravel()).tolist():
                for m in range(nm_):
                    if fr[(ri, m)][c]:
                        s = key(ri, m, 8, c)
                        gsc[s] = 0.0
                        par[s] = None
                        heapq.heappush(heap, (hl[c], 0.0, s))
        if not heap:
            return None, "source cells all blocked"
        exp = 0
        found = None
        while heap:
            f, g, s = heapq.heappop(heap)
            if g > gsc.get(s, 1e18):
                continue
            exp += 1
            if exp > cap:
                return None, f"search cap {cap} hit"
            ri, m, d, c = unkey(s)
            if d != 8 and gl[ri][c] >= 0:
                found = (s, gl[ri][c])
                break
            if dl is not None and dl[c] and fr[(ri, m)][c]:
                found = (s, "plane")              # a stitching via, at a start cell too
                break
            nbrs = []
            if d == 8:
                for nd in range(8):
                    e = run_ok(ri, m, c, nd, lmin[m])
                    if e >= 0:
                        nbrs.append((key(ri, m, nd, e), lmin[m] * steps[nd][1] * cost_m[m]))
            else:
                e = run_ok(ri, m, c, d, 1)
                if e >= 0:
                    nbrs.append((key(ri, m, d, e), steps[d][1] * cost_m[m]))
                for nd in ((d + 1) % 8, (d + 7) % 8):
                    e = run_ok(ri, m, c, nd, lmin[m])
                    if e >= 0:
                        nbrs.append((key(ri, m, nd, e), lmin[m] * steps[nd][1] * cost_m[m] + TURN))
                for nm2 in range(nm_):
                    if nm2 != m:
                        e = run_ok(ri, nm2, c, d, lmin[nm2])
                        if e >= 0:
                            nbrs.append((key(ri, nm2, d, e), lmin[nm2] * steps[d][1] * cost_m[nm2] + SWITCH))
                if vl[c] and m != nm_ - 1:
                    for r2 in range(len(ROUTE)):
                        if r2 != ri and fr[(r2, m)][c]:
                            nbrs.append((key(r2, m, 8, c), VIA_COST))
            for s2, cst in nbrs:
                g2 = g + cst
                if g2 < gsc.get(s2, 1e18):
                    gsc[s2] = g2
                    par[s2] = s
                    heapq.heappush(heap, (g2 + wt * hl[s2 % NC], g2, s2))
        if found is None:
            return None, f"no path ({exp} expansions)"
        # ---- back to geometry ----
        s, tgt = found
        chain = []
        while s is not None:
            chain.append(unkey(s))
            s = par[s]
        chain.reverse()

        def xy(c):
            return (X0 + (i0 + c % WX) * G, Y0 + (j0 + c // WX) * G)

        segs, vias = [], []
        cur = None                                   # (ri, m, d, start cell)
        prev = chain[0]
        for st in chain[1:]:
            ri, m, d, c = st
            pri, pm, pd, pc = prev
            if ri != pri:                            # via at c
                if cur is not None and cur[3] != pc:
                    segs.append((ROUTE[cur[0]], modes[cur[1]], xy(cur[3]), xy(pc)))
                vias.append(xy(c))
                cur = None
            elif cur is None or (d, m) != (cur[2], cur[1]):
                if cur is not None and cur[3] != pc:
                    segs.append((ROUTE[cur[0]], modes[cur[1]], xy(cur[3]), xy(pc)))
                cur = (ri, m, d, pc)
            prev = st
        if cur is not None and cur[3] != prev[3]:
            segs.append((ROUTE[cur[0]], modes[cur[1]], xy(cur[3]), xy(prev[3])))
        if tgt == "plane":
            vias.append(xy(prev[3]))
        # exact acceptance: every segment and via against the copper as it is
        for li, wd, a, b in segs:
            inb = all(F.box[0] <= p[0] <= F.box[2] and F.box[1] <= p[1] <= F.box[3] for p in (a, b))
            if not F.exact_ok(li, k, a, b, wd / 2, inb):
                return None, f"exact check failed at {a}-{b} w{wd}: {F.why}"
        for v in vias:
            for li in range(4):
                if not F.exact_ok(li, k, v, v, VIA_D / 2, False):
                    return None, f"exact via check failed at {v}"
            for hx, hy, hr in F.holes:
                if math.hypot(hx - v[0], hy - v[1]) < hr + VIA_DRILL / 2 + HOLE2HOLE + 0.001:
                    return None, f"hole spacing at {v}"
        # ---- commit ----
        ni = nets[net]
        for li, wd, a, b in segs:
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(nm(a[0]), nm(a[1])))
            t.SetEnd(pcbnew.VECTOR2I(nm(b[0]), nm(b[1])))
            t.SetWidth(nm(wd))
            t.SetLayer(LAYERS[li])
            t.SetNet(ni)
            board.Add(t)
            ob = ("seg", a[0], a[1], b[0], b[1], wd / 2)
            F.add(li, k, ob)
            ends.add((li, nm(a[0]), nm(a[1])))
            ends.add((li, nm(b[0]), nm(b[1])))
            own.setdefault(net, []).append((li, ob, "track", t.m_Uuid.AsString()))
            src_items.append((li, ob, "track"))
            newtracks.append((net, li, a, b))
            added["tracks"] += 1
        for v in vias:
            t = pcbnew.PCB_VIA(board)
            t.SetPosition(pcbnew.VECTOR2I(nm(v[0]), nm(v[1])))
            t.SetWidth(nm(VIA_D))
            t.SetDrill(nm(VIA_DRILL))
            t.SetNet(ni)
            board.Add(t)
            F.add_hole(v[0], v[1], VIA_DRILL / 2)
            for li in range(4):
                ob = ("circ", v[0], v[1], VIA_D / 2)
                F.add(li, k, ob)
                own.setdefault(net, []).append((li, ob, "via", t.m_Uuid.AsString()))
                src_items.append((li, ob, "via"))
            added["vias"] += 1
        return tgt, f"{len(segs)} segs {len(vias)} vias, {exp} expansions"

    def escape_balls(net):
        """Dogbone each bare FX3 ball of a supply net to a via in a free diagonal gap, so the
        net leaves the field on B.Cu instead of on F.Cu between the row A/B exits."""
        k = nid(net)
        u2 = board.FindFootprintByReference("U2")
        c = u2.GetPosition()
        cx, cy = mm(c.x), mm(c.y)
        done = 0
        for p in u2.Pads():
            if p.GetNetname() != net:
                continue
            q = p.GetPosition()
            px, py = mm(q.x), mm(q.y)
            if any(t.GetNetname() == net and t.GetClass() == "PCB_TRACK" and
                   (p.HitTest(t.GetStart()) or p.HitTest(t.GetEnd())) for t in tracks(board)):
                continue
            cands = sorted(((px + sx * 0.4, py + sy * 0.4) for sx in (1, -1) for sy in (1, -1)),
                           key=lambda v: -math.hypot(v[0] - cx, v[1] - cy))
            for vx, vy in cands:
                if any(cx0 - 0.45 < vx < cx1 + 0.45 and cy0 - 0.45 < vy < cy1 + 0.45
                       for cx0, cy0, cx1, cy1 in CHANNELS):
                    continue
                if any(math.hypot(hx - vx, hy - vy) < hr + VIA_DRILL / 2 + HOLE2HOLE + 0.001
                       for hx, hy, hr in F.holes):
                    continue
                if not all(F.exact_ok(li, k, (vx, vy), (vx, vy), VIA_D / 2, True) for li in range(4)):
                    continue
                wd = next((w for w in (0.25, 0.1) if F.exact_ok(0, k, (px, py), (vx, vy), w / 2, True)), None)
                if wd is None:
                    continue
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pcbnew.VECTOR2I(nm(px), nm(py)))
                t.SetEnd(pcbnew.VECTOR2I(nm(vx), nm(vy)))
                t.SetWidth(nm(wd))
                t.SetLayer(pcbnew.F_Cu)
                t.SetNet(nets[net])
                board.Add(t)
                ob = ("seg", px, py, vx, vy, wd / 2)
                F.add(0, k, ob)
                ends.add((0, nm(px), nm(py)))
                ends.add((0, nm(vx), nm(vy)))
                own.setdefault(net, []).append((0, ob, "track", t.m_Uuid.AsString()))
                attach[t.m_Uuid.AsString()] = p.m_Uuid.AsString()
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(nm(vx), nm(vy)))
                v.SetWidth(nm(VIA_D))
                v.SetDrill(nm(VIA_DRILL))
                v.SetNet(nets[net])
                board.Add(v)
                F.add_hole(vx, vy, VIA_DRILL / 2)
                for li in range(4):
                    ob = ("circ", vx, vy, VIA_D / 2)
                    F.add(li, k, ob)
                    own.setdefault(net, []).append((li, ob, "via", v.m_Uuid.AsString()))
                attach[v.m_Uuid.AsString()] = p.m_Uuid.AsString()
                added["tracks"] += 1
                added["vias"] += 1
                done += 1
                break
            else:
                print(f"  escape: no free gap for U2.{p.GetNumber()} {net}")
        if done:
            print(f"  escape: {done} {net} ball(s) dogboned")

    attach = {}
    seen = set()
    for net in ESCAPE:
        if net in nets and (not only or net in only):
            escape_balls(net)
    for pat in ORDER:
        for net in sorted(n for n in map(str, nets.keys()) if fnmatch.fnmatchcase(n, pat)):
            if net in seen or (only and net not in only):
                continue
            seen.add(net)
            route(net)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(path))
    print(f"p2p: {added['tracks']} segments, {added['vias']} vias added in {time.time() - t0:.0f} s")
    for r in report:
        print("  " + r)


if __name__ == "__main__":
    main()
