#!/usr/bin/env python3
"""Join what Freerouting left apart: read the unconnected pairs from a DRC run, route each pair on
F.Cu / B.Cu (45 degree A* on a 0.05 mm grid, vias allowed) and write the tracks into the board.

Same idea as the root tools/p2p_route.py (islands of one net joined point to point against
distance fields), cut down to what the hat needs: the pair comes from the DRC report instead of the
board connectivity, the obstacle raster is rebuilt per pair in a window around it. A cell is free
for a net when its centre is at least clearance + half the track width + one grid step from every
other net's copper and from the board edge.

usage: join_hat.py [board.kicad_pcb] [--rounds N]
"""
import heapq
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pcbnew
from pcbnew import FromMM as MM, ToMM as TOMM
from scipy import ndimage

HAT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HAT / "tools"))
G = 0.05
CLR, EDGE = 0.15, 0.4
VIA_EXTRA = 0.04          # hole to hole (0.4 mm between drill walls) and rounding
VIA_D, VIA_DRILL = 0.45, 0.2
F, B, I2 = pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In2_Cu
LAYERS = (F, I2, B)


def class_width_fn(pcb):
    import fnmatch
    d = json.loads(Path(pcb).with_suffix(".kicad_pro").read_text())["net_settings"]
    widths = {c["name"]: c["track_width"] for c in d["classes"]}

    def width(net):
        for pat in d.get("netclass_patterns", []):
            if fnmatch.fnmatchcase(net, pat["pattern"]):
                return widths[pat["netclass"]]
        return widths["Default"]
    return width


def drc_pairs(pcb, out):
    subprocess.run(["kicad-cli", "pcb", "drc", "--format", "json", "--severity-all", "--refill-zones",
                    "-o", str(out), str(pcb)], check=True, capture_output=True)
    d = json.loads(Path(out).read_text())
    pairs = []
    for v in d.get("unconnected_items", []):
        its = v["items"]
        if len(its) == 2:
            pairs.append([(i["description"], i["pos"]["x"], i["pos"]["y"]) for i in its])
    return pairs


def net_of(desc):
    import re
    m = re.search(r"\[([^\]]+)\]", desc)
    return m.group(1) if m else None


def layers_of(desc):
    if "PTH pad" in desc or "Via" in desc:
        return {F, B}
    if "B.Cu" in desc:
        return {B}
    return {F}


class Router:
    def __init__(self, board, pcb):
        self.b = board
        self.width = class_width_fn(pcb)
        ol = pcbnew.SHAPE_POLY_SET()
        board.GetBoardPolygonOutlines(ol, True)
        o = ol.COutline(0)
        self.poly = [(TOMM(o.CPoint(i).x), TOMM(o.CPoint(i).y)) for i in range(o.PointCount())]
        self.shapes = []
        self.refresh()

    def refresh(self):
        """(layer, net, kind, geometry) of all copper, kept in plain Python."""
        self.items = []
        for fp in self.b.GetFootprints():
            for p in fp.Pads():
                bb = p.GetBoundingBox()
                rect = (TOMM(bb.GetLeft()), TOMM(bb.GetTop()), TOMM(bb.GetRight()), TOMM(bb.GetBottom()))
                for L in (F, B):
                    if p.IsOnLayer(L):
                        self.items.append((L, p.GetNetname(), "rect", rect))
                if p.GetDrillSize().x > 0:      # plated hole: copper on In2 too
                    self.items.append((I2, p.GetNetname(), "rect", rect))
                if p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:       # min_hole_clearance 0.2
                    q = p.GetPosition()
                    for L in (F, I2, B):
                        self.items.append((L, "", "disc", (TOMM(q.x), TOMM(q.y), TOMM(p.GetDrillSize().x) / 2 + 0.06)))
        for t in self.b.GetTracks():
            if t.GetClass() == "PCB_VIA":
                v = pcbnew.Cast_to_PCB_VIA(t)
                q = v.GetPosition()
                for L in (F, I2, B):
                    self.items.append((L, v.GetNetname(), "disc", (TOMM(q.x), TOMM(q.y), TOMM(v.GetWidth(F)) / 2)))
            else:
                s, e = t.GetStart(), t.GetEnd()
                self.items.append((t.GetLayer(), t.GetNetname(), "seg",
                                   (TOMM(s.x), TOMM(s.y), TOMM(e.x), TOMM(e.y), TOMM(t.GetWidth()) / 2)))

        # power pours on In2 (not the GND background fill): a signal wire must not cut them
        for i in range(self.b.GetAreaCount()):
            z = self.b.GetArea(i)
            if z.GetIsRuleArea() or z.GetLayer() != I2 or z.GetNetname() == "GND":
                continue
            ol = z.Outline()
            for k in range(ol.OutlineCount()):
                c = ol.COutline(k)
                self.items.append((I2, z.GetNetname(), "poly",
                                   [(TOMM(c.CPoint(j).x), TOMM(c.CPoint(j).y)) for j in range(c.PointCount())]))

    def dist(self, layer, net, X, Y, G, polys=True):
        """Distance from every grid centre to the nearest other-net copper on `layer` (exact for
        rectangles, discs and segments; one cell of margin for the pour polygons)."""
        d = np.full(X.shape, 1e9)
        x0, x1, y0, y1 = X.min() - 3, X.max() + 3, Y.min() - 3, Y.max() + 3
        occ = np.zeros(X.shape, bool)
        for L, n, kind, g in self.items:
            if L != layer or n == net:
                continue
            if kind == "rect":
                if g[2] < x0 or g[0] > x1 or g[3] < y0 or g[1] > y1:
                    continue
                dx = np.maximum(np.maximum(g[0] - X, X - g[2]), 0)
                dy = np.maximum(np.maximum(g[1] - Y, Y - g[3]), 0)
                d = np.minimum(d, np.hypot(dx, dy))
            elif kind == "poly":
                if not polys:
                    continue
                ins = np.zeros(X.shape, bool)
                for k in range(len(g)):
                    (ax, ay), (bx, by) = g[k], g[(k + 1) % len(g)]
                    if ay == by:
                        continue
                    cond = ((ay <= Y) & (Y < by)) | ((by <= Y) & (Y < ay))
                    ins ^= cond & (X < ax + (Y - ay) * (bx - ax) / (by - ay))
                occ |= ins
            elif kind == "disc":
                if g[0] + g[2] < x0 or g[0] - g[2] > x1 or g[1] + g[2] < y0 or g[1] - g[2] > y1:
                    continue
                d = np.minimum(d, np.maximum(np.hypot(X - g[0], Y - g[1]) - g[2], 0))
            else:
                ax, ay, bx, by, r = g
                if max(ax, bx) + r < x0 or min(ax, bx) - r > x1 or max(ay, by) + r < y0 or min(ay, by) - r > y1:
                    continue
                dx, dy = bx - ax, by - ay
                L2 = dx * dx + dy * dy
                t = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0, 1) if L2 else 0
                d = np.minimum(d, np.maximum(np.hypot(X - ax - t * dx, Y - ay - t * dy) - r, 0))
        if occ.any():
            d = np.minimum(d, np.maximum(ndimage.distance_transform_edt(~occ) * G - 0.75 * G, 0))
        return d

    def inside_dist(self, X, Y):
        P = self.poly
        ins = np.zeros(X.shape, bool)
        d = np.full(X.shape, 1e9)
        for k in range(len(P)):
            (ax, ay), (bx, by) = P[k], P[(k + 1) % len(P)]
            if ay != by:
                cond = ((ay <= Y) & (Y < by)) | ((by <= Y) & (Y < ay))
                ins ^= cond & (X < ax + (Y - ay) * (bx - ax) / (by - ay))
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = np.clip(((X - ax) * dx + (Y - ay) * dy) / L2, 0, 1)
            d = np.minimum(d, np.hypot(X - ax - t * dx, Y - ay - t * dy))
        return np.where(ins, d, -1.0)


def route_pair(router, net, a, b, la, lb, w, G=0.05, va=(), vb=(), cut=False):
    """Wrapper that keeps parents properly (second, simpler A* returning the path)."""
    pad = 4.0
    x0, x1 = min(a[0], b[0]) - pad, max(a[0], b[0]) + pad
    y0, y1 = min(a[1], b[1]) - pad, max(a[1], b[1]) + pad
    x0 = a[0] - math.floor((a[0] - x0) / G) * G       # the start sits exactly on a grid node
    y0 = a[1] - math.floor((a[1] - y0) / G) * G
    nx, ny = int((x1 - x0) / G) + 1, int((y1 - y0) / G) + 1
    gx = x0 + np.arange(nx) * G
    gy = y0 + np.arange(ny) * G
    X, Y = np.meshgrid(gx, gy, indexing="ij")
    edge = router.inside_dist(X, Y)
    ok_edge = edge >= EDGE + w / 2
    free, viaok = {}, {}
    for L in LAYERS:
        dist = router.dist(L, net, X, Y, G)
        free[L] = (dist >= CLR + w / 2 + 0.004) & ok_edge
        viaok[L] = ((router.dist(L, net, X, Y, G, polys=False) if cut else dist) >= CLR + VIA_D / 2 + VIA_EXTRA) & ok_edge
        if L == LAYERS[0]:
            same = np.full(X.shape, 1e9)
            for L2, n2, kind, g in router.items:
                if L2 == F and n2 == net and kind == "disc":
                    same = np.minimum(same, np.hypot(X - g[0], Y - g[1]))
            viaok_same = same >= VIA_D + CLR + VIA_EXTRA     # centre to centre of two vias of one net
    for L in LAYERS:
        viaok[L] &= viaok_same

    def cell(p):
        return (int(round((p[0] - x0) / G)), int(round((p[1] - y0) / G)))
    ca, cb = cell(a), cell(b)
    for L in LAYERS:
        for c in (ca, cb):
            R = max(1, int(round(0.1 / G)))
            for di in range(-R, R + 1):
                for dj in range(-R, R + 1):
                    i, j = c[0] + di, c[1] + dj
                    if 0 <= i < nx and 0 <= j < ny and di * di + dj * dj <= R * R:
                        free[L][i, j] = True
    dirs = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    pq = []
    best = {}
    parent = {}
    for L in la:
        st = (ca, L, -1)
        best[st] = 0.0
        heapq.heappush(pq, (0.0, 0.0, st))
    exist = {cell((i[3][0], i[3][1])) for i in router.items if i[2] == 'disc' and i[0] == F and i[1] == net}
    # vias of the same net sitting at the start item can be left on any layer (and likewise at the goal)
    gcells = {cb: set(lb)}
    for v in va:
        for L in LAYERS:
            st = (cell(v), L, -1)
            best[st] = 0.0
            heapq.heappush(pq, (0.0, 0.0, st))
    for v in vb:
        gcells[cell(v)] = set(LAYERS)
    found = None
    exp = 0
    while pq:
        f, g, st = heapq.heappop(pq)
        if best.get(st, 1e18) < g - 1e-12:
            continue
        c, L, d = st
        exp += 1
        if exp > 4000000:
            break
        if c in gcells and L in gcells[c]:
            found = st
            break
        for k, (dx, dy) in enumerate(dirs):
            n = (c[0] + dx, c[1] + dy)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or not free[L][n]:
                continue
            if dx and dy and not (free[L][c[0] + dx, c[1]] and free[L][c[0], c[1] + dy]):
                continue
            ng = g + G * (1.4142 if (dx and dy) else 1.0) + (0.6 if d not in (-1, k) else 0.0)
            ns = (n, L, k)
            if ng < best.get(ns, 1e18):
                best[ns] = ng
                parent[ns] = st
                heapq.heappush(pq, (ng + math.hypot(n[0] - cb[0], n[1] - cb[1]) * G, ng, ns))
        at_via = c in exist
        if at_via or all(viaok[x][c] for x in LAYERS):
            for L2 in LAYERS:
                if L2 == L:
                    continue
                ns = (c, L2, -1)
                ng = g + (0.1 if at_via else 1.5)
                if ng < best.get(ns, 1e18):
                    best[ns] = ng
                    parent[ns] = st
                    heapq.heappush(pq, (ng + math.hypot(c[0] - cb[0], c[1] - cb[1]) * G, ng, ns))
    if not found:
        return None
    path = [found]
    while path[-1] in parent:
        path.append(parent[path[-1]])
    path.reverse()
    segs, vias = [], []
    cur, pts = path[0][1], [path[0][0]]
    for (c, L, d), (c2, L2, d2) in zip(path, path[1:]):
        if L2 != L:
            vias.append(c)
            segs.append((cur, pts))
            cur, pts = L2, [c]
        else:
            pts.append(c2)
    segs.append((cur, pts))
    out = []
    for L, pts in segs:
        # keep the corners only
        keep = [pts[0]]
        for i in range(1, len(pts) - 1):
            d1 = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            d2 = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            if d1 != d2:
                keep.append(pts[i])
        keep.append(pts[-1])
        out.append((L, [(x0 + c[0] * G, y0 + c[1] * G) for c in keep]))
    first = out[0][1]
    if path[0][0] == ca:                     # started at the item itself, not at one of its vias
        first[0] = a
    last = out[-1][1]
    if path[-1][0] == cb and math.hypot(last[-1][0] - b[0], last[-1][1] - b[1]) < 0.06:
        last[-1] = b
    return out, [(x0 + c[0] * G, y0 + c[1] * G) for c in vias]


def main():
    pcb = Path(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else HAT / "ps1-av-hat.kicad_pcb"
    rounds = int(sys.argv[sys.argv.index("--rounds") + 1]) if "--rounds" in sys.argv else 3
    tmp = Path("/tmp/join-drc.json") if "--tmp" in sys.argv else HAT / "build" / "join-drc.json"
    for rnd in range(rounds):
        pairs = drc_pairs(pcb, tmp)
        print(f"join round {rnd}: {len(pairs)} unconnected pairs", flush=True)
        if not pairs:
            break
        board = pcbnew.LoadBoard(str(pcb))
        nets = {str(n): board.FindNet(str(n)) for n in board.GetNetsByName()}
        router = Router(board, pcb)
        added = 0
        # signals first: they have the fewest ways round; the wide supply wires then take what is left
        pairs.sort(key=lambda pr: router.width(net_of(pr[0][0]) or ""))
        for (da, ax, ay), (db, bx, by) in pairs:
            net = net_of(da) or net_of(db)
            if net is None or net_of(da) != net_of(db) or "Zone" in da or "Zone" in db:
                print("  skip", da, "|", db)
                continue
            base = router.width(net)
            res = None
            vs = [(i[3][0], i[3][1]) for i in router.items if i[2] == "disc" and i[0] == F and i[1] == net]
            def attached(desc, px, py):
                """Vias that are part of the item itself: the item is a via, or a track that ends on one."""
                import re
                if desc.startswith("Via"):
                    return [v for v in vs if math.hypot(v[0] - px, v[1] - py) < 0.05]
                m = re.search(r"length ([\d.]+) mm", desc)
                if not desc.startswith("Track") or not m:
                    return []
                ln = float(m.group(1))
                return [v for v in vs if abs(math.hypot(v[0] - px, v[1] - py) - ln) < 0.03]
            va = attached(da, ax, ay)
            vb = attached(db, bx, by)
            for w in sorted({base, 0.3, 0.25, 0.2, 0.15}, reverse=True):
                if w > base:
                    continue
                area = (abs(ax - bx) + 8) * (abs(ay - by) + 8)
                for Gr, cut in [(g_, False) for g_ in ((0.05, 0.025) if area < 450 else (0.05,))] + ([(0.05, True)] if "--cut" in sys.argv and base <= 0.2 else []):
                    res = route_pair(router, net, (ax, ay), (bx, by), layers_of(da), layers_of(db), w, Gr, va, vb, cut)
                    if res:
                        if cut:
                            print("  (via cuts another net's In2 pour here)")
                        break
                if res:
                    break
            if not res:
                print("  FAILED", net, da, "->", db)
                continue
            segs, vias = res
            for L, pts in segs:
                for p, q in zip(pts, pts[1:]):
                    t = pcbnew.PCB_TRACK(board)
                    t.SetStart(pcbnew.VECTOR2I(MM(p[0]), MM(p[1])))
                    t.SetEnd(pcbnew.VECTOR2I(MM(q[0]), MM(q[1])))
                    t.SetWidth(MM(w))
                    t.SetLayer(L)
                    t.SetNet(nets[net])
                    board.Add(t)
                    t.thisown = 0
                    router.items.append((L, net, "seg", (p[0], p[1], q[0], q[1], w / 2)))
            for v in vias:
                if any(i[2] == "disc" and i[0] == F and i[1] == net and math.hypot(i[3][0] - v[0], i[3][1] - v[1]) < 0.1
                       for i in router.items):
                    continue          # the path switches layers at a via that is already there
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(pcbnew.VECTOR2I(MM(v[0]), MM(v[1])))
                via.SetViaType(pcbnew.VIATYPE_THROUGH)
                via.SetWidth(MM(VIA_D))
                via.SetDrill(MM(VIA_DRILL))
                via.SetNet(nets[net])
                via.SetLayerPair(F, B)
                board.Add(via)
                via.thisown = 0
                for L in LAYERS:
                    router.items.append((L, net, "disc", (v[0], v[1], VIA_D / 2)))
            print(f"  joined {net} w={w}: {sum(len(p) - 1 for _, p in segs)} segments, {len(vias)} vias", flush=True)
            added += 1
        pcbnew.SaveBoard(str(pcb), board)
        sys.stdout.flush()
        if not added:
            break
    import os
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
