#!/usr/bin/env python3
"""FX3 BGA escape, phase 2: a grid maze router for U2's signal balls out to the field edge.

Runs after escape.py on the same board. Inside ESC_BOX (U2's ball field plus a skirt) tracks
neck down to 0.1 mm at 0.1 mm clearance, so one track passes between two vias in neighbouring
cells (0.35 mm gap) - the custom rule in ps1-av-capture.kicad_dru holds DRC to the same numbers
there and nowhere else. Rings 1-2 route on F.Cu from the ball; rings 3+ from their dogbone via
on B.Cu, falling back to In2.Cu. Each net runs to the box face its bundle leaves from (GPIF
G/B and syncs west towards U1, GPIF R north, everything else its nearest face) and stops on the
box edge; the bundle beyond is the next script's job. A* on a 0.05 mm grid, 8-connected, one
net at a time, no rip-up: prints what it routed and what it could not.
"""
import fnmatch
import heapq
import math
import sys
from pathlib import Path

import numpy as np
import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_usb import OX, OY, PCB  # noqa: E402
from escape import ring, SKIP_NETS  # noqa: E402
from build_pcb import POWER_NETS  # noqa: E402

G = 0.05                                   # grid step, mm
W, CLR = 0.1, 0.1                          # neckdown track width and clearance inside the box
ESC_BOX = (55.0, 43.0, 69.0, 57.0)         # x0, y0, x1, y1 (board mm); must match the .kicad_dru
LAYERS = [pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
NX = int(round((ESC_BOX[2] - ESC_BOX[0]) / G)) + 1
NY = int(round((ESC_BOX[3] - ESC_BOX[1]) / G)) + 1


def mm(v):
    return pcbnew.ToMM(v)


def face_for(net, x, y):
    if net.startswith("VD_R"):
        return "N"
    if net.startswith("VD_"):
        return "W"
    d = {"W": x - ESC_BOX[0], "E": ESC_BOX[2] - x, "N": y - ESC_BOX[1], "S": ESC_BOX[3] - y}
    d["S"] += 99                           # south is the USB corridor: nothing else leaves there
    return min(d, key=d.get)


def main():
    board = pcbnew.LoadBoard(str(PCB))
    nets = board.GetNetsByName()
    netid = {}

    def nid(n):
        return netid.setdefault(n, len(netid) + 1)

    # owner[l][j, i]: 0 free, k>0 only net k may use the cell, -1 blocked
    owner = [np.zeros((NY, NX), dtype=np.int32) for _ in LAYERS]

    def stamp(li, net, kind, g, ext):
        """Mark cells whose centre is within ext + CLR + W/2 of the item."""
        r = ext + CLR + W / 2
        if kind == "circle":
            (cx, cy), bx = g, (g[0] - r, g[1] - r, g[0] + r, g[1] + r)
        elif kind == "rect":
            bx = (g[0] - r, g[1] - r, g[2] + r, g[3] + r)
        else:
            (a, b) = g
            bx = (min(a[0], b[0]) - r, min(a[1], b[1]) - r, max(a[0], b[0]) + r, max(a[1], b[1]) + r)
        i0, i1 = max(0, int((bx[0] - ESC_BOX[0]) / G)), min(NX - 1, int((bx[2] - ESC_BOX[0]) / G) + 1)
        j0, j1 = max(0, int((bx[1] - ESC_BOX[1]) / G)), min(NY - 1, int((bx[3] - ESC_BOX[1]) / G) + 1)
        if i0 > i1 or j0 > j1:
            return
        xs = ESC_BOX[0] + np.arange(i0, i1 + 1) * G
        ys = ESC_BOX[1] + np.arange(j0, j1 + 1) * G
        X, Y = np.meshgrid(xs, ys)
        if kind == "circle":
            d = np.hypot(X - cx, Y - cy)
        elif kind == "rect":
            d = np.hypot(np.maximum(np.maximum(g[0] - X, X - g[2]), 0), np.maximum(np.maximum(g[1] - Y, Y - g[3]), 0))
        else:
            ax, ay = a
            vx, vy = b[0] - ax, b[1] - ay
            L2 = vx * vx + vy * vy or 1e-12
            t = np.clip(((X - ax) * vx + (Y - ay) * vy) / L2, 0, 1)
            d = np.hypot(X - ax - t * vx, Y - ay - t * vy)
        hit = d < r - 1e-6
        sub = owner[li][j0:j1 + 1, i0:i1 + 1]
        k = nid(net)
        sub[hit & (sub == 0)] = k
        sub[hit & (sub != k) & (sub != 0)] = -1

    for fp in board.GetFootprints():
        for p in fp.Pads():
            net = p.GetNetname() or f"<nc {fp.GetReference()}.{p.GetNumber()}>"
            pos = p.GetPosition()
            for li, L in enumerate(LAYERS):
                if not p.IsOnLayer(L):
                    continue
                if p.GetShape() == pcbnew.PAD_SHAPE_CIRCLE:
                    stamp(li, net, "circle", (mm(pos.x) - OX, mm(pos.y) - OY), mm(p.GetSize().x) / 2)
                else:
                    bb = p.GetBoundingBox()
                    stamp(li, net, "rect", (mm(bb.GetLeft()) - OX, mm(bb.GetTop()) - OY,
                                            mm(bb.GetRight()) - OX, mm(bb.GetBottom()) - OY), 0.0)
    vias = {}
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            xy = (mm(t.GetPosition().x) - OX, mm(t.GetPosition().y) - OY)
            vias.setdefault(t.GetNetname(), []).append(xy)
            for li in range(len(LAYERS)):
                stamp(li, t.GetNetname(), "circle", xy, mm(t.GetWidth(pcbnew.F_Cu)) / 2)
        else:
            for li, L in enumerate(LAYERS):
                if t.GetLayer() == L:
                    stamp(li, t.GetNetname(), "seg", ((mm(t.GetStart().x) - OX, mm(t.GetStart().y) - OY),
                                                      (mm(t.GetEnd().x) - OX, mm(t.GetEnd().y) - OY)),
                          mm(t.GetWidth()) / 2)

    def cell(x, y):
        return int(round((y - ESC_BOX[1]) / G)), int(round((x - ESC_BOX[0]) / G))

    def on_face(j, i, face):
        return {"W": i == 0, "E": i == NX - 1, "N": j == 0, "S": j == NY - 1}[face]

    def astar(li, k, start, face):
        grid = owner[li]
        ok = lambda j, i: grid[j, i] == 0 or grid[j, i] == k
        goal_d = {"W": lambda j, i: i, "E": lambda j, i: NX - 1 - i,
                  "N": lambda j, i: j, "S": lambda j, i: NY - 1 - j}[face]
        if not ok(*start):
            return None
        openq = [(goal_d(*start), 0.0, start, None)]
        came, best = {}, {start: 0.0}
        steps = [(0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),
                 (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        while openq:
            _, g, c, par = heapq.heappop(openq)
            if c in came:
                continue
            came[c] = par
            if on_face(*c, face):
                path = [c]
                while came[path[-1]] is not None:
                    path.append(came[path[-1]])
                return path[::-1]
            for dj, di, cost in steps:
                n = (c[0] + dj, c[1] + di)
                if not (0 <= n[0] < NY and 0 <= n[1] < NX) or not ok(*n):
                    continue
                if dj and di and not (ok(c[0] + dj, c[1]) and ok(c[0], c[1] + di)):
                    continue                       # no corner-cutting past an obstacle
                turn = 0.3 if par is not None and (c[0] - par[0], c[1] - par[1]) != (dj, di) else 0
                ng = g + cost + turn
                if ng < best.get(n, 1e18):
                    best[n] = ng
                    heapq.heappush(openq, (ng + goal_d(*n), ng, n, c))
        return None

    def simplify(path):
        out = [path[0]]
        for a, b, c in zip(path, path[1:], path[2:]):
            if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
                out.append(b)
        out.append(path[-1])
        return out

    def via_fits(k, x, y):
        """A 0.45 via at (x, y) keeps CLR on every layer: the grid is inflated by W/2 for a
        track centreline, so every cell within 0.225 - W/2 of the centre must be ours."""
        r = int(math.ceil((0.225 - W / 2 + CLR * 0) / G))
        j, i = cell(x, y)
        for lj in range(len(LAYERS)):
            for dj in range(-r, r + 1):
                for di in range(-r, r + 1):
                    if math.hypot(dj, di) * G > 0.225 - W / 2 + 1e-6:
                        continue
                    jj, ii = j + dj, i + di
                    if not (0 <= jj < NY and 0 <= ii < NX):
                        return False
                    o = owner[lj][jj, ii]
                    if o != 0 and o != k:
                        return False
        return True

    def line_ok(li, k, a, b):
        n = max(2, int(math.dist(a, b) / (G / 2)))
        for t in range(1, n):
            j, i = cell(a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n)
            o = owner[li][j, i]
            if o != 0 and o != k:
                return False
        return True

    def add_track(li, net, a, b, w):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(OX + a[0]), pcbnew.FromMM(OY + a[1])))
        t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(OX + b[0]), pcbnew.FromMM(OY + b[1])))
        t.SetWidth(pcbnew.FromMM(w))
        t.SetLayer(LAYERS[li])
        t.SetNet(nets[net])
        t.SetLocked(True)
        board.Add(t)
        stamp(li, net, "seg", (a, b), w / 2)

    u2 = board.FindFootprintByReference("U2")
    jobs = []
    for p in u2.Pads():
        net = p.GetNetname()
        if net in SKIP_NETS or net.startswith(("unconnected", "USB3_")):
            continue
        if any(fnmatch.fnmatchcase(net, pat) for pat in POWER_NETS):
            continue                  # supply nets keep their class width: point-to-point step
        px, py = mm(p.GetPosition().x) - OX, mm(p.GetPosition().y) - OY
        rg = ring(p.GetNumber())
        if rg <= 2:
            starts = [(0, (px, py))]
        else:
            v = min(vias.get(net, []), key=lambda q: math.hypot(q[0] - px, q[1] - py), default=None)
            if v is None or math.hypot(v[0] - px, v[1] - py) > 0.6:
                print("  no dogbone for", p.GetNumber(), net)
                continue
            starts = [(2, v), (1, v)]
        face = face_for(net, px, py)
        dist = {"W": px - ESC_BOX[0], "E": ESC_BOX[2] - px, "N": py - ESC_BOX[1], "S": ESC_BOX[3] - py}[face]
        jobs.append((rg, dist, p.GetNumber(), net, starts, face))
    # F.Cu: ring 2 before ring 1 (a ring-1 ball is on the edge and needs one short stub, while
    # a ring-2 ball has to thread between two of them), then the via nets, inner rings last
    jobs.sort(key=lambda j: (j[0] > 2, -j[0] if j[0] <= 2 else j[0], j[1]))
    done, failed, length = [], [], {}
    seen_net = set()
    for rg, _, ball, net, starts, face in jobs:
        if net in seen_net:                        # FX3_FSLC has several balls: one exit
            continue
        seen_net.add(net)
        k = nid(net)
        # preferred face first, then the others nearest first (south, the USB corridor, last)
        x0, y0 = starts[0][1]
        rest = sorted("WENS".replace(face, ""), key=lambda f: ({"W": x0 - ESC_BOX[0], "E": ESC_BOX[2] - x0,
                      "N": y0 - ESC_BOX[1], "S": ESC_BOX[3] - y0 + 99}[f]))
        path = None
        for f in [face] + rest:
            for li, (x, y) in starts:
                path = astar(li, k, cell(x, y), f)
                if path:
                    break
            if path:
                face = f
                break
        if not path and rg <= 2:
            # boxed in on F.Cu by its neighbours' stubs: drop a dogbone via of its own and
            # leave on an inner layer instead
            bx, by = starts[0][1]
            for vx, vy in sorted([(bx + sx * 0.4, by + sy * 0.4) for sx in (1, -1) for sy in (1, -1)],
                                 key=lambda q: math.hypot(q[0] - ESC_BOX[0] if face == "W" else 0,
                                                          q[1] - ESC_BOX[1] if face == "N" else 0)):
                if not via_fits(k, vx, vy) or not line_ok(0, k, (bx, by), (vx, vy)):
                    continue
                for f in [face] + rest:
                    for li in (2, 1):
                        path = astar(li, k, cell(vx, vy), f)
                        if path:
                            break
                    if path:
                        face = f
                        break
                if path:
                    add_track(0, net, (bx, by), (vx, vy), 0.2)
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(OX + vx), pcbnew.FromMM(OY + vy)))
                    v.SetWidth(pcbnew.FromMM(0.45))
                    v.SetDrill(pcbnew.FromMM(0.2))
                    v.SetNet(nets[net])
                    v.SetLocked(True)
                    board.Add(v)
                    for lj in range(len(LAYERS)):
                        stamp(lj, net, "circle", (vx, vy), 0.225)
                    x, y = vx, vy
                    break
        if not path:
            failed.append(f"{ball} {net} ring {rg} -> {face}")
            continue
        pts = [(ESC_BOX[0] + i * G, ESC_BOX[1] + j * G) for j, i in simplify(path)]
        pts[0] = (x, y)               # the ball or via centre, not its nearest grid point
        def bends_ok(ps):
            """Every corner turns by 45 degrees or less (check_geometry's ACUTE test)."""
            for u, v, w in zip(ps, ps[1:], ps[2:]):
                a1 = math.atan2(v[1] - u[1], v[0] - u[0])
                a2 = math.atan2(w[1] - v[1], w[0] - v[0])
                if abs((a2 - a1 + math.pi) % (2 * math.pi) - math.pi) > math.pi / 4 + 1e-3:
                    return False
            return True

        # no sub-width segments: drop an interior point next to a short segment when the
        # straight line that replaces it still clears everything (line_ok on the raster)
        changed = True
        while changed and len(pts) > 2:
            changed = False
            for q in range(1, len(pts) - 1):
                trial = pts[:q] + pts[q + 1:]
                if (min(math.dist(pts[q - 1], pts[q]), math.dist(pts[q], pts[q + 1])) < 2 * W
                        and line_ok(li, k, pts[q - 1], pts[q + 1]) and bends_ok(trial)):
                    del pts[q]
                    changed = True
                    break
        if len(pts) > 2 and math.dist(pts[-2], pts[-1]) < 2 * W:
            pts[-2] = pts[-1]          # the last grid step onto the box edge
            del pts[-1]
        L = 0.0
        for a, b in zip(pts, pts[1:]):
            if math.dist(a, b) < 1e-6:
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(OX + a[0]), pcbnew.FromMM(OY + a[1])))
            t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(OX + b[0]), pcbnew.FromMM(OY + b[1])))
            t.SetWidth(pcbnew.FromMM(W))
            t.SetLayer(LAYERS[li])
            t.SetNet(nets[net])
            t.SetLocked(True)
            board.Add(t)
            for lj in range(len(LAYERS)):
                if lj == li:
                    stamp(lj, net, "seg", (a, b), W / 2)
            L += math.dist(a, b)
        length[net] = L
        done.append(f"{ball} {net} {pcbnew.LayerName(LAYERS[li]) if False else board.GetLayerName(LAYERS[li])} -> {face} {L:.2f} mm")
    area = pcbnew.ZONE(board)                      # rule area the .kicad_dru keys on
    area.SetIsRuleArea(True)
    area.SetZoneName("BGA_ESC")
    area.SetDoNotAllowTracks(False)
    area.SetDoNotAllowVias(False)
    area.SetDoNotAllowPads(False)
    area.SetDoNotAllowFootprints(False)
    area.SetDoNotAllowZoneFills(False)
    ls = pcbnew.LSET()
    for L in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
        ls.AddLayer(L)
    area.SetLayerSet(ls)
    ol = area.Outline()
    ol.NewOutline()
    for x, y in ((ESC_BOX[0], ESC_BOX[1]), (ESC_BOX[2], ESC_BOX[1]), (ESC_BOX[2], ESC_BOX[3]), (ESC_BOX[0], ESC_BOX[3])):
        ol.Append(pcbnew.FromMM(OX + x), pcbnew.FromMM(OY + y))
    board.Add(area)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(PCB))
    print(f"bga_route: {len(done)} nets out of the field, {len(failed)} failed")
    for d in done:
        print("  ok", d)
    for f in failed:
        print("  FAILED", f)


if __name__ == "__main__":
    main()
