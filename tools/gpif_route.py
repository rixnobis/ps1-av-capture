#!/usr/bin/env python3
"""GPIF bundle: each VD_* net from the end of its FX3 escape (tools/bga_route.py) to the
VD-side pad of its 22R series resistor next to the TVP7002 (build_pcb.py SERIES).

Runs after bga_route.py on the same board. A* on a 0.05 mm grid over BUNDLE_BOX, three
signal layers (F.Cu, In2.Cu, B.Cu), vias allowed outside the neckdown zone. The neckdown zone
is the escape box plus a 1 mm skirt (rule area BGA_ESC, widened here): 0.1 mm track at 0.1 mm
clearance inside it, 0.127 mm track at the 0.15 mm netclass clearance outside, so the bundle
fans out from the escape pitch before it widens. Nets are routed nearest-target-first; prints
per-net length and what failed.
"""
import heapq
import math
import sys
from pathlib import Path

import numpy as np
import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_usb import OX, OY, PCB  # noqa: E402
from bga_route import ESC_BOX, SKIRT  # noqa: E402

G = 0.05
NECK = (ESC_BOX[0] - SKIRT, ESC_BOX[1] - SKIRT, ESC_BOX[2] + SKIRT, ESC_BOX[3] + SKIRT)
W_IN, CLR_IN, W_OUT, CLR_OUT = 0.1, 0.1, 0.127, 0.15
VIA_R, VIA_COST = 0.225, 25.0
VIA_KEEP = 2.0                      # mm around the series-resistor pads with no via
BUNDLE_BOX = (14.0, 17.0, 72.0, 58.0)
LAYERS = [pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
NX = int(round((BUNDLE_BOX[2] - BUNDLE_BOX[0]) / G)) + 1
NY = int(round((BUNDLE_BOX[3] - BUNDLE_BOX[1]) / G)) + 1
XS = BUNDLE_BOX[0] + np.arange(NX) * G
YS = BUNDLE_BOX[1] + np.arange(NY) * G
INNECK = ((XS[None, :] >= NECK[0]) & (XS[None, :] <= NECK[2]) &
          (YS[:, None] >= NECK[1]) & (YS[:, None] <= NECK[3]))
INFL = np.where(INNECK, CLR_IN + W_IN / 2, CLR_OUT + W_OUT / 2)    # per-cell track inflation


def mm(v):
    return pcbnew.ToMM(v)


def in_neck(x, y):
    return NECK[0] <= x <= NECK[2] and NECK[1] <= y <= NECK[3]


def main():
    board = pcbnew.LoadBoard(str(PCB))
    nets = board.GetNetsByName()
    netid = {}
    nid = lambda n: netid.setdefault(n, len(netid) + 1)  # noqa: E731
    owner = [np.zeros((NY, NX), dtype=np.int32) for _ in LAYERS]

    def stamp(li, net, kind, g, ext):
        r = ext + CLR_OUT + W_OUT / 2 + 0.01
        if kind == "circle":
            bx = (g[0] - r, g[1] - r, g[0] + r, g[1] + r)
        elif kind == "rect":
            bx = (g[0] - r, g[1] - r, g[2] + r, g[3] + r)
        else:
            a, b = g
            bx = (min(a[0], b[0]) - r, min(a[1], b[1]) - r, max(a[0], b[0]) + r, max(a[1], b[1]) + r)
        i0, i1 = max(0, int((bx[0] - BUNDLE_BOX[0]) / G)), min(NX - 1, int((bx[2] - BUNDLE_BOX[0]) / G) + 1)
        j0, j1 = max(0, int((bx[1] - BUNDLE_BOX[1]) / G)), min(NY - 1, int((bx[3] - BUNDLE_BOX[1]) / G) + 1)
        if i0 > i1 or j0 > j1:
            return
        X, Y = np.meshgrid(XS[i0:i1 + 1], YS[j0:j1 + 1])
        if kind == "circle":
            d = np.hypot(X - g[0], Y - g[1])
        elif kind == "rect":
            d = np.hypot(np.maximum(np.maximum(g[0] - X, X - g[2]), 0),
                         np.maximum(np.maximum(g[1] - Y, Y - g[3]), 0))
        else:
            (ax, ay), (bx2, by2) = g
            vx, vy = bx2 - ax, by2 - ay
            t = np.clip(((X - ax) * vx + (Y - ay) * vy) / (vx * vx + vy * vy or 1e-12), 0, 1)
            d = np.hypot(X - ax - t * vx, Y - ay - t * vy)
        hit = d < ext + INFL[j0:j1 + 1, i0:i1 + 1] - 1e-6
        sub = owner[li][j0:j1 + 1, i0:i1 + 1]
        k = nid(net)
        sub[hit & (sub == 0)] = k
        sub[hit & (sub != k) & (sub != 0)] = -1

    pads = {}
    for fp in board.GetFootprints():
        for p in fp.Pads():
            net = p.GetNetname() or f"<nc {fp.GetReference()}.{p.GetNumber()}>"
            bb = p.GetBoundingBox()
            rect = (mm(bb.GetLeft()) - OX, mm(bb.GetTop()) - OY, mm(bb.GetRight()) - OX, mm(bb.GetBottom()) - OY)
            pads.setdefault(net, []).append((fp.GetReference(), rect, p))
            for li, L in enumerate(LAYERS):
                if p.IsOnLayer(L):
                    if p.GetShape() == pcbnew.PAD_SHAPE_CIRCLE:
                        stamp(li, net, "circle", ((rect[0] + rect[2]) / 2, (rect[1] + rect[3]) / 2),
                              (rect[2] - rect[0]) / 2)
                    else:
                        stamp(li, net, "rect", rect, 0.0)
    ends = {}            # net -> list of (layer index, x, y) track end points
    for t in board.GetTracks():
        n = t.GetNetname()
        if t.GetClass() == "PCB_VIA":
            xy = (mm(t.GetPosition().x) - OX, mm(t.GetPosition().y) - OY)
            for li in range(len(LAYERS)):
                stamp(li, n, "circle", xy, mm(t.GetWidth(pcbnew.F_Cu)) / 2)
            continue
        a = (mm(t.GetStart().x) - OX, mm(t.GetStart().y) - OY)
        b = (mm(t.GetEnd().x) - OX, mm(t.GetEnd().y) - OY)
        for li, L in enumerate(LAYERS):
            if t.GetLayer() == L:
                stamp(li, n, "seg", (a, b), mm(t.GetWidth()) / 2)
                ends.setdefault(n, []).extend([(li, *a), (li, *b)])

    def cell(x, y):
        return int(round((y - BUNDLE_BOX[1]) / G)), int(round((x - BUNDLE_BOX[0]) / G))

    # No layer change within VIA_KEEP of a series-resistor pad: a via under the row blocks the
    # approach to the pads on either side of it, so every net arrives on F.Cu from out there.
    NOVIA = np.zeros((NY, NX), dtype=bool)
    for net, lst in pads.items():
        if not net.startswith("VD_"):
            continue
        for r, rect, _ in lst:
            if r.startswith("R"):
                X, Y = np.meshgrid(XS, YS)
                d = np.hypot(np.maximum(np.maximum(rect[0] - X, X - rect[2]), 0),
                             np.maximum(np.maximum(rect[1] - Y, Y - rect[3]), 0))
                NOVIA |= d < VIA_KEEP

    via_disk = [(dj, di) for dj in range(-6, 7) for di in range(-6, 7)
                if math.hypot(dj, di) * G <= VIA_R - W_OUT / 2 + G + 1e-6]   # + one cell: grid quantisation

    def via_ok(k, j, i):
        if INNECK[j, i] or NOVIA[j, i]:
            return False
        for lj in range(len(LAYERS)):
            o = owner[lj]
            for dj, di in via_disk:
                jj, ii = j + dj, i + di
                if not (0 <= jj < NY and 0 <= ii < NX) or o[jj, ii] not in (0, k):
                    return False
        return True

    def astar(k, starts, goal, occ=None, hist=None, pf=0.0):
        """starts: [(li, j, i)]; goal: set of (li, j, i). 8-connected plus vias. With occ/hist,
        a step into a cell other bundle nets occupy costs pf per occupant plus the cell's history
        (negotiated congestion); without them those cells are simply free."""
        gj = sum(g[1] for g in goal) / len(goal)
        gi = sum(g[2] for g in goal) / len(goal)
        h = lambda j, i: math.hypot(j - gj, i - gi) * 1.3  # noqa: E731  (weighted: fast, near-optimal)
        openq = [(h(s[1], s[2]), 0.0, s, None) for s in starts]
        heapq.heapify(openq)
        came, best = {}, {s: 0.0 for s in starts}
        steps = [(0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1),
                 (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
        n_exp = 0
        while openq:
            _, g, c, par = heapq.heappop(openq)
            if c in came:
                continue
            came[c] = par
            n_exp += 1
            if c in goal:
                path = [c]
                while came[path[-1]] is not None:
                    path.append(came[path[-1]])
                return path[::-1]
            if n_exp > 400000:
                return None
            li, j, i = c
            grid = owner[li]
            for dj, di, cost in steps:
                nj, ni = j + dj, i + di
                if not (0 <= nj < NY and 0 <= ni < NX) or grid[nj, ni] not in (0, k):
                    continue
                if dj and di and (grid[j + dj, i] not in (0, k) or grid[j, i + di] not in (0, k)):
                    continue
                turn = 0.3 if par is not None and par[0] == li and (j - par[1], i - par[2]) != (dj, di) else 0
                n = (li, nj, ni)
                ng = g + cost + turn
                if occ is not None:
                    ng += cost * (hist[li][nj, ni] + pf * occ[li][nj, ni])
                if ng < best.get(n, 1e18):
                    best[n] = ng
                    heapq.heappush(openq, (ng + h(nj, ni), ng, n, c))
            for lj in range(len(LAYERS)):
                if lj != li and via_ok(k, j, i):
                    n = (lj, j, i)
                    ng = g + VIA_COST
                    if occ is not None:
                        ng += 4 * pf * sum(o[j, i] for o in occ)
                    if ng < best.get(n, 1e18):
                        best[n] = ng
                        heapq.heappush(openq, (ng + h(j, i), ng, n, c))
        return None

    jobs = []
    for net in sorted(str(n) for n in nets.keys() if str(n).startswith("VD_")):
        tgt = [(r, rect) for r, rect, p in pads.get(net, []) if r.startswith("R") and p.IsOnLayer(pcbnew.F_Cu)]
        # the escape track's free end: the end point on the escape box edge
        cand = [(li, x, y) for li, x, y in ends.get(net, [])
                if min(abs(x - ESC_BOX[0]), abs(x - ESC_BOX[2]), abs(y - ESC_BOX[1]), abs(y - ESC_BOX[3])) < 0.03]
        if len(tgt) != 1 or not cand:
            print("  skipped", net, "targets", len(tgt), "escape ends", len(cand))
            continue
        ref, rect = tgt[0]
        jobs.append((net, cand[0], ref, rect))
    # Lane order: a bus running past a row of pads and peeling off one net per pad crosses
    # nothing only if the first net to peel off owns the lane nearest the pads. So the row under
    # U1 (pads entered from below) routes east to west, the R column (entered from the east)
    # south to north, and the far syncs last.
    def lane(j):
        net, _, _, r = j
        cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
        if cx < 25.0:
            return (2, cy)                 # sync column west of U1
        if cy > 34.0:
            return (0, -cx)                # row under U1: east first
        return (1, -cy)                    # R column east of U1: south first
    jobs.sort(key=lane)

    def line_ok(li, k, a, b):
        n = max(2, int(math.dist(a, b) / (G / 2)))
        for t in range(n + 1):
            j, i = cell(a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n)
            if owner[li][j, i] not in (0, k):
                return False
        return True

    def bends_ok(ps):
        for u, v, w in zip(ps, ps[1:], ps[2:]):
            a1 = math.atan2(v[1] - u[1], v[0] - u[0])
            a2 = math.atan2(w[1] - v[1], w[0] - v[0])
            if abs((a2 - a1 + math.pi) % (2 * math.pi) - math.pi) > math.pi / 4 + 1e-3:
                return False
        return True

    def attempt(order):
      done, failed, total = [], [], {}
      new_items = []
      for net, (sli, sx, sy), ref, rect in order:
          k = nid(net)
          goal = set()
          j0, i0 = cell(rect[0], rect[1])
          j1, i1 = cell(rect[2], rect[3])
          for jj in range(j0, j1 + 1):
              for ii in range(i0, i1 + 1):
                  if owner[0][jj, ii] in (0, k):
                      goal.add((0, jj, ii))
          sj, si = cell(sx, sy)
          path = astar(k, [(sli, sj, si)], goal) if goal else None
          if not path:
              failed.append(f"{net} -> {ref}")
              continue
          # cells -> runs: break at direction change, layer change, and neck boundary
          pts = [path[0]]
          for a, b, c in zip(path, path[1:], path[2:]):
              if (b[0] != a[0] or c[0] != b[0] or (b[1] - a[1], b[2] - a[2]) != (c[1] - b[1], c[2] - b[2])
                      or INNECK[a[1], a[2]] != INNECK[c[1], c[2]]):
                  pts.append(b)
          pts.append(path[-1])
          xy = lambda c: (BUNDLE_BOX[0] + c[2] * G, BUNDLE_BOX[1] + c[1] * G)  # noqa: E731
          runs, cur_li, cur = [], pts[0][0], [(sx, sy)]
          for q in pts[1:]:
              if q[0] != cur_li:
                  runs.append((cur_li, cur))
                  cur_li, cur = q[0], [xy(q)]
              else:
                  cur.append(xy(q))
          runs.append((cur_li, cur))
          L, vias = 0.0, 0
          for n_run, (li, run) in enumerate(runs):
              # merge away sub-width steps where the straight replacement clears the raster,
              # keeps 45-degree bends and does not straddle the neckdown boundary
              changed = True
              while changed and len(run) > 2:
                  changed = False
                  for q in range(1, len(run) - 1):
                      if (min(math.dist(run[q - 1], run[q]), math.dist(run[q], run[q + 1])) < 2 * W_OUT
                              and in_neck(*run[q - 1]) == in_neck(*run[q + 1])
                              and line_ok(li, k, run[q - 1], run[q + 1]) and bends_ok(run[:q] + run[q + 1:])):
                          del run[q]
                          changed = True
                          break
              if n_run:
                  new_items.append(("via", net, run[0]))
                  for lj in range(len(LAYERS)):
                      stamp(lj, net, "circle", run[0], VIA_R)
                  vias += 1
              for pa, pb in zip(run, run[1:]):
                  if math.dist(pa, pb) < 1e-6:
                      continue
                  w = W_IN if in_neck(*pa) and in_neck(*pb) else W_OUT
                  new_items.append(("track", net, li, pa, pb, w))
                  stamp(li, net, "seg", (pa, pb), w / 2)
                  L += math.dist(pa, pb)
          total[net] = L
          done.append(f"{net:9s} -> {ref:4s} {L:6.2f} mm, {vias} via(s)")
      return done, failed, total, new_items

    def region(kind, g, ext):
        """(j0, i0, bool mask) of the cells within ext + INFL of an item: where another net's
        track centre may not go."""
        r = ext + CLR_OUT + W_OUT / 2 + 0.01
        if kind == "circle":
            bx = (g[0] - r, g[1] - r, g[0] + r, g[1] + r)
        else:
            a_, b_ = g
            bx = (min(a_[0], b_[0]) - r, min(a_[1], b_[1]) - r, max(a_[0], b_[0]) + r, max(a_[1], b_[1]) + r)
        i0, i1 = max(0, int((bx[0] - BUNDLE_BOX[0]) / G)), min(NX - 1, int((bx[2] - BUNDLE_BOX[0]) / G) + 1)
        j0, j1 = max(0, int((bx[1] - BUNDLE_BOX[1]) / G)), min(NY - 1, int((bx[3] - BUNDLE_BOX[1]) / G) + 1)
        X, Y = np.meshgrid(XS[i0:i1 + 1], YS[j0:j1 + 1])
        if kind == "circle":
            d = np.hypot(X - g[0], Y - g[1])
        else:
            (ax, ay), (bx2, by2) = g
            vx, vy = bx2 - ax, by2 - ay
            t = np.clip(((X - ax) * vx + (Y - ay) * vy) / (vx * vx + vy * vy or 1e-12), 0, 1)
            d = np.hypot(X - ax - t * vx, Y - ay - t * vy)
        return j0, i0, d < ext + INFL[j0:j1 + 1, i0:i1 + 1] - 1e-6

    def build(net, k, path, sx, sy):
        """Grid path -> (items, length, vias); sub-width steps merged where the straight line
        clears the fixed obstacles, keeps 45-degree bends and stays on one side of the neck edge."""
        xy = lambda c: (BUNDLE_BOX[0] + c[2] * G, BUNDLE_BOX[1] + c[1] * G)  # noqa: E731
        pts = [path[0]]
        for a_, b_, c_ in zip(path, path[1:], path[2:]):
            if (b_[0] != a_[0] or c_[0] != b_[0] or (b_[1] - a_[1], b_[2] - a_[2]) != (c_[1] - b_[1], c_[2] - b_[2])
                    or INNECK[a_[1], a_[2]] != INNECK[c_[1], c_[2]]):
                pts.append(b_)
        pts.append(path[-1])
        runs, cur_li, cur = [], pts[0][0], [(sx, sy)]
        for q in pts[1:]:
            if q[0] != cur_li:
                runs.append((cur_li, cur))
                cur_li, cur = q[0], [xy(q)]
            else:
                cur.append(xy(q))
        runs.append((cur_li, cur))
        items, L, nv = [], 0.0, 0
        for n_run, (li, run) in enumerate(runs):
            changed = True
            while changed and len(run) > 2:
                changed = False
                for q in range(1, len(run) - 1):
                    if (min(math.dist(run[q - 1], run[q]), math.dist(run[q], run[q + 1])) < 2 * W_OUT
                            and in_neck(*run[q - 1]) == in_neck(*run[q + 1])
                            and line_ok(li, k, run[q - 1], run[q + 1]) and bends_ok(run[:q] + run[q + 1:])):
                        del run[q]
                        changed = True
                        break
            if n_run:
                items.append(("via", net, run[0]))
                nv += 1
            for pa, pb in zip(run, run[1:]):
                if math.dist(pa, pb) > 1e-6:
                    items.append(("track", net, li, pa, pb, W_IN if in_neck(*pa) and in_neck(*pb) else W_OUT))
                    L += math.dist(pa, pb)
        return items, L, nv

    def masks_of(items):
        """One boolean union per layer: a net occupies a cell once however many of its own
        segments cover it (a sum would count every joint as a conflict with itself)."""
        out = [np.zeros((NY, NX), dtype=bool) for _ in LAYERS]
        for it in items:
            parts = ([(lj, region("circle", it[2], VIA_R)) for lj in range(len(LAYERS))] if it[0] == "via"
                     else [(it[2], region("seg", (it[3], it[4]), it[5] / 2))])
            for li, (j0, i0, m) in parts:
                out[li][j0:j0 + m.shape[0], i0:i0 + m.shape[1]] |= m
        return out

    def apply(occ, ms, sign):
        for li, m in enumerate(ms):
            occ[li] += sign * m

    def negotiate(order, iters=10):
        """PathFinder-style: every net routes every iteration, sharing allowed at a cost that
        grows each round (pf) plus a per-cell history of past conflicts, until no path centre
        sits in another net's clearance region."""
        occ = [np.zeros((NY, NX), dtype=np.float32) for _ in LAYERS]
        hist = [np.zeros((NY, NX), dtype=np.float32) for _ in LAYERS]
        routes = {}                                   # net -> (items, L, vias, masks, path)
        pf = 0.5
        for it_n in range(iters):
            for net, (sli, sx, sy), ref, rect in order:
                k = nid(net)
                if net in routes:
                    apply(occ, routes[net][3], -1)
                goal = set()
                j0, i0 = cell(rect[0], rect[1])
                j1, i1 = cell(rect[2], rect[3])
                for jj in range(j0, j1 + 1):
                    for ii in range(i0, i1 + 1):
                        if owner[0][jj, ii] in (0, k):
                            goal.add((0, jj, ii))
                sj, si = cell(sx, sy)
                path = astar(k, [(sli, sj, si)], goal, occ, hist, pf) if goal else None
                if not path:
                    routes.pop(net, None)
                    continue
                items, L, nv = build(net, k, path, sx, sy)
                ms = masks_of(items)
                apply(occ, ms, +1)
                routes[net] = (items, L, nv, ms, path)
            bad = []
            for net, (items, L, nv, ms, path) in routes.items():
                c = [(li, j, i) for li, j, i in path if occ[li][j, i] > 1.5]
                for li, j, i in c:
                    hist[li][j, i] += 1.0
                if c:
                    bad.append(net)
            missing = [j[0] for j in order if j[0] not in routes]
            print(f"  negotiate iter {it_n + 1}: {len(routes)} routed, {len(bad)} in conflict, "
                  f"{len(missing)} unroutable, pf {pf:.2f}", flush=True)
            if not bad and not missing:
                break
            pf *= 1.6
        # commit in order, dropping any net that still overlaps a committed one
        committed = [np.zeros((NY, NX), dtype=np.float32) for _ in LAYERS]
        centres = [np.zeros((NY, NX), dtype=bool) for _ in LAYERS]    # committed path centres
        via_pts = []          # a via's clearance region is sized for a track, not another via
        done, failed, total, new_items = [], [], {}, []
        for net, _, ref, _ in order:
            if net not in routes:
                failed.append(f"{net} -> {ref}")
                continue
            items, L, nv, ms, path = routes[net]
            # both directions: my centre line in their clearance, and their centre line in mine
            my_vias = [it[2] for it in items if it[0] == "via"]
            if (any(committed[li][j, i] > 0 for li, j, i in path)
                    or any((ms[li] & centres[li]).any() for li in range(len(LAYERS)))
                    or any(math.dist(v, w) < 2 * VIA_R + CLR_OUT for v in my_vias for w in via_pts)):
                failed.append(f"{net} -> {ref} (conflict)")
                continue
            apply(committed, ms, +1)
            for li, j, i in path:
                centres[li][j, i] = True
            via_pts.extend(my_vias)
            new_items += items
            total[net] = L
            done.append(f"{net:9s} -> {ref:4s} {L:6.2f} mm, {nv} via(s)")
        return done, failed, total, new_items

    # Two orders on copies of the raster, keep the better: nearest target first, then the
    # first pass's failures first
    import os
    if os.environ.get("GPIF_ONLY"):
        jobs = [j for j in jobs if j[0] in os.environ["GPIF_ONLY"].split(",")]
        for net, st, ref, rect in jobs:
            k = nid(net)
            sj, si = cell(st[1], st[2])
            print("  debug", net, "start", st, "owner there", owner[st[0]][sj, si], "k", k,
                  "neighbours free", sum(owner[st[0]][sj + a, si + b] in (0, k) for a in (-1, 0, 1) for b in (-1, 0, 1)),
                  "target", ref, [round(v, 2) for v in rect])
    # Greedy (attempt) is the default: on this board it routes 22 of 29. The negotiated router
    # (--negotiate) did not converge in 10 rounds (about 20 nets left in conflict) and committed
    # 16; it stays here for the next attempt at the crossing problem.
    if "--negotiate" in sys.argv:
        result = negotiate(jobs)
    else:
        base = [o.copy() for o in owner]
        result = attempt(jobs)
        print(f"  pass 1: {len(result[0])} routed, {len(result[1])} failed")
        if result[1]:
            names = {f.split()[0] for f in result[1]}
            order2 = [j for j in jobs if j[0] in names] + [j for j in jobs if j[0] not in names]
            for li in range(len(LAYERS)):
                owner[li][:] = base[li]
            r2 = attempt(order2)
            print(f"  pass 2 (failures first): {len(r2[0])} routed, {len(r2[1])} failed")
            if len(r2[1]) < len(result[1]):
                result = r2
    done, failed, total, new_items = result
    for it in new_items:
        if it[0] == "via":
            _, net, (x, y) = it
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(OX + x), pcbnew.FromMM(OY + y)))
            v.SetWidth(pcbnew.FromMM(2 * VIA_R))
            v.SetDrill(pcbnew.FromMM(0.2))
        else:
            _, net, li, a, b, w = it
            v = pcbnew.PCB_TRACK(board)
            v.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(OX + a[0]), pcbnew.FromMM(OY + a[1])))
            v.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(OX + b[0]), pcbnew.FromMM(OY + b[1])))
            v.SetWidth(pcbnew.FromMM(w))
            v.SetLayer(LAYERS[li])
        v.SetNet(nets[net])
        v.SetLocked(True)
        board.Add(v)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(PCB))
    print(f"gpif_route: {len(done)} nets routed, {len(failed)} failed")
    for d in done:
        print("  ok", d)
    for f in failed:
        print("  FAILED", f)
    if total:
        print(f"  length spread {min(total.values()):.1f} .. {max(total.values()):.1f} mm")


if __name__ == "__main__":
    main()
