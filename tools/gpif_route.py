#!/usr/bin/env python3
"""GPIF bundle: each VD_* net from the end of its FX3 escape (tools/bga_route.py) to the
VD-side pad of its 22R series resistor next to the TVP7002 (build_pcb.py SERIES).

Runs after bga_route.py on the same board. A* on a 0.05 mm grid over BUNDLE_BOX, three
signal layers (F.Cu, In2.Cu, B.Cu), vias allowed outside the neckdown zone. The neckdown zone
is the escape box plus a 1 mm skirt (rule area BGA_ESC, widened here): 0.1 mm track at 0.1 mm
clearance inside it, 0.127 mm track at the 0.15 mm netclass clearance outside, so the bundle
fans out from the escape pitch before it widens. Nets are routed nearest-target-first; prints
per-net length and what failed.

Each route carries on from its escape stub within 45 degrees; an end sealed by its neighbours
branches off the stub instead (the stub trimmed back to the branch). The grid path is then
simplified (simplify): staircases, reversals and sub-width steps merged away and grid corners
chamfered to 45 degrees, every new segment held to the DRC clearances exactly (clear_ok).
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
CLR_MARGIN = 0.00001               # mm over the DRC clearance for off-grid segments (nm rounding)
GPIF_VIA_IN_NECK = __import__("os").environ.get("GPIF_VIA_IN_NECK", "1") == "1"
EXIT_RESERVE = 0.8                 # mm reserved straight out of each escape end
LAST_FAIL = [""]                    # why the last astar() returned None
EXP_CAP = int(__import__("os").environ.get("GPIF_EXP_CAP", "400000"))   # A* node expansions per net
VIA_KEEP = float(__import__("os").environ.get("GPIF_VIA_KEEP", "2.0"))                     # mm around the series-resistor pads with no via
BUNDLE_BOX = (14.0, 17.0, 72.0, 58.0)
LAYERS = [pcbnew.F_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
NX = int(round((BUNDLE_BOX[2] - BUNDLE_BOX[0]) / G)) + 1
NY = int(round((BUNDLE_BOX[3] - BUNDLE_BOX[1]) / G)) + 1
XS = BUNDLE_BOX[0] + np.arange(NX) * G
YS = BUNDLE_BOX[1] + np.arange(NY) * G
INNECK = ((XS[None, :] >= NECK[0]) & (XS[None, :] <= NECK[2]) &
          (YS[:, None] >= NECK[1]) & (YS[:, None] <= NECK[3]))
INFL = np.where(INNECK, CLR_IN + W_IN / 2, CLR_OUT + W_OUT / 2)    # per-cell track inflation
INBOX = ((XS[None, :] > ESC_BOX[0] + 1e-6) & (XS[None, :] < ESC_BOX[2] - 1e-6) &
         (YS[:, None] > ESC_BOX[1] + 1e-6) & (YS[:, None] < ESC_BOX[3] - 1e-6))   # strictly inside


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
    # every keep-out stamped, as exact geometry: what a segment the simplifier makes (off the
    # grid lines, where the raster says nothing between cell centres) is checked against.
    # Per layer: rows of (net id, ax, ay, bx, by, ext, centre line enters the neckdown zone)
    prims = [[] for _ in LAYERS]
    prim_arr = [None for _ in LAYERS]
    VIA_PTS = []            # every via centre, board's and ours: hole-to-hole spacing

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
        if kind == "circle":
            pr = [(g, g, ext)]
        elif kind == "rect":
            c4 = [(g[0], g[1]), (g[2], g[1]), (g[2], g[3]), (g[0], g[3])]
            pr = [(c4[n], c4[(n + 1) % 4], ext) for n in range(4)]
        else:
            pr = [(g[0], g[1], ext)]
        for a, b, e in pr:
            # in the zone, for the 0.1 mm rule: its centre line enters it (the DRC asks whether
            # the copper does, so this errs to 0.15 mm)
            n_s = max(1, int(math.dist(a, b) / 0.02))
            inz = any(in_neck(a[0] + (b[0] - a[0]) * t / n_s, a[1] + (b[1] - a[1]) * t / n_s)
                      for t in range(n_s + 1))
            prims[li].append((k, *a, *b, e, inz))
        prim_arr[li] = None

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
    segs = {}            # net -> list of (layer index, a, b, track) existing tracks
    net_vias = {}        # net -> existing via centres
    for t in board.GetTracks():
        n = t.GetNetname()
        if t.GetClass() == "PCB_VIA":
            xy = (mm(t.GetPosition().x) - OX, mm(t.GetPosition().y) - OY)
            VIA_PTS.append(xy)
            net_vias.setdefault(n, []).append(xy)
            for li in range(len(LAYERS)):
                stamp(li, n, "circle", xy, mm(t.GetWidth(pcbnew.F_Cu)) / 2)
            continue
        a = (mm(t.GetStart().x) - OX, mm(t.GetStart().y) - OY)
        b = (mm(t.GetEnd().x) - OX, mm(t.GetEnd().y) - OY)
        for li, L in enumerate(LAYERS):
            if t.GetLayer() == L:
                stamp(li, n, "seg", (a, b), mm(t.GetWidth()) / 2)
                ends.setdefault(n, []).extend([(li, *a), (li, *b)])
                segs.setdefault(n, []).append((li, a, b, t))

    n_board_vias = len(VIA_PTS)

    def cell(x, y):
        return int(round((y - BUNDLE_BOX[1]) / G)), int(round((x - BUNDLE_BOX[0]) / G))

    def pad_cells(net, rect):
        """The cells of a target pad's bounding box whose centre is on the pad itself: a route
        must end on the pad, not on a cell that rounds to just past its edge or sits in the
        rounded corner of a roundrect."""
        pad = TARGET_PAD[net]
        out = []
        for j in range(math.floor((rect[1] - BUNDLE_BOX[1]) / G), math.ceil((rect[3] - BUNDLE_BOX[1]) / G) + 1):
            for i in range(math.floor((rect[0] - BUNDLE_BOX[0]) / G), math.ceil((rect[2] - BUNDLE_BOX[0]) / G) + 1):
                x, y = BUNDLE_BOX[0] + i * G, BUNDLE_BOX[1] + j * G
                if pad.HitTest(pcbnew.VECTOR2I(pcbnew.FromMM(OX + x), pcbnew.FromMM(OY + y))):
                    out.append((j, i))
        return out

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
        # vias are allowed in the neckdown skirt too: the B.Cu escapes on the west face meet
        # the B.Cu decap column 1.5 mm out and can only leave by changing layer before it
        if NOVIA[j, i] or (INNECK[j, i] and not GPIF_VIA_IN_NECK):
            return False
        # hole to hole: 0.2 mm drills 0.4 mm apart edge to edge (board rule), any net, own too
        x, y = BUNDLE_BOX[0] + i * G, BUNDLE_BOX[1] + j * G
        if any(math.dist((x, y), v) < 0.2 + 0.4 - 1e-6 for v in VIA_PTS):
            return False
        for lj in range(len(LAYERS)):
            o = owner[lj]
            for dj, di in via_disk:
                jj, ii = j + dj, i + di
                if not (0 <= jj < NY and 0 <= ii < NX) or o[jj, ii] not in (0, k):
                    return False
        return True

    def astar(k, starts, goal, occ=None, hist=None, pf=0.0, leads=None, nobox=None):
        """starts: [(li, j, i)]; goal: set of (li, j, i). 8-connected plus vias. With occ/hist,
        a step into a cell other bundle nets occupy costs pf per occupant plus the cell's history
        (negotiated congestion); without them those cells are simply free.
        leads = {start: (ux, uy)}: the escape stub arrives at that start heading (ux, uy); the
        first step from it stays within 45 degrees of that. nobox = layer: never re-enter the
        escape box on it (the stub's own clearance band reads as free cells, and a path back
        along it doubles the copper and leaves a 0-degree spike)."""
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
            if n_exp > EXP_CAP:
                LAST_FAIL[0] = f"cap hit after {n_exp} expansions"
                return None
            li, j, i = c
            grid = owner[li]
            for dj, di, cost in steps:
                nj, ni = j + dj, i + di
                if not (0 <= nj < NY and 0 <= ni < NX) or grid[nj, ni] not in (0, k):
                    continue
                if li == nobox and INBOX[nj, ni]:
                    continue
                if par is None and leads and c in leads and (
                        (di * leads[c][0] + dj * leads[c][1]) / cost < math.cos(math.pi / 4) - 1e-3):
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
        LAST_FAIL[0] = f"search space exhausted after {n_exp} expansions"
        return None

    jobs = []
    LEAD = {}            # net -> the stub point before its escape end
    CHAIN = {}           # net -> (stub points V0..Vn, stub tracks T0..Tn-1, V0 is a via)
    TARGET_PAD = {}      # net -> its series resistor's pad
    for net in sorted(str(n) for n in nets.keys() if str(n).startswith("VD_")):
        tgt = [(r, rect) for r, rect, p in pads.get(net, []) if r.startswith("R") and p.IsOnLayer(pcbnew.F_Cu)]
        TARGET_PAD.update({net: p for r, rect, p in pads.get(net, []) if r.startswith("R") and p.IsOnLayer(pcbnew.F_Cu)})
        # the escape track's free end: the end point on the escape box edge
        cand = [(li, x, y) for li, x, y in ends.get(net, [])
                if min(abs(x - ESC_BOX[0]), abs(x - ESC_BOX[2]), abs(y - ESC_BOX[1]), abs(y - ESC_BOX[3])) < 0.03]
        if len(tgt) != 1 or not cand:
            print("  skipped", net, "targets", len(tgt), "escape ends", len(cand))
            continue
        ref, rect = tgt[0]
        jobs.append((net, cand[0], ref, rect))
        # The escape stub as a chain V0 .. Vn = (sx, sy) on its layer, V0 at the net's dogbone
        # via when there is one: a route that cannot carry straight on out of the box (its end
        # sealed by its neighbours and the decap column) may branch off anywhere along it.
        sli, sx, sy = cand[0]
        chain_p, chain_t, used = [(sx, sy)], [], set()
        while True:
            nxt = [(b if math.dist(a, chain_p[-1]) < 1e-4 else a, t) for li, a, b, t in segs.get(net, [])
                   if li == sli and id(t) not in used and math.dist(a, b) > 1e-4
                   and min(math.dist(a, chain_p[-1]), math.dist(b, chain_p[-1])) < 1e-4]
            if len(nxt) != 1 or (len(chain_p) > 1 and any(math.dist(v, chain_p[-1]) < 1e-4
                                                          for v in net_vias.get(net, []))):
                break
            used.add(id(nxt[0][1]))
            chain_p.append(nxt[0][0])
            chain_t.append(nxt[0][1])
        if len(chain_p) > 1:
            LEAD[net] = chain_p[1]
            CHAIN[net] = (chain_p[::-1], chain_t[::-1],
                          any(math.dist(v, chain_p[-1]) < 1e-4 for v in net_vias.get(net, [])))
    # Lane order: a bus running past a row of pads and peeling off one net per pad crosses
    # nothing only if the first net to peel off owns the lane nearest the pads. So the row under
    # U1 (pads entered from below) routes east to west, the R column (entered from the east)
    # south to north, and the far syncs last.
    def lane(j):
        net, _, _, r = j
        cx, cy = (r[0] + r[2]) / 2, (r[1] + r[3]) / 2
        if cy > 34.0:
            return (0, -cx)                # row under U1: east first
        return (1, -cy)                    # R column east of U1: south first
    jobs.sort(key=lane)

    # Every escape end sits on the escape-box edge next to its neighbours', so a net routed
    # early can run straight across a later net's end and seal it in (measured: the nets that
    # failed did so after ~600-2300 expansions, search space exhausted, not on the cap).
    # Reserve each end's first EXIT_RESERVE mm straight out of the box for its own net.
    for net, (sli, sx, sy), _, _ in jobs:
        d = {abs(sx - ESC_BOX[0]): (-1, 0), abs(sx - ESC_BOX[2]): (1, 0),
             abs(sy - ESC_BOX[1]): (0, -1), abs(sy - ESC_BOX[3]): (0, 1)}
        ux, uy = d[min(d)]
        stamp(sli, net, "seg", ((sx, sy), (sx + ux * EXIT_RESERVE, sy + uy * EXIT_RESERVE)), W_IN / 2)

    def start_tiers(net, sli, sx, sy):
        """Ways to start, tried in order; each is (starts, leads, nobox) for astar with
        starts = {node: (start point, point the copper arrives from, stub trim)}.
          1. the escape end, carrying straight on out of the box;
          2. anywhere along the net's own escape stub (or its dogbone via, on another layer),
             the stub then trimmed back to there: sealed ends need a via inside the box, and
             a path that walks back along the stub to one doubles the copper;
          3. the escape end, free (no stub known)."""
        sj, si = cell(sx, sy)
        tiers = []
        if net in LEAD:
            px, py = LEAD[net]
            n = math.hypot(sx - px, sy - py)
            node = (sli, sj, si)
            tiers.append(({node: ((sx, sy), LEAD[net], None)}, {node: ((sx - px) / n, (sy - py) / n)}, sli))
        if net in CHAIN:
            V, _, via0 = CHAIN[net]
            st, leads = {}, {}
            for m in range(len(V) - 2, -1, -1):          # from the escape end inwards
                a, b = V[m], V[m + 1]
                Ls = math.dist(a, b)
                e = ((b[0] - a[0]) / Ls, (b[1] - a[1]) / Ls)
                for t in [Ls] + [n_s * G for n_s in range(int(Ls / G + 1e-6), 0, -1) if n_s * G < Ls - 1e-6]:
                    j, i = cell(a[0] + e[0] * t, a[1] + e[1] * t)
                    cx, cy = BUNDLE_BOX[0] + i * G, BUNDLE_BOX[1] + j * G
                    tp = min(Ls, max(1e-6, (cx - a[0]) * e[0] + (cy - a[1]) * e[1]))
                    D = (a[0] + e[0] * tp, a[1] + e[1] * tp)
                    if math.dist(D, (cx, cy)) > 0.75 * G or (sli, j, i) in st:
                        continue
                    D = (round(D[0], 6), round(D[1], 6))
                    st[(sli, j, i)] = (D, a, (m, D))
                    leads[(sli, j, i)] = e
            if via0:
                j, i = cell(*V[0])
                for lj in range(len(LAYERS)):
                    if lj != sli:
                        st[(lj, j, i)] = (V[0], None, (-1, V[0]))
            tiers.append((st, leads, None))
        tiers.append(({(sli, sj, si): ((sx, sy), LEAD.get(net), None)}, {}, None))
        return tiers

    def slide_of(net, trim):
        """Where a start on the escape stub may move to: (point, stub point before it, track
        index m) every G/2 along the whole chain; None when the start is fixed."""
        if not trim or trim[0] < 0:
            return None
        V = CHAIN[net][0]
        opts = []
        for m in range(len(V) - 1):
            a, b = V[m], V[m + 1]
            Ls = math.dist(a, b)
            for n_s in range(1, int(Ls / (G / 2) + 1e-6) + 2):
                t = min(Ls, n_s * G / 2)
                opts.append(((round(a[0] + (b[0] - a[0]) * t / Ls, 6), round(a[1] + (b[1] - a[1]) * t / Ls, 6)), a, m))
                if t >= Ls:
                    break
        return opts

    def route_net(net, k, sli, sx, sy, goal, occ=None, hist=None, pf=0.0):
        """(path, start point, arrival point, stub trim) or None."""
        for n_t, (st, leads, nobox) in enumerate(start_tiers(net, sli, sx, sy)):
            path = astar(k, list(st), goal, occ, hist, pf, leads, nobox)
            if path:
                if n_t and occ is None:
                    print(f"  {net}: start tier {n_t + 1}", flush=True)
                return (path, *st[path[0]])
        return None

    def clear_ok(li, k, a, b, w, zone):
        """Exact clearance of track a-b (width w; zone: it touches the neckdown zone) from every
        other net's stamped copper on layer li: the DRC rule's 0.1 mm where both are in the
        zone, 0.15 mm otherwise; plus CLR_MARGIN for an off-grid segment."""
        if prim_arr[li] is None:
            prim_arr[li] = np.array(prims[li], dtype=float).reshape(-1, 7)
        P = prim_arr[li]
        r = w / 2 + CLR_OUT + 0.5
        m = ((P[:, 0] != k) & (np.minimum(P[:, 1], P[:, 3]) - P[:, 5] < max(a[0], b[0]) + r)
             & (np.maximum(P[:, 1], P[:, 3]) + P[:, 5] > min(a[0], b[0]) - r)
             & (np.minimum(P[:, 2], P[:, 4]) - P[:, 5] < max(a[1], b[1]) + r)
             & (np.maximum(P[:, 2], P[:, 4]) + P[:, 5] > min(a[1], b[1]) - r))
        Q = P[m]
        if not len(Q):
            return True
        ax, ay, bx, by = Q[:, 1], Q[:, 2], Q[:, 3], Q[:, 4]

        def pt_seg(px, py, x0, y0, x1, y1):
            vx, vy = x1 - x0, y1 - y0
            L2 = vx * vx + vy * vy
            t = np.clip(((px - x0) * vx + (py - y0) * vy) / np.where(L2 > 0, L2, 1.0), 0, 1)
            return np.hypot(px - x0 - t * vx, py - y0 - t * vy)

        d = np.minimum.reduce([pt_seg(a[0], a[1], ax, ay, bx, by), pt_seg(b[0], b[1], ax, ay, bx, by),
                               pt_seg(ax, ay, a[0], a[1], b[0], b[1]), pt_seg(bx, by, a[0], a[1], b[0], b[1])])
        o1 = (b[0] - a[0]) * (ay - a[1]) - (b[1] - a[1]) * (ax - a[0])
        o2 = (b[0] - a[0]) * (by - a[1]) - (b[1] - a[1]) * (bx - a[0])
        o3 = (bx - ax) * (a[1] - ay) - (by - ay) * (a[0] - ax)
        o4 = (bx - ax) * (b[1] - ay) - (by - ay) * (b[0] - ax)
        d = np.where((o1 * o2 < 0) & (o3 * o4 < 0), 0.0, d)
        # ends on the grid are whole nanometres, so the distance is exact there: no margin
        on_grid = all(abs(v / G - round(v / G)) < 1e-6 for v in (a[0] - BUNDLE_BOX[0], a[1] - BUNDLE_BOX[1],
                                                               b[0] - BUNDLE_BOX[0], b[1] - BUNDLE_BOX[1]))
        need = (w / 2 + Q[:, 5] + np.where((Q[:, 6] > 0) & zone, CLR_IN, CLR_OUT)
                + (-1e-7 if on_grid else CLR_MARGIN))
        return bool((d >= need).all())

    def attempt(order):
      del VIA_PTS[n_board_vias:]           # a second pass starts from the board's own vias
      done, failed, total = [], [], {}
      new_items = []
      for net, (sli, sx, sy), ref, rect in order:
          k = nid(net)
          goal = set()
          for jj, ii in pad_cells(net, rect):
              if owner[0][jj, ii] in (0, k):
                  goal.add((0, jj, ii))
          LAST_FAIL[0] = "no goal cell free" if not goal else ""
          r = route_net(net, k, sli, sx, sy, goal) if goal else None
          if not r:
              failed.append(f"{net} -> {ref}: {LAST_FAIL[0]}")
              continue
          path, p0, pre, trim = r
          items, L, vias, p0, m_end = build(net, k, path, *p0, pre, slide_of(net, trim), trim and trim[0])
          if trim:
              items.append(("trim", net, m_end, p0))
          for it in items:
              if it[0] == "via":
                  VIA_PTS.append(it[2])
                  for lj in range(len(LAYERS)):
                      stamp(lj, net, "circle", it[2], VIA_R)
              elif it[0] == "track":
                  stamp(it[2], net, "seg", (it[3], it[4]), it[5] / 2)
          new_items += items
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

    def seg_ok(li, k, a, b):
        """A replacement segment a-b: a 0.127 mm segment (either end outside the neckdown zone)
        does not reach into the zone except at an end on its edge, and the segment clears every
        other net's copper (clear_ok)."""
        if not (in_neck(*a) and in_neck(*b)):
            n = max(2, int(math.dist(a, b) / (G / 4)))
            for t in range(1, n):
                x, y = a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n
                if NECK[0] + 1e-6 < x < NECK[2] - 1e-6 and NECK[1] + 1e-6 < y < NECK[3] - 1e-6:
                    return False
            for e in (a, b):
                if (NECK[0] + 1e-6 < e[0] < NECK[2] - 1e-6 and NECK[1] + 1e-6 < e[1] < NECK[3] - 1e-6):
                    return False
        # width by the neckdown rule; clearance by the DRC's: a track that so much as touches
        # BGA_ESC (an end on its edge) gets the 0.1 mm against copper that is in it too
        return clear_ok(li, k, a, b, W_IN if in_neck(*a) and in_neck(*b) else W_OUT,
                        in_neck(*a) or in_neck(*b))

    def turn(u, v, w):
        """Direction change at v in degrees (0 = straight on); None if a leg is degenerate."""
        a = (v[0] - u[0], v[1] - u[1])
        b = (w[0] - v[0], w[1] - v[1])
        na, nb = math.hypot(*a), math.hypot(*b)
        if na < 1e-9 or nb < 1e-9:
            return None
        return math.degrees(math.acos(max(-1.0, min(1.0, (a[0] * b[0] + a[1] * b[1]) / (na * nb)))))

    MAXTURN = 45.0 + 1e-3
    SHORT = 2 * W_OUT          # segments under this are merged away where the geometry allows

    def simplify(li, k, run, pre, slide=None, m0=None):
        """Grid run (fixed ends) -> polyline without sub-width steps, reversals or turns over
        45 degrees, as far as the clearances allow. pre: the point the copper arrives from before
        run[0] (the escape stub), or None.

        Every candidate is normalised (a vertex inserted where a segment crosses the neckdown
        edge, so the 0.1 / 0.127 mm split stays on it), every new segment must pass seg_ok, and
        the candidate must strictly lower (sharp turns, short segments, length, vertices): so
        the loop ends, and no move trades a short step for a sharp turn. Moves, in order:
          - drop a vertex (collinear points, reversals, staircase steps), or two or three in
            a row (pairwise where a whole staircase does not clear);
          - a short step: extend its neighbours to meet, or rebuild the corner they make as the
            longest chamfer that fits;
          - slide a neckdown-edge vertex along the edge;
          - a corner sharper than 45 degrees: the longest chamfer that fits;
          - slide (start tier 2): run[0] branches off the escape stub and may move anywhere
            along it (slide_of; the stub is trimmed to wherever it ends up), keeping the stump
            of the stub track it lands on >= SHORT.
        Returns (run, point the copper arrives at run[0] from, stub track index)."""
        PRE, M = [pre], [m0]
        cache = {}

        def ok_seg(a, b):
            key_ = (a, b)
            if key_ not in cache:
                cache[key_] = cache[(b, a)] = seg_ok(li, k, a, b)
            return cache[key_]

        def bend_at(ps, q):
            u = ps[q - 1] if q > 0 else PRE[0]
            if u is None or q + 1 >= len(ps):
                return 0.0
            t = turn(u, ps[q], ps[q + 1])
            return 180.0 if t is None else t

        def edge_cross(a, b):
            """Where a-b (one end strictly inside the neckdown zone, one outside) leaves it."""
            if not in_neck(*a):
                a, b = b, a
            t_out = 1.0
            for v0, v1, lo, hi in ((a[0], b[0], NECK[0], NECK[2]), (a[1], b[1], NECK[1], NECK[3])):
                if v1 > hi:
                    t_out = min(t_out, (hi - v0) / (v1 - v0))
                elif v1 < lo:
                    t_out = min(t_out, (lo - v0) / (v1 - v0))
            return (round(a[0] + (b[0] - a[0]) * t_out, 6), round(a[1] + (b[1] - a[1]) * t_out, 6))

        def normal(ps):
            out = [ps[0]]
            for b in ps[1:]:
                a = out[-1]
                if in_neck(*a) != in_neck(*b):
                    x = edge_cross(a, b)
                    if min(math.dist(x, a), math.dist(x, b)) > 1e-6:
                        out.append(x)
                if math.dist(out[-1], b) > 1e-6:
                    out.append(b)
            return out

        def key(ps):
            stump = bool(slide) and math.dist(PRE[0], ps[0]) < SHORT
            return (sum(bend_at(ps, q) > MAXTURN for q in range(len(ps))),
                    sum(math.dist(a, b) < SHORT for a, b in zip(ps, ps[1:])) + stump,
                    round(sum(math.dist(a, b) for a, b in zip(ps, ps[1:])), 5),
                    len(ps))

        def accept_slide(ps, cand, opt):
            """accept() for a candidate whose start moved along the stub to opt."""
            old = PRE[0], M[0]
            k_old = key(ps)
            PRE[0], M[0] = opt[1], opt[2]
            cand = normal(cand)
            if cand[0] == opt[0] and cand[-1] == ps[-1] and key(cand) < k_old and all(
                    ok_seg(a, b) for a, b in zip(cand, cand[1:])):
                return cand
            PRE[0], M[0] = old
            return None

        def accept(ps, cand):
            cand = normal(cand)
            if cand[0] != ps[0] or cand[-1] != ps[-1] or key(cand) >= key(ps):
                return None
            if not all(ok_seg(a, b) for a, b in zip(cand, cand[1:])):
                return None
            return cand

        def corner(ps, lo, hi, x):
            """ps[lo..hi] -> a chamfered corner at x between legs ps[lo-1]->x and x->ps[hi+1],
            longest chamfer first, no leg left under SHORT."""
            a, d = ps[lo - 1], ps[hi + 1]
            lin, lout = math.dist(a, x), math.dist(x, d)
            if lin < 1e-6 or lout < 1e-6:
                return None
            e_in = ((x[0] - a[0]) / lin, (x[1] - a[1]) / lin)
            e_out = ((d[0] - x[0]) / lout, (d[1] - x[1]) / lout)
            cap_in = lin if lo - 1 == 0 else lin / 2       # leave the far corner its share
            cap_out = lout if hi + 1 == len(ps) - 1 else lout / 2
            for n_c in range(int(min(cap_in, cap_out, 1.5) / (G / 2) + 1e-6), 0, -1):
                c = n_c * G / 2
                ci, co = min(c, lin), min(c, lout)
                if 1e-6 < lin - ci < SHORT or 1e-6 < lout - co < SHORT:
                    continue
                p1 = a if lin - ci <= 1e-6 else (round(x[0] - e_in[0] * ci, 6), round(x[1] - e_in[1] * ci, 6))
                p2 = d if lout - co <= 1e-6 else (round(x[0] + e_out[0] * co, 6), round(x[1] + e_out[1] * co, 6))
                cand = accept(ps, ps[:lo - 1] + [a, p1, p2, d] + ps[hi + 2:])
                if cand:
                    return cand
            return None

        def meet(a, b, c, d):
            """Where line a-b, extended past b, meets line d-c extended past c."""
            u = (b[0] - a[0], b[1] - a[1])
            w = (d[0] - c[0], d[1] - c[1])
            den = u[0] * w[1] - u[1] * w[0]
            if abs(den) < 1e-9:
                return None
            t = ((c[0] - a[0]) * w[1] - (c[1] - a[1]) * w[0]) / den
            s_ = ((c[0] - a[0]) * u[1] - (c[1] - a[1]) * u[0]) / den
            x = (a[0] + t * u[0], a[1] + t * u[1])
            if t > 1e-6 and s_ < 1 - 1e-6 and max(math.dist(x, b), math.dist(x, c)) < 1.0:
                return (round(x[0], 6), round(x[1], 6))
            return None

        def on_edge(p):
            return in_neck(*p) and not (NECK[0] + 1e-6 < p[0] < NECK[2] - 1e-6 and NECK[1] + 1e-6 < p[1] < NECK[3] - 1e-6)

        def moves(ps):
            n = len(ps)
            if slide and n > 1:                            # slide the branch point along the stub
                for opt in slide:
                    if math.dist(opt[0], ps[0]) > 1e-6 and math.dist(opt[0], ps[0]) < 2.0:
                        for drop in range(0, min(3, n - 1)):    # and maybe drop what it bypasses
                            yield lambda opt=opt, drop=drop: accept_slide(ps, [opt[0]] + ps[1 + drop:], opt)
            for q in range(1, n - 1):                      # collinear / reversal
                t = turn(ps[q - 1], ps[q], ps[q + 1])
                if t is not None and (t < 1e-3 or t > 180 - 1e-3):
                    yield lambda q=q: accept(ps, ps[:q] + ps[q + 1:])
            for Lq, q in sorted((math.dist(ps[q], ps[q + 1]), q) for q in range(n - 1)):
                if Lq >= SHORT:
                    break
                if 0 < q and q + 2 < n:
                    x = meet(ps[q - 1], ps[q], ps[q + 1], ps[q + 2])
                    if x is not None:
                        yield lambda q=q, x=x: accept(ps, ps[:q] + [x] + ps[q + 2:])
                # drop one, two or three vertices in a row at the step (a staircase collapses
                # into one segment, or pairwise where the whole run does not clear)
                for w_ in (1, 2, 3):
                    for st in range(max(1, q + 1 - w_), min(q + 2, n - w_)):
                        yield lambda st=st, w_=w_: accept(ps, ps[:st] + ps[st + w_:])
                if 0 < q and q + 2 < n:
                    x = meet(ps[q - 1], ps[q], ps[q + 1], ps[q + 2])
                    if x is not None:
                        yield lambda q=q, x=x: corner(ps, q, q + 1, x)
                for qq in (q, q + 1):
                    if 0 < qq < n - 1:
                        yield lambda qq=qq: corner(ps, qq, qq, ps[qq])
                for qq in (q, q + 1):                      # slide an edge vertex along the edge
                    if 0 < qq < n - 1 and on_edge(ps[qq]):
                        p = ps[qq]
                        vert = abs(p[0] - NECK[0]) < 1e-6 or abs(p[0] - NECK[2]) < 1e-6
                        for m in range(1, 9):
                            for sg in (1, -1):
                                dlt = sg * m * G / 2
                                p2 = (p[0], round(p[1] + dlt, 6)) if vert else (round(p[0] + dlt, 6), p[1])
                                yield lambda qq=qq, p2=p2: accept(ps, ps[:qq] + [p2] + ps[qq + 1:])
            for q in range(1, n - 1):
                if bend_at(ps, q) > MAXTURN:
                    yield lambda q=q: corner(ps, q, q, ps[q])

        run = normal([run[0]] + [p for p, q in zip(run[1:], run) if math.dist(p, q) > 1e-6])
        for a, b in zip(run, run[1:]):       # A*'s own segments stand; only new ones are checked
            cache[(a, b)] = cache[(b, a)] = True
        for _ in range(5000):
            for mv in moves(run):
                cand = mv()
                if cand:
                    run = cand
                    break
            else:
                break
        return run, PRE[0], M[0]

    def build(net, k, path, sx, sy, pre=None, slide=None, m0=None):
        """Grid path -> (items, length, vias). The path is cut into runs at layer changes and at
        the neckdown edge (one vertex on the edge cell: 0.1 mm inside, 0.127 mm outside), and each
        run is simplified (simplify above). pre: where the escape stub arrives at (sx, sy) from."""
        xy = lambda c: (BUNDLE_BOX[0] + c[2] * G, BUNDLE_BOX[1] + c[1] * G)  # noqa: E731
        pts = [path[0]]
        for a_, b_, c_ in zip(path, path[1:], path[2:]):
            if (b_[0] != a_[0] or c_[0] != b_[0] or (b_[1] - a_[1], b_[2] - a_[2]) != (c_[1] - b_[1], c_[2] - b_[2])
                    or (INNECK[b_[1], b_[2]] and not (INNECK[a_[1], a_[2]] and INNECK[c_[1], c_[2]]))):
                pts.append(b_)
        if len(path) > 1:
            pts.append(path[-1])
        runs, cur_li, cur = [], pts[0][0], [(sx, sy)]
        for q in pts[1:]:
            if q[0] != cur_li:
                runs.append((cur_li, cur))
                cur_li, cur = q[0], [xy(q)]
            else:
                cur.append(xy(q))
        runs.append((cur_li, cur))
        items, L, nv, start, m_end = [], 0.0, 0, (sx, sy), m0
        for n_run, (li, run) in enumerate(runs):
            if len(run) > 1:
                if n_run == 0:
                    run, _, m_end = simplify(li, k, run, pre, slide, m0)
                else:
                    run = simplify(li, k, run, None)[0]
            if n_run == 0:
                start = run[0]
            if n_run:
                items.append(("via", net, run[0]))
                nv += 1
            for pa, pb in zip(run, run[1:]):
                if math.dist(pa, pb) > 1e-6:
                    items.append(("track", net, li, pa, pb, W_IN if in_neck(*pa) and in_neck(*pb) else W_OUT))
                    L += math.dist(pa, pb)
        return items, L, nv, start, m_end

    def masks_of(items):
        """One boolean union per layer: a net occupies a cell once however many of its own
        segments cover it (a sum would count every joint as a conflict with itself)."""
        out = [np.zeros((NY, NX), dtype=bool) for _ in LAYERS]
        for it in items:
            if it[0] not in ("via", "track"):
                continue
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
                for jj, ii in pad_cells(net, rect):
                    if owner[0][jj, ii] in (0, k):
                        goal.add((0, jj, ii))
                r = route_net(net, k, sli, sx, sy, goal, occ, hist, pf) if goal else None
                if not r:
                    routes.pop(net, None)
                    continue
                path, p0, pre, trim = r
                items, L, nv, p0, m_end = build(net, k, path, *p0, pre, slide_of(net, trim), trim and trim[0])
                if trim:
                    items.append(("trim", net, m_end, p0))
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
        base_p = [list(pl) for pl in prims]
        result = attempt(jobs)
        print(f"  pass 1: {len(result[0])} routed, {len(result[1])} failed")
        if result[1]:
            names = {f.split()[0] for f in result[1]}
            order2 = [j for j in jobs if j[0] in names] + [j for j in jobs if j[0] not in names]
            for li in range(len(LAYERS)):
                owner[li][:] = base[li]
                prims[li][:] = base_p[li]
                prim_arr[li] = None
            r2 = attempt(order2)
            print(f"  pass 2 (failures first): {len(r2[0])} routed, {len(r2[1])} failed")
            if len(r2[1]) < len(result[1]):
                result = r2
    done, failed, total, new_items = result
    # Stub trims first. A route that branched off its escape stub at D (start tier 2) leaves the
    # stub past D redundant: the track holding D is cut back to it, and the tracks beyond are
    # reused for the route's own segments (pcbnew is not asked to Remove anything).
    spare = {}                                   # net -> stub tracks free for reuse
    trimmed = 0.0
    for it in new_items:
        if it[0] != "trim":
            continue
        _, net, m, D = it
        V, T, _ = CHAIN[net]
        if m >= 0 and math.dist(D, V[m + 1]) > 1e-6:
            t = T[m]
            far = V[m + 1]
            pt = pcbnew.VECTOR2I(pcbnew.FromMM(OX + D[0]), pcbnew.FromMM(OY + D[1]))
            if math.dist((mm(t.GetEnd().x) - OX, mm(t.GetEnd().y) - OY), far) < 1e-4:
                t.SetEnd(pt)
            else:
                t.SetStart(pt)
            trimmed += math.dist(D, far)
        for t in T[m + 1:]:
            trimmed += mm(t.GetLength())
            spare.setdefault(net, []).append(t)
    for it in new_items:
        if it[0] == "trim":
            continue
        if it[0] == "via":
            _, net, (x, y) = it
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(OX + x), pcbnew.FromMM(OY + y)))
            v.SetWidth(pcbnew.FromMM(2 * VIA_R))
            v.SetDrill(pcbnew.FromMM(0.2))
        else:
            _, net, li, a, b, w = it
            reuse = bool(spare.get(net))
            v = spare[net].pop() if reuse else pcbnew.PCB_TRACK(board)
            v.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(OX + a[0]), pcbnew.FromMM(OY + a[1])))
            v.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(OX + b[0]), pcbnew.FromMM(OY + b[1])))
            v.SetWidth(pcbnew.FromMM(w))
            v.SetLayer(LAYERS[li])
            if reuse:
                continue
        v.SetNet(nets[net])
        v.SetLocked(True)
        board.Add(v)
    left = [n for n, lst in spare.items() if lst]
    if left:
        sys.exit(f"gpif_route: stub tracks left over after trimming: {left}")
    if trimmed:
        print(f"  escape stubs trimmed back by {trimmed:.2f} mm in all")
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
