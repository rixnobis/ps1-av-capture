#!/usr/bin/env python3
"""Planes, stitching vias, power pours and the Freerouting hand-off for the hat.

  route_hat.py prep    - GND/power zones, vias for every SMD GND and power pad, buck loop on L1,
                         DSN for Freerouting (build/ps1-av-hat.dsn)
  route_hat.py import  - SES back into the board, stitching, zone refill, save

Stackup (docs/stackup.md): L1 signals+parts, In1 solid GND, In2 power pours (+ GND fill), B.Cu
signals + GND fill. Every SMD GND pad gets a via to In1 next to it, every pour-net pad gets a
via to its In2 pour. The In2 pours are a tree of strips found by a grid router (ownership raster),
then drawn as zones. For the DSN the pour strips are also written as In2 tracks so Freerouting
sees the pour nets as already connected; those tracks are removed again before the pours are
filled. Freerouting is not deterministic: the build gates on DRC and check_geometry.py only.
"""
import heapq
import math
import sys
from pathlib import Path

import numpy as np
import pcbnew
from pcbnew import FromMM as MM, ToMM as TOMM, VECTOR2I

HAT = Path(__file__).resolve().parent.parent
PCB = HAT / "ps1-av-hat.kicad_pcb"
DSN = HAT / "build" / "ps1-av-hat.dsn"
SES = HAT / "build" / "ps1-av-hat.ses"
ROUTE = HAT / "build" / "route.kicad_pcb"

VIA_D, VIA_DRILL = 0.45, 0.2
CLR = 0.17                  # clearance used when placing (netclass 0.15 plus margin)
GRID = 0.25
# pour nets, hardest first: (net, strip width mm)
POURS = [("+3V3", 1.5), ("+5V", 1.5), ("+2V5", 1.25), ("+3V3A", 1.0), ("+1V9D", 1.0),
         ("TVP_AVDD", 1.0), ("+3V3_AUD", 1.0), ("TVP_PLLVDD", 0.75), ("OSC_VDD", 0.75),
         ("+1V9A", 0.75), ("VBUS_AUD", 1.0)]
STUB_W = {"+5V": 0.6, "+2V5": 0.6}
POUR_NETS = [n for n, _ in POURS]
L1, IN1, IN2, L4 = pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu


def v2(x, y):
    return VECTOR2I(MM(float(x)), MM(float(y)))


class Obstacles:
    """Copper that new vias and stubs must keep clear of, with a numpy bbox prefilter."""

    def __init__(self):
        self.items = []     # (net, shape, owner)
        self.bb = np.zeros((0, 4))
        self._rows = []

    def add(self, net, shape, owner=None, layer=None):
        b = shape.BBox()
        self._rows.append((TOMM(b.GetX()), TOMM(b.GetY()), TOMM(b.GetRight()), TOMM(b.GetBottom())))
        self.items.append((net, shape, owner, layer))
        self.bb = None

    def near(self, x0, y0, x1, y1, m):
        if self.bb is None:
            self.bb = np.array(self._rows).reshape(-1, 4)
        bb = self.bb
        idx = np.nonzero((bb[:, 0] <= x1 + m) & (bb[:, 2] >= x0 - m) &
                         (bb[:, 1] <= y1 + m) & (bb[:, 3] >= y0 - m))[0]
        return [self.items[i] for i in idx]

    def hit(self, shape, clr, skip_owner=None, same_net_ok=None):
        b = shape.BBox()
        for net, sh, owner, lay in self.near(TOMM(b.GetX()), TOMM(b.GetY()), TOMM(b.GetRight()),
                                        TOMM(b.GetBottom()), clr):
            if owner is not None and owner == skip_owner:
                continue
            if same_net_ok is not None and net == same_net_ok and clr > 0:
                pass
            if pcbnew.SHAPE.Collide(sh, shape, MM(clr)):
                return True
        return False


class Ctx:
    def __init__(self, board, tracks=False):
        self.b = board
        self.ol = pcbnew.SHAPE_POLY_SET()
        board.GetBoardPolygonOutlines(self.ol, True)
        o = self.ol.COutline(0)
        P = [(o.CPoint(i).x, o.CPoint(i).y) for i in range(o.PointCount())]
        self.edges = [(VECTOR2I(*P[i]), VECTOR2I(*P[(i + 1) % len(P)])) for i in range(len(P))]
        self.obs = Obstacles()
        self.vias = []          # (net, x, y)
        self.new = []
        self.skip = set()       # (ref, pad) already joined by hand
        for fp in board.GetFootprints():
            for p in fp.Pads():
                if p.IsOnLayer(L1) or p.IsOnLayer(L4) or p.GetDrillSize().x > 0:
                    self.obs.add(p.GetNetname(), p.GetEffectiveShape(L1), pad_key(p),
                             None if p.GetDrillSize().x > 0 else (L1 if p.IsOnLayer(L1) else L4))
        if tracks:      # tracks and vias already on the board (after the SES import)
            for t in board.GetTracks():
                if t.GetClass() == "PCB_VIA":
                    self.obs.add(t.GetNetname(), pcbnew.SHAPE_CIRCLE(t.GetPosition(), pcbnew.Cast_to_PCB_VIA(t).GetWidth(L1) // 2), None)
                else:
                    self.obs.add(t.GetNetname(), pcbnew.SHAPE_SEGMENT(t.GetStart(), t.GetEnd(), t.GetWidth()), None, t.GetLayer())
        self.nets = {}
        for n in board.GetNetsByName():
            self.nets[str(n)] = board.FindNet(str(n))

    def edge_dist(self, x, y):
        best = 1e9
        for a, b in self.edges:
            ax, ay, bx, by = TOMM(a.x), TOMM(a.y), TOMM(b.x), TOMM(b.y)
            dx, dy = bx - ax, by - ay
            t = max(0, min(1, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
            best = min(best, math.hypot(x - ax - t * dx, y - ay - t * dy))
        return best

    def inside(self, x, y):
        return self.ol.Collide(v2(x, y), 0)

    def add_via(self, net, x, y):
        v = pcbnew.PCB_VIA(self.b)
        v.SetPosition(v2(x, y))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        try:
            v.SetWidth(MM(VIA_D))
        except TypeError:
            v.SetWidth(L1, MM(VIA_D))
        v.SetDrill(MM(VIA_DRILL))
        v.SetNet(self.nets[net])
        v.SetLayerPair(L1, L4)
        v.SetLocked(True)
        self.b.Add(v)
        v.thisown = 0
        self.vias.append((net, x, y))
        self.obs.add(net, pcbnew.SHAPE_CIRCLE(v2(x, y), MM(VIA_D / 2)), None)
        return v

    def add_track(self, net, a, b, w, layer):
        t = pcbnew.PCB_TRACK(self.b)
        t.SetStart(v2(*a))
        t.SetEnd(v2(*b))
        t.SetWidth(MM(w))
        t.SetLayer(layer)
        t.SetNet(self.nets[net])
        t.SetLocked(True)       # Freerouting keeps locked items (type protect in the DSN)
        self.b.Add(t)
        t.thisown = 0
        if layer == L1:
            self.obs.add(net, pcbnew.SHAPE_SEGMENT(v2(*a), v2(*b), MM(w)), None, L1)
        return t


def pad_key(p):
    return (p.GetParentFootprint().GetReference(), p.GetNumber())


def stub_width(pad, net):
    cls = STUB_W.get(net, 0.4 if net in POUR_NETS or net == "GND" else 0.2)
    sz = pad.GetSize()
    return min(cls, TOMM(min(sz.x, sz.y)))


DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1), (0.7071, 0.7071), (0.7071, -0.7071),
        (-0.7071, 0.7071), (-0.7071, -0.7071)]


def place_pad_via(ctx, pad, net, tmax=4.0, join=True):
    """Via near an SMD pad with a short L1 stub. Returns (via xy, distance pad edge to via centre)."""
    c = pad.GetPosition()
    cx, cy = TOMM(c.x), TOMM(c.y)
    shape = pad.GetEffectiveShape(L1)
    w = stub_width(pad, net)
    cands = []
    fp = pad.GetParentFootprint()
    big = len(fp.Pads()) > 40           # the TVP7002: vias go inside the pad ring, out of the escape channels
    fc = fp.GetPosition()
    ix, iy = TOMM(fc.x) - cx, TOMM(fc.y) - cy
    for k, (dx, dy) in enumerate(DIRS):
        pen = 0.0 if not big or dx * ix + dy * iy > 0 else 1.5
        t = 0.25
        while t <= tmax:
            cands.append((t + (0.0 if k < 4 else 0.25) + pen, t, dx, dy))
            t += 0.05
    cands.sort()
    for _, t, dx, dy in cands:
        x, y = cx + dx * t, cy + dy * t
        if not ctx.inside(x, y) or ctx.edge_dist(x, y) < 0.55 + VIA_D / 2:
            continue
        via = pcbnew.SHAPE_CIRCLE(v2(x, y), MM(VIA_D / 2))
        if pcbnew.SHAPE.Collide(shape, via, MM(0.16)):
            continue
        if ctx.obs.hit(via, CLR, skip_owner=pad_key(pad)):
            continue
        seg = pcbnew.SHAPE_SEGMENT(c, v2(x, y), MM(w))
        if ctx.obs.hit(seg, CLR, skip_owner=pad_key(pad)):
            continue
        ctx.add_via(net, x, y)
        ctx.add_track(net, (cx, cy), (x, y), w, L1)
        edge_gap = 0.0 if pcbnew.SHAPE.Collide(shape, pcbnew.SHAPE_CIRCLE(v2(x, y), 1), MM(0.5)) else 9.9
        return (x, y), edge_gap
    return None, None


# --------------------------------------------------------------------------- pour router
def build_pours(ctx, terminals):
    """terminals: {net: [(x, y), ...]}. Returns {net: set of (ix, iy) owned cells}, failures."""
    bb = ctx.ol.BBox()
    ox, oy = TOMM(bb.GetX()) - 1, TOMM(bb.GetY()) - 1
    W = int((TOMM(bb.GetWidth()) + 2) / GRID) + 2
    H = int((TOMM(bb.GetHeight()) + 2) / GRID) + 2
    X, Y = np.meshgrid(np.arange(W), np.arange(H), indexing="ij")
    px, py = ox + X * GRID, oy + Y * GRID

    def disc_mask(cx, cy, r):
        return (px - cx) ** 2 + (py - cy) ** 2 <= r * r

    def cell(x, y):
        return (int(round((x - ox) / GRID)), int(round((y - oy) / GRID)))

    pts = [(TOMM(a.x), TOMM(a.y)) for a, _ in ctx.edges]
    inside = np.zeros((W, H), bool)
    for k in range(len(pts)):          # even-odd ray cast, vectorised
        (x0, y0), (x1, y1) = pts[k], pts[(k + 1) % len(pts)]
        if y0 == y1:
            continue
        cond = ((y0 <= py) & (py < y1)) | ((y1 <= py) & (py < y0))
        xi = x0 + (py - y0) * (x1 - x0) / (y1 - y0)
        inside ^= cond & (px < xi)
    edge = np.full((W, H), 1e9)
    for k in range(len(pts)):
        (ax, ay), (bx, by) = pts[k], pts[(k + 1) % len(pts)]
        dx, dy = bx - ax, by - ay
        t = np.clip(((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy), 0, 1)
        edge = np.minimum(edge, np.hypot(px - ax - t * dx, py - ay - t * dy))
    edge = np.where(inside, edge, -1.0)
    # foreign drills: vias, TH pads (any net), stored as (net, x, y, r)
    drills = [(n, x, y, VIA_D / 2) for n, x, y in ctx.vias]
    for fp in ctx.b.GetFootprints():
        for p in fp.Pads():
            if p.GetDrillSize().x > 0:
                bbp = p.GetBoundingBox()
                q = p.GetPosition()
                drills.append((p.GetNetname(), TOMM(q.x), TOMM(q.y),
                               TOMM(max(bbp.GetWidth(), bbp.GetHeight())) / 2))
    from scipy.ndimage import distance_transform_edt
    ZC = 0.15                                 # zone clearance (pour to foreign copper)
    H0 = 0.15                                 # half width of the routing strip (0.3 mm)
    SQ2 = math.sqrt(2)
    def route_thin(order):
      thin, fails = {}, []
      taken = np.zeros((W, H), bool)
      if True:
        for net, wd in order:
            terms = terminals.get(net, [])
            if not terms:
                continue
            bl = ~inside | (edge < 0.4 + H0)
            for n, x, y, r in drills:
                if n != net:
                    bl |= disc_mask(x, y, r + ZC + H0)
            if taken.any():
                bl |= (distance_transform_edt(~taken) * GRID) < (ZC + H0)
            tcells = [cell(x, y) for x, y in terms]
            for tc in tcells:
                bl[tc] = False
            tree = {tcells[0]}
            remaining = list(tcells[1:])
            while remaining:
                goal = set(remaining)
                pq = [(0, s, None) for s in tree]
                heapq.heapify(pq)
                prev = {}
                found = None
                while pq:
                    g, cur, par = heapq.heappop(pq)
                    if cur in prev:
                        continue
                    prev[cur] = par
                    if cur in goal:
                        found = cur
                        break
                    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                        nx, ny = cur[0] + dx, cur[1] + dy
                        if not (0 <= nx < W and 0 <= ny < H) or (nx, ny) in prev or bl[nx, ny]:
                            continue
                        if dx and dy and (bl[cur[0] + dx, cur[1]] or bl[cur[0], cur[1] + dy]):
                            continue
                        step = SQ2 if (dx and dy) else 1.0
                        if (nx, ny) in tree:
                            step = 0.05
                        if par is not None and (cur[0] - par[0], cur[1] - par[1]) != (dx, dy):
                            step += 0.2
                        heapq.heappush(pq, (g + step, (nx, ny), cur))
                if found is None:
                    fails.append((net, terms[tcells.index(remaining[0])]))
                    remaining = remaining[1:]
                    continue
                n = found
                while n is not None:
                    tree.add(n)
                    n = prev[n]
                remaining = [t for t in remaining if t not in tree]
            body = np.zeros((W, H), bool)
            for (i0, j0) in tree:
                body[i0, j0] = True
            thin[net] = (tree, tcells, body)
            taken |= disc_mask_from(body, H0, px, py, ox, oy, W, H)
      return thin, fails

    order = sorted(POURS, key=lambda t: len(terminals.get(t[0], [])))
    best = None
    for attempt in range(10):
        thin, fails = route_thin(order)
        if best is None or len(fails) < len(best[1]):
            best = (thin, fails, list(order))
        if not fails:
            break
        bad = {n for n, _ in fails}
        order = [t for t in order if t[0] in bad] + [t for t in order if t[0] not in bad]
        print("pour retry", attempt, "failed nets", sorted(bad), flush=True)
    thin, fails, order = best
    # thicken, hardest net first, never closer than ZC to another net's strip or a foreign drill
    owned = {}
    final = {n: disc_mask_from(thin[n][2], H0, px, py, ox, oy, W, H) for n in thin}
    for net, wd in POURS:
        if net not in thin:
            continue
        tree, tcells, body = thin[net]
        allow = inside & (edge >= 0.4)
        for n, x, y, r in drills:
            if n != net:
                allow &= ~disc_mask(x, y, r + ZC)
        others = np.zeros((W, H), bool)
        for n2 in final:
            if n2 != net:
                others |= final[n2]
        if others.any():
            allow &= (distance_transform_edt(~others) * GRID) >= ZC
        grow = disc_mask_from(body, wd / 2, px, py, ox, oy, W, H)
        for tc in tcells:
            if tc in tree:      # a terminal the router never reached must not seed a loose island
                grow |= disc_mask(ox + tc[0] * GRID, oy + tc[1] * GRID, 0.7)
        final[net] = (grow & allow) | final[net]
        # keep only the piece the first terminal sits in: a terminal cut off from it (and its via)
        # is left for Freerouting to wire instead of sitting in a loose island of the pour
        from scipy.ndimage import label
        lab, _ = label(final[net], structure=np.ones((3, 3)))
        root = lab[tcells[0]]
        assert root > 0
        final[net] = lab == root
        owned[net] = (final[net], tree, (ox, oy))
    return owned, fails, (ox, oy, W, H)


def disc_mask_from(body, r, px, py, ox, oy, W, H):
    """Dilate a boolean raster by a disc of radius r (mm)."""
    from scipy.ndimage import distance_transform_edt
    if not body.any():
        return body.copy()
    return (distance_transform_edt(~body) * GRID) <= r


def mask_to_poly(mask, ox, oy):
    ps = pcbnew.SHAPE_POLY_SET()
    W, H = mask.shape
    first = True
    for j in range(H):
        i = 0
        while i < W:
            if mask[i, j]:
                k = i
                while k + 1 < W and mask[k + 1, j]:
                    k += 1
                x0, x1 = ox + i * GRID - GRID / 2, ox + k * GRID + GRID / 2
                y0, y1 = oy + j * GRID - GRID / 2, oy + j * GRID + GRID / 2
                r = pcbnew.SHAPE_POLY_SET()
                r.NewOutline()
                for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
                    r.Append(MM(x), MM(y))
                ps.BooleanAdd(r)
                i = k + 1
            else:
                i += 1
    ps.Simplify()
    return ps


def make_zone(board, layer, net, poly, prio, name):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    z.SetOutline(poly)
    poly.thisown = 0
    z.SetAssignedPriority(prio)
    z.SetLocalClearance(MM(0.15))
    z.SetMinThickness(MM(0.2))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetZoneName(name)
    board.Add(z)
    z.thisown = 0
    return z


# --------------------------------------------------------------------------- prep
def prep():
    board = pcbnew.LoadBoard(str(PCB))
    assert not list(board.GetTracks()) and not list(board.Zones()), "prep needs the unrouted board from build_hat_pcb.py"
    ctx = Ctx(board)
    report = []

    # 1. buck loop on L1 (hand routed): SW node, input cap and bootstrap straight on the top layer
    hand_buck(ctx)

    # 2. vias for pour-net pads (power first, so their vias claim the space beside the pads)
    terminals = {n: [] for n in POUR_NETS}
    fails = []
    pour_pads = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            n = p.GetNetname()
            if n in POUR_NETS:
                pour_pads.append((fp.GetReference(), p, n))
    # ICs last so decoupling caps keep first pick of the space around them
    pour_pads.sort(key=lambda t: (t[0][0] in "UJ", t[0]))
    for ref, p, n in pour_pads:
        if (ref, p.GetNumber()) in ctx.skip:
            continue
        c = p.GetPosition()
        if p.GetDrillSize().x > 0:
            terminals[n].append((TOMM(c.x), TOMM(c.y)))
            continue
        # a pad already joined to a same-net via by the buck hand-routing needs no new one
        if any(vn == n and math.hypot(vx - TOMM(c.x), vy - TOMM(c.y)) < 0.01 for vn, vx, vy in ctx.vias):
            terminals[n].append((TOMM(c.x), TOMM(c.y)))
            continue
        xy, _ = place_pad_via(ctx, p, n)
        if xy is None:
            fails.append((ref, p.GetNumber(), n))
        else:
            terminals[n].append(xy)

    # 3. GND vias: capacitor pads first, then everything else
    gnd = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == "GND" and p.GetDrillSize().x == 0 and p.IsOnLayer(L1):
                gnd.append((fp.GetReference(), p))
    gnd.sort(key=lambda t: (0 if t[0][0] == "C" else 1, t[0]))
    gnd_far = []
    for ref, p in gnd:
        c = p.GetPosition()
        if any(vn == "GND" and math.hypot(vx - TOMM(c.x), vy - TOMM(c.y)) < 0.01 for vn, vx, vy in ctx.vias):
            continue
        xy, gap = place_pad_via(ctx, p, "GND")
        if xy is None:
            fails.append((ref, p.GetNumber(), "GND"))
        elif ref[0] == "C" and gap > 0.5:
            gnd_far.append((ref, p.GetNumber()))
    print("via placement failures:", fails)
    print("cap GND vias beyond 0.5 mm of the pad:", gnd_far)

    # 4. In1 GND, In2 pours, then (after the routing copy is saved) GND fill on In2 and B.Cu
    owned, pfails, (ox, oy, W, H) = build_pours(ctx, terminals)
    print("pour terminals not reached:", pfails)
    gndnet = ctx.nets["GND"]
    outline = pcbnew.SHAPE_POLY_SET(ctx.ol)
    make_zone(board, IN1, gndnet, pcbnew.SHAPE_POLY_SET(outline), 0, "GND_In1")
    for i, (net, wd) in enumerate(POURS):
        if net not in owned:
            continue
        own, tree, _ = owned[net]
        poly = mask_to_poly(own, ox, oy)
        make_zone(board, IN2, ctx.nets[net], poly, 10 + i, f"{net}_In2")
    (HAT / "build").mkdir(exist_ok=True)
    # Freerouting works on build/route.kicad_pcb: the DSN exporter turns zones into planes it
    # treats as conduction areas, so the whole-board GND fills of In2 and B.Cu stay out of it
    # (finish() adds them to the final board)
    pcbnew.SaveBoard(str(ROUTE), board)
    print("route copy saved, vias:", len(ctx.vias), flush=True)


def dsn():
    """DSN for Freerouting from build/route.kicad_pcb (loaded fresh: exporting the in-memory board
    right after prep crashes the exporter). In1 and In2 are power layers: no signal routing there.
    A rule-area ring keeps tracks and vias 0.6 mm off the board edge (the exporter ignores the
    copper-edge clearance)."""
    board = pcbnew.LoadBoard(str(ROUTE))
    ol = pcbnew.SHAPE_POLY_SET()
    board.GetBoardPolygonOutlines(ol, True)
    inner = pcbnew.SHAPE_POLY_SET(ol)
    inner.Inflate(MM(-0.6), pcbnew.CORNER_STRATEGY_ROUND_ALL_CORNERS, MM(0.01))
    ring = pcbnew.SHAPE_POLY_SET(ol)
    ring.BooleanSubtract(inner)
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(True)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    z.SetDoNotAllowZoneFills(False)
    ls = pcbnew.LSET()
    ls.AddLayer(L1)
    ls.AddLayer(L4)
    z.SetLayerSet(ls)
    z.SetOutline(ring)
    ring.thisown = 0
    board.Add(z)
    z.thisown = 0
    board.SetLayerType(IN1, pcbnew.LT_POWER)
    board.SetLayerType(IN2, pcbnew.LT_POWER)
    assert pcbnew.ExportSpecctraDSN(board, str(DSN)), "DSN export failed"
    print("dsn", DSN.stat().st_size, "bytes")


def pad_xy(ctx, ref, num):
    for p in ctx.b.FindFootprintByReference(ref).Pads():
        if p.GetNumber() == num:
            q = p.GetPosition()
            return TOMM(q.x), TOMM(q.y)
    raise KeyError((ref, num))


def hand_buck(ctx):
    """Buck (U2 TPS563201, L1, C39, C42) on L1: SW node U2.2 -> L1.1 straight and wide, VIN U2.3 ->
    C39.1 short and wide, bootstrap cap across U2.6 / SW. C40 was moved out of the SW corridor."""
    u2 = {n: pad_xy(ctx, "U2", n) for n in "123456"}
    ctx.add_track("SW_2V5", u2["2"], pad_xy(ctx, "L1", "1"), 0.6, L1)
    ctx.add_track("+5V", u2["3"], pad_xy(ctx, "C39", "1"), 0.6, L1)
    ctx.add_track("BST_2V5", u2["6"], pad_xy(ctx, "C42", "1"), 0.25, L1)
    c42s = pad_xy(ctx, "C42", "2")
    ctx.add_track("SW_2V5", c42s, (c42s[0], u2["2"][1]), 0.25, L1)
    ctx.add_track("SW_2V5", (c42s[0], u2["2"][1]), u2["2"], 0.25, L1)
    ctx.skip.add(("U2", "3"))       # joined to C39.1 by hand; C39.1 carries the +5V pour via


def stitch_edge(ctx, pitch=4.0, inset=0.9):
    """GND vias around the board edge (they tie B.Cu and In2 fills to In1)."""
    n = 0
    for a, b in ctx.edges:
        ax, ay, bx, by = TOMM(a.x), TOMM(a.y), TOMM(b.x), TOMM(b.y)
        L = math.hypot(bx - ax, by - ay)
        if L < 0.3:
            continue
        ux, uy = (bx - ax) / L, (by - ay) / L
        nx, ny = -uy, ux
        if not ctx.inside(ax + ux * L / 2 + nx * 0.3, ay + uy * L / 2 + ny * 0.3):
            nx, ny = -nx, -ny
        steps = max(1, int(L // pitch))
        for k in range(steps):
            s = (k + 0.5) * L / steps
            for d in (inset, inset + 0.4, inset + 0.8):
                x, y = ax + ux * s + nx * d, ay + uy * s + ny * d
                if not ctx.inside(x, y) or ctx.edge_dist(x, y) < 0.6:
                    continue
                via = pcbnew.SHAPE_CIRCLE(v2(x, y), MM(VIA_D / 2))
                if ctx.obs.hit(via, 0.25):
                    continue
                ctx.add_via("GND", x, y)
                n += 1
                break
    return n


def imp():
    """SES into build/route.kicad_pcb (in place, so another Freerouting round starts from it)."""
    board = pcbnew.LoadBoard(str(ROUTE))
    assert pcbnew.ImportSpecctraSES(board, str(SES)), "SES import failed"
    pcbnew.SaveBoard(str(ROUTE), board)
    print("tracks", len([t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]),
          "vias", len([t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]))


def prune_redundant(board):
    """Freerouting does not see that a via beside a pad ties it to a plane or pour, so it adds its own
    wires between pads of GND and the pour nets. Remove every unlocked wire of those nets whose
    removal leaves each pad as plane-connected as before, then the unlocked vias left dangling."""
    from collections import defaultdict, deque
    zones = {}
    for i in range(board.GetAreaCount()):
        z = board.GetArea(i)
        if z.GetIsRuleArea():
            continue
        if z.GetLayer() == IN1 and z.GetNetname() == "GND":
            zones["GND"] = z
        elif z.GetLayer() == IN2 and z.GetNetname() in POUR_NETS:
            zones[z.GetNetname()] = z
    removed_t = removed_v = 0
    doomed = []
    for net, z in zones.items():
        items = [t for t in board.GetTracks() if t.GetNetname() == net]
        trks = [t for t in items if t.GetClass() != "PCB_VIA"]
        vias = [pcbnew.Cast_to_PCB_VIA(t) for t in items if t.GetClass() == "PCB_VIA"]
        pads = [(fp.GetReference(), p) for fp in board.GetFootprints() for p in fp.Pads() if p.GetNetname() == net]
        adj = defaultdict(set)

        def link(a, b, key=None):
            adj[a].add((b, key))
            adj[b].add((a, key))
        for k, t in enumerate(trks):
            s, e = t.GetStart(), t.GetEnd()
            ka, kb = ("t", t.GetLayer(), s.x, s.y), ("t", t.GetLayer(), e.x, e.y)
            link(ka, kb, k)
            for q, kk in ((s, ka), (e, kb)):
                for pi, (ref, p) in enumerate(pads):
                    if p.IsOnLayer(t.GetLayer()) and pcbnew.SHAPE.Collide(p.GetEffectiveShape(t.GetLayer()), pcbnew.SHAPE_CIRCLE(q, 1), 0):
                        link(kk, ("p", pi))
                for vi, v in enumerate(vias):
                    if (v.GetPosition() - q).EuclideanNorm() <= v.GetWidth(L1) // 2 + 100:
                        link(kk, ("v", vi))
        for vi, v in enumerate(vias):
            if z.GetLayer() == IN1 or z.Outline().Collide(v.GetPosition(), 0):
                link("PLANE", ("v", vi))
        for pi, (ref, p) in enumerate(pads):
            if p.GetDrillSize().x > 0 and (z.GetLayer() == IN1 or z.Outline().Collide(p.GetPosition(), 0)):
                link("PLANE", ("p", pi))

        def reach(skip):
            seen, dq = {"PLANE"}, deque(["PLANE"])
            while dq:
                n = dq.popleft()
                for m, key in adj[n]:
                    if key is not None and key in skip or m in seen:
                        continue
                    seen.add(m)
                    dq.append(m)
            return seen
        base = reach(set())
        need = {("p", pi) for pi in range(len(pads))} & base
        skip = set()
        dead = []
        for k, t in sorted(enumerate(trks), key=lambda kt: -pcbnew.ToMM(kt[1].GetLength())):
            if t.IsLocked():
                continue
            r = reach(skip | {k})
            if need <= r:
                skip.add(k)
                dead.append(t)
        # trim dangling leaves the removals left behind (an end touching nothing)
        def touches(k, q, layer):
            for k2, t2 in enumerate(trks):
                if k2 == k or k2 in skip or t2.GetLayer() != layer:
                    continue
                for q2 in (t2.GetStart(), t2.GetEnd()):
                    if (q2 - q).EuclideanNorm() < 200:
                        return True
            for v in vias:
                if (v.GetPosition() - q).EuclideanNorm() <= v.GetWidth(L1) // 2 + 100:
                    return True
            for ref, p in pads:
                if p.IsOnLayer(layer) and pcbnew.SHAPE.Collide(p.GetEffectiveShape(layer), pcbnew.SHAPE_CIRCLE(q, 1), 0):
                    return True
            return False
        again = True
        while again:
            again = False
            for k, t in enumerate(trks):
                if k in skip or t.IsLocked():
                    continue
                if not touches(k, t.GetStart(), t.GetLayer()) or not touches(k, t.GetEnd(), t.GetLayer()):
                    skip.add(k)
                    dead.append(t)
                    again = True
        doomed.extend(dead)
        removed_t += len(dead)
        # vias left with no track and no pad contact (unlocked only)
        keep_v = set()
        for k, t in enumerate(trks):
            if k in skip:
                continue
            for q in (t.GetStart(), t.GetEnd()):
                for vi, v in enumerate(vias):
                    if (v.GetPosition() - q).EuclideanNorm() <= v.GetWidth(L1) // 2 + 100:
                        keep_v.add(vi)
        for vi, v in enumerate(vias):
            if v.IsLocked() or vi in keep_v:
                continue
            doomed.append(v)
            removed_v += 1
    for t in doomed:       # removed last: Remove() invalidates the SWIG proxies still in use above
        board.Remove(t)
    return removed_t, removed_v


def class_widths():
    import fnmatch
    import json
    d = json.loads(PCB.with_suffix(".kicad_pro").read_text())["net_settings"]
    widths = {c["name"]: c["track_width"] for c in d["classes"]}

    def width(net):
        for pat in d.get("netclass_patterns", []):
            if fnmatch.fnmatchcase(net, pat["pattern"]):
                return widths[pat["netclass"]]
        return widths["Default"]
    return width


def heal(board, ctx):
    """Freerouting necks tracks down to 75 % at pin exits. Widen each back to its class width where
    that keeps 0.15 mm clearance to everything else (tracks, vias, pads, board edge); a neck below
    the 0.1 mm floor that cannot reach its class width is widened as far as it will go (0.12, 0.1)."""
    width = class_widths()
    tr = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
    fixed = 0

    def fits(t, w):
        a, b = t.GetStart(), t.GetEnd()
        seg = pcbnew.SHAPE_SEGMENT(a, b, w)
        if ctx.edge_dist(TOMM(a.x), TOMM(a.y)) <= 0.4 + TOMM(w) / 2 or \
                ctx.edge_dist(TOMM(b.x), TOMM(b.y)) <= 0.4 + TOMM(w) / 2:
            return None
        bb = seg.BBox()
        for net, sh, owner, lay in ctx.obs.near(TOMM(bb.GetX()), TOMM(bb.GetY()), TOMM(bb.GetRight()),
                                                TOMM(bb.GetBottom()), 0.2):
            if net == t.GetNetname() or (lay is not None and lay != t.GetLayer()):
                continue
            if pcbnew.SHAPE.Collide(sh, seg, MM(0.155)):
                return None
        return seg
    for t in tr:
        want = MM(width(t.GetNetname()))
        if t.IsLocked() or t.GetWidth() >= want - MM(0.005):
            continue     # locked = my pad stubs, deliberately pad-width
        for w in (want, MM(0.12), MM(0.1)):
            if w <= t.GetWidth() or (w != want and t.GetWidth() >= MM(0.1)):
                continue
            seg = fits(t, w)
            if seg is not None:
                t.SetWidth(w)
                ctx.obs.add(t.GetNetname(), seg, None, t.GetLayer())
                fixed += 1
                break
    return fixed


def prune():
    """Own process: the SWIG proxies (and the heap) are unreliable after Remove(), so save and _exit."""
    import os
    board = pcbnew.LoadBoard(str(ROUTE))
    print("redundant router wires/vias removed (tracks, vias):", prune_redundant(board), flush=True)
    pcbnew.SaveBoard(str(HAT / "build" / "pruned.kicad_pcb"), board)
    sys.stdout.flush()
    os._exit(0)


def tidy():
    """Clean Freerouting's output (own process, saves and _exits like prune): merge collinear
    chains and overlapping duplicates, contract jogs shorter than the track width where the move
    keeps 0.15 mm clearance. Locked items (my pad stubs) and nodes at pads/vias are never moved."""
    import os
    from collections import defaultdict
    src = Path(sys.argv[2]) if len(sys.argv) > 2 else HAT / "build" / "pruned.kicad_pcb"
    board = pcbnew.LoadBoard(str(src))
    ctx = Ctx(board, tracks=True)
    tr = [t for t in board.GetTracks() if t.GetClass() != "PCB_VIA" and not t.IsLocked()]
    vias = [pcbnew.Cast_to_PCB_VIA(t) for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    pads = [p for fp in board.GetFootprints() for p in fp.Pads()]
    fixed_pts = defaultdict(set)     # layer -> {(x, y)} where a pad, via or locked item sits
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            continue
        if t.IsLocked():
            for q in (t.GetStart(), t.GetEnd()):
                fixed_pts[t.GetLayer()].add((q.x, q.y))
    live = {id(t): t for t in tr}
    dead = []
    stats = {"merged": 0, "contracted": 0, "zero": 0}

    def node_fixed(layer, q):
        if (q.x, q.y) in fixed_pts[layer]:
            return True
        c = pcbnew.SHAPE_CIRCLE(q, 1)
        for v in vias:
            if (v.GetPosition() - q).EuclideanNorm() <= v.GetWidth(L1) // 2 + 100:
                return True
        for p in pads:
            if p.IsOnLayer(layer):
                bb = p.GetBoundingBox()
                if bb.Contains(q) and pcbnew.SHAPE.Collide(p.GetEffectiveShape(layer), c, 0):
                    return True
        return False

    def clear_ok(t, a, b):
        seg = pcbnew.SHAPE_SEGMENT(a, b, t.GetWidth())
        bb = seg.BBox()
        for net, sh, owner, lay in ctx.obs.near(TOMM(bb.GetX()), TOMM(bb.GetY()), TOMM(bb.GetRight()),
                                                TOMM(bb.GetBottom()), 0.25):
            if net == t.GetNetname() or (lay is not None and lay != t.GetLayer()):
                continue
            if pcbnew.SHAPE.Collide(sh, seg, MM(0.155)):
                return False
        return True

    # overlapping collinear duplicates of one net: keep the union as one wire
    by = defaultdict(list)
    for t in live.values():
        by[(t.GetNetname(), t.GetLayer())].append(t)
    for key, lst in by.items():
        for i, A in enumerate(lst):
            for Bt in lst[i + 1:]:
                if id(A) not in live or id(Bt) not in live:
                    continue
                a0, a1, b0, b1 = A.GetStart(), A.GetEnd(), Bt.GetStart(), Bt.GetEnd()
                dx, dy = a1.x - a0.x, a1.y - a0.y
                L2 = dx * dx + dy * dy
                if L2 == 0:
                    continue
                Ln = math.sqrt(L2)
                d0 = abs((b0.x - a0.x) * dy - (b0.y - a0.y) * dx) / Ln
                d1 = abs((b1.x - a0.x) * dy - (b1.y - a0.y) * dx) / Ln
                if d0 > 1000 or d1 > 1000 or A.GetWidth() != Bt.GetWidth():       # 1 um
                    continue
                ta = [0.0, 1.0]
                tb = [((q.x - a0.x) * dx + (q.y - a0.y) * dy) / L2 for q in (b0, b1)]
                lo, hi = min(ta + tb), max(ta + tb)
                if max(min(tb), 0.0) > min(max(tb), 1.0):        # disjoint
                    continue
                A.SetStart(pcbnew.VECTOR2I(int(a0.x + lo * dx), int(a0.y + lo * dy)))
                A.SetEnd(pcbnew.VECTOR2I(int(a0.x + hi * dx), int(a0.y + hi * dy)))
                del live[id(Bt)]
                dead.append(Bt)
                stats["merged"] += 1

    changed = True
    while changed:
        changed = False
        nodes = defaultdict(list)
        for t in live.values():
            for end, q in (("s", t.GetStart()), ("e", t.GetEnd())):
                nodes[(t.GetNetname(), t.GetLayer(), q.x, q.y)].append((t, end))
        for (net, layer, x, y), lst in list(nodes.items()):
            if len(lst) != 2:
                continue
            (A, ea), (B, eb) = lst
            if A is B or id(A) not in live or id(B) not in live:
                continue
            q = pcbnew.VECTOR2I(x, y)
            if node_fixed(layer, q):
                continue
            a_far = A.GetEnd() if ea == "s" else A.GetStart()
            b_far = B.GetEnd() if eb == "s" else B.GetStart()
            la = (a_far - q).EuclideanNorm()
            lb = (b_far - q).EuclideanNorm()
            if la == 0 or lb == 0:
                victim, keep, kfar = (A, B, b_far) if la == 0 else (B, A, a_far)
                del live[id(victim)]
                dead.append(victim)
                stats["zero"] += 1
                changed = True
                continue
            cross = (a_far.x - x) * (b_far.y - y) - (a_far.y - y) * (b_far.x - x)
            dot = (a_far.x - x) * (b_far.x - x) + (a_far.y - y) * (b_far.y - y)
            collinear = abs(cross) <= 2e-4 * la * lb and dot < 0
            if collinear and A.GetWidth() == B.GetWidth():
                # A takes over B's far end, B goes
                (A.SetEnd if ea == "e" else A.SetStart)(b_far)
                del live[id(B)]
                dead.append(B)
                stats["merged"] += 1
                changed = True
                continue
            w = A.GetWidth()
            for S, Sfar, N, Nend, Nfar in ((A, a_far, B, eb, b_far), (B, b_far, A, ea, a_far)):
                # S is the short jog between this node and Sfar; N is the other wire; move N's end to Sfar
                Slen = (Sfar - q).EuclideanNorm()
                if Slen < S.GetWidth():
                    newa, newb = (Sfar, Nfar)
                    if clear_ok(N, newa, newb):
                        (N.SetStart if Nend == "s" else N.SetEnd)(Sfar)
                        del live[id(S)]
                        dead.append(S)
                        stats["contracted"] += 1
                        changed = True
                        break
    # 90 degree corners become two 45 degree ones where the cut keeps clearance
    nodes = defaultdict(list)
    for t in live.values():
        for end, q in (("s", t.GetStart()), ("e", t.GetEnd())):
            nodes[(t.GetNetname(), t.GetLayer(), q.x, q.y)].append((t, end))
    stats["chamfered"] = 0
    for (net, layer, x, y), lst in list(nodes.items()):
        if len(lst) != 2:
            continue
        (A, ea), (Bt, eb) = lst
        if A is Bt or A.GetWidth() != Bt.GetWidth():
            continue
        q = pcbnew.VECTOR2I(x, y)
        if node_fixed(layer, q):
            continue
        af = A.GetEnd() if ea == "s" else A.GetStart()
        bf = Bt.GetEnd() if eb == "s" else Bt.GetStart()
        la_, lb_ = (af - q).EuclideanNorm(), (bf - q).EuclideanNorm()
        if la_ == 0 or lb_ == 0:
            continue
        ux, uy = (af.x - x) / la_, (af.y - y) / la_
        vx, vy = (bf.x - x) / lb_, (bf.y - y) / lb_
        if abs(ux * vx + uy * vy) > 0.12:
            continue
        w = A.GetWidth()
        c = min(MM(0.3), 0.45 * la_, 0.45 * lb_)
        if c * 1.4142 < w:
            continue
        pa = pcbnew.VECTOR2I(int(x + ux * c), int(y + uy * c))
        pb = pcbnew.VECTOR2I(int(x + vx * c), int(y + vy * c))
        if not clear_ok(A, pa, pb):
            continue
        (A.SetStart if ea == "s" else A.SetEnd)(pa)
        (Bt.SetStart if eb == "s" else Bt.SetEnd)(pb)
        d = pcbnew.PCB_TRACK(board)
        d.SetStart(pa)
        d.SetEnd(pb)
        d.SetWidth(w)
        d.SetLayer(layer)
        d.SetNet(A.GetNet())
        board.Add(d)
        d.thisown = 0
        ctx.obs.add(net, pcbnew.SHAPE_SEGMENT(pa, pb, w), None, layer)
        stats["chamfered"] += 1
    for t in dead:
        board.Remove(t)
    print("tidy:", stats, flush=True)
    pcbnew.SaveBoard(str(src), board)
    sys.stdout.flush()
    os._exit(0)


def trim():
    """Last clean-up (own process, saves and _exits): delete unlocked wires with an end that touches
    no copper of its own net (pad, via, another wire, T-junctions included), repeatedly, and exact
    duplicates left by join_hat.py."""
    import os
    from collections import defaultdict
    board = pcbnew.LoadBoard(str(PCB))
    trks = [t for t in board.GetTracks() if t.GetClass() != "PCB_VIA"]
    shapes = defaultdict(list)       # net -> [(kind, owner index or None, layer or None, shape)]
    for fp in board.GetFootprints():
        for p in fp.Pads():
            shapes[p.GetNetname()].append((None, p, pcbnew.SHAPE_POLY_SET()))
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            v = pcbnew.Cast_to_PCB_VIA(t)
            shapes[t.GetNetname()].append(("via", None, pcbnew.SHAPE_CIRCLE(v.GetPosition(), v.GetWidth(L1) // 2)))
    dead = set()
    seen = {}
    for k, t in enumerate(trks):
        if t.GetLength() == 0 and not t.IsLocked():       # zero-length wires
            dead.add(k)
    for k, t in enumerate(trks):                       # exact duplicates
        a_, b_ = (t.GetStart().x, t.GetStart().y), (t.GetEnd().x, t.GetEnd().y)
        key = (t.GetNetname(), t.GetLayer(), min(a_, b_), max(a_, b_))
        if key in seen and not t.IsLocked():
            dead.add(k)
        else:
            seen[key] = k
    by_net = defaultdict(list)
    for k, t in enumerate(trks):
        by_net[t.GetNetname()].append(k)
    dead_vias = []
    vlist = [pcbnew.Cast_to_PCB_VIA(t) for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    for i, v in enumerate(vlist):                       # two vias of one net on the same spot: keep one
        for w in vlist[:i]:
            if w not in dead_vias and v.GetNetname() == w.GetNetname() and \
                    (v.GetPosition() - w.GetPosition()).EuclideanNorm() < pcbnew.FromMM(0.3):
                dead_vias.append(v)
                break

    def touches(k, q, t):
        probe = pcbnew.SHAPE_CIRCLE(q, 5000)       # the end point itself (5 um)
        for k2 in by_net[t.GetNetname()]:
            t2 = trks[k2]
            if k2 == k or k2 in dead or t2.GetLayer() != t.GetLayer():
                continue
            if pcbnew.SHAPE.Collide(pcbnew.SHAPE_SEGMENT(t2.GetStart(), t2.GetEnd(), t2.GetWidth()), probe, 0):
                return True
        for kind, p, sh in shapes[t.GetNetname()]:
            if kind == "via":
                if pcbnew.SHAPE.Collide(sh, probe, 0):
                    return True
            elif p.IsOnLayer(t.GetLayer()) and pcbnew.SHAPE.Collide(p.GetEffectiveShape(t.GetLayer()), probe, 0):
                return True
        return False
    again = True
    while again:
        again = False
        for k, t in enumerate(trks):
            if k in dead or t.IsLocked():
                continue
            if not touches(k, t.GetStart(), t) or not touches(k, t.GetEnd(), t):
                dead.add(k)
                again = True
    for k in sorted(dead):
        board.Remove(trks[k])
    for v in dead_vias:
        board.Remove(v)
    print("trim: removed", len(dead), "dangling or duplicate wires,", len(dead_vias), "duplicate vias", flush=True)
    pcbnew.SaveBoard(str(PCB), board)
    sys.stdout.flush()
    os._exit(0)


def refill():
    """Refill every zone (after join_hat.py added wires) and save."""
    board = pcbnew.LoadBoard(str(PCB))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(PCB), board)
    print("zones refilled")


def finish():
    """Final board: GND fills on In2 and B.Cu, edge stitching vias, widths healed, zones filled."""
    board = pcbnew.LoadBoard(str(HAT / "build" / "pruned.kicad_pcb"))
    ctx = Ctx(board, tracks=True)
    gndnet = ctx.nets["GND"]
    print("edge stitching vias:", stitch_edge(ctx))
    print("tracks widened back to class width:", heal(board, ctx))
    outline = pcbnew.SHAPE_POLY_SET(ctx.ol)
    make_zone(board, L4, gndnet, pcbnew.SHAPE_POLY_SET(outline), 0, "GND_B")
    make_zone(board, IN2, gndnet, pcbnew.SHAPE_POLY_SET(outline), 0, "GND_In2")
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(str(PCB), board)
    print("tracks", len([t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]),
          "vias", len([t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]))


if __name__ == "__main__":
    {"prep": prep, "dsn": dsn, "import": imp, "prune": prune, "tidy": tidy, "finish": finish, "trim": trim, "refill": refill}[sys.argv[1]]()
