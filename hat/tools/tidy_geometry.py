#!/usr/bin/env python3
"""Post-route geometry tidy for the hat (after the routing stages, before the final gates).

Works on the committed, DRC-clean board in place; never re-routes. Batches, each verified with
kicad-cli DRC (no new violation type/count versus the start, 0 unconnected, 0 parity) and bisected:
a failing batch is split until the offending edits are found and dropped.

  1 overlap   delete redundant collinear copper (check_geometry OVERLAP)
  2 staircase collapse chains of unfixed 2-way nodes to their Douglas-Peucker polyline (jogs, staircases)
  3 short     contract segments shorter than their width into a neighbour node (SHORT)
  4 snap      nudge a 2-way vertex a few um so a ~135 degree corner reads 135 (SES rounding)
  5 chamfer   cut 90..134 degree 2-way corners with a 45 degree chamfer where the legs allow
  6 neck      widen necked segments to class width where 0.17 mm clearance to other nets holds

usage: tidy_geometry.py [board.kicad_pcb]
"""
import contextlib
import io
import math
import re
import shutil
import subprocess
import sys
import tempfile
from collections import Counter, defaultdict
from pathlib import Path

import pcbnew
from pcbnew import FromMM as MM, ToMM as TOMM, VECTOR2I

HAT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HAT.parent / "tools"))
import check_geometry as cg            # noqa: E402  (read only: findings drive the neck batch)

ANG_OK = 135 - 0.01
WORK = D = None          # set by main(); the worker processes need no scratch dir


def log(*a):
    print("tidy:", *a, flush=True)


# ---------------------------------------------------------------- DRC gate
def drc(path):
    rpt = WORK / "drc.rpt"
    shutil.copy(path, D / "ps1-av-hat.kicad_pcb")
    path = D / "ps1-av-hat.kicad_pcb"
    r = subprocess.run(["kicad-cli", "pcb", "drc", "--format", "report", "--severity-all", "--schematic-parity",
                        "--refill-zones", "-o", str(rpt), str(path)], capture_output=True, text=True, cwd=HAT)
    txt = rpt.read_text()
    types = Counter(re.findall(r"^\[(\w+)\]", txt, re.M))
    unc = int(re.search(r"Found (\d+) unconnected", txt).group(1))
    par = int(re.search(r"Found (\d+) schematic parity", r.stdout).group(1))
    return types, unc, par


def drc_ok(res, base):
    types, unc, par = res
    return unc == 0 and par == 0 and all(types[k] <= base[0][k] for k in types)


# ---------------------------------------------------------------- board model
class Model:
    def __init__(self, path):
        self.path = path
        self.board = pcbnew.LoadBoard(str(path))
        b = self.board
        self.tracks = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"]
        self.vias = [pcbnew.Cast_to_PCB_VIA(t) for t in b.GetTracks() if t.GetClass() == "PCB_VIA"]
        self.pads = [p for fp in b.GetFootprints() for p in fp.Pads()]
        self.dead = set()
        self.nodes = defaultdict(list)          # (net, layer, x, y) -> [track]
        for t in self.tracks:
            for q in (t.GetStart(), t.GetEnd()):
                self.nodes[(t.GetNetCode(), t.GetLayer(), q.x, q.y)].append(t)
        self._fixed = {}

    def key(self, t, q):
        return (t.GetNetCode(), t.GetLayer(), q.x, q.y)

    def fixed(self, t, q):
        """node at a pad, via or locked item: never moved"""
        k = self.key(t, q)
        if k in self._fixed:
            return self._fixed[k]
        r = any(x.IsLocked() for x in self.nodes[k])
        if not r:
            c = pcbnew.SHAPE_CIRCLE(q, 1)
            for p in self.pads:
                if p.IsOnLayer(t.GetLayer()) and p.GetBoundingBox().Contains(q) and \
                        pcbnew.SHAPE.Collide(p.GetEffectiveShape(t.GetLayer()), c, 0):
                    r = True
                    break
        if not r:
            r = any(math.hypot(v.GetPosition().x - q.x, v.GetPosition().y - q.y) <= v.GetWidth(t.GetLayer()) // 2 + 100
                    for v in self.vias)
        self._fixed[k] = r
        return r

    def at(self, t, q):
        return [x for x in self.nodes[self.key(t, q)] if id(x) not in self.dead]

    def save(self, path):
        pcbnew.SaveBoard(str(path), self.board)

    def remove(self, t):
        self.dead.add(id(t))
        self.board.Remove(t)

    def new_track(self, like, a, b, width=None):
        n = pcbnew.PCB_TRACK(self.board)
        n.SetLayer(like.GetLayer())
        n.SetNetCode(like.GetNetCode())
        n.SetWidth(width if width is not None else like.GetWidth())
        n.SetStart(a)
        n.SetEnd(b)
        self.board.Add(n)
        return n


def other(t, q):
    return t.GetEnd() if (t.GetStart().x, t.GetStart().y) == (q.x, q.y) else t.GetStart()


def angle(q, a, b):
    v1, v2 = (a.x - q.x, a.y - q.y), (b.x - q.x, b.y - q.y)
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 == 0 or n2 == 0:
        return 180.0
    return math.degrees(math.acos(max(-1, min(1, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))))


def min_angle(q, others):
    m = 180.0
    for i in range(len(others)):
        for j in range(i + 1, len(others)):
            m = min(m, angle(q, others[i], others[j]))
    return m


def length(t):
    return math.hypot(t.GetEnd().x - t.GetStart().x, t.GetEnd().y - t.GetStart().y)


# ---------------------------------------------------------------- planners; each returns independent ops
def plan_overlap(m):
    ops, touched = [], set()
    groups = defaultdict(list)
    for t in m.tracks:
        groups[(t.GetNetCode(), t.GetLayer())].append(t)
    for lst in groups.values():
        for i, a in enumerate(lst):
            for b in lst[i + 1:]:
                if id(a) in touched or id(b) in touched:
                    continue
                ins = [p for p in (b.GetStart(), b.GetEnd()) if cg.on_segment(p, a.GetStart(), a.GetEnd(), cg.TOL)]
                ins += [p for p in (a.GetStart(), a.GetEnd()) if cg.on_segment(p, b.GetStart(), b.GetEnd(), cg.TOL)]
                if len({(p.x, p.y) for p in ins}) < 2:
                    continue
                touched |= {id(a), id(b)}
                ops.append(lambda a=a, b=b: _fix_overlap(m, a, b))
    return ops


def _within(p, t):
    return cg.on_segment(p, t.GetStart(), t.GetEnd(), cg.TOL)


def _fix_overlap(m, a, b):
    if a.IsLocked() and not b.IsLocked():
        a, b = b, a                      # a is the one we may change
    if a.IsLocked():
        return
    if all(_within(p, b) for p in (a.GetStart(), a.GetEnd())) or length(a) == 0:
        m.remove(a)                      # a lies wholly inside b
        return
    if all(_within(p, a) for p in (b.GetStart(), b.GetEnd())) and not b.IsLocked() and a.GetWidth() >= b.GetWidth():
        m.remove(b)
        return
    if not b.IsLocked() and a.GetWidth() == b.GetWidth():
        pts = [a.GetStart(), a.GetEnd(), b.GetStart(), b.GetEnd()]
        d = (a.GetEnd().x - a.GetStart().x, a.GetEnd().y - a.GetStart().y)
        pts.sort(key=lambda p: p.x * d[0] + p.y * d[1])
        a.SetStart(pts[0])
        a.SetEnd(pts[-1])
        m.remove(b)
        return
    # a partly overlaps b: cut a back to where it leaves b
    for p, o in ((a.GetStart(), a.GetEnd()), (a.GetEnd(), a.GetStart())):
        if _within(p, b) and not _within(o, b):
            e = min((b.GetStart(), b.GetEnd()), key=lambda q: math.hypot(q.x - o.x, q.y - o.y))
            (a.SetStart if p == a.GetStart() else a.SetEnd)(e)
            return


def dp(pts, tol):
    """Douglas-Peucker on a list of (x, y) in nm"""
    if len(pts) < 3:
        return pts
    (x0, y0), (x1, y1) = pts[0], pts[-1]
    L = math.hypot(x1 - x0, y1 - y0)
    best, bi = -1, 0
    for i in range(1, len(pts) - 1):
        d = (math.hypot(pts[i][0] - x0, pts[i][1] - y0) if L == 0 else
             abs((pts[i][0] - x0) * (y1 - y0) - (pts[i][1] - y0) * (x1 - x0)) / L)
        if d > best:
            best, bi = d, i
    if best <= tol:
        return [pts[0], pts[-1]]
    return dp(pts[:bi + 1], tol)[:-1] + dp(pts[bi:], tol)


def plan_chain(m, tol=MM(0.03)):
    ops, seen = [], set()

    def interior(t, q):
        return (not m.fixed(t, q)) and len(m.nodes[m.key(t, q)]) == 2 and not any(
            x.IsLocked() for x in m.nodes[m.key(t, q)])

    for t in m.tracks:
        if id(t) in seen or t.IsLocked():
            continue
        # walk to a chain end
        chain = [t]
        seen.add(id(t))
        for end in (0, 1):
            cur = t
            q = cur.GetEnd() if end else cur.GetStart()
            while interior(cur, q):
                nxt = [x for x in m.nodes[m.key(cur, q)] if x is not cur][0]
                if id(nxt) in seen or nxt.GetWidth() != t.GetWidth():
                    break
                seen.add(id(nxt))
                chain.insert(len(chain), nxt) if end else chain.insert(0, nxt)
                q = other(nxt, q)
                cur = nxt
        if len(chain) < 2:
            continue
        # order points
        pts = [(chain[0].GetStart().x, chain[0].GetStart().y), (chain[0].GetEnd().x, chain[0].GetEnd().y)]
        ok = True
        for c in chain[1:]:
            s, e = (c.GetStart().x, c.GetStart().y), (c.GetEnd().x, c.GetEnd().y)
            if pts[-1] == s:
                pts.append(e)
            elif pts[-1] == e:
                pts.append(s)
            else:                       # chain[0] was oriented the wrong way round
                pts.reverse()
                if pts[-1] == s:
                    pts.append(e)
                elif pts[-1] == e:
                    pts.append(s)
                else:
                    ok = False
                    break
        if not ok or pts[0] == pts[-1]:
            continue
        new = dp(pts, tol)
        if len(new) >= len(pts):
            continue
        ops.append(lambda chain=chain, new=new: _apply_chain(m, chain, new))
    return ops


def _apply_chain(m, chain, new):
    like = chain[0]
    for c in chain:
        m.remove(c)
    for a, b in zip(new, new[1:]):
        m.new_track(like, VECTOR2I(*a), VECTOR2I(*b))


def plan_short(m):
    ops, touched = [], set()
    for t in m.tracks:
        if t.IsLocked() or length(t) >= t.GetWidth() or id(t) in touched:
            continue
        P, Q = t.GetStart(), t.GetEnd()
        if length(t) == 0:
            if len(m.at(t, P)) > 1 or True:
                touched.add(id(t))
                ops.append(lambda t=t: m.remove(t))
            continue
        best = None
        for M, T in ((P, Q), (Q, P)):
            if m.fixed(t, M):
                continue
            fol = [x for x in m.at(t, M) if x is not t]
            if any(x.IsLocked() or id(x) in touched for x in fol):
                continue
            far = [other(x, M) for x in fol]
            if any((f.x, f.y) == (T.x, T.y) for f in far):
                continue
            tgt = [other(x, T) for x in m.at(t, T) if x is not t] + far
            before = min(min_angle(M, [other(t, M)] + far), min_angle(T, [other(x, T) for x in m.at(t, T)]))
            after = min_angle(T, tgt)
            if after < min(before, ANG_OK) - 0.01:
                continue
            sc = (len(fol), -after)
            if best is None or sc < best[0]:
                best = (sc, M, T, fol)
        if best is None:
            continue
        _, M, T, fol = best
        touched |= {id(t)} | {id(x) for x in fol}
        ops.append(lambda t=t, M=M, T=T, fol=fol: _contract(m, t, M, T, fol))
    return ops


def _contract(m, t, M, T, fol):
    M, T = VECTOR2I(M.x, M.y), VECTOR2I(T.x, T.y)
    for x in fol:
        if (x.GetStart().x, x.GetStart().y) == (M.x, M.y):
            x.SetStart(T)
        else:
            x.SetEnd(T)
    m.remove(t)


def plan_snap(m):
    ops, done = [], set()
    for k, lst in m.nodes.items():
        if len(lst) != 2 or k in done:
            continue
        a, b = lst
        if a.IsLocked() or b.IsLocked() or a is b:
            continue
        q = VECTOR2I(k[2], k[3])
        if m.fixed(a, q):
            continue
        A, B = other(a, q), other(b, q)
        ang = angle(q, A, B)
        if not (120 <= ang < ANG_OK):
            continue
        # neighbour vertices must not get worse
        nb = []
        for t, F in ((a, A), (b, B)):
            nl = m.at(t, F)
            nb.append((F, [other(x, F) for x in nl]))
        base = [min_angle(F, o) for F, o in nb]
        best = None
        r = MM(0.04)
        step = MM(0.002)
        n = int(r // step)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                p = VECTOR2I(q.x + i * step, q.y + j * step)
                d = math.hypot(i, j)
                if best is not None and d >= best[0]:
                    continue
                if angle(p, A, B) < 135.002:
                    continue
                if any(min_angle(F, [p if (x.x, x.y) == (q.x, q.y) else x for x in o]) < min(bs, ANG_OK) - 0.01
                       for (F, o), bs in zip(nb, base)):
                    continue
                best = (d, p)
        if best:
            done.add(k)
            ops.append(lambda a=a, b=b, q=q, p=best[1]: _move(a, b, q, p))
    return ops


def _move(a, b, q, p):
    for t in (a, b):
        if (t.GetStart().x, t.GetStart().y) == (q.x, q.y):
            t.SetStart(p)
        else:
            t.SetEnd(p)


def plan_chamfer(m):
    ops, touched = [], set()
    for k, lst in m.nodes.items():
        if len(lst) != 2:
            continue
        a, b = lst
        if a is b or a.IsLocked() or b.IsLocked() or id(a) in touched or id(b) in touched:
            continue
        q = VECTOR2I(k[2], k[3])
        if m.fixed(a, q):
            continue
        A, B = other(a, q), other(b, q)
        ang = angle(q, A, B)
        if not (90 - 0.01 <= ang < ANG_OK):
            continue
        La, Lb = length(a), length(b)
        w = max(a.GetWidth(), b.GetWidth())
        pick = None
        for d in (MM(x) for x in (0.5, 0.4, 0.3, 0.25, 0.2, 0.15)):
            if La - d >= a.GetWidth() and Lb - d >= b.GetWidth() and d * 2 ** 0.5 >= w * 0.5:
                pick = d
                break
        if pick is None:
            continue
        P1 = VECTOR2I(int(q.x + (A.x - q.x) * pick / La), int(q.y + (A.y - q.y) * pick / La))
        P2 = VECTOR2I(int(q.x + (B.x - q.x) * pick / Lb), int(q.y + (B.y - q.y) * pick / Lb))
        touched |= {id(a), id(b)}
        ops.append(lambda a=a, b=b, q=q, P1=P1, P2=P2: _chamfer(m, a, b, q, P1, P2))
    return ops


def _chamfer(m, a, b, q, P1, P2):
    for t, P in ((a, P1), (b, P2)):
        if (t.GetStart().x, t.GetStart().y) == (q.x, q.y):
            t.SetStart(P)
        else:
            t.SetEnd(P)
    m.new_track(a, P1, P2, width=min(a.GetWidth(), b.GetWidth()))


def make_plan_neck(wanted):
    def plan(m):
        # obstacles on every net/layer
        obs = []
        for t in m.tracks:
            obs.append((t.GetNetCode(), t.GetLayer(), t.GetEffectiveShape(), t))
        for v in m.vias:
            for lay in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
                obs.append((v.GetNetCode(), lay, v.GetEffectiveShape(lay), v))
        for p in m.pads:
            for lay in (pcbnew.F_Cu, pcbnew.B_Cu):
                if p.IsOnLayer(lay):
                    obs.append((p.GetNetCode(), lay, p.GetEffectiveShape(lay), p))
        by = {(t.GetNetname(), t.GetLayer(), round(TOMM(t.GetStart().x), 3), round(TOMM(t.GetStart().y), 3)): t
              for t in m.tracks}
        ops = []
        for key, want in wanted:
            t = by.get(key)
            if t is None or t.IsLocked():
                continue
            seg = pcbnew.SHAPE_SEGMENT(t.GetStart(), t.GetEnd(), want)
            ok = True
            for net, lay, sh, o in obs:
                if o is t or net == t.GetNetCode() or lay != t.GetLayer():
                    continue
                if pcbnew.SHAPE.Collide(sh, seg, MM(0.19)):
                    ok = False
                    break
            if ok:
                ops.append(lambda t=t, want=want: t.SetWidth(want))
        return ops
    return plan


# ---------------------------------------------------------------- batch driver with bisection
def run_batch(name, cur, base, plan_fn, minimum_gain=True):
    def trial(idx):
        out = WORK / "trial.kicad_pcb"
        subprocess.run([sys.executable, __file__, "--worker", plan_fn, str(cur), str(out), ",".join(map(str, idx))],
                       check=True, stderr=subprocess.DEVNULL)
        return out, drc(out)

    n = int(subprocess.run([sys.executable, __file__, "--count", plan_fn, str(cur)], check=True,
                           capture_output=True, text=True).stdout.split()[-1])
    if n == 0:
        log(f"{name}: nothing to do")
        return
    acc = []

    def go(idx):
        nonlocal acc
        if not idx:
            return
        out, res = trial(acc + idx)
        if drc_ok(res, base):
            acc = acc + idx
            shutil.copy(out, WORK / "good.kicad_pcb")
        elif len(idx) > 1:
            h = len(idx) // 2
            go(idx[:h])
            go(idx[h:])

    go(list(range(n)))
    log(f"{name}: {len(acc)}/{n} edits kept")
    if acc:
        shutil.copy(WORK / "good.kicad_pcb", cur)


def findings(path):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        cg.main(path)
    return [l for l in buf.getvalue().splitlines() if l.split()[0] in ("ACUTE", "SHORT", "NECKED", "OVERLAP")]


def census(path):
    return Counter(l.split()[0] for l in findings(path))


def wanted_neck(path):
    out = []
    for l in findings(path):
        mt = re.match(r"NECKED ([\d.]+)<([\d.]+)mm (\S+) (\S+) at \(([-\d.]+), ([-\d.]+)\)", l)
        if mt:
            lay = pcbnew.BOARD().GetLayerID(mt.group(4)) if False else {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu,
                                                                          "In1.Cu": pcbnew.In1_Cu, "In2.Cu": pcbnew.In2_Cu}[mt.group(4)]
            out.append(((mt.group(3), lay, float(mt.group(5)), float(mt.group(6))), MM(float(mt.group(2)))))
    return out


PLANS = {"overlap": plan_overlap, "staircase": plan_chain, "short": plan_short, "snap": plan_snap,
         "chamfer": plan_chamfer, "neck": lambda m: make_plan_neck(wanted_neck(m.path))(m)}


def worker():
    import os
    mode, name, path = sys.argv[1], sys.argv[2], Path(sys.argv[3])
    m = Model(path)
    ops = PLANS[name](m)
    if mode == "--count":
        print(len(ops))
    else:
        for i in (int(x) for x in sys.argv[5].split(",") if x):
            ops[i]()
        m.save(sys.argv[4])
    sys.stdout.flush()
    os._exit(0)


def main():
    global WORK, D
    WORK = Path(tempfile.mkdtemp(prefix="tidy_"))
    D = WORK / "d"
    D.mkdir()
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else HAT / "ps1-av-hat.kicad_pcb"
    cur = WORK / "cur.kicad_pcb"
    shutil.copy(src, cur)
    shutil.copy(src.with_suffix(".kicad_pro"), cur.with_suffix(".kicad_pro"))
    shutil.copy(src.with_suffix(".kicad_pro"), D / "ps1-av-hat.kicad_pro")
    shutil.copy(src.with_suffix(".kicad_dru"), D / "ps1-av-hat.kicad_dru")
    shutil.copy(src.with_suffix(".kicad_prl"), D / "ps1-av-hat.kicad_prl")
    for s in HAT.glob("*.kicad_sch"):
        shutil.copy(s, D / s.name)
    (D / "lib").symlink_to(HAT / "lib")
    for f in ("fp-lib-table",):
        if (HAT / f).exists():
            shutil.copy(HAT / f, D / f)
    base = drc(cur)
    log("start DRC", dict(base[0]), "unconnected", base[1], "parity", base[2])
    log("start geometry", dict(census(cur)))
    run_batch("overlap", cur, base, "overlap")
    run_batch("staircase", cur, base, "staircase")
    run_batch("short", cur, base, "short")
    run_batch("snap", cur, base, "snap")
    run_batch("chamfer", cur, base, "chamfer")
    run_batch("short2", cur, base, "short")
    run_batch("neck", cur, base, "neck")
    log("end geometry", dict(census(cur)))
    shutil.copy(cur, src)
    log("end DRC", dict(drc(src)[0]))
    shutil.rmtree(WORK, ignore_errors=True)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] in ("--worker", "--count"):
        worker()
    else:
        main()
