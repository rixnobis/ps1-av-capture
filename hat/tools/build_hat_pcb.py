#!/usr/bin/env python3
"""Build ps1-av-hat.kicad_pcb: outline, headers, mounting holes, placement. No routing.

  build_hat_pcb.py            (run from anywhere; reads hat/build/netlist.xml)

Geometry comes from the dev-board DXF via tools/mg_dxf_geometry.json (tools/extract_mg_dxf.py).
Working coordinates ("h-coords") are DXF x and hy = -DXF y, i.e. KiCad's y-down orientation
with no mirroring; every KiCad position is h-coords + (OFFX, OFFY).

Mechanical rule: the hat sits 8-11 mm above the dev board, so its outline stays clear of the
board's tall parts (barrel jack, screw terminal, RJ45, USB-A, see KEEPOUTS_DXF). The two
sockets reach x = -35.6 but the RJ45/USB-A start at x = -43.1/-38.0, so the outline is a plain
rectangle only on the left; on the right it has a notch between two ears (the ears carry the
two right-hand M3 holes).
"""
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pcbnew

TOOLS = Path(__file__).resolve().parent
HAT = TOOLS.parent
ROOT = HAT.parent
PCB = HAT / "ps1-av-hat.kicad_pcb"
NETLIST = HAT / "build" / "netlist.xml"
LIBS = {"ps1-av-capture": ROOT / "lib" / "av.pretty", "ps1-av-hat": HAT / "lib" / "hat.pretty"}
FPDIR = Path("/usr/share/kicad/footprints")
GEO = json.loads((TOOLS / "mg_dxf_geometry.json").read_text())

# ---- the one switch that is still unresolved: which DXF header is the board's U12 ------------
TOP_IS_U12 = True            # True: DXF top header = U12 = J4, bottom = U13 = J5. Flip + rebuild.

OFFX, OFFY = 190.0, 0.0      # sheet offset added to h-coords (keeps the hat on the A4 page)
PITCH = 2.54
COL0_X = -83.185             # DXF x of header column 0
HDR_TOP = {"pin1_dxf_y": -76.962}
HDR_BOT = {"pin1_dxf_y": -128.651}

# ---- tall parts of the dev board, DXF coordinates (silkscreen cluster bounding boxes) --------
KEEPOUTS_DXF = {
    "barrel jack (top-left)": [-116.65, -83.40, -102.50, -74.39],
    "screw terminal (left)": [-114.81, -96.52, -107.06, -87.00],
    "USB receptacle shells (left edge, lower)": [-116.65, -124.74, -115.17, -99.54],
    "RJ45": [-43.12, -99.18, -21.52, -83.08],
    "USB-A (right edge)": [-38.04, -117.45, -21.54, -104.95],
}
CLEAR = 1.0
# hat outline in DXF coords
Y_EXTRA = 0.25                           # beyond the board outline so header copper has >= 1 mm
L_DXF = KEEPOUTS_DXF["barrel jack (top-left)"][2] + CLEAR            # -101.5
RN_DXF = KEEPOUTS_DXF["RJ45"][0] - CLEAR                             # -44.12 (notch floor)
R_EAR_DXF = -26.0                        # ears reach just past the right-hand holes
NOTCH_Y = (KEEPOUTS_DXF["USB-A (right edge)"][1] - CLEAR, KEEPOUTS_DXF["RJ45"][3] + CLEAR)  # DXF -118.45..-82.08
BOARD_Y = (GEO["outline_bbox"][1], GEO["outline_bbox"][3])            # -130.245..-70.245
CORNER_R = 1.0
EDGE = 0.7                               # parts stay this far inside the outline

GAP = 0.15                               # half the courtyard-to-courtyard spacing used by the placer
ANCHOR_SKIP = {"GND"}


def mm(v):
    return pcbnew.FromMM(float(v))


def K(hx, hy):
    return pcbnew.VECTOR2I(mm(hx + OFFX), mm(hy + OFFY))


def h(p):
    """KiCad VECTOR2I -> h-coords (mm)."""
    return pcbnew.ToMM(p.x) - OFFX, pcbnew.ToMM(p.y) - OFFY


# outline in h-coords (hy = -DXF y): top edge hy = -BOARD_Y[1] - Y_EXTRA
H_T, H_B = -BOARD_Y[1] - Y_EXTRA, -BOARD_Y[0] + Y_EXTRA
H_L, H_RN, H_R = L_DXF, RN_DXF, R_EAR_DXF
H_N0, H_N1 = -NOTCH_Y[1], -NOTCH_Y[0]    # notch spans hy 82.08..118.45


def header_centres():
    """(J4 centre, J5 centre) in h-coords; pad 1 is at (-22.86, +2.54) from the centre."""
    cx = COL0_X + 18 * PITCH / 2
    top = (cx, -HDR_TOP["pin1_dxf_y"] - PITCH)
    bot = (cx, -HDR_BOT["pin1_dxf_y"] - PITCH)
    return (top, bot) if TOP_IS_U12 else (bot, top)


J4C, J5C = header_centres()
HOLES = {"H1": (-97.845, 73.745), "H2": (-97.845, 126.745), "H3": (-29.845, 73.745), "H4": (-29.845, 126.745)}

# Hand placement: ref -> (hx, hy, angle). Everything else is placed by anchor (see place_rest).
FIXED = {
    "J4": (*J4C, 0), "J5": (*J5C, 0),
    "U1": (-70.5, 105.0, 270),    # TVP7002: analog pins (1-25) face J1 on the left, B/G data pins face J5
    "J1": (-97.0, 96.0, 270),     # AV pigtail on the left edge, pin 1 at the top, grounds on the edge side
    "J3": (-97.30, 87.9, 270),    # USB-C, mouth towards -x, flush with the left outline edge
    "U7": (-85.5, 88.0, 90),      # PCM2903C next to J3
    "U8": (-88.0, 118.0, 0),      # 74LVC1G08 next to J5 pins 1-3
    "U2": (-72.0, 84.0, 0),       # buck, under J4 pins 7-9 (+5V), far from the analog input side
    "L1": (-65.0, 84.0, 0),
    "U3": (-56.0, 82.5, 0), "U4": (-56.0, 89.0, 0),     # 1V9 LDOs
    "U5": (-48.5, 86.0, 0),                              # 3V3A LDO
    "C40": (-67.0, 80.8, 0),                             # buck input cap 2: out of the SW node corridor U2.2 -> L1.1
    "TP1": (-64.4, 79.9, 0), "TP2": (-61.9, 79.9, 0), "TP3": (-59.4, 79.9, 0),   # MCU test points along the J4 side
}
FIXED.update({k: (*v, 0) for k, v in HOLES.items()})
# Series 22R (U1 side = pad 1): single row under U1's south pins, ordered by the U1 pad x
SERIES_Y_OFF = 10.9
SERIES_PITCH = 1.05


def read_netlist():
    r = ET.parse(NETLIST).getroot()
    comps = {}
    for c in r.find("components"):
        comps[c.get("ref")] = {
            "value": c.findtext("value"), "footprint": c.findtext("footprint"),
            "uuid": c.findtext("tstamps"), "part": c.find("libsource").get("part"),
            "fields": {f.get("name"): f.text for f in c.iter("field")},
            "dnp": any(p.get("name") == "dnp" for p in c.findall("property")),
            "sheet": c.find("sheetpath").get("names")}
    pads = {}
    for n in r.find("nets"):
        for node in n:
            pads[(node.get("ref"), node.get("pin"))] = n.get("name")
    return comps, pads


def load_fp(fpid):
    lib, name = fpid.split(":")
    fp = pcbnew.FootprintLoad(str(LIBS.get(lib, FPDIR / f"{lib}.pretty")), name)
    assert fp is not None, fpid
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def add_outline(board):
    def seg(a, b):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(K(*a))
        s.SetEnd(K(*b))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.05))
        board.Add(s)

    def arc(a, m, b):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_ARC)
        s.SetArcGeometry(K(*a), K(*m), K(*b))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.05))
        board.Add(s)

    r = CORNER_R
    d = r * (1 - 2 ** -0.5)           # corner-arc midpoint offset
    seg((H_L + r, H_T), (H_R - r, H_T))
    arc((H_R - r, H_T), (H_R - d, H_T + d), (H_R, H_T + r))
    seg((H_R, H_T + r), (H_R, H_N0))
    seg((H_R, H_N0), (H_RN, H_N0))
    seg((H_RN, H_N0), (H_RN, H_N1))
    seg((H_RN, H_N1), (H_R, H_N1))
    seg((H_R, H_N1), (H_R, H_B - r))
    arc((H_R, H_B - r), (H_R - d, H_B - d), (H_R - r, H_B))
    seg((H_R - r, H_B), (H_L + r, H_B))
    arc((H_L + r, H_B), (H_L + d, H_B - d), (H_L, H_B - r))
    seg((H_L, H_B - r), (H_L, H_T + r))
    arc((H_L, H_T + r), (H_L + d, H_T + d), (H_L + r, H_T))


# ---- footprint geometry cache: box and pad offsets at each rotation -----------------------
_geo = {}


def geom(fpid, ang):
    """(xmin, ymin, xmax, ymax of pads+courtyard, {padnum: (dx, dy)}) at origin, rotated."""
    key = (fpid, ang % 360)
    if key not in _geo:
        fp = load_fp(fpid)
        fp.SetPosition(pcbnew.VECTOR2I(0, 0))
        fp.SetOrientationDegrees(ang)
        xs, ys, pp = [], [], {}
        for p in fp.Pads():
            b = p.GetBoundingBox()
            xs += [b.GetLeft(), b.GetRight()]
            ys += [b.GetTop(), b.GetBottom()]
            pp.setdefault(p.GetNumber(), []).append((pcbnew.ToMM(p.GetX()), pcbnew.ToMM(p.GetY())))
        for g in fp.GraphicalItems():
            if g.GetLayer() == pcbnew.F_CrtYd:
                b = g.GetBoundingBox()
                xs += [b.GetLeft(), b.GetRight()]
                ys += [b.GetTop(), b.GetBottom()]
        _geo[key] = (pcbnew.ToMM(min(xs)), pcbnew.ToMM(min(ys)), pcbnew.ToMM(max(xs)),
                     pcbnew.ToMM(max(ys)), pp)
    return _geo[key]


class Placed:
    """Collision boxes (numpy) of everything placed so far."""

    def __init__(self):
        self.b = np.zeros((0, 4))

    def add(self, box):
        self.b = np.vstack([self.b, box])

    def hit(self, box):
        b = self.b
        return bool(len(b) and np.any((box[0] < b[:, 2]) & (b[:, 0] < box[2]) & (box[1] < b[:, 3]) & (b[:, 1] < box[3])))


def in_outline(box, e=EDGE):
    x0, y0, x1, y1 = box
    if x0 < H_L + e or y0 < H_T + e or y1 > H_B - e:
        return False
    if x1 <= H_RN - e:
        return True
    return x1 <= H_R - e and (y1 <= H_N0 - e or y0 >= H_N1 + e)


def main():
    pcbnew.KIID.SeedGenerator(1)  # deterministic board item UUIDs
    comps, pads = read_netlist()
    board = pcbnew.NewBoard(str(PCB))
    board.SetCopperLayerCount(4)
    nets = {}
    for name in sorted(set(pads.values())):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni
    fps = {}
    for ref, c in sorted(comps.items()):
        fp = load_fp(c["footprint"])
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH("/" + c["uuid"]))
        if c["fields"].get("LCSC"):
            fp.SetField("LCSC", c["fields"]["LCSC"])
            fp.GetField("LCSC").SetVisible(False)
        for fname, fval in c["fields"].items():     # schematic fields the footprint must carry (parity)
            if fname in ("Footprint", "Datasheet", "Description", "LCSC") or not fval:
                continue
            fp.SetField(fname, fval)
            fp.GetField(fname).SetVisible(False)
        if ref.startswith("TP"):     # in the schematic BOM; the library test point pad is excluded by default
            fp.SetAttributes(fp.GetAttributes() & ~pcbnew.FP_EXCLUDE_FROM_BOM)
        if c["dnp"]:
            fp.SetDNP(True)
        for pad in fp.Pads():
            n = pads.get((ref, pad.GetNumber()))
            if n:
                pad.SetNet(nets[n])
        board.Add(fp)
        fps[ref] = fp
    add_outline(board)

    placed = Placed()
    where = {}                          # ref -> (x, y, ang)
    boxes = {}

    def put(ref, x, y, a, margin=GAP, ring=0.0):
        fp = fps[ref]
        fp.SetOrientationDegrees(a)
        fp.SetPosition(K(x, y))
        g = geom(comps[ref]["footprint"], a)
        m = margin + ring
        box = (x + g[0] - m, y + g[1] - m, x + g[2] + m, y + g[3] + m)
        placed.add(box)
        boxes[ref] = box
        where[ref] = (x, y, a)

    def padh(ref, num):
        x, y, a = where[ref]
        return [(x + dx, y + dy) for dx, dy in geom(comps[ref]["footprint"], a)[4][num]]

    # headers first: their body (silk outline +0.5) is the keep-out band, not just the pad box
    for r in ("J4", "J5"):
        x, y, a = FIXED[r]
        put(r, x, y, a)
        placed.b = placed.b[:-1]
        body = (x - 24.19 - 0.5, y - 3.8 - 0.5, x + 24.19 + 0.5, y + 3.7 + 0.5)
        placed.add(body)
        boxes[r] = body
    for r in HOLES:
        put(r, *FIXED[r][:2], 0, margin=0.3)
    for r in ("U1", "J1", "J3", "U7", "U8", "U2", "L1", "U3", "U4", "U5", "C40", "TP1", "TP2", "TP3"):
        put(r, *FIXED[r], ring=0.1 if r == "U1" else 0.0)
    for r, (x, y, a) in FIXED.items():
        assert in_outline(boxes[r], 0.0) or r in HOLES or r in ("J4", "J5") or r == "J3", ("outside outline", r, boxes[r])

    # ---- orientation assertions ---------------------------------------------------------
    u1x, u1y, _ = where["U1"]
    p13 = padh("U1", "13")[0]
    p38 = padh("U1", "38")[0]
    p80 = padh("U1", "80")[0]
    assert p13[0] < u1x - 5, ("U1 analog side not facing J1 (left)", p13)
    assert p38[1] > u1y + 5, ("U1 B/G data side not facing J5 (down)", p38)
    assert p80[1] < u1y - 5, ("U1 pin 80 (EXT_CLK) not on the north side", p80)
    j1 = {n: padh("J1", n)[0] for n in map(str, range(1, 15))}
    assert sum(j1[str(n)][0] for n in range(1, 15, 2)) > \
        sum(j1[str(n)][0] for n in range(2, 15, 2)), "J1 signals must face U1, grounds on the edge side"
    assert j1["1"][1] < j1["13"][1], "J1 pin 1 should be at the top"
    print("orientation ok: U1 pin13", tuple(round(v, 2) for v in p13), "pin38", tuple(round(v, 2) for v in p38),
          "pin80", tuple(round(v, 2) for v in p80))

    # ---- series 22R row between U1 and J5 -----------------------------------------------
    series = []
    for ref, c in comps.items():
        if c["value"] == "22" and (pads.get((ref, "2")) or "").startswith("VD_"):
            tvp_net = pads[(ref, "1")]
            pin = [p for (r, p), n in pads.items() if r == "U1" and n == tvp_net][0]
            series.append((padh("U1", pin)[0][0], ref))
    series.sort()
    assert len(series) == 21, len(series)
    x0 = u1x - (len(series) - 1) * SERIES_PITCH / 2
    for i, (_, ref) in enumerate(series):
        put(ref, x0 + i * SERIES_PITCH, u1y + SERIES_Y_OFF, 270, margin=0.0)   # pad 1 up (towards U1)
        p1, p2 = padh(ref, "1")[0], padh(ref, "2")[0]
        assert p1[1] < p2[1], ("series R pad 1 not towards U1", ref)
    # ---- decoupling-cap anchors ---------------------------------------------------------
    fan = {}
    for (r, _), n in pads.items():
        fan[n] = fan.get(n, 0) + 1
    caps_on = {}

    def is_ic(r):
        return r[0] in "UXYLD" or r[:2] == "FB"

    def anchors_for(ref, fanmax=8):
        nets_ = [(p.GetNumber(), pads.get((ref, p.GetNumber()))) for p in fps[ref].Pads()]
        nets_ = [(pn, n) for pn, n in nets_ if n and n not in ANCHOR_SKIP and not n.startswith("unconnected")]
        decap = ref[0] == "C" and any(pads.get((ref, p.GetNumber())) == "GND" for p in fps[ref].Pads())
        cands = []
        for pn, n in nets_:
            for (r2, pin), n2 in pads.items():
                if n2 != n or r2 == ref or r2 not in where:
                    continue
                if decap and fan[n] > 4:
                    if not (is_ic(r2) and r2[0] in "UXY" and comps[r2]["sheet"] == comps[ref]["sheet"]):
                        continue
                if not decap and fan[n] > fanmax:
                    continue
                cands.append((caps_on.get((r2, pin), 0) if decap else 0, fan[n], r2, pin, n, pn))
        return decap, cands

    # offsets sorted by distance, 0.25 mm grid out to 16 mm
    g = np.arange(-16, 16.01, 0.25)
    gx, gy = np.meshgrid(g, g)
    offs = sorted(zip(gx.ravel(), gy.ravel()), key=lambda t: t[0] ** 2 + t[1] ** 2)
    offs = [o for o in offs if o[0] ** 2 + o[1] ** 2 <= 256]

    dist_report = []

    def place_near(ref, ax, ay, my_pad, ring=0.0):
        fpid = comps[ref]["footprint"]
        best = None
        for a in (0, 90, 180, 270):
            gm = geom(fpid, a)
            dxp, dyp = gm[4][my_pad][0]
            for ox, oy in offs:
                x, y = ax + ox - dxp, ay + oy - dyp          # pad lands at anchor + offset
                box = (x + gm[0] - GAP, y + gm[1] - GAP, x + gm[2] + GAP, y + gm[3] + GAP)
                if not in_outline(box) or placed.hit(box):
                    continue
                d = (ox * ox + oy * oy) ** 0.5
                if best is None or d < best[0] - 1e-9:
                    best = (d, x, y, a)
                break          # offs are sorted: first free spot for this rotation is its nearest
        return best

    todo = sorted((r for r in comps if r not in where), key=lambda r: (not is_ic(r), r[0] in "RC" and r[0] != "C", r))
    todo.sort(key=lambda r: (0 if is_ic(r) else 1 if r[0] == "C" else 2 if r[0] == "R" else 3))
    stuck, fanmax = 0, 8
    while todo:
        if stuck >= len(todo):
            assert fanmax < 10 ** 6, ("parts with no placed neighbour", todo)
            fanmax, stuck = 10 ** 6, 0     # last resort: anchor on big nets too
        ref = todo.pop(0)
        decap, cands = anchors_for(ref, fanmax)
        if not cands:
            todo.append(ref)
            stuck += 1
            continue
        stuck = 0
        cands.sort(key=lambda c: (c[0], c[1]))
        # try the best two anchors, keep the closer outcome
        res = None
        for c in cands[:3]:
            ax, ay = padh(c[2], c[3])[0]
            b = place_near(ref, ax, ay, c[5])
            if b and (res is None or b[0] < res[0][0]):
                res = (b, c)
        assert res, ("no room for", ref)
        (d, x, y, a), c = res
        caps_on[(c[2], c[3])] = caps_on.get((c[2], c[3]), 0) + 1
        put(ref, x, y, a)
        dist_report.append((ref, c[2], c[3], round(d, 2), c[4]))
    assert not todo, ("parts with no placed neighbour", todo)
    n_far = [d for d in dist_report if d[3] > 2.0 and comps[d[0]]["value"] in ("100n", "10n", "1n", "1u")
             and d[0][0] == "C"]
    print("placed", len(where), "parts;", len(dist_report), "by anchor;",
          "caps farther than 2 mm from their pin:", [(d[0], d[1], d[3]) for d in n_far])

    # ---- header assertions ---------------------------------------------------------------
    pad1 = {tuple(round(v, 3) for v in p) for p in GEO["header_square_pads"]}
    for jref, dy in (("J4", J4C[1]), ("J5", J5C[1])):
        # the DXF square pad this header must land on: the one whose row is the header's pin-1 row
        want = min(GEO["header_square_pads"], key=lambda p: abs(-p[1] - (dy + PITCH)))
        for n, (ex, ey) in (("1", (want[0], -want[1])), ("2", (want[0], -want[1] - PITCH)),
                            ("3", (want[0], -want[1] - 2 * PITCH))):
            p = padh(jref, n)[0]
            assert abs(p[0] - ex) < 0.01 and abs(p[1] - ey) < 0.01, (jref, n, p, (ex, ey))
        p1, p2, p3 = padh(jref, "1")[0], padh(jref, "2")[0], padh(jref, "3")[0]
        assert p1[1] > p2[1] > p3[1], "pins 2/3 must move away from the board bottom edge"
        print(f"{jref}: pad1 h=({p1[0]:.3f},{p1[1]:.3f}) = DXF ({p1[0]:.3f},{-p1[1]:.3f}) vs DXF square pad "
              f"({want[0]:.3f},{want[1]:.3f}) OK; pad2 y {p2[1]:.3f}, pad3 y {p3[1]:.3f} (DXF up: away from bottom edge) OK")
    print("TOP_IS_U12 =", TOP_IS_U12, "-> J4 on the", "top" if TOP_IS_U12 else "bottom", "header")
    print("sheet offset: KiCad = h-coords + (%.1f, %.1f); h = (DXF x, -DXF y)" % (OFFX, OFFY))
    print("outline h-coords: x %.3f..%.3f (ears %.3f), y %.3f..%.3f, notch x>%.3f for y %.3f..%.3f -> %.2f x %.2f mm"
          % (H_L, H_RN, H_R, H_T, H_B, H_RN, H_N0, H_N1, H_R - H_L, H_B - H_T))
    # reference text: passives and test points to F.Fab (silk would overlap on 1 mm spacing), holes hidden
    for r, fp in fps.items():
        if r[0] == "H":
            fp.Reference().SetVisible(False)
        elif r[0] in "RC" or r[:2] in ("FB", "TP"):
            fp.Reference().SetLayer(pcbnew.F_Fab)
    back = [r for r, fp in fps.items() if fp.GetLayer() != pcbnew.F_Cu]
    print("parts on B.Cu:", back or "none")
    board.Save(str(PCB))
    print("wrote", PCB)


if __name__ == "__main__":
    main()
