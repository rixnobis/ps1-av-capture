#!/usr/bin/env python3
"""Build ps1-av-capture.kicad_pcb from the schematic netlist.

  build_pcb.py place   - board, stackup, net classes, outline, part placement

Coordinates are millimetres from the board's top-left corner. Every IC and connector
position is set here by hand; passives are placed next to the pin they serve by
place_passives(). Orientation of each IC is asserted against its pads after placing.
"""
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
NAME = "ps1-av-capture"
PCB = ROOT / f"{NAME}.kicad_pcb"
PRO = ROOT / f"{NAME}.kicad_pro"
SCH = ROOT / f"{NAME}.kicad_sch"
LIBS = {"ps1-av-capture": ROOT / "lib" / "av.pretty"}
FPDIR = Path("/usr/share/kicad/footprints")

OX, OY = 100.0, 100.0           # sheet offset of the board origin
W, H = 100.0, 80.0              # board size

# Hand placement: ref -> (x, y, angle). Angles are asserted below against pad positions.
FIXED = {
    "J1": (6.0, 12.0, 270),      # AV pigtail JST PHD 2x7: pin 1 at top, grounds (even) on the edge side, signals face U1
    "U1": (33.0, 26.0, 270),     # TVP7002: pins 1-25 face J1, data pins face U2
    "U2": (62.0, 50.0, 180),     # FX3: row A (USB) at the bottom, GPIF towards U1
    "J2": (62.0, 67.35, 0),      # USB3-B: body ends at +13.65 mm (mating face), flush with the bottom edge
    "H1": (3.5, 3.5, 0), "H2": (W - 3.5, 3.5, 0), "H3": (3.5, H - 3.5, 0), "H4": (W - 3.5, H - 3.5, 0),
    "J3": (22.0, 74.0, 0),       # USB-C (audio) at the bottom edge
    "PCM2903C": (22.0, 60.0, 0), # keyed by part; resolved to its reference in place()
}
# Second tier, also by hand: the power block on the right edge (one buck template repeated,
# VBUS comes up from J2), the TVP7002's LDOs above U1, and the debug/boot parts top right.
BUCKS = [("U5", "L1"), ("U6", "L2"), ("U7", "L3")]   # 3V3, 2V5, 1V2
BUCK_X, BUCK_Y, BUCK_DY = 80.0, 49.0, 9.5
HAND = {
    "U8": (46.0, 9.0, 0), "U9": (52.0, 9.0, 0),      # 2V5 -> 1V9A / 1V9D LDOs, above U1
    "U10": (20.0, 9.0, 0),                           # VBUS -> 3V3A LDO, above U1's analog side
    "U3": (78.0, 30.0, 0), "U4": (78.0, 38.0, 0),    # boot EEPROM + DNP spare
    "J4": (90.0, 13.0, 0), "J5": (90.0, 25.0, 0), "JP1": (90.0, 35.0, 0),   # JTAG, UART, USB boot
    "SW1": (80.0, 12.0, 0),
    "D3": (70.0, 4.0, 0), "D6": (75.0, 4.0, 0),      # status RGB + power LED on the top edge
}
# USB 3 corridor between U2's row A and J2 (tools/route_usb.py routes it). SS TX caps under the TX
# balls, the 4-channel ESD flow-through across all four SS lines, the D+/D- ESD to the left.
HAND.update({
    "C77": (61.9, 56.6, 270),     # USB3_TXN_C -> USB3_TXN, pad 1 up; 1.0 mm apart clears courtyards
    "C78": (62.9, 56.6, 270),     # USB3_TXP_C -> USB3_TXP
    "D4": (63.0, 59.5, 90),       # pads 1..5 left to right: TXN TXP GND RXP RXN
    "D5": (56.0, 57.5, 180),      # D- on the top row, D+ on the bottom row
})
KEEPOUT = [(52.0, 55.5, 70.0, 62.4)]   # nothing auto-placed in the corridor
for i, (u, l) in enumerate(BUCKS):
    HAND[u] = (BUCK_X, BUCK_Y + i * BUCK_DY, 0)
    HAND[l] = (BUCK_X + 6.0, BUCK_Y + i * BUCK_DY, 0)
FIXED_TVP = "U1"
ANCHOR_SKIP = {"GND"}          # nets never used to decide where a part goes
EDGE = 1.0                     # keep parts this far inside the outline
PREFER = {"75": "J1"}          # value -> host: video terminations sit at the input connector
GAP = 0.25                     # clearance between part outlines when packing
# Escape ring kept clear around the fine-pitch ICs: a first Freerouting pass with passives packed
# 0.25 mm from the bodies escaped only 499 of 802 SMD pins.
RING = {"U1": 1.5, "U2": 1.5}
DIFF_NETS = ["USB3_TXP*", "USB3_TXN*", "USB3_RXP", "USB3_RXN", "USB3_DP", "USB3_DM",
             "AUD_DP*", "AUD_DM*"]
POWER_NETS = ["VBUS", "VBUS_AUD", "+3V3*", "+2V5", "+1V2", "+1V9*", "TVP_AVDD",
              "TVP_PLLVDD", "FX3_AVDD", "FX3_U3*", "FX3_CVDDQ", "SW_*", "OSC_VDD"]


def mm(v):
    return pcbnew.FromMM(v)


def pt(x, y):
    return pcbnew.VECTOR2I(mm(OX + x), mm(OY + y))


def netlist():
    xml = ROOT / "build" / "netlist.xml"
    xml.parent.mkdir(exist_ok=True)
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadxml",
                    "-o", str(xml), str(SCH)], check=True, capture_output=True)
    r = ET.parse(xml).getroot()
    comps = {}
    for c in r.find("components"):
        fields = {f.get("name"): f.text for f in c.iter("field")}
        dnp = any(p.get("name") == "dnp" for p in c.findall("property"))
        comps[c.get("ref")] = {"value": c.findtext("value"), "footprint": c.findtext("footprint"),
                               "uuid": c.findtext("tstamps"), "fields": fields, "dnp": dnp,
                               "part": c.find("libsource").get("part"),
                               "sheet": c.find("sheetpath").get("names")}
    pads = {}
    for n in r.find("nets"):
        for node in n:
            pads[(node.get("ref"), node.get("pin"))] = n.get("name")
    return comps, pads


def write_project():
    d = json.loads(PRO.read_text())
    board = d.setdefault("board", {})
    ds = board.setdefault("design_settings", {})
    ns = d.setdefault("net_settings", {})
    base = {"bus_width": 12, "clearance": 0.15, "diff_pair_gap": 0.15, "diff_pair_via_gap": 0.25,
            "diff_pair_width": 0.25, "line_style": 0, "microvia_diameter": 0.3, "microvia_drill": 0.1,
            "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)",
            "tuning_profile": "", "via_diameter": 0.45, "via_drill": 0.2, "wire_width": 6}
    ns["classes"] = [dict(base, name="Default", priority=2147483647, track_width=0.2),
                     dict(base, name="Power", priority=1, track_width=0.5),
                     # GND is the L2 plane; tracks only stub small pads to vias
                     dict(base, name="Ground", priority=1, track_width=0.3),
                     # 90 ohm on JLC04161H-3313A L1/L2 per docs/stackup.md
                     dict(base, name="USB", priority=0, track_width=0.25, clearance=0.15)]
    ns["netclass_patterns"] = ([{"netclass": "USB", "pattern": n} for n in DIFF_NETS] +
                               [{"netclass": "Power", "pattern": n} for n in POWER_NETS] +
                               [{"netclass": "Ground", "pattern": "GND"}])
    ns["netclass_assignments"] = {}
    rules = ds.setdefault("rules", {})
    # JLCPCB 4-layer capabilities, with margin: 0.127 mm track/space, 0.2 mm via drill
    rules.update({"min_clearance": 0.127, "min_track_width": 0.127, "min_via_diameter": 0.45,
                  "min_via_annular_width": 0.1, "min_through_hole_diameter": 0.2,
                  "min_copper_edge_clearance": 0.4, "min_hole_clearance": 0.2,
                  "min_hole_to_hole": 0.4, "min_silk_clearance": 0.0, "min_text_height": 0.8,
                  "min_text_thickness": 0.15})
    PRO.write_text(json.dumps(d, indent=2) + "\n")


def load_fp(fpid):
    lib, name = fpid.split(":")
    path = LIBS.get(lib, FPDIR / f"{lib}.pretty")
    fp = pcbnew.FootprintLoad(str(path), name)
    assert fp is not None, fpid
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def outline(board):
    rad = 1.5
    segs = [((rad, 0), (W - rad, 0)), ((W, rad), (W, H - rad)),
            ((W - rad, H), (rad, H)), ((0, H - rad), (0, rad))]
    for a, b in segs:
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(pt(*a))
        s.SetEnd(pt(*b))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        board.Add(s)
    for cx, cy, sx, sy in [(rad, rad, 0, rad), (W - rad, rad, W - rad, 0),
                           (W - rad, H - rad, W, H - rad), (rad, H - rad, rad, H)]:
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_ARC)
        s.SetCenter(pt(cx, cy))
        s.SetStart(pt(sx, sy))
        s.SetArcAngleAndEnd(pcbnew.EDA_ANGLE(90, pcbnew.DEGREES_T), True)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(mm(0.1))
        board.Add(s)


def padxy(board, ref, num):
    fp = board.FindFootprintByReference(ref)
    p = [q for q in fp.Pads() if q.GetNumber() == num][0].GetPosition()
    return round(pcbnew.ToMM(p.x) - OX, 3), round(pcbnew.ToMM(p.y) - OY, 3)


def check_orientation(board):
    """Each placed IC must face where the floorplan says. Fails loudly on a wrong angle."""
    ux, uy = FIXED["U1"][:2]
    p13 = padxy(board, "U1", "13")      # middle of the analog side
    p38 = padxy(board, "U1", "38")      # B data side
    p60 = padxy(board, "U1", "60")      # R data side
    assert p13[0] < ux - 5, ("U1 analog side not facing left", p13)
    assert p38[1] > uy + 5, ("U1 B/G data side not facing down", p38)
    assert p60[0] > ux + 5, ("U1 R data side not facing right", p60)
    fx, fy = FIXED["U2"][:2]
    a4 = padxy(board, "U2", "A4")       # SSRXP, row A
    f10 = padxy(board, "U2", "F10")     # GPIO[0]
    assert a4[1] > fy + 3, ("U2 row A not at the bottom", a4)
    assert f10[0] < fx - 2, ("U2 DQ[0] not on the side facing U1", f10)
    print("orientation ok: U1 pin13", p13, "pin38", p38, "pin60", p60, "| U2 A4", a4, "F10", f10)


def fp_box(fp, margin=0.0):
    """Outline of a footprint from its pads and fab/courtyard graphics, in board mm."""
    xs, ys = [], []
    for p in fp.Pads():
        bb = p.GetBoundingBox()
        xs += [bb.GetLeft(), bb.GetRight()]
        ys += [bb.GetTop(), bb.GetBottom()]
    for g in fp.GraphicalItems():
        if g.GetLayer() in (pcbnew.F_CrtYd, pcbnew.F_Fab, pcbnew.B_CrtYd) and g.GetClass() != "PCB_TEXT":
            bb = g.GetBoundingBox()
            xs += [bb.GetLeft(), bb.GetRight()]
            ys += [bb.GetTop(), bb.GetBottom()]
    f = lambda v, o: pcbnew.ToMM(v) - o  # noqa: E731
    return (min(f(x, OX) for x in xs) - margin, min(f(y, OY) for y in ys) - margin,
            max(f(x, OX) for x in xs) + margin, max(f(y, OY) for y in ys) + margin)


def overlaps(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def place_rest(board, comps, pads):
    """Hand tier (HAND), then every other part next to the pad it serves.

    A part's anchor is the pad of an already-placed part on the net it shares with the fewest
    other pads, GND excluded. A decoupling cap (rail + GND) goes to the rail pin of that IC with
    the fewest caps so far, so caps spread over the rail's pins instead of piling on one.
    The part is pushed out from its anchor's footprint centre until it clears everything placed.
    """
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    placed = set(FIXED)
    for ref, (x, y, a) in HAND.items():
        fps[ref].SetPosition(pt(x, y))
        fps[ref].SetOrientationDegrees(a)
        placed.add(ref)
    boxes = {r: fp_box(fps[r], GAP + RING.get(r, 0.0)) for r in placed}
    fan = {}
    for (r, _), n in pads.items():
        fan[n] = fan.get(n, 0) + 1
    caps_on = {}
    rails = {n for n, k in fan.items() if k >= 6}     # multi-pin supply nets
    todo = sorted(set(comps) - placed, key=lambda r: (r[0] in "RC", r))
    stuck = 0
    while todo and stuck < len(todo):
        ref = todo.pop(0)
        fp = fps[ref]
        nets = [pads.get((ref, p.GetNumber())) for p in fp.Pads()]
        nets = [n for n in nets if n and n not in ANCHOR_SKIP and not n.startswith("unconnected")]
        decap = ref[0] == "C" and "GND" in [pads.get((ref, p.GetNumber())) for p in fp.Pads()]
        cands = []
        for n in nets:
            for (r2, pin), n2 in pads.items():
                if n2 == n and r2 != ref and r2 in placed:
                    if decap and n in rails and not (r2[0] in "UL" and comps[r2]["sheet"] == comps[ref]["sheet"]):
                        continue      # a rail cap decouples an IC on its own sheet, not a header
                    cands.append((fan[n], caps_on.get((r2, pin), 0), r2, pin, n))
        if not cands:
            todo.append(ref)
            stuck += 1
            continue
        stuck = 0
        key = (lambda c: (c[1], c[0])) if decap else (lambda c: (c[0], c[1]))
        pref = [c for c in cands if c[2] == PREFER.get(comps[ref]["value"])]
        _, _, r2, pin, net = min(pref or cands, key=key)
        caps_on[(r2, pin)] = caps_on.get((r2, pin), 0) + 1
        ax, ay = padxy(board, r2, pin)
        host = boxes[r2]
        cx, cy = (host[0] + host[2]) / 2, (host[1] + host[3]) / 2
        dx, dy = ax - cx, ay - cy
        horiz = abs(dx) * (host[3] - host[1]) >= abs(dy) * (host[2] - host[0])
        ux, uy = ((1 if dx >= 0 else -1), 0) if horiz else (0, (1 if dy >= 0 else -1))
        fp.SetOrientationDegrees(0 if horiz else 90)
        mine = [p for p in fp.Pads() if pads.get((ref, p.GetNumber())) == net]
        if mine and len(list(fp.Pads())) == 2:
            # pad on the anchor net faces the anchor
            fp.SetPosition(pt(0, 0))
            px = pcbnew.ToMM(mine[0].GetPosition().x) - OX
            py = pcbnew.ToMM(mine[0].GetPosition().y) - OY
            if (px * ux + py * uy) > 0:
                fp.SetOrientationDegrees(fp.GetOrientationDegrees() + 180)
        best = None
        for step in range(0, 80):
            d = step * 0.5
            for lat in [0] + [s * k * 0.5 for k in range(1, 25) for s in (1, -1)]:
                x = (host[2] if ux > 0 else host[0] if ux < 0 else ax + lat) if horiz else ax + lat
                y = (host[3] if uy > 0 else host[1] if uy < 0 else ay + lat) if not horiz else ay + lat
                fp.SetPosition(pt(x, y))
                bx = fp_box(fp, GAP)
                w2, h2 = (bx[2] - bx[0]) / 2, (bx[3] - bx[1]) / 2
                x += ux * (w2 + d)
                y += uy * (h2 + d)
                fp.SetPosition(pt(x, y))
                bx = fp_box(fp, GAP)
                if bx[0] < EDGE or bx[1] < EDGE or bx[2] > W - EDGE or bx[3] > H - EDGE:
                    continue
                if any(overlaps(bx, o) for o in list(boxes.values()) + KEEPOUT):
                    continue
                best = bx
                break
            if best:
                break
        assert best, ("no room for", ref, "near", r2, pin)
        boxes[ref] = best
        placed.add(ref)
    assert not todo, ("parts with no placed neighbour", todo)
    print("placed", len(placed) - len(FIXED) - len(HAND), "parts next to their anchors")


def place():
    write_project()
    comps, pads = netlist()
    for part in [k for k in FIXED if not k[:1].isupper() or len(k) > 4]:
        refs = [r for r, c in comps.items() if c["part"] == part]
        assert len(refs) == 1, (part, refs)
        FIXED[refs[0]] = FIXED.pop(part)
    board = pcbnew.NewBoard(str(PCB))
    board.SetCopperLayerCount(4)
    nets = {}
    for name in sorted(set(pads.values())):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni
    stage_x, stage_y = W + 10, 0.0          # unplaced passives wait off-board, in a column
    for ref, c in sorted(comps.items()):
        fp = load_fp(c["footprint"])
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH("/" + c["uuid"]))
        if c["fields"].get("LCSC"):
            fp.SetField("LCSC", c["fields"]["LCSC"])
            fp.GetField("LCSC").SetVisible(False)
        if c["dnp"]:
            fp.SetDNP(True)
        board.Add(fp)
        if ref in FIXED:
            x, y, a = FIXED[ref]
        else:
            x, y, a = stage_x, stage_y, 0
            stage_y += 3.0
            if stage_y > 150:
                stage_x, stage_y = stage_x + 6, 0.0
        fp.SetPosition(pt(x, y))
        fp.SetOrientationDegrees(a)
        for pad in fp.Pads():
            net = pads.get((ref, pad.GetNumber()))
            if net:
                pad.SetNet(nets[net])
            if ref == FIXED_TVP and pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH:
                # thermal-pad via grid joins the GND plane solid; spokes starve at 0.8 mm pitch
                pad.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    check_orientation(board)
    place_rest(board, comps, pads)
    outline(board)
    board.Save(str(PCB))
    print("fixed", len(FIXED), "hand", len(HAND))


if __name__ == "__main__":
    {"place": place}[sys.argv[1]]()
