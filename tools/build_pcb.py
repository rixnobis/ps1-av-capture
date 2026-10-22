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
DIFF_NETS = ["USB3_TXP*", "USB3_TXN*", "USB3_RXP", "USB3_RXN", "USB3_DP", "USB3_DM",
             "AUD_DP*", "AUD_DM*"]
POWER_NETS = ["GND", "VBUS", "VBUS_AUD", "+3V3*", "+2V5", "+1V2", "+1V9*", "TVP_AVDD",
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
                               "part": c.find("libsource").get("part")}
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
            "tuning_profile": "", "via_diameter": 0.6, "via_drill": 0.3, "wire_width": 6}
    ns["classes"] = [dict(base, name="Default", priority=2147483647, track_width=0.2),
                     dict(base, name="Power", priority=1, track_width=0.5),
                     # 90 ohm on JLC04161H-3313A L1/L2 per docs/stackup.md
                     dict(base, name="USB", priority=0, track_width=0.25, clearance=0.2)]
    ns["netclass_patterns"] = ([{"netclass": "USB", "pattern": n} for n in DIFF_NETS] +
                               [{"netclass": "Power", "pattern": n} for n in POWER_NETS])
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
    check_orientation(board)
    outline(board)
    board.Save(str(PCB))
    print("placed", len(FIXED), "fixed parts;", len(comps) - len(FIXED), "staged off-board")


if __name__ == "__main__":
    {"place": place}[sys.argv[1]]()
