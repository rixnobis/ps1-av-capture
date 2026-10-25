#!/usr/bin/env python3
"""FX3 BGA escape, phase 1: a dogbone via for every inner signal ball.

Runs after fanout.py on the same board. Rings 1 and 2 of U2 (the outer two rows/columns)
escape on L1 between the balls; every signal ball from ring 3 inward gets a 0.45/0.2 via in
one of its four diagonal cells, so the bundle can leave on an inner layer. The cell picked is
the free one pointing the way the net has to go (towards U1 for the GPIF/TVP nets, outward
otherwise). Prints every via it placed and every ball it could not serve.
"""
import math
import sys
from pathlib import Path

import pcbnew

sys.path.insert(0, str(Path(__file__).resolve().parent))
from route_usb import OX, OY, PCB, footprints, tracks  # noqa: E402

VIA_D, VIA_DRILL, CLEAR, STUB = 0.45, 0.2, 0.15, 0.2
ROWS = "ABCDEFGHJKL"
SKIP_NETS = {"GND", "+3V3", "+1V2"}
U1_XY = (33.0, 26.0)           # build_pcb FIXED["U1"]


def mm(v):
    return pcbnew.ToMM(v)


def ring(num):
    r, c = ROWS.index(num[0]), int(num[1:]) - 1
    return min(r, c, 10 - r, 10 - c) + 1


def main():
    board = pcbnew.LoadBoard(str(PCB))
    nets = board.GetNetsByName()
    u2 = board.FindFootprintByReference("U2")
    cx, cy = mm(u2.GetPosition().x) - OX, mm(u2.GetPosition().y) - OY
    circles = []                                   # (net, x, y, r): balls and vias
    for p in u2.Pads():
        circles.append((p.GetNetname(), mm(p.GetPosition().x) - OX, mm(p.GetPosition().y) - OY,
                        mm(p.GetSize().x) / 2))
    for t in tracks(board):
        if t.GetClass() == "PCB_VIA":
            circles.append((t.GetNetname(), mm(t.GetPosition().x) - OX, mm(t.GetPosition().y) - OY,
                            mm(t.GetWidth(pcbnew.F_Cu)) / 2))
    segs = [(t.GetNetname(), (mm(t.GetStart().x) - OX, mm(t.GetStart().y) - OY),
             (mm(t.GetEnd().x) - OX, mm(t.GetEnd().y) - OY), mm(t.GetWidth()) / 2, t.GetLayer())
            for t in tracks(board) if t.GetClass() == "PCB_TRACK"]

    def seg_dist(px, py, a, b):
        L2 = (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2
        t = 0 if L2 == 0 else max(0, min(1, ((px - a[0]) * (b[0] - a[0]) + (py - a[1]) * (b[1] - a[1])) / L2))
        return math.hypot(px - a[0] - t * (b[0] - a[0]), py - a[1] - t * (b[1] - a[1]))

    def via_ok(net, x, y):
        for n, qx, qy, r in circles:
            d = math.hypot(x - qx, y - qy)
            if d < 0.05:
                return False                       # cell already holds a via
            if n != net and d - r < VIA_D / 2 + CLEAR:
                return False
        # any track on any layer passing the cell (a via spans them all)
        return all(n == net or seg_dist(x, y, a, b) - hw >= VIA_D / 2 + CLEAR
                   for n, a, b, hw, _ in segs)

    balls = sorted((p for p in u2.Pads() if p.GetNetname() not in SKIP_NETS
                    and not p.GetNetname().startswith(("unconnected", "USB3_"))
                    and ring(p.GetNumber()) >= 3), key=lambda p: -ring(p.GetNumber()))
    placed, failed = [], []
    for p in balls:
        net = p.GetNetname()
        px, py = mm(p.GetPosition().x) - OX, mm(p.GetPosition().y) - OY
        tx, ty = U1_XY if net.startswith(("VD_", "TVP_")) else (px + (px - cx), py + (py - cy))
        cands = [(px + sx * 0.4, py + sy * 0.4) for sx in (1, -1) for sy in (1, -1)]
        cands.sort(key=lambda q: math.hypot(q[0] - tx, q[1] - ty))
        for vx, vy in cands:
            if not via_ok(net, vx, vy):
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(OX + px), pcbnew.FromMM(OY + py)))
            t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(OX + vx), pcbnew.FromMM(OY + vy)))
            t.SetWidth(pcbnew.FromMM(STUB))
            t.SetLayer(pcbnew.F_Cu)
            t.SetNet(nets[net])
            t.SetLocked(True)
            board.Add(t)
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(OX + vx), pcbnew.FromMM(OY + vy)))
            v.SetWidth(pcbnew.FromMM(VIA_D))
            v.SetDrill(pcbnew.FromMM(VIA_DRILL))
            v.SetNet(nets[net])
            v.SetLocked(True)
            board.Add(v)
            circles.append((net, vx, vy, VIA_D / 2))
            segs.append((net, (px, py), (vx, vy), STUB / 2, pcbnew.F_Cu))
            placed.append(f"{p.GetNumber()} {net} ({vx:.1f},{vy:.1f})")
            break
        else:
            failed.append(f"{p.GetNumber()} {net}")
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(PCB))
    print(f"escape: {len(placed)} inner signal balls on dogbone vias, {len(failed)} not served")
    for f in failed:
        print("  unserved", f)


if __name__ == "__main__":
    main()
