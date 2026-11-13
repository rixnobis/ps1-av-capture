#!/usr/bin/env python3
"""In1 GND plane check: filled area as a percentage of the board area, islands and breaks.

  islands: filled pieces other than the largest one (a piece cut off from the main plane)
  breaks:  holes in the fill larger than --min mm2 (default 0.25), listed with position and the
           copper that causes them: a header pad or via antipad is expected, a track slot or a
           hole with nothing inside is a real break.
usage: check_plane.py [board.kicad_pcb] [--min 0.25]     (board must have filled zones)
"""
import sys
from pathlib import Path

import pcbnew

HAT = Path(__file__).resolve().parent.parent


def area_mm2(chain):
    return abs(chain.Area()) / 1e12


def main(path, thr):
    board = pcbnew.LoadBoard(str(path))
    ol = pcbnew.SHAPE_POLY_SET()
    board.GetBoardPolygonOutlines(ol, True)
    board_area = sum(area_mm2(ol.COutline(i)) for i in range(ol.OutlineCount()))
    zones = [z for z in (board.GetArea(i) for i in range(board.GetAreaCount()))
             if z.GetLayer() == pcbnew.In1_Cu and z.GetNetname() == "GND" and not z.GetIsRuleArea()]
    assert zones, "no In1 GND zone"
    fill_total, pieces, holes = 0.0, [], []
    for z in zones:
        fp = pcbnew.SHAPE_POLY_SET(z.GetFilledPolysList(pcbnew.In1_Cu))
        fp.Unfracture()
        for i in range(fp.OutlineCount()):
            a = area_mm2(fp.COutline(i))
            h_total = 0.0
            for j in range(fp.HoleCount(i)):
                ha = area_mm2(fp.CHole(i, j))
                h_total += ha
                if ha > thr:
                    bb = fp.CHole(i, j).BBox()
                    holes.append((ha, pcbnew.ToMM(bb.Centre().x), pcbnew.ToMM(bb.Centre().y),
                                  pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())))
            pieces.append((a - h_total, i))
            fill_total += a - h_total
    pieces.sort(reverse=True)
    print(f"In1 GND fill {fill_total:.1f} mm2 of board {board_area:.1f} mm2 = {100 * fill_total / board_area:.1f} %")
    small = [p for p in pieces[1:] if p[0] > thr]
    print(f"filled pieces: {len(pieces)}, largest {pieces[0][0]:.1f} mm2; islands > {thr} mm2: {len(small)}")
    for a, i in small:
        print(f"  island {a:.2f} mm2 (outline {i})")
    # classify holes: something drilled / padded inside?
    pads = [(p.GetPosition(), p.GetBoundingBox()) for fp in board.GetFootprints() for p in fp.Pads()
            if p.GetDrillSize().x > 0]
    vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    plain = []
    for ha, x, y, w, hgt in sorted(holes, reverse=True):
        pt = pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))
        tag = None
        for pos, bb in pads:
            if bb.Contains(pt):
                tag = "TH pad antipad"
                break
        if not tag:
            for v in vias:
                if (v.GetPosition() - pt).EuclideanNorm() < pcbnew.FromMM(1.2):
                    tag = "via antipad cluster"
                    break
        if not tag:
            plain.append((ha, x, y, w, hgt))
    print(f"holes > {thr} mm2: {len(holes)} (explained by TH pad / via antipads: {len(holes) - len(plain)})")
    for ha, x, y, w, hgt in plain:
        print(f"  BREAK hole {ha:.2f} mm2 at ({x:.2f}, {y:.2f}) size {w:.2f} x {hgt:.2f} mm")
    return 1 if small or plain else 0


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("board", nargs="?", default=str(HAT / "ps1-av-hat.kicad_pcb"))
    ap.add_argument("--min", type=float, default=0.25)
    a = ap.parse_args()
    sys.exit(main(a.board, a.min))
