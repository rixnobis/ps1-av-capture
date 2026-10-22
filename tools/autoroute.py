#!/usr/bin/env python3
"""Freerouting pass for everything the hand router left: export DSN, run, import SES.

  autoroute.py export   - inner layers marked as planes, DSN to build/
  autoroute.py import   - SES back into the board, zones refilled
Freerouting itself runs between the two (tools/build.sh); it is not deterministic, so the
build gates on DRC and check_geometry.py, never on a particular result.
"""
import sys
from pathlib import Path

import pcbnew

ROOT = Path(__file__).resolve().parent.parent
PCB = ROOT / "ps1-av-capture.kicad_pcb"
DSN = ROOT / "build" / "ps1-av-capture.dsn"
SES = ROOT / "build" / "ps1-av-capture.ses"


def export():
    board = pcbnew.LoadBoard(str(PCB))
    # L2 is the solid GND plane. L3 carries the supply pours AND signals: two signal layers
    # stalled Freerouting at 89-131 unrouted. A rule area keeps tracks off L3 under the USB
    # corridor, where L3 is the D+/D- reference (route_usb.py).
    board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)
    board.Save(str(PCB))
    assert pcbnew.ExportSpecctraDSN(board, str(DSN)), "DSN export failed"
    print("dsn", DSN.stat().st_size, "bytes")


def imp():
    board = pcbnew.LoadBoard(str(PCB))
    assert pcbnew.ImportSpecctraSES(board, str(SES)), "SES import failed"
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(str(PCB))
    print("tracks", len(board.GetTracks()))


if __name__ == "__main__":
    {"export": export, "import": imp}[sys.argv[1]]()
