#!/usr/bin/env python3
"""Corrections applied to the LCSC land patterns in lib/av.pretty (easyeda2kicad output).

- TVP7002 HTQFP-100: the LCSC exposed pad is 5.445 mm and carries 9 unnumbered, irregularly
  placed vias. The OSSC, which builds this chip in production (marqs85/ossc_pcb,
  TQFP-100_thermal), uses a 6.2 x 6.2 mm pad with a 7 x 7 grid of 0.33 mm vias at 0.8 mm
  pitch. Same here: pad 101 is 6.2 mm and the vias are numbered 101 so they join GND.
- FX3 BGA-121: LCSC pads are 0.40 mm. AN70707 recommends 11 mil (0.28 mm) for pad, mask
  and paste.
- USB3-B: the shell pads are numbered 10/11; the USB3_B symbol calls the shell SH.
Each edit asserts its anchor, so a changed source file fails loudly. Already-applied edits are
reported and skipped.
"""
import re
from pathlib import Path

LIB = Path(__file__).resolve().parent.parent / "lib" / "av.pretty"


def edit(path, fn):
    s = path.read_text()
    out = fn(s)
    if out == s:
        print(f"{path.name}: already applied")
    else:
        path.write_text(out)
        print(f"{path.name}: updated")


def tvp(s):
    if "(size 6.200 6.200)" in s:
        return s
    old_ep = "(pad 101 smd rect (at 0.00 0.00 0.00) (size 5.445 5.445) (layers F.Cu F.Paste F.Mask))"
    assert old_ep in s, "TVP EP anchor"
    vias = re.findall(r'\t\(pad "" thru_hole circle [^\n]*\n', s)
    assert len(vias) == 9, f"expected 9 unnumbered vias, found {len(vias)}"
    for v in vias:
        s = s.replace(v, "", 1)
    grid = "".join(
        f'\t(pad 101 thru_hole circle (at {x * 0.8:.2f} {y * 0.8:.2f}) (size 0.60 0.60) (drill 0.30)'
        f' (layers *.Cu))\n' for y in range(-3, 4) for x in range(-3, 4))
    return s.replace(old_ep, "(pad 101 smd rect (at 0.00 0.00 0.00) (size 6.200 6.200) "
                             "(layers F.Cu F.Paste F.Mask))\n" + grid.rstrip("\n"), 1)


def bga(s):
    n = s.count("(size 0.400 0.400)")
    if n == 0:
        assert s.count("(size 0.280 0.280)") == 121, "BGA neither original nor fixed"
        return s
    assert n == 121, f"expected 121 BGA pads at 0.40 mm, found {n}"
    return s.replace("(size 0.400 0.400)", "(size 0.280 0.280)")


def usb3b(s):
    if "(pad SH thru_hole" in s:
        return s
    for old in ("(pad 10 thru_hole circle (at -6.02 3.35 0.00)", "(pad 11 thru_hole circle (at 6.02 3.35 0.00)"):
        assert old in s, old
        s = s.replace(old, old.replace("(pad 10 ", "(pad SH ").replace("(pad 11 ", "(pad SH "), 1)
    return s


edit(LIB / "HTQFP-100_L14.0-W14.0-P0.50-LS16.0-BL-EP6.2.kicad_mod", tvp)
edit(LIB / "BGA-121_L10.0-W10.0-R11-C11-P0.80-TL.kicad_mod", bga)
edit(LIB / "USB-B-TH_9P-L18.5-W12.0.kicad_mod", usb3b)
