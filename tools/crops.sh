#!/bin/sh
# Zoomed (130 px/mm, about 4x the board renders) F.Cu | B.Cu crops of every pad row and
# the resistor row, for looking at routing detail the board-scale renders hide.
# B.Cu is NOT mirrored here, so both halves of a crop share the same coordinates.
# usage: tools/crops.sh [board.kicad_pcb] [outdir]
set -e
cd "$(dirname "$0")/.."
pcb=${1:-ps1-av-capture.kicad_pcb}
out=${2:-renders/crops}
PXMM=130
mkdir -p "$out"
tmp=$(mktemp -d)
for L in F.Cu B.Cu; do
    kicad-cli pcb export svg --mode-single --fit-page-to-board --exclude-drawing-sheet \
        --page-size-mode 2 --layers "$L,Edge.Cuts" -o "$tmp/$L.svg" "$pcb" >/dev/null
    # board is 40 x 51 mm; the svg is fitted to the board outline
    rsvg-convert -b white -w $((40 * PXMM)) "$tmp/$L.svg" -o "$tmp/$L.png"
done
crop() {  # name x0 y0 w h (mm, board coordinates)
    g="$(($4 * PXMM))x$(($5 * PXMM))+$(($2 * PXMM))+$(($3 * PXMM))"
    magick "$tmp/F.Cu.png" -crop "$g" +repage -bordercolor black -border 4 "$tmp/f.png"
    magick "$tmp/B.Cu.png" -crop "$g" +repage -bordercolor black -border 4 "$tmp/b.png"
    magick "$tmp/f.png" "$tmp/b.png" +append "$out/$1.png"
}
crop J1-pins1-5   5  6 7 14
crop J1-pins6-10  5 19 7 14
crop J2-pins1-5  28  6 7 14
crop J2-pins6-10 28 19 7 14
crop R-J3-left    8 34 7 14
crop R-J3-midL   14 34 7 14
crop R-J3-midR   20 34 7 14
crop R-J3-right  26 34 7 14
rm -r "$tmp"
ls "$out"
