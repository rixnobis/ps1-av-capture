"""Symbol handling and schematic emission for gen_sch.py.

Stock symbols are copied out of the KiCad library (a derived symbol is replaced by its
base, renamed, so the embedded copy is self-contained). Symbols for parts the library
lacks are built from the pin tables in tools/pins/.
Every pin gets a short stub and a global label, so connectivity is carried by net names
across all sheets.
"""
import math
import re
import uuid as uuidlib
from pathlib import Path

SYMDIR = Path("/usr/share/kicad/symbols")
PROJECT = "ps1-av-capture"


def uid():
    return str(uuidlib.uuid4())


def fmt(v):
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def esc(s):
    return s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


def _sexpr_at(text, start):
    depth = 0
    i = start
    in_str = False
    while True:
        c = text[i]
        if in_str:
            if c == "\\":
                i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        i += 1


_libcache = {}


def _raw_symbol(lib, name):
    text = _libcache.setdefault(lib, (SYMDIR / f"{lib}.kicad_sym").read_text())
    m = re.search(r'\n\t\(symbol "%s"\n' % re.escape(name), text)
    if not m:
        raise SystemExit(f"symbol {lib}:{name} not found")
    return _sexpr_at(text, m.start() + 2)


def _pins_of(body):
    pins = {}
    for pm in re.finditer(
            r'\(pin (\w+) \w+\s*\(at ([-\d.]+) ([-\d.]+) ([-\d.]+)\)\s*\(length ([-\d.]+)\)'
            r'(?:\s*\(hide yes\))?\s*\(name "([^"]*)"(?:.|\n)*?\(number "([^"]+)"', body):
        etype, x, y, a, ln, name, num = pm.groups()
        # unit of this pin: the enclosing sub-symbol "<name>_<unit>_<style>"
        head = body[:pm.start()]
        sub = re.findall(r'\(symbol "[^"]*_(\d+)_(\d+)"', head)
        unit = int(sub[-1][0]) if sub else 1
        pins.setdefault(num, []).append((float(x), float(y), float(a), unit, etype, name))
    return pins


class Sym:
    """A symbol ready to embed: lib_id, body text, pins {num: [(x,y,ang,unit,type,name)]}."""

    def __init__(self, lib_id, body, units=1):
        self.lib_id = lib_id
        self.body = body
        self.pins = _pins_of(body)
        self.units = max([1] + [p[3] for v in self.pins.values() for p in v])

    def bbox(self, unit=1):
        pts = [(p[0], p[1]) for v in self.pins.values() for p in v if p[3] in (unit, 0)]
        if not pts:
            return (-2.54, -2.54, 2.54, 2.54)
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return (min(xs), min(ys), max(xs), max(ys))


def stock(lib, name, base=None):
    """Copy lib:name. If it is derived (extends), embed its base under lib:name instead."""
    body = _raw_symbol(lib, name)
    ext = re.search(r'\(extends "([^"]+)"\)', body)
    if ext:
        base = ext.group(1)
        body = _raw_symbol(lib, base)
        body = body.replace(f'"{base}_', f'"{name}_')
        body = body.replace(f'(symbol "{base}"', f'(symbol "{name}"', 1)
    body = body.replace(f'(symbol "{name}"', f'(symbol "{lib}:{name}"', 1)
    return Sym(f"{lib}:{name}", body)


def _pin_sexpr(num, name, etype, x, y, ang, length=2.54, hide_name=False):
    return (f'(pin {etype} line (at {fmt(x)} {fmt(y)} {ang}) (length {length})'
            f' (name "{esc(name)}" (effects (font (size 1.27 1.27))))'
            f' (number "{esc(num)}" (effects (font (size 1.27 1.27)))))')


def custom(name, units, ref="U", footprint=""):
    """units: list of dicts {left:[(num,name,type)], right:[...], top:[], bottom:[]}.
    Pins on a 2.54 grid; body sized to fit the longest side and the longest pin name."""
    parts = [f'(symbol "{PROJECT}:{name}" (pin_names (offset 1.016)) (exclude_from_sim no)'
             f' (in_bom yes) (on_board yes)',
             f'(property "Reference" "{ref}" (at 0 2.54 0) (effects (font (size 1.27 1.27))))',
             f'(property "Value" "{name}" (at 0 -2.54 0) (effects (font (size 1.27 1.27))))',
             f'(property "Footprint" "{footprint}" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))',
             f'(property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) (hide yes)))']
    for u, spec in enumerate(units, 1):
        L, R = spec.get("left", []), spec.get("right", [])
        T, B = spec.get("top", []), spec.get("bottom", [])
        rows = max(len(L), len(R), 1)
        cols = max(len(T), len(B), 0)
        namew = max([len(p[1]) for p in L + R] + [4]) * 1.27 * 0.75
        w = max(2 * namew + 5.08, cols * 2.54 + 5.08)
        w = math.ceil(w / 2.54) * 2.54
        h = (rows + 1) * 2.54
        x0, y0 = -w / 2, h / 2
        body = [f'(symbol "{name}_{u}_1"',
                f'(rectangle (start {fmt(x0)} {fmt(y0)}) (end {fmt(-x0)} {fmt(-y0)})'
                f' (stroke (width 0.254) (type default)) (fill (type background)))']
        for i, (num, pname, et) in enumerate(L):
            body.append(_pin_sexpr(num, pname, et, x0 - 2.54, y0 - 2.54 * (i + 1), 0))
        for i, (num, pname, et) in enumerate(R):
            body.append(_pin_sexpr(num, pname, et, -x0 + 2.54, y0 - 2.54 * (i + 1), 180))
        for i, (num, pname, et) in enumerate(T):
            body.append(_pin_sexpr(num, pname, et, x0 + 2.54 * (i + 1), y0 + 2.54, 270))
        for i, (num, pname, et) in enumerate(B):
            body.append(_pin_sexpr(num, pname, et, x0 + 2.54 * (i + 1), -y0 - 2.54, 90))
        body.append(")")
        parts.append(" ".join(body))
    parts.append(")")
    return Sym(f"{PROJECT}:{name}", "\n".join(parts))


class Sheet:
    def __init__(self, name, fname, title, paper="A3"):
        self.name, self.fname, self.title, self.paper = name, fname, title, paper
        self.uuid = uid()
        self.items = []
        self.syms = {}
        self.cursor = None

    def text(self, s, x, y, size=1.27):
        self.items.append(
            f'(text "{esc(s)}" (exclude_from_sim no) (at {fmt(x)} {fmt(y)} 0)'
            f' (effects (font (size {size} {size})) (justify left bottom)) (uuid "{uid()}"))')

    def place(self, sym, ref, value, x, y, nets, unit=1, footprint="", fields=(), dnp=False,
              in_bom=True, path=None):
        """nets: {pin_number: netname | None}. None draws a no-connect flag."""
        self.syms[sym.lib_id] = sym
        pin_uuids = " ".join(f'(pin "{n}" (uuid "{uid()}"))' for n in sym.pins)
        extra = " ".join(
            f'(property "{k}" "{esc(v)}" (at {fmt(x)} {fmt(y)} 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            for k, v in fields)
        bx0, by0, bx1, by1 = sym.bbox(unit)
        self.items.append(
            f'(symbol (lib_id "{sym.lib_id}") (at {fmt(x)} {fmt(y)} 0) (unit {unit})'
            f' (exclude_from_sim no) (in_bom {"yes" if in_bom else "no"}) (on_board yes)'
            f' (dnp {"yes" if dnp else "no"}) (uuid "{uid()}")'
            f' (property "Reference" "{ref}" (at {fmt(x + bx0)} {fmt(y - by1 - 3.81)} 0)'
            f' (effects (font (size 1.27 1.27)) (justify left)))'
            f' (property "Value" "{esc(value)}" (at {fmt(x + bx0)} {fmt(y - by0 + 3.81)} 0)'
            f' (effects (font (size 1.27 1.27)) (justify left)))'
            f' (property "Footprint" "{esc(footprint)}" (at {fmt(x)} {fmt(y)} 0)'
            f' (effects (font (size 1.27 1.27)) (hide yes)))'
            f' (property "Datasheet" "" (at {fmt(x)} {fmt(y)} 0) (effects (font (size 1.27 1.27)) (hide yes)))'
            f' {extra} {pin_uuids}'
            f' (instances (project "{PROJECT}" (path "{path}" (reference "{ref}") (unit {unit})))))')
        for num, net in nets.items():
            cands = [p for p in sym.pins[str(num)] if p[3] in (unit, 0)]
            if not cands:
                raise SystemExit(f"{ref}: pin {num} not in unit {unit}")
            for px, py, a, *_ in cands:
                sx, sy = x + px, y - py
                if net is None:
                    self.items.append(f'(no_connect (at {fmt(sx)} {fmt(sy)}) (uuid "{uid()}"))')
                    continue
                dx, dy = -math.cos(math.radians(a)), math.sin(math.radians(a))
                ex, ey = sx + 2.54 * dx, sy + 2.54 * dy
                self.items.append(
                    f'(wire (pts (xy {fmt(sx)} {fmt(sy)}) (xy {fmt(ex)} {fmt(ey)}))'
                    f' (stroke (width 0) (type default)) (uuid "{uid()}"))')
                if abs(dx) > 0.5:
                    ang, just = (180, "right") if dx < 0 else (0, "left")
                else:
                    ang, just = (90, "left") if dy < 0 else (270, "right")
                self.items.append(
                    f'(global_label "{esc(net)}" (shape passive) (at {fmt(ex)} {fmt(ey)} {ang})'
                    f' (fields_autoplaced yes) (effects (font (size 1.0 1.0)) (justify {just}))'
                    f' (uuid "{uid()}"))')

    def render(self, root_uuid=None):
        libs = "\n".join(s.body for s in self.syms.values())
        sheet_uuid = root_uuid or self.uuid
        return (f'(kicad_sch (version 20260306) (generator "eeschema") (generator_version "10.0")'
                f' (uuid "{sheet_uuid}") (paper "{self.paper}")'
                f' (title_block (title "{esc(self.title)}") (rev "A") (company "rixnobis"))\n'
                f'(lib_symbols\n{libs}\n)\n' + "\n".join(self.items) + "\n")
