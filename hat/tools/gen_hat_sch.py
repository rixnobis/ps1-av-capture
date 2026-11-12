#!/usr/bin/env python3
"""Generate the ps1-av-hat schematic: a one-channel daughterboard for the Maker go CH32H417QEU6.

Root sheet plus input, tvp7002, power, audio and mcu sheets. Derived from tools/gen_sch.py
(the FX3 design); the symbol/emission library tools/sch_lib.py is imported unchanged with its
PROJECT constant overridden. Run hat/build.sh to regenerate, run ERC and export the PDF.
"""
import csv
import re
import sys
from pathlib import Path

HAT = Path(__file__).resolve().parent.parent
REPO = HAT.parent
sys.path.insert(0, str(REPO / "tools"))
import sch_lib as L  # noqa: E402

L.PROJECT = "ps1-av-hat"

# Deterministic UUIDs: sch_lib.uid() is random; override it at runtime with uuid5 over a stable key
# (sheet name + ref + unit for symbols, sheet name + kind for the rest, plus a per-key counter).
import uuid as _uuid
_NS = _uuid.UUID("2f0c7a54-9b1e-4d36-8a77-5c1d0e9b4a10")
_ctx = ["lib"]
_cnt = {}


def _det_uid():
    k = _ctx[-1]
    _cnt[k] = _cnt.get(k, 0) + 1
    return str(_uuid.uuid5(_NS, f"{k}#{_cnt[k]}"))


def _with_ctx(fn, keyf):
    def wrapper(self, *a, **kw):
        _ctx.append(keyf(self, *a, **kw))
        try:
            return fn(self, *a, **kw)
        finally:
            _ctx.pop()
    return wrapper


L.uid = _det_uid
_orig_init = L.Sheet.__init__


def _init(self, name, *a, **kw):
    _ctx.append(f"sheet:{name}")
    try:
        _orig_init(self, name, *a, **kw)
    finally:
        _ctx.pop()


L.Sheet.__init__ = _init
L.Sheet.place = _with_ctx(L.Sheet.place, lambda self, sym, ref, value, x, y, nets, unit=1, **kw: f"{self.name}:{ref}:u{unit}")
L.Sheet.text = _with_ctx(L.Sheet.text, lambda self, *a, **kw: f"{self.name}:text")
ROOT = HAT                      # sheets are written next to ps1-av-hat.kicad_pro
PINS = REPO / "tools" / "pins"  # TVP7002 / PCM2903C pin tables are shared with the FX3 design
ROOT_UUID = "7b1d4e86-3a52-4c9f-8d07-1e5a9b3c6f22"

# ---------------------------------------------------------------- footprints, parts
FP = {
    "R0402": "Resistor_SMD:R_0402_1005Metric",
    "C0402": "Capacitor_SMD:C_0402_1005Metric",
    "C0603": "Capacitor_SMD:C_0603_1608Metric",
    "C0805": "Capacitor_SMD:C_0805_2012Metric",
    "FB0603": "Inductor_SMD:L_0603_1608Metric",
    "L4030": "Inductor_SMD:L_Sunlord_SWPA4030S",
    "SOT23-5": "Package_TO_SOT_SMD:SOT-23-5",
    "SOT23-6": "Package_TO_SOT_SMD:SOT-23-6",
    "BUCK": "ps1-av-capture:SOT-23-6_L2.9-W1.6-P0.95-LS2.8-BR",
    "TVP": "ps1-av-capture:HTQFP-100_L14.0-W14.0-P0.50-LS16.0-BL-EP6.2",
    "FX3": "ps1-av-capture:BGA-121_L10.0-W10.0-R11-C11-P0.80-TL",
    "PCM": "Package_SO:SSOP-28_5.3x10.2mm_P0.65mm",
    "USB3B": "ps1-av-capture:USB-B-TH_9P-L18.5-W12.0",
    "USBC": "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12",
    "XTAL3225": "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
    "OSC3225": "Oscillator:Oscillator_SMD_Abracon_ASE-4Pin_3.2x2.5mm",
    "SOIC8": "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm",
    "USON10": "Package_SON:USON-10_2.5x1.0mm_P0.5mm",
    "LED0603": "LED_SMD:LED_0603_1608Metric",
    "LED1615RGB": "ps1-av-capture:LED-ARRAY-SMD_4P-L1.6-W1.5-BR_XL-1615RGBC-RF",
    "JTAG": "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD",
    "PHD14": "Connector_JST:JST_PHD_B14B-PHDSS_2x07_P2.00mm_Vertical",
    "HDR3": "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
    "HDR2": "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
    "SW": "Button_Switch_SMD:SW_SPST_TL3342",
    "MH": "MountingHole:MountingHole_3.2mm_M3",
    "SOCKET": "ps1-av-hat:Socket_3x19_P2.54_MakerGo",
    "TP": "TestPoint:TestPoint_Pad_D1.0mm",
}

# LCSC numbers: tools/check_lcsc.py reads each one back from JLC and checks the
# description against the value before the BOM is trusted.
LCSC_PASSIVE = {
    ("R0402", "22"): "C25092", ("R0402", "0"): "C17168", ("R0402", "75"): "C25133", ("R0402", "200 1%"): "C25087",
    ("R0402", "1k"): "C11702", ("R0402", "100"): "C25076", ("R0402", "470"): "C25117", ("R0402", "1.5k"): "C25867", ("R0402", "2k"): "C4109",
    ("R0402", "2.2k"): "C25879", ("R0402", "5.1k"): "C25905", ("R0402", "6.04k 1%"): "C97819",
    ("R0402", "10k"): "C25744", ("R0402", "22.6k 1%"): "C26960", ("R0402", "33.2k 1%"): "C122548",
    ("R0402", "5.76k 1%"): "C5153969", ("R0402", "100k"): "C25741", ("R0402", "1M"): "C26083",
    ("C0402", "33p"): "C1562", ("C0402", "12p"): "C1547", ("C0402", "330p"): "C13533",
    ("C0402", "1n"): "C1523", ("C0402", "2.2n"): "C1531", ("C0402", "4.7n"): "C1538",
    ("C0402", "10n"): "C15195", ("C0402", "100n"): "C1525", ("C0402", "1u"): "C52923",
    ("C0603", "2.2u"): "C23630", ("C0603", "10u"): "C19702", ("C0805", "22u"): "C45783",
    ("FB0603", "600R"): "C108301", ("L4030", "2.2u"): "C36409",
}

# ---------------------------------------------------------------- symbols
STK = {}


def stk(lib, name):
    return STK.setdefault((lib, name), L.stock(lib, name))


def table(name):
    return [(r[0], r[1]) for r in csv.reader(open(PINS / f"{name}.csv"))][1:]


POWER_NAMES = re.compile(r"^(VDD|VSS|AVDD|AVSS|U3\w+VDDQ|U3VSSQ|CVDDQ|VIO\d|VBUS|VBATT|"
                         r"A33VDD|A33GND|AGND|DVDD|DGND|GND|IOVDD|IOGND|PLL_AVDD|PLL_AGND|NSUB|"
                         r"VCCC|VCCP\d|VCCX|AGNDC|AGNDP|AGNDX|DGNDU)$")


def et(name):
    return "power_in" if POWER_NAMES.match(name) else "passive"


def build_tvp():
    rows = table("tvp7002")
    pwr = [(p, n) for p, n in rows if POWER_NAMES.match(n)]
    data = [(p, n) for p, n in rows if re.match(r"^[RGB]_\d$", n) or n == "DATACLK"]
    rest = [(p, n) for p, n in rows if (p, n) not in pwr and (p, n) not in data]
    pwr.append(("101", "EP"))
    ins = [r for r in rest if re.match(r"(RIN|GIN|BIN|SOGIN|HSYNC|VSYNC|CLAMP|COAST|EXT_CLK|FILT|PLL_F)", r[1])]
    ctl = [r for r in rest if r not in ins]
    u1 = {"left": [(p, n, "passive") for p, n in ins], "right": [(p, n, "passive") for p, n in ctl]}
    data.sort(key=lambda r: ("RGBD".index(r[1][0]), -int(r[1][2]) if r[1][0] in "RGB" else 0))
    h = (len(data) + 1) // 2
    u2 = {"left": [(p, n, "passive") for p, n in data[:h]], "right": [(p, n, "passive") for p, n in data[h:]]}
    pos = [(p, n, "power_in") for p, n in pwr if "VDD" in n]
    neg = [(p, n, "power_in") for p, n in pwr if "VDD" not in n]
    u3 = {"left": pos, "right": neg}
    return L.custom("TVP7002", [u1, u2, u3], footprint=FP["TVP"])


def build_pcm():
    rows = table("pcm2903c")
    left = [(p, n, et(n)) for p, n in rows if n in ("D+", "D-", "VBUS", "SEL0", "SEL1", "HID0", "HID1",
                                                    "HID2", "DIN", "DOUT", "~{SSPND}", "XTI", "XTO")]
    right = [(p, n, et(n)) for p, n in rows if (p, n, et(n)) not in left]
    return L.custom("PCM2903C", [{"left": left, "right": right}], footprint=FP["PCM"])

TVP = build_tvp()
PCM = build_pcm()

# ---------------------------------------------------------------- sheets and placement
SHEETS = {k: L.Sheet(*v) for k, v in {
    "input": ("Input", "input.kicad_sch", "AV multi-out input, termination, ESD", "A3"),
    "tvp": ("TVP7002", "tvp7002.kicad_sch", "TVP7002 RGB digitiser", "A2"),
    "power": ("Power", "power.kicad_sch", "Power from the dev-board headers", "A2"),
    "audio": ("Audio", "audio.kicad_sch", "PCM2903C USB audio", "A2"),
    "mcu": ("MCU", "mcu.kicad_sch", "CH32H417 dev-board headers and bus control", "A2"),
}.items()}
for s in SHEETS.values():
    s.path = f"/{ROOT_UUID}/{s.uuid}"
    s.cx, s.cy, s.rowh = 30.0, 45.0, 0.0
    s.bx0, s.bw, s.btop, s.bbot = 20.0, 0.0, 30.0, 30.0   # current block
    s.shx, s.shy, s.shh = 20.0, 30.0, 0.0                 # current shelf of blocks

PAPER = {"A3": (420.0, 297.0), "A2": (594.0, 420.0), "A1": (841.0, 594.0)}
REFS = {"U": 1, "J": 5}  # U1 TVP7002, J1 pigtail, J3 audio, J4/J5 board headers are assigned by hand
ALL = []  # (ref, sym, unit, nets, value)
USED = set()


def nextref(p):
    REFS[p] = REFS.get(p, 0) + 1
    return f"{p}{REFS[p]}"


def heading(sheet, s, width=175.0):
    """Start a block: blocks sit side by side on a shelf, a new shelf when the page is full."""
    import textwrap
    sh = SHEETS[sheet]
    sh.bbot = max(sh.bbot, sh.cy + sh.rowh)
    sh.shh = max(sh.shh, sh.bbot - sh.shy)
    nx = sh.bx0 + sh.bw + (8.0 if sh.bw else 0.0)
    if nx + width > PAPER[sh.paper][0] - 20:
        sh.shy, sh.shh, nx = sh.shy + sh.shh + 6.0, 0.0, 20.0
    sh.bx0, sh.bw = nx, width
    lines = textwrap.wrap(s, int(width / 1.75))
    sh.text("\n".join(lines), nx, sh.shy + 4.0, 1.8)
    sh.cx, sh.cy, sh.rowh = nx, sh.shy + 4.0 + 3.2 * len(lines) + 2.0, 0.0
    sh.bbot = sh.cy


def put(sheet, sym, prefix, value, nets, fp="", unit=1, lcsc=None, dnp=False, ref=None, note=None):
    sh = SHEETS[sheet]
    x0, y0, x1, y1 = sym.bbox(unit)
    angs = {p[2] for v in sym.pins.values() for p in v if p[3] in (unit, 0)}
    margin = 24.0 if angs & {0.0, 180.0} else 4.0       # side labels need room
    vmargin = 16.0 if angs & {90.0, 270.0} else 5.0     # labels above / below
    w, h = (x1 - x0) + 2 * margin, (y1 - y0) + 2 * vmargin
    if sh.cx + w > sh.bx0 + max(sh.bw, w) + 0.01:
        sh.cx, sh.cy, sh.rowh = sh.bx0, sh.cy + sh.rowh, 0.0
    x, y = sh.cx + margin - x0, sh.cy + vmargin + y1
    x, y = round(x / 2.54) * 2.54, round(y / 2.54) * 2.54
    ref = ref or nextref(prefix)
    assert ref not in USED or unit > 1, f"duplicate reference {ref}"
    USED.add(ref)
    if lcsc is None and fp in ("R0402", "C0402", "C0603", "C0805", "FB0603", "L4030"):
        lcsc = LCSC_PASSIVE[(fp, value)]
    fields = (("LCSC", lcsc),) if lcsc else ()
    if note:
        fields += (("Note", note),)
    sh.place(sym, ref, value, x, y, {str(k): v for k, v in nets.items()}, unit=unit,
             footprint=FP.get(fp, fp), fields=fields, dnp=dnp, path=sh.path,
             in_bom=sym.lib_id != "Mechanical:MountingHole" and not sym.lib_id.startswith("power:"))
    sh.cx += w
    sh.rowh = max(sh.rowh, h)
    ALL.append((ref, sym, unit, nets, value))
    return ref


R = lambda: stk("Device", "R")  # noqa: E731
C = lambda: stk("Device", "C")  # noqa: E731


def res(sheet, value, a, b, fp="R0402", **kw):
    return put(sheet, R(), "R", value, {1: a, 2: b}, fp, **kw)


def cap(sheet, value, a, b="GND", fp=None, **kw):
    fp = fp or ("C0805" if value == "22u" else "C0603" if value in ("10u", "2.2u") else "C0402")
    return put(sheet, C(), "C", value, {1: a, 2: b}, fp, **kw)


def bead(sheet, a, b):
    return put(sheet, stk("Device", "FerriteBead_Small"), "FB", "600R", {1: a, 2: b}, "FB0603")

# ================================================================ input sheet
heading("input", "PS1 AV multi-out via a short pigtail to a multi-AV male plug (pinout in "
        "the FX3 project docs/design.md). Multi-out +5V (pin 3) and S-Video C (pin 6) are not carried.")
put("input", stk("Connector_Generic", "Conn_02x07_Odd_Even"), "J", "AV_PIGTAIL", {
    1: "VIN_R", 2: "GND", 3: "VIN_G", 4: "GND", 5: "VIN_B", 6: "GND", 7: "VIN_Y", 8: "GND",
    9: "VIN_CVBS", 10: "GND", 11: "AIN_L", 12: "GND", 13: "AIN_R", 14: "GND"}, "PHD14", ref="J1",
    lcsc="C471471", dnp=True, note="Hand-soldered, not in the JLC assembly. JST PHD 2x7 (B14B-PHDSS), keyed and latching; each signal pairs with the ground beside it")
heading("input", "75 ohm termination at the connector, then ESD, then AC coupling (TVP7002: 0.1 uF; "
        "SOG inputs: 1 nF). CVBS and Y also reach spare ADC inputs, so luma can be digitised.")
for n in ("R", "G", "B", "CVBS", "Y"):
    res("input", "75", f"VIN_{n}", "GND")
put("input", stk("Power_Protection", "TPD4E05U06DQA"), "D", "TPD4E05U06DQAR",
    {1: "VIN_R", 2: "VIN_G", 4: "VIN_B", 5: "VIN_CVBS", 3: "GND", 8: "GND",
     6: None, 7: None, 9: None, 10: None}, "USON10", lcsc="C138714")
put("input", stk("Power_Protection", "TPD4E05U06DQA"), "D", "TPD4E05U06DQAR",
    {1: "VIN_Y", 2: "AIN_L_C", 4: "AIN_R_C", 5: None, 3: "GND", 8: "GND",
     6: None, 7: None, 9: None, 10: None}, "USON10", lcsc="C138714",
    note="Audio channels sit after the coupling cap, where the signal rides on VCOM")
cap("input", "100n", "VIN_R", "TVP_RIN1")
cap("input", "100n", "VIN_G", "TVP_GIN1")
cap("input", "100n", "VIN_B", "TVP_BIN1")
cap("input", "100n", "VIN_CVBS", "TVP_GIN2")
cap("input", "100n", "VIN_Y", "TVP_GIN3")
cap("input", "1n", "VIN_G", "TVP_SOGIN1")
cap("input", "1n", "VIN_CVBS", "TVP_SOGIN2")
cap("input", "1n", "VIN_Y", "TVP_SOGIN3")
heading("input", "Unused TVP7002 analog inputs: 10 nF to GND (SLES206C Table 1).")
for n in ("TVP_RIN2", "TVP_RIN3", "TVP_GIN4", "TVP_BIN2", "TVP_BIN3"):
    cap("input", "10n", n)

# ================================================================ TVP7002 sheet
tvp_nets = {}
for p, n in table("tvp7002"):
    m = re.match(r"^([RGB])_(\d)$", n)
    if m:
        bit = int(m.group(2))
        # Only G[9:2] (Y) and B[9:2] (C) are used; R[9:2] and all bits below 2 are left open.
        tvp_nets[p] = f"TVP_{m.group(1)}{bit}" if (bit >= 2 and m.group(1) in "GB") else None
        continue
    tvp_nets[p] = {
        "RIN_1": "TVP_RIN1", "RIN_2": "TVP_RIN2", "RIN_3": "TVP_RIN3", "GIN_1": "TVP_GIN1",
        "GIN_2": "TVP_GIN2", "GIN_3": "TVP_GIN3", "GIN_4": "TVP_GIN4", "BIN_1": "TVP_BIN1",
        "BIN_2": "TVP_BIN2", "BIN_3": "TVP_BIN3", "SOGIN_1": "TVP_SOGIN1", "SOGIN_2": "TVP_SOGIN2",
        "SOGIN_3": "TVP_SOGIN3", "DATACLK": "TVP_DATACLK", "EXT_CLK": "TVP_EXTCLK",
        "PWDN": "TVP_PWDN", "RESETB": "TVP_RESETn", "TMS": "GND", "I2CA": "GND",
        "FILT1": "TVP_FILT1", "FILT2": "TVP_FILT2", "PLL_F": "TVP_PLLF", "SCL": "I2C_SCL",
        "SDA": "I2C_SDA", "CLAMP": "GND", "COAST": "GND", "VSYNC_A": "GND", "VSYNC_B": "GND",
        "HSYNC_A": "GND", "HSYNC_B": "GND", "FIDOUT": "TVP_FIDOUT", "VSOUT": "TVP_VSOUT",
        "HSOUT": "TVP_HSOUT", "SOGOUT": "TVP_SOGOUT",
        "NSUB": "GND", "A33VDD": "+3V3A", "A33GND": "GND", "AGND": "GND", "AVDD": "TVP_AVDD",
        "PLL_AVDD": "TVP_PLLVDD", "PLL_AGND": "GND", "DGND": "GND", "GND": "GND",
        "DVDD": "+1V9D", "IOGND": "GND", "IOVDD": "+3V3",
    }[n]
tvp_nets["101"] = "GND"
heading("tvp", "TVP7002 on 3.3 V IOVDD. The H417 VIO18 rail MUST be 3.3 V (Maker go R39 fitted 0R, or "
        "firmware PWR_CTLR VIO18=3.3V) before RESETB is released: TVP IOVDD is 3.3 V and the VIO18 pins "
        "are not 5V/over-voltage tolerant.")
heading("tvp", "Sync comes from SOGIN (composite or luma; the PS1 has no sync pin), so the "
        "digital HSYNC/VSYNC inputs are grounded as unused. I2C address B8h (I2CA low).")
for u in (1, 2, 3):
    put("tvp", TVP, "U", "TVP7002PZP", {k: v for k, v in tvp_nets.items()
                                         if any(pp[3] == u for pp in TVP.pins[str(k)])},
        "TVP", unit=u, lcsc="C3824085", ref="U1")
heading("tvp", "H-PLL loop filter (SLES206C Figure 4): 1.5k + 100n FILT1-PLL_F, 4.7n FILT2-PLL_F.")
res("tvp", "1.5k", "TVP_FILT1", "TVP_FILT1R")
cap("tvp", "100n", "TVP_FILT1R", "TVP_PLLF")
cap("tvp", "4.7n", "TVP_FILT2", "TVP_PLLF")
heading("tvp", "Control. RESETB is held low by 2.2k until the CH32H417 releases it (TVP_RESETn is an MCU "
        "GPIO, PA4), so the TVP7002 stays in reset until firmware is up (SLES206C: no I2C activity before "
        "reset completes). PWDN 2.2k to GND (Figure 8). I2C bus: 2.2k pull-ups to +3V3.")
res("tvp", "2.2k", "TVP_RESETn", "GND")
res("tvp", "2.2k", "TVP_PWDN", "GND")
res("tvp", "2.2k", "I2C_SCL", "+3V3")
res("tvp", "2.2k", "I2C_SDA", "+3V3")
heading("tvp", "27 MHz reference on EXT_CLK for mode detection (SLES206C recommends 27 MHz; "
        "the internal reference is about 6.5 MHz and imprecise).")
put("tvp", stk("Oscillator", "ASE-xxxMHz"), "X", "27MHz", {1: "OSC_VDD", 2: "GND", 3: "TVP_EXTCLK", 4: "OSC_VDD"},
    "OSC3225", lcsc="C725995")
bead("tvp", "+3V3", "OSC_VDD")
cap("tvp", "100n", "OSC_VDD")
cap("tvp", "10n", "OSC_VDD")
heading("tvp", "Output bus: 22R series at the source, only the 16 bits used plus clock and syncs. "
        "G[9:2] -> VD_Y0..7 (luma), B[9:2] -> VD_C0..7 (chroma); R[9:2] are unconnected.")
for ch, name in (("G", "Y"), ("B", "C")):
    for bit in range(2, 10):
        res("tvp", "22", f"TVP_{ch}{bit}", f"VD_{name}{bit - 2}")
for src, dst in (("TVP_DATACLK", "VD_PCLK"), ("TVP_HSOUT", "VD_HS"), ("TVP_VSOUT", "VD_VS"),
                 ("TVP_FIDOUT", "VD_FID"), ("TVP_SOGOUT", "VD_SOG")):
    res("tvp", "22", src, dst)
heading("tvp", "Supply decoupling. SLES206C gives no values; this follows the OSSC: ferrites on "
        "AVDD and PLL_AVDD, 100n per pin, bulk per rail.")
bead("tvp", "+1V9A", "TVP_AVDD")
bead("tvp", "+1V9A", "TVP_PLLVDD")
for _ in range(4):
    cap("tvp", "100n", "TVP_AVDD")
cap("tvp", "10u", "TVP_AVDD")
for _ in range(2):
    cap("tvp", "100n", "TVP_PLLVDD")
cap("tvp", "1u", "TVP_PLLVDD")
for _ in range(2):
    cap("tvp", "100n", "+1V9D")
cap("tvp", "10u", "+1V9D")
for _ in range(4):
    cap("tvp", "100n", "+3V3A")
cap("tvp", "10u", "+3V3A")
for _ in range(4):
    cap("tvp", "100n", "+3V3")
cap("tvp", "10u", "+3V3")

# ================================================================ power sheet
heading("power", "Power comes from the dev-board headers: +5V (J4 pins 7-9) and +3V3 (J4 pins 10-12); "
        "no USB 3 connector. +3V3 feeds TVP7002 IOVDD, the oscillator, pull-ups and the bus-control gate. "
        "+5V feeds one TPS563201 buck to 2.5 V (SLVSD90B Table 7-2: R2 = 10k, L = 2.2 uH, 2 x 22 uF out), "
        "which is the pre-regulator for the 1.9 V LDOs (OSSC pattern). EN has an internal pull-down, so it is "
        "tied up through 100k.")
cap("power", "10u", "+5V")
for out, r1 in (("+2V5", "22.6k 1%"),):
    tag = out.strip("+")
    put("power", stk("Regulator_Switching", "TPS563201"), "U", "TPS563201DDCR", {
        1: "GND", 2: f"SW_{tag}", 3: "+5V", 4: f"FB_{tag}", 5: f"EN_{tag}", 6: f"BST_{tag}"},
        "BUCK", lcsc="C116592")
    cap("power", "10u", "+5V")
    cap("power", "100n", "+5V")
    res("power", "100k", "+5V", f"EN_{tag}")
    cap("power", "100n", f"BST_{tag}", f"SW_{tag}")
    put("power", stk("Device", "L"), "L", "2.2u", {1: f"SW_{tag}", 2: out}, "L4030")
    cap("power", "22u", out)
    cap("power", "22u", out)
    res("power", r1, out, f"FB_{tag}")
    res("power", "10k", f"FB_{tag}", "GND")
heading("power", "LDOs: 2.5 V -> 1.9 V for TVP7002 AVDD/PLL and DVDD (two TLV75519, 500 mA); "
        "+5V -> 3.3 V for TVP7002 A33VDD alone (TLV75733, 1 A), kept off the board's 3V3 rail.")
for out in ("+1V9A", "+1V9D"):
    put("power", stk("Regulator_Linear", "TLV75519PDBV"), "U", "TLV75519PDBVR",
        {1: "+2V5", 2: "GND", 3: "+2V5", 4: None, 5: out}, "SOT23-5", lcsc="C2865381")
    cap("power", "1u", "+2V5")
    cap("power", "1u", out)
put("power", stk("Regulator_Linear", "TLV75733PDBV"), "U", "TLV75733PDBVR",
    {1: "+5V", 2: "GND", 3: "+5V", 4: None, 5: "+3V3A"}, "SOT23-5", lcsc="C485517")
cap("power", "1u", "+5V")
cap("power", "1u", "+3V3A")
heading("power", "Power LED on +3V3, mounting holes.")
put("power", stk("Device", "LED"), "D", "GREEN", {1: "PWR_LED_K", 2: "+3V3"}, "LED0603", lcsc="C2986059")
res("power", "1k", "PWR_LED_K", "GND")
for _ in range(4):
    put("power", stk("Mechanical", "MountingHole"), "H", "M3", {}, "MH")

# ================================================================ audio sheet
heading("audio", "PCM2903C on its own USB 2.0 Type-C port. TI documents the PCM2903C self-powered only "
        "(SBFS038A Figure 41), so a 3.3 V LDO makes its supply from this port's VBUS.")
put("audio", stk("Connector", "USB_C_Receptacle_USB2.0_16P"), "J", "USB_C_AUDIO", {
    "A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND", "A4": "VBUS_AUD", "A9": "VBUS_AUD",
    "B4": "VBUS_AUD", "B9": "VBUS_AUD", "A5": "AUD_CC1", "B5": "AUD_CC2", "A6": "AUD_DP_C",
    "B6": "AUD_DP_C", "A7": "AUD_DM_C", "B7": "AUD_DM_C", "A8": None, "B8": None, "SH": "GND"},
    "USBC", lcsc="C165948", ref="J3")
res("audio", "5.1k", "AUD_CC1", "GND")
res("audio", "5.1k", "AUD_CC2", "GND")
put("audio", stk("Power_Protection", "USBLC6-2SC6"), "D", "USBLC6-2SC6",
    {1: "AUD_DP_C", 6: "AUD_DP_C", 3: "AUD_DM_C", 4: "AUD_DM_C", 2: "GND", 5: "VBUS_AUD"}, "SOT23-6", lcsc="C7519")
heading("audio", "D+/D-: 22R series; 1.5k D+ pull-up on the connector side (Figure 41). TI gates the "
        "pull-up with an AND of VDD and VBUS; here VDD only exists when VBUS does, so it goes to +3V3_AUD.")
res("audio", "22", "AUD_DP_C", "AUD_DP")
res("audio", "22", "AUD_DM_C", "AUD_DM")
res("audio", "1.5k", "AUD_DP_C", "+3V3_AUD")
put("audio", stk("Regulator_Linear", "TLV75533PDBV"), "U", "TLV75533PDBVR",
    {1: "VBUS_AUD", 2: "GND", 3: "VBUS_AUD", 4: None, 5: "+3V3_AUD"}, "SOT23-5", lcsc="C404027")
cap("audio", "1u", "VBUS_AUD")
cap("audio", "10u", "+3V3_AUD")
heading("audio", "PCM2903C wiring per Figure 41: VBUS pin and SEL0 to VDD; SEL1 to the port VBUS with "
        "1M to GND; 10 uF on every supply pin and VCOM; 12 MHz crystal with 1M across it. "
        "DAC, S/PDIF and HID pins unused.")
pcm_nets = {"1": "AUD_DP", "2": "AUD_DM", "3": "+3V3_AUD", "4": "GND", "5": None, "6": None, "7": None,
            "8": "+3V3_AUD", "9": "AUD_SEL1", "10": "+3V3_AUD", "11": "GND", "12": "AUD_VINL",
            "13": "AUD_VINR", "14": "AUD_VCOM", "15": None, "16": None, "17": "+3V3_AUD", "18": "GND",
            "19": "+3V3_AUD", "20": "AUD_XO", "21": "AUD_XI", "22": "GND", "23": "+3V3_AUD",
            "24": None, "25": None, "26": "GND", "27": "+3V3_AUD", "28": None}
put("audio", PCM, "U", "PCM2903CDBR", pcm_nets, "PCM", lcsc="C2861593")
res("audio", "1M", "VBUS_AUD", "AUD_SEL1")
res("audio", "1M", "AUD_SEL1", "GND")
for _ in range(5):
    cap("audio", "10u", "+3V3_AUD")
for _ in range(5):
    cap("audio", "100n", "+3V3_AUD")
cap("audio", "10u", "AUD_VCOM")
put("audio", stk("Device", "Crystal_GND24"), "Y", "12MHz", {1: "AUD_XI", 3: "AUD_XO", 2: "GND", 4: "GND"},
    "XTAL3225", lcsc="C9002")
res("audio", "1M", "AUD_XI", "AUD_XO")
cap("audio", "33p", "AUD_XI")
cap("audio", "33p", "AUD_XO")
heading("audio", "Line input: 2.2 uF coupling, then 10k into the 30k ADC input (x0.75, so 2.6 Vpp "
        "maps to the 1.98 Vpp full scale) with 330 pF to GND (anti-alias pole ~48 kHz).")
for ch in ("L", "R"):
    cap("audio", "2.2u", f"AIN_{ch}", f"AIN_{ch}_C")
    res("audio", "10k", f"AIN_{ch}_C", f"AUD_VIN{ch}")
    cap("audio", "330p", f"AUD_VIN{ch}")


# ================================================================ mcu sheet
# Pin -> CH32H417 port, as wired by the Maker go CH32H417QEU6 headers. Pins not listed are open.
# Column-wise numbering: column k holds pins 3k-2, 3k-1, 3k.
#   J4 (U12): 1-3 DCDC_VIN (unused)  4-6,13-16 GND  7-9 +5V  10-12 +3V3
#     17 MCU_RD_N  PB11     18 MCU_RDNE PB10     20 MCU_WRNF PE14     24 MCU_AF_N PE10
#     28 I2C_SDA   PF13     29 I2C_SCL  PF12     30 VD_PCLK  PF11
#     43 VD_HS     PA0      42 VD_VS    PA1      41 VD_FID   PA2      40 VD_SOG   PA3
#     39 TVP_RESETn PA4     38 CAP_EN_N PA5
#   J5 (U13): 1 MCU_OE_N PB12   2 UHSIF_WR_N PB13   3 UHSIF_CS_N PB14
#     7 VD_Y0 PD10   8 VD_Y1 PD11   9 VD_Y2 PD12   10 VD_Y3 PD13   11 VD_Y4 PD14   12 VD_Y5 PD15
#     13 VD_Y6 PF0   14 VD_Y7 PF1   15 VD_C0 PF2   16 VD_C1 PC6    17 VD_C2 PC7    18 VD_C3 PC8
#     19 VD_C4 PC9   25 VD_C5 PA13  26 VD_C6 PA14  27 VD_C7 PA15
J4_NETS = {1: None, 2: None, 3: None, 4: "GND", 5: "GND", 6: "GND", 7: "+5V", 8: "+5V", 9: "+5V",
           10: "+3V3", 11: "+3V3", 12: "+3V3", 13: "GND", 14: "GND", 15: "GND", 16: "GND",
           17: "MCU_RD_N", 18: "MCU_RDNE", 20: "MCU_WRNF", 24: "MCU_AF_N",
           28: "I2C_SDA", 29: "I2C_SCL", 30: "VD_PCLK", 43: "VD_HS", 42: "VD_VS", 41: "VD_FID",
           40: "VD_SOG", 39: "TVP_RESETn", 38: "CAP_EN_N"}
J5_NETS = {1: "MCU_OE_N", 2: "UHSIF_WR_N", 3: "UHSIF_CS_N"}
J5_NETS.update({7 + i: f"VD_Y{i}" for i in range(8)})
J5_NETS.update({15 + i: f"VD_C{i}" for i in range(5)})
J5_NETS.update({25: "VD_C5", 26: "VD_C6", 27: "VD_C7"})
SOCKET_NOTE = "Female 2.54 mm 3x19 socket to be sourced (no LCSC part chosen yet); mates with the Maker go header"
for jref, jval, jnets in (("J4", "MAKERGO_U12", J4_NETS), ("J5", "MAKERGO_U13", J5_NETS)):
    heading("mcu", f"{jref}: Maker go CH32H417QEU6 header {jval[-3:]} (3x19, 2.54 mm, pins 1..57 column-wise). "
            "Pin-to-port table in gen_hat_sch.py.", width=70.0)
    put("mcu", stk("Connector_Generic", "Conn_01x57"), "J", jval,
        {i: jnets.get(i) for i in range(1, 58)}, "SOCKET", ref=jref, note=SOCKET_NOTE)
heading("mcu", "+3V3 bulk at the header: 10u + 100n.")
cap("mcu", "10u", "+3V3")
cap("mcu", "100n", "+3V3")
heading("mcu", "Bus control. UHSIF CS# and WR# (active low) are driven low together while capturing: "
        "Y = CAP_EN_N AND CAP_QUAL. CAP_EN_N idles high through 10k; CAP_QUAL is tied high by a 0R, with a "
        "DNP 0R to VD_HS as an option to gate on HSOUT later. The 0R CS#-WR# link lets them be split.")
put("mcu", stk("74xGxx", "74LVC1G08"), "U", "74LVC1G08GV", {
    1: "CAP_EN_N", 2: "CAP_QUAL", 3: "GND", 4: "UHSIF_CS_N", 5: "+3V3"}, "SOT23-5", lcsc="C19829591")
cap("mcu", "100n", "+3V3")
res("mcu", "10k", "CAP_EN_N", "+3V3")
res("mcu", "0", "CAP_QUAL", "+3V3")
res("mcu", "0", "VD_HS", "CAP_QUAL", dnp=True)
res("mcu", "0", "UHSIF_CS_N", "UHSIF_WR_N")
heading("mcu", "MCU_OE_N and MCU_RD_N: 1k pull-ups to +3V3 (the board loads OE# with an LED to VDDIO). "
        "MCU_RDNE, MCU_WRNF, MCU_AF_N: test points only.")
res("mcu", "1k", "MCU_OE_N", "+3V3")
res("mcu", "1k", "MCU_RD_N", "+3V3")
for n in ("MCU_RDNE", "MCU_WRNF", "MCU_AF_N"):
    put("mcu", stk("Connector", "TestPoint"), "TP", n, {1: n}, "TP")

# ================================================================ power flags, write
def overflow():
    out = []
    for s in SHEETS.values():
        pw, ph = PAPER[s.paper]
        body = "\n".join(s.items)
        xs = [float(m[1]) for m in re.finditer(r"\(at ([-\d.]+) [-\d.]+", body)]
        ys = [float(m[1]) for m in re.finditer(r"\(at [-\d.]+ ([-\d.]+)", body)]
        if xs and (max(xs) > pw - 12 or max(ys) > ph - 45):  # 45 mm keeps clear of the title block
            out.append(f"{s.name}: content to x={max(xs):.0f}, y={max(ys):.0f} on {s.paper}")
    return out


nets_pins = {}
for ref, sym, unit, nets, value in ALL:
    for k, v in nets.items():
        if v is None:
            continue
        for p in sym.pins[str(k)]:
            if p[3] in (unit, 0):
                nets_pins.setdefault(v, []).append(p[4])
flagged = [n for n, types in nets_pins.items() if "power_in" in types and "power_out" not in types]
heading("power", "Power flags (ERC): " + ", ".join(sorted(flagged)))
for n in sorted(flagged):
    put("power", stk("power", "PWR_FLAG"), "#FLG", "PWR_FLAG", {1: n})


root = L.Sheet("root", "ps1-av-hat.kicad_sch", "ps1-av-hat: one-channel PS1 RGB + audio hat for CH32H417")
root.uuid = ROOT_UUID
root.text("ps1-av-hat, one channel. TVP7002 digitises the PS1 analog RGB (16-bit Y/C bus) into the "
          "CH32H417 dev board via two 3x19 headers,\na PCM2903C presents the line audio as a USB "
          "Audio Class device on its own port.", 20, 30, 2.0)
sheet_nodes, pages = [], []
for i, s in enumerate(SHEETS.values()):
    x, y = 25 + (i % 3) * 75, 60 + (i // 3) * 45
    sheet_nodes.append(
        f'(sheet (at {x} {y}) (size 60 30) (exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no)'
        f' (fields_autoplaced yes) (stroke (width 0.1524) (type solid)) (fill (color 0 0 0 0.0))'
        f' (uuid "{s.uuid}")'
        f' (property "Sheetname" "{s.name}" (at {x} {y - 0.7} 0) (effects (font (size 1.27 1.27)) (justify left bottom)))'
        f' (property "Sheetfile" "{s.fname}" (at {x} {y + 30.6} 0) (effects (font (size 1.27 1.27)) (justify left top)))'
        f' (instances (project "{L.PROJECT}" (path "/{ROOT_UUID}" (page "{i + 2}")))))')
    pages.append(f'(path "/{s.uuid}" (page "{i + 2}"))')
    (ROOT / s.fname).write_text(s.render(L.uid()) + ")\n")
(ROOT / root.fname).write_text(
    root.render(ROOT_UUID) + "\n".join(sheet_nodes) +
    f'\n(sheet_instances (path "/" (page "1")) {" ".join(pages)})\n(embedded_fonts no)\n)\n')
ov = overflow()
if ov:
    raise SystemExit("schematic content runs off the page: " + "; ".join(ov))
print("parts:", len({a[0] for a in ALL}), "nets:", len(nets_pins), "flags:", len(flagged))
