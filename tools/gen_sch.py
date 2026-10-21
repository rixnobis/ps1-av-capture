#!/usr/bin/env python3
"""Generate the ps1-av-capture schematic: a root sheet plus five sub-sheets.

Nets are carried by global labels on every pin stub, so a net name is the same on every
sheet. Run tools/build.sh to regenerate, run ERC and export the PDF.
"""
import csv
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sch_lib as L  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
PINS = ROOT / "tools" / "pins"
ROOT_UUID = "2f6a9c51-7d3e-4b08-9e1a-6c4d2b8f0a11"

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
    "JTAG": "Connector_PinHeader_1.27mm:PinHeader_2x05_P1.27mm_Vertical_SMD",
    "HDR13": "Connector_PinHeader_2.54mm:PinHeader_1x13_P2.54mm_Vertical",
    "HDR3": "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical",
    "HDR2": "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
    "SW": "Button_Switch_SMD:SW_SPST_TL3342",
    "MH": "MountingHole:MountingHole_3.2mm_M3",
}

# LCSC numbers: tools/check_lcsc.py reads each one back from JLC and checks the
# description against the value before the BOM is trusted.
LCSC_PASSIVE = {
    ("R0402", "22"): "C25092", ("R0402", "75"): "C25133", ("R0402", "200 1%"): "C25087",
    ("R0402", "1k"): "C11702", ("R0402", "1.5k"): "C25867", ("R0402", "2k"): "C4109",
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


def build_fx3():
    rows = table("fx3")
    gpio = lambda b, n: (b, n, et(n))  # noqa: E731
    by = {n: [] for _, n in rows}
    for b, n in rows:
        by.setdefault(n, []).append(b)
    num = lambda n: int(re.search(r"\[(\d+)\]", n).group(1)) if "[" in n else -1  # noqa: E731
    gpif = sorted([(b, n) for b, n in rows if n.startswith("GPIO[") and num(n) <= 45], key=lambda r: num(r[1]))
    gpif.append([(b, n) for b, n in rows if n == "INT#"][0])
    misc_names = [n for _, n in rows if not n.startswith("GPIO[") and not POWER_NAMES.match(n)
                  and n != "INT#" or (n.startswith("GPIO[") and num(n) > 45)]
    misc = [(b, n) for b, n in rows if n in misc_names]
    pwr = [(b, n) for b, n in rows if POWER_NAMES.match(n)]
    assert len(gpif) + len(misc) + len(pwr) == 121, (len(gpif), len(misc), len(pwr))
    half = (len(gpif) + 1) // 2
    u1 = {"left": [gpio(b, n) for b, n in gpif[:half]], "right": [gpio(b, n) for b, n in gpif[half:]]}
    misc.sort(key=lambda r: (r[1].startswith("GPIO") or r[1].startswith("I2C"), r[1]))
    h2 = (len(misc) + 1) // 2
    u2 = {"left": [gpio(b, n) for b, n in misc[:h2]], "right": [gpio(b, n) for b, n in misc[h2:]]}
    pos = [(b, n) for b, n in pwr if not re.search("VSS|GND", n)]
    neg = [(b, n) for b, n in pwr if re.search("VSS|GND", n)]
    pos.sort(key=lambda r: r[1])
    u3 = {"left": [gpio(b, n) for b, n in pos], "right": [gpio(b, n) for b, n in neg]}
    return L.custom("CYUSB3014-BZXI", [u1, u2, u3], footprint=FP["FX3"])


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


FX3 = build_fx3()
TVP = build_tvp()
PCM = build_pcm()

# ---------------------------------------------------------------- sheets and placement
SHEETS = {k: L.Sheet(*v) for k, v in {
    "input": ("Input", "input.kicad_sch", "AV multi-out input, termination, ESD", "A3"),
    "tvp": ("TVP7002", "tvp7002.kicad_sch", "TVP7002 RGB digitiser", "A2"),
    "fx3": ("FX3", "fx3.kicad_sch", "FX3 USB 3 bridge", "A2"),
    "power": ("Power", "power.kicad_sch", "USB 3 connector and power", "A2"),
    "audio": ("Audio", "audio.kicad_sch", "PCM2903C USB audio", "A2"),
}.items()}
for s in SHEETS.values():
    s.path = f"/{ROOT_UUID}/{s.uuid}"
    s.cx, s.cy, s.rowh = 30.0, 45.0, 0.0
    s.bx0, s.bw, s.btop, s.bbot = 20.0, 0.0, 30.0, 30.0   # current block
    s.shx, s.shy, s.shh = 20.0, 30.0, 0.0                 # current shelf of blocks

PAPER = {"A3": (420.0, 297.0), "A2": (594.0, 420.0), "A1": (841.0, 594.0)}
REFS = {"U": 2, "J": 3}  # U1 TVP7002, U2 FX3, J1-J3 connectors are assigned by hand
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
heading("input", "PS1 AV multi-out (psx-spx pinout). Pin 3 (+5V) is not taken: the board is "
        "powered from USB. Pin 6 (S-Video C) is not used.")
put("input", stk("Connector_Generic", "Conn_01x13"), "J", "AV_MULTI_OUT", {
    1: "VIN_G", 2: "VIN_R", 3: None, 4: "VIN_B", 5: "GND", 6: None, 7: "VIN_CVBS", 8: "VIN_Y",
    9: "AIN_L", 10: "GND", 11: "AIN_R", 12: "GND", 13: "GND"}, "HDR13", ref="J1",
    note="Pins 1-12 = multi-out pins 1-12, pin 13 = shield")
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
        tvp_nets[p] = f"TVP_{m.group(1)}{bit}" if bit >= 2 else None
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
heading("tvp", "TVP7002. Sync comes from SOGIN (composite or luma; the PS1 has no sync pin), so the "
        "digital HSYNC/VSYNC inputs are grounded as unused. I2C address B8h (I2CA low).")
for u in (1, 2, 3):
    put("tvp", TVP, "U", "TVP7002PZP", {k: v for k, v in tvp_nets.items()
                                         if any(pp[3] == u for pp in TVP.pins[str(k)])},
        "TVP", unit=u, lcsc="C3824085", ref="U1")
heading("tvp", "H-PLL loop filter (SLES206C Figure 4): 1.5k + 100n FILT1-PLL_F, 4.7n FILT2-PLL_F.")
res("tvp", "1.5k", "TVP_FILT1", "TVP_FILT1R")
cap("tvp", "100n", "TVP_FILT1R", "TVP_PLLF")
cap("tvp", "4.7n", "TVP_FILT2", "TVP_PLLF")
heading("tvp", "Control. RESETB is held low by 2.2k until the FX3 firmware releases it, so the TVP7002 "
        "stays in reset while the FX3 boots from the shared I2C bus (SLES206C: no I2C activity before "
        "reset completes). PWDN 2.2k to GND (Figure 8).")
res("tvp", "2.2k", "TVP_RESETn", "GND")
res("tvp", "2.2k", "TVP_PWDN", "GND")
heading("tvp", "27 MHz reference on EXT_CLK for mode detection (SLES206C recommends 27 MHz; "
        "the internal reference is about 6.5 MHz and imprecise).")
put("tvp", stk("Oscillator", "ASE-xxxMHz"), "X", "27MHz", {1: "OSC_VDD", 2: "GND", 3: "TVP_EXTCLK", 4: "OSC_VDD"},
    "OSC3225", lcsc="C725995")
bead("tvp", "+3V3", "OSC_VDD")
cap("tvp", "100n", "OSC_VDD")
cap("tvp", "10n", "OSC_VDD")
heading("tvp", "Output bus: 22R series at the source (FX3 AN70707 checklist item 9). Top 8 bits of "
        "each channel. 4:4:4 uses DQ[23:0]; 20-bit 4:2:2 uses DQ[15:0] (Y on G, CbCr on B).")
fx3_dq = {}
for i, (ch, bit) in enumerate([(c, b) for c in "BGR" for b in range(2, 10)]):
    fx3_dq[i] = f"{ch}{bit}"
    res("tvp", "22", f"TVP_{ch}{bit}", f"VD_{ch}{bit}")
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

# ================================================================ FX3 sheet
fx3_nets = {}
for b, n in table("fx3"):
    g = re.match(r"^GPIO\[(\d+)\]$", n)
    if g:
        k = int(g.group(1))
        if k <= 15:
            fx3_nets[b] = "VD_" + fx3_dq[k]
        elif 33 <= k <= 40:
            fx3_nets[b] = "VD_" + fx3_dq[k - 17]
        else:
            fx3_nets[b] = {16: "VD_PCLK", 17: "VD_HS", 18: "VD_VS", 19: "VD_FID", 20: "VD_SOG",
                           30: "FX3_PMODE0", 31: "FX3_PMODE1", 32: "FX3_PMODE2", 45: "TVP_RESETn",
                           55: "FX3_UART_TX", 56: "FX3_UART_RX", 57: "FX3_LED"}.get(k)
        continue
    fx3_nets[b] = {
        "I2C_GPIO[58]": "I2C_SCL", "I2C_GPIO[59]": "I2C_SDA", "INT#": None, "NC": None,
        "CLKIN": None, "CLKIN_32": None, "OTG_ID": None, "TRST#": None,
        "FSLC[0]": "FX3_FSLC", "FSLC[1]": "FX3_FSLC", "FSLC[2]": "FX3_FSLC",
        "RESET#": "FX3_RESETn", "XTALIN": "FX3_XI", "XTALOUT": "FX3_XO",
        "TDI": "FX3_TDI", "TDO": "FX3_TDO", "TMS": "FX3_TMS", "TCK": "FX3_TCK",
        "DP": "USB3_DP", "DM": "USB3_DM", "SSRXM": "USB3_RXN", "SSRXP": "USB3_RXP",
        "SSTXM": "USB3_TXN_C", "SSTXP": "USB3_TXP_C", "R_USB2": "FX3_RUSB2", "R_USB3": "FX3_RUSB3",
        "VBUS": "VBUS", "VBATT": "VBUS", "VDD": "+1V2", "AVDD": "FX3_AVDD",
        "U3TXVDDQ": "FX3_U3TX", "U3RXVDDQ": "FX3_U3RX", "CVDDQ": "FX3_CVDDQ",
        "VIO1": "+3V3", "VIO2": "+3V3", "VIO3": "+3V3", "VIO4": "+3V3", "VIO5": "+3V3",
        "VSS": "GND", "AVSS": "GND", "U3VSSQ": "GND",
    }[n]
heading("fx3", "CYUSB3014. GPIF II takes the TVP7002 bus: DQ[7:0]=B[9:2], DQ[15:8]=G[9:2] (VIO1), "
        "DQ[23:16]=R[9:2] (VIO2), PCLK=DATACLK, CTL[0..3]=HS/VS/FID/SOG. All VIO at 3.3 V.")
for u in (1, 2, 3):
    put("fx3", FX3, "U", "CYUSB3014-BZXI", {k: v for k, v in fx3_nets.items()
                                             if any(pp[3] == u for pp in FX3.pins[k])},
        "FX3", unit=u, lcsc="C57294", ref="U2")
heading("fx3", "19.2 MHz crystal, FSLC[2:0]=000 through one 10k (AN70707 Figure 9). No series "
        "resistor on the crystal (AN70707). Load caps sized for a 12 pF crystal with ~5 pF stray.")
put("fx3", stk("Device", "Crystal_GND24"), "Y", "19.2MHz", {1: "FX3_XI", 3: "FX3_XO", 2: "GND", 4: "GND"},
    "XTAL3225", lcsc="C411225")
cap("fx3", "12p", "FX3_XI")
cap("fx3", "12p", "FX3_XO")
res("fx3", "10k", "FX3_FSLC", "GND")
heading("fx3", "Boot: PMODE F1F = I2C EEPROM, USB fallback (AN70707 Table 9). 10k up/down footprints "
        "on every PMODE pin, only PMODE1 up fitted. Jumper JP1 pulls PMODE0 high = F11, USB boot only.")
for i in range(3):
    res("fx3", "10k", f"FX3_PMODE{i}", "+3V3", dnp=(i != 1))
    res("fx3", "10k", f"FX3_PMODE{i}", "GND", dnp=True)
put("fx3", stk("Connector_Generic", "Conn_01x02"), "JP", "USB_BOOT", {1: "FX3_PMODE0", 2: "FX3_PMODE0_UP"}, "HDR2")
res("fx3", "1k", "FX3_PMODE0_UP", "+3V3")
heading("fx3", "Reset: RC holds RESET# low > 1 ms after power (tRPW 1 ms, tRR 5 ms with a crystal); "
        "button and the JTAG header can also pull it.")
res("fx3", "10k", "FX3_RESETn", "FX3_CVDDQ")
cap("fx3", "1u", "FX3_RESETn")
put("fx3", stk("Switch", "SW_Push"), "SW", "RESET", {1: "FX3_RESETn", 2: "GND"}, "SW", lcsc="C318884")
heading("fx3", "I2C: boot EEPROM and TVP7002 on one bus, 2k pull-ups to VIO5 (AN70707). "
        "24LC1025 A2 must be tied high; A1:A0 = 00.")
put("fx3", stk("Memory_EEPROM", "24LC1025"), "U", "24LC1025-I/SN", {
    1: "GND", 2: "GND", 3: "+3V3", 4: "GND", 5: "I2C_SDA", 6: "I2C_SCL", 7: "GND", 8: "+3V3"},
    "SOIC8", lcsc="C615697")
put("fx3", stk("Memory_EEPROM", "24LC1025"), "U", "24LC1025-I/SN", {
    1: "+3V3", 2: "GND", 3: "+3V3", 4: "GND", 5: "I2C_SDA", 6: "I2C_SCL", 7: "GND", 8: "+3V3"},
    "SOIC8", lcsc="C615697", dnp=True,
    note="Second 128 KB for a firmware image over 128 KB: AN76405 Table 17 device 2 (A2/A1/A0 = Vcc/0/1)")
cap("fx3", "100n", "+3V3")
res("fx3", "2k", "I2C_SCL", "+3V3")
res("fx3", "2k", "I2C_SDA", "+3V3")
heading("fx3", "USB reference resistors (AN70707): R_USB2 6.04k 1%, R_USB3 200R 1%.")
res("fx3", "6.04k 1%", "FX3_RUSB2", "GND")
res("fx3", "200 1%", "FX3_RUSB3", "GND")
heading("fx3", "Debug: JTAG (no external pulls; FX3 has internal ones), UART on GPIO55/56, LED on GPIO57.")
put("fx3", stk("Connector", "Conn_ARM_JTAG_SWD_10"), "J", "JTAG", {
    1: "+3V3", 2: "FX3_TMS", 3: "GND", 4: "FX3_TCK", 5: "GND", 6: "FX3_TDO", 7: None, 8: "FX3_TDI",
    9: "GND", 10: "FX3_RESETn"}, "JTAG")
put("fx3", stk("Connector_Generic", "Conn_01x03"), "J", "UART", {1: "FX3_UART_TX", 2: "FX3_UART_RX", 3: "GND"}, "HDR3")
put("fx3", stk("Device", "LED"), "D", "GREEN", {1: "FX3_LED_K", 2: "+3V3"}, "LED0603", lcsc="C2986059")
res("fx3", "1k", "FX3_LED_K", "FX3_LED")
heading("fx3", "Decoupling per AN70707 Table 3. AVDD, U3TXVDDQ, U3RXVDDQ and CVDDQ each behind a "
        "ferrite (AN70707 Figure 1, Appendix B); 22 uF on each USB 3 supply (datasheet).")
for _ in range(8):
    cap("fx3", "100n", "+1V2")
for _ in range(4):
    cap("fx3", "10n", "+1V2")
cap("fx3", "22u", "+1V2")
bead("fx3", "+1V2", "FX3_AVDD")
cap("fx3", "2.2u", "FX3_AVDD")
cap("fx3", "100n", "FX3_AVDD")
for rail in ("FX3_U3TX", "FX3_U3RX"):
    bead("fx3", "+1V2", rail)
    cap("fx3", "22u", rail)
    cap("fx3", "100n", rail)
bead("fx3", "+3V3", "FX3_CVDDQ")
cap("fx3", "100n", "FX3_CVDDQ")
cap("fx3", "10n", "FX3_CVDDQ")
for _ in range(6):
    cap("fx3", "100n", "+3V3")
for _ in range(5):
    cap("fx3", "10n", "+3V3")
cap("fx3", "1u", "VBUS")
cap("fx3", "100n", "VBUS")

# ================================================================ power sheet
heading("power", "USB 3 Standard-B (FX3). SS TX AC-coupled with 100 nF at the connector (AN70707 p.22, "
        "p.40). ESD at the connector. Shield to GND through 100k || 2.2 nF (AN70707 Figure 35).")
put("power", stk("Connector", "USB3_B"), "J", "USB3_B", {
    1: "VBUS", 2: "USB3_DM", 3: "USB3_DP", 4: "GND", 5: "USB3_TXN", 6: "USB3_TXP", 7: "GND",
    8: "USB3_RXN", 9: "USB3_RXP", "SH": "USB3_SHIELD"}, "USB3B", lcsc="C7501849", ref="J2")
cap("power", "100n", "USB3_TXN_C", "USB3_TXN")
cap("power", "100n", "USB3_TXP_C", "USB3_TXP")
put("power", stk("Power_Protection", "TPD4E05U06DQA"), "D", "TPD4E05U06DQAR",
    {1: "USB3_TXP", 2: "USB3_TXN", 4: "USB3_RXP", 5: "USB3_RXN", 3: "GND", 8: "GND",
     6: None, 7: None, 9: None, 10: None}, "USON10", lcsc="C138714")
put("power", stk("Power_Protection", "USBLC6-2SC6"), "D", "USBLC6-2SC6",
    {1: "USB3_DP", 6: "USB3_DP", 3: "USB3_DM", 4: "USB3_DM", 2: "GND", 5: "VBUS"}, "SOT23-6", lcsc="C7519")
res("power", "100k", "USB3_SHIELD", "GND")
cap("power", "2.2n", "USB3_SHIELD")
cap("power", "10u", "VBUS")
heading("power", "Three TPS563201 bucks from VBUS (SLVSD90B Table 7-2: R2 = 10k, L = 2.2 uH, "
        "2 x 22 uF out). 3.3 V digital; 2.5 V pre-regulator for the TVP7002 1.9 V LDOs "
        "(OSSC pattern); 1.2 V FX3 core. EN has an internal pull-down, so it is tied up through 100k.")
for out, r1 in (("+3V3", "33.2k 1%"), ("+2V5", "22.6k 1%"), ("+1V2", "5.76k 1%")):
    tag = out.strip("+")
    put("power", stk("Regulator_Switching", "TPS563201"), "U", "TPS563201DDCR", {
        1: "GND", 2: f"SW_{tag}", 3: "VBUS", 4: f"FB_{tag}", 5: f"EN_{tag}", 6: f"BST_{tag}"},
        "BUCK", lcsc="C116592")
    cap("power", "10u", "VBUS")
    cap("power", "100n", "VBUS")
    res("power", "100k", "VBUS", f"EN_{tag}")
    cap("power", "100n", f"BST_{tag}", f"SW_{tag}")
    put("power", stk("Device", "L"), "L", "2.2u", {1: f"SW_{tag}", 2: out}, "L4030")
    cap("power", "22u", out)
    cap("power", "22u", out)
    res("power", r1, out, f"FB_{tag}")
    res("power", "10k", f"FB_{tag}", "GND")
heading("power", "LDOs: 2.5 V -> 1.9 V for TVP7002 AVDD/PLL and DVDD (two TLV75519, 500 mA); "
        "VBUS -> 3.3 V for TVP7002 A33VDD alone (TLV75733, 1 A), kept off the buck rail.")
for out in ("+1V9A", "+1V9D"):
    put("power", stk("Regulator_Linear", "TLV75519PDBV"), "U", "TLV75519PDBVR",
        {1: "+2V5", 2: "GND", 3: "+2V5", 4: None, 5: out}, "SOT23-5", lcsc="C2865381")
    cap("power", "1u", "+2V5")
    cap("power", "1u", out)
put("power", stk("Regulator_Linear", "TLV75733PDBV"), "U", "TLV75733PDBVR",
    {1: "VBUS", 2: "GND", 3: "VBUS", 4: None, 5: "+3V3A"}, "SOT23-5", lcsc="C485517")
cap("power", "1u", "VBUS")
cap("power", "1u", "+3V3A")
heading("power", "Power LED, mounting holes.")
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

root = L.Sheet("root", "ps1-av-capture.kicad_sch", "ps1-av-capture: PS1 RGB + audio to USB")
root.uuid = ROOT_UUID
root.text("ps1-av-capture, one channel. TVP7002 digitises the PS1 analog RGB, an FX3 bridges it to "
          "USB 3 (UVC firmware, AN75779 pattern),\na PCM2903C presents the line audio as a USB "
          "Audio Class device on its own port. Design notes: docs/design.md.", 20, 30, 2.0)
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
