#!/usr/bin/env python3
"""Read every LCSC number in the netlist back from JLCPCB and check it against the value.

The passive LCSC numbers in gen_sch.py were typed, so each one is looked up and its
description must carry the value (and the 1% for precision resistors) and the package.
Usage: check_lcsc.py netlist.xml   (exit 1 on any mismatch or zero stock)
"""
import json
import re
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

URL = 'https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList'


def lookup(code):
    body = json.dumps({'currentPage': 1, 'pageSize': 10, 'keyword': code, 'searchSource': 'search'}).encode()
    rq = urllib.request.Request(URL, body, {'Content-Type': 'application/json', 'User-Agent': 'Mozilla/5.0'})
    d = json.load(urllib.request.urlopen(rq, timeout=30))
    rows = (d.get('data') or {}).get('componentPageInfo', {}).get('list') or []
    return next((r for r in rows if r.get('componentCode') == code), None)


def value_tokens(value, fp):
    """Strings one of which must appear in the JLC description for this value."""
    v = value.split()[0]
    m = re.match(r'^([\d.]+)([pnumkM]?)(R?)$', v.replace('u', 'u'))
    if not m:
        return [v]
    num, mult = m.group(1), m.group(2)
    if fp.startswith('Resistor'):
        unit = {'': 'Ω', 'k': 'kΩ', 'M': 'MΩ'}[mult]
        return [f'{num}{unit}']
    if fp.startswith('Capacitor'):
        unit = {'p': 'pF', 'n': 'nF', 'u': 'uF'}[mult]
        alt = {'n': (lambda x: f'{float(x) * 1000:g}pF'), 'u': (lambda x: f'{float(x) * 1000:g}nF')}
        toks = [f'{num}{unit}', f'{num}{unit.replace("u", "µ")}']
        if mult in alt:
            toks.append(alt[mult](num))
        return toks
    return [v]


root = ET.parse(sys.argv[1]).getroot()
seen = {}
for c in root.find('components'):
    f = {x.get('name'): x.text for x in c.iter('field')}
    code = f.get('LCSC')
    if code:
        seen.setdefault(code, []).append((c.get('ref'), c.findtext('value'), c.findtext('footprint') or ''))
bad = []
for code, uses in sorted(seen.items()):
    r = lookup(code)
    time.sleep(1.2)
    if r is None:
        bad.append(f'{code}: not found')
        print(f'{code:10} NOT FOUND for {uses[0]}')
        continue
    desc = f"{r.get('componentModelEn')} {r.get('componentSpecificationEn')} {r.get('describe')}"
    ref, value, fp = uses[0]
    toks = value_tokens(value, fp.split(':')[0])
    ok = any(t in desc for t in toks) or not fp.split(':')[0] in ('Resistor_SMD', 'Capacitor_SMD')
    if '1%' in value and '1%' not in desc:
        ok = False
    pkg = re.search(r'_(0402|0603|0805)_', fp)
    if pkg and pkg.group(1) not in desc:
        ok = False
    stock = r.get('stockCount') or 0
    print(f"{code:10} {'ok ' if ok else 'BAD'} stock {stock:<8} x{len(uses):<3} {value:10} {desc[:90]}")
    if not ok:
        bad.append(f'{code}: {value} {fp} vs "{desc[:80]}"')
    if stock < len(uses):
        bad.append(f'{code}: stock {stock} for {len(uses)} uses')
    elif stock < 10 * len(uses):
        print(f'           note: {code} stock {stock} is under 10 builds of this board')
print(f'codes {len(seen)}, findings {len(bad)}')
for b in bad:
    print('  ' + b)
sys.exit(1 if bad else 0)
