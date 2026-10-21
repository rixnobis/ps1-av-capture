#!/usr/bin/env python3
"""Stock check against JLCPCB's parts search. Usage: jlc_stock.py TVP7002 CYUSB3014 ...
The search is fuzzy, so only parts whose model number starts with the keyword are printed."""
import json, sys, time, urllib.request
URL = 'https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList'
for kw in sys.argv[1:]:
    body = json.dumps({'currentPage': 1, 'pageSize': 50, 'keyword': kw, 'searchSource': 'search'}).encode()
    rq = urllib.request.Request(URL, body, {'Content-Type': 'application/json', 'User-Agent': 'Mozilla/5.0'})
    d = json.load(urllib.request.urlopen(rq, timeout=30))
    rows = (d.get('data') or {}).get('componentPageInfo', {}).get('list') or []
    hits = [c for c in rows if (c.get('componentModelEn') or '').upper().startswith(kw.upper())]
    print('== %s: %d results, %d match' % (kw, len(rows), len(hits)))
    for c in hits:
        p1 = (c.get('componentPrices') or [{}])[0].get('productPrice')
        print('  %-12s %-20s %-24s %-7s stock %-6s $%s' % (c.get('componentCode'), c.get('componentModelEn'),
              c.get('componentSpecificationEn'), c.get('componentLibraryType'), c.get('stockCount'), p1))
    time.sleep(1.5)
