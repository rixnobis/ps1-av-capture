# Parts and stock

Checked 2026-10-21 against JLCPCB's assembly parts search (`tools/jlc_stock.py`),
before any footprint was chosen. Prices are the single-unit tier in USD.

| Part | Use | LCSC | Package | Stock | Price |
|---|---|---|---|---|---|
| TVP7002PZP | RGB digitiser | C3824085 | HTQFP-100 | 2 | 8.96 |
| TVP70025IPZPR | same, industrial grade | C5216145 | HTQFP-100 | 3 | 13.38 |
| CYUSB3014-BZXI | FX3, USB 3 bridge | C57294 | FBGA-121 10x10 | 4868 | 16.48 |
| CYUSB3014-BZXC | FX3, commercial | C462526 | FBGA-121 10x10 | 3348 | 17.04 |
| PCM2903CDBR | USB audio codec | C2861593 | SSOP-28 | 151 | 10.42 |
| PCM2903CDB | same, tube | C1521850 | SSOP-28 | 38 | 15.59 |
| TUSB8041RGCR | 4-port USB 3 hub (later) | C486066 | VQFN-64 9x9 | 2334 | 7.05 |
| USB2514B-AEZC-TR | 4-port USB 2 hub (later) | C16251 | VQFN-36 6x6 | 13055 | 1.81 |

TI lists the TVP7002 as active. JLC holds five across the two grades, enough for this
one-channel board and a respin. A three-channel board needs the TVP7002 bought
elsewhere and supplied to the assembler.
The VL817 and GL3520 hubs show no stock at all, so the hub is a TUSB8041 if it ever
goes on a board.

The single-channel board carries no hub: the FX3 and the PCM2903C each get their own
USB connector.
