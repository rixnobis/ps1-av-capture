// Run JLCPCB's impedance calculator headless for a 90 ohm edge-coupled pair on a 4-layer 1.6 mm board
// and dump every stackup it evaluates. Usage: NODE_PATH=<dir with playwright-core> node tools/jlc_impedance.js <spacing_mil> 14
// Writes diff_out.json (request args + solver result per stackup) in the working directory.
const { chromium } = require('playwright-core');
(async () => {
  const b = await chromium.launch({ headless: true, executablePath: process.env.CHROMIUM || '/usr/bin/chromium' });
  const p = await b.newPage({ viewport: { width: 1280, height: 1600 } });
  const out = [];
  p.on('response', async r => {
    if (r.url().endsWith('/impedance/calc')) {
      let body = ''; try { body = await r.text(); } catch (e) {}
      out.push({ req: JSON.parse(r.request().postData() || '{}'), resp: JSON.parse(body || '{}') });
    }
  });
  await p.goto('https://jlcpcb.com/pcb-impedance-calculator', { waitUntil: 'networkidle', timeout: 60000 });
  await p.waitForTimeout(1500);
  const ins = p.locator('input');
  await ins.nth(10).click();
  await p.waitForTimeout(500);
  await p.locator('.el-select-dropdown__item:visible', { hasText: 'Differential Pair (Non coplanar)' }).first().click();
  await p.waitForTimeout(500);
  await ins.nth(9).fill('90');
  const vals = await p.$$eval('input', es => es.map((e, i) => i + ':' + e.value + ':' + (e.offsetParent ? 'v' : 'h')).filter(x => x.endsWith(':v')));
  console.log(vals.join(' | '));
  const sp = p.locator('input').nth(+process.argv[3] || 14);
  await sp.fill(process.argv[2] || '6');
  await p.locator('button:has-text("Calculate")').first().click();
  await p.waitForTimeout(8000);
  require('fs').writeFileSync('diff_out.json', JSON.stringify(out));
  await p.screenshot({ path: 'diff.png', fullPage: true });
  console.log('calls', out.length, out.length ? out[0].req.impedance_calc_mark : '');
  await b.close();
})();
