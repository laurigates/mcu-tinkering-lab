// AliExpress console helpers, pasted into a signed-in tab with the Chrome
// extension's javascript_tool. Read-only: nothing here adds to a cart or orders.
// See SKILL.md for where each helper must run and what the output filter blocks.

// --- Order history (run on https://www.aliexpress.com/p/order/index.html) ---
// 1. Capture the page's own request: hook lib.mtop.request, click "View orders".
window.__cap = [];
const __orig = window.lib.mtop.request.bind(window.lib.mtop);
window.lib.mtop.request = function (p, ...r) {
  window.__cap.push(JSON.parse(JSON.stringify(p)));
  return __orig(p, ...r);
};
document.querySelector('.order-more button').click();

// 2. Replay it with any page index and size.
window.__page = async (idx, size = 200) => {
  const p = JSON.parse(JSON.stringify(window.__cap.find((c) => c.api === 'mtop.aliexpress.trade.buyer.order.list')));
  const outer = JSON.parse(p.data.params);
  const inner = JSON.parse(outer.data);
  const body = Object.keys(inner).find((k) => k.startsWith('pc_om_list_body'));
  inner[body].fields.pageIndex = idx;
  inner[body].fields.pageSize = size;
  outer.data = JSON.stringify(inner);
  p.data.params = JSON.stringify(outer);
  return await window.lib.mtop.request(p);
};

// 3. Every order line in one request. Paging is unstable: orders placed
//    together shuffle between pages, so a 50-per-page walk returns some twice
//    and drops others (SKILL.md trap 2). One page of 200 held all 187 orders on
//    2026-10-30; if hasMore is still true, raise the size rather than paging.
window.__orders = [];
{
  const d = (await window.__page(1)).data.data;
  const orders = Object.keys(d).filter((k) => k.startsWith('pc_om_list_order_')).map((k) => d[k].fields);
  const body = d[Object.keys(d).find((k) => k.startsWith('pc_om_list_body'))];
  if (body && body.fields.hasMore) throw new Error('order list truncated: raise the page size');
  if (new Set(orders.map((o) => String(o.orderId))).size !== orders.length) throw new Error('duplicate orders in one page');
  for (const o of orders)
    for (const l of o.orderLines || [])
      window.__orders.push({
        date: o.orderDateText, status: o.statusText, pid: l.productId, title: l.itemTitle,
        sku: (Array.isArray(l.skuAttrs) ? l.skuAttrs : Object.values(l.skuAttrs || {}))
          .map((x) => x && x.text).filter((t) => t && !/China/i.test(t)).join(' '),
        price: l.itemPriceText, qty: l.quantity,
      });
}

// --- Search (any aliexpress.com page) ---
// The listing price is the cheapest variant, often a 1- or 2-piece pack.
window.__search = async (q, n = 8) => {
  const h = await fetch('/w/wholesale-' + encodeURIComponent(q.replace(/ /g, '-')) + '.html', { credentials: 'include' }).then((r) => r.text());
  const out = [], seen = new Set();
  for (const s of h.split('"productId":"').slice(1)) {
    const pid = s.slice(0, s.indexOf('"'));
    if (seen.has(pid) || !/^\d+$/.test(pid)) continue;
    const t = (s.match(/"displayTitle":"((?:[^"\\]|\\.)*)"/) || [])[1];
    if (!t) continue;
    seen.add(pid);
    out.push({
      pid, t: t.slice(0, 70),
      p: (s.match(/"salePrice":\{[^}]*?"formattedPrice":"([^"]*)"/) || [])[1],
      tr: (s.match(/"tradeDesc":"([^"]*)"/) || [])[1] || '',
    });
    if (out.length >= n) break;
  }
  return out;
};

// --- Variant prices and shipping (run on any /item/<id>.html page) ---
// Copy the page's own pdp query parameters (ship-to country, currency).
const __u = new URL(performance.getEntriesByType('resource').map((e) => e.name).find((n) => /pdp\.pc\.query/.test(n)));
window.__pdpReq = { data: JSON.parse(__u.searchParams.get('data')) };
window.__pdp = async (pid) => window.lib.mtop.request({
  api: 'mtop.aliexpress.pdp.pc.query', v: '1.0', type: 'GET', dataType: 'originaljsonp',
  data: { ...window.__pdpReq.data, productId: String(pid) },
});

// One line per item: shipping, Choice threshold, delivery days, every salable
// variant with its price. '?', '&' and '=' are stripped: the tool's output
// filter blocks anything that looks like a query string.
window.__item = async (pid, filter, max = 14) => {
  const R = (await window.__pdp(pid)).data.result;
  if (!R || !R.SKU) return pid + ' | no data';
  const names = {};
  for (const p of R.SKU.skuProperties)
    for (const v of p.skuPropertyValues) names[p.skuPropertyId + ':' + v.propertyValueIdLong] = v.propertyValueDisplayName;
  const pm = R.PRICE.skuPriceInfoMap || {};
  let rows = R.SKU.skuPaths.filter((s) => s.salable).map((s) =>
    s.path.split(';').map((x) => names[x] || x).join('/') + ' ' + ((pm[s.skuIdStr] || pm[s.skuId] || {}).salePriceString || 'NA'));
  if (filter) rows = rows.filter((x) => new RegExp(filter, 'i').test(x));
  const b = (R.SHIPPING.deliveryLayoutInfo || [])[0]?.bizData || {};
  const title = ((R.PRODUCT_TITLE && R.PRODUCT_TITLE.text) || '').slice(0, 60);
  const choice = b.choiceFreeShipping === 'yes' ? ' Choice free from ' + b.logisticsComposeThreshold : '';
  return `${pid} ${title} || ship ${b.formattedAmount || 'NA'}${choice} ${b.deliveryDayMin || ''}-${b.deliveryDayMax || ''}d || ${rows.slice(0, max).join(' ; ')}`.replace(/[?&=]/g, ' ');
};

// Search, keep titles matching `re`, price each: one string per candidate.
window.__scan = async (q, re, filter, n = 6) => {
  const hits = (await window.__search(q, 20)).filter((x) => new RegExp(re, 'i').test(x.t)).slice(0, n);
  const out = [];
  for (const x of hits) out.push((x.tr ? '[' + x.tr + '] ' : '') + (await window.__item(x.pid, filter, 8)));
  return out;
};

// --- Add to cart (only when Lauri asks; run on an /item/<id>.html page) ---
// 1. Capture the page's own cart.add request: install the hook, then click
//    "Add to cart" once by hand or with
//    [...document.querySelectorAll('button')].find((b) => b.innerText.trim() === 'Add to cart').click()
window.__cap = [];
const __origAdd = window.lib.mtop.request.bind(window.lib.mtop);
window.lib.mtop.request = function (p, ...r) {
  if (p.api === 'mtop.aliexpress.trade.cart.add') window.__cap.push(JSON.parse(JSON.stringify(p)));
  return __origAdd(p, ...r);
};
// 2. After the click: window.__cartReq = window.__cap[0];

// One variant, matched by name to exactly one skuId. Call it a few lines at a
// time, seconds apart: 25 rapid adds brought up a slider CAPTCHA (SKILL.md trap 8).
window.__add = async (pid, re, qty = 1) => {
  const R = (await window.__pdp(pid)).data.result;
  const names = {};
  for (const p of R.SKU.skuProperties)
    for (const v of p.skuPropertyValues) names[p.skuPropertyId + ':' + v.propertyValueIdLong] = v.propertyValueDisplayName;
  const hits = R.SKU.skuPaths.filter((s) => s.salable)
    .map((s) => ({ sku: s.skuIdStr, attr: s.path.split(';').map((x) => names[x] || x).join('/') }))
    .filter((x) => !re || new RegExp(re, 'i').test(x.attr));
  if (hits.length !== 1) return `${pid} SKIPPED: ${hits.length} matches`;
  const svc = (R.SHIPPING.deliveryLayoutInfo || [])[0]?.bizData?.deliveryOptionCode;
  const p = JSON.parse(JSON.stringify(window.__cartReq));
  p.data.addItems = JSON.stringify([{ itemId: String(pid), skuId: hits[0].sku, fulfillmentservice: svc, quantity: qty, attributes: { carAdditionalInfo: '{}', sourceType: '' } }]);
  const res = await window.lib.mtop.request(p);
  return `${pid} ${hits[0].attr} x${qty}: ${res.ret[0].split('::')[0]}`;
};
