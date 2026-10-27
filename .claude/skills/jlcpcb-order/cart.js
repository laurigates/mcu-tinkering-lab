// Read the JLCPCB cart and the coupons it qualifies for, through the cart
// page's own API. Run on a cart.jlcpcb.com target:
//   node cdp.mjs eval cart.jlcpcb @cart.js
// Prices from the API are USD; EUR uses the site's own rate table.
(async () => {
  const post = (path, body) =>
    fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    }).then((r) => r.json());

  const rates = (await fetch('/api/overseas-basic-data/v1/queryRateList').then((r) => r.json())).data;
  const eur = rates.find((r) => r.afterCountryCode === 'EUR').rate;

  const cart = await post('/api/overseas-core-platform/shoppingCart/page', {
    pageNum: 1, pageSize: 100, keyWord: null, expired: false, type: 'ALL',
  });
  if (cart.code !== 200) return { error: cart.message ?? `code ${cart.code}`, hint: 'signed out?' };

  const goods = (l) => [
    ['PCB', l.pcbGoods], ['PCBA', l.smtGoods], ['Stencil', l.steelmeshGoods], ['3DP', l.tdpGoods],
  ].filter(([, g]) => g);

  // A PCB and its PCBA share one cart line and one grossWeight; count it once.
  const lines = cart.data.list.flatMap((l) => goods(l).map(([kind, g], i) => ({
    kind,
    code: g.goodsCode,
    file: g.gerberFile,
    qty: g.quantity,
    buildTime: g.achieveDateShow ?? g.buildTime,
    usd: g.price,
    eur: +(g.price * eur).toFixed(2),
    kg: i > 0 ? 0 : l.grossWeight ?? (g.tdpGoodsVO?.goodsWeightEstimation ?? 0) / 1000,
    businessType: g.businessType,
    cartId: g.shoppingCartAccessId,
  })));

  const usd = lines.reduce((s, l) => s + l.usd, 0);
  // The cart page sends the selection's business types (1 PCB, 4 PCBA, 7 3DP),
  // plus 2 whenever a PCB is in it, and one access id per good. Shipping-only
  // coupons need the shipping amount, which this leaves at 0.
  const types = new Set(lines.map((l) => l.businessType));
  if (types.has(1)) types.add(2);
  const coupons = await post('/api/overseas-core-platform/coupons/queryPossibleCoupons', {
    orderType: [...types].sort((a, b) => a - b), isSpecialCost: false,
    merchandiseTotal: +usd.toFixed(2), shippingMoney: 0,
    shopcartAccessIds: lines.map((l) => l.cartId),
  });
  // USD amounts, as the cart prices are.
  const coupon = ({ couponsRecordVo: c, unavailableDescription: why }) =>
    `${c.titleE} | $${c.denomination} off over $${c.minAmount} | expires ${new Date(c.expiryDate).toISOString().slice(0, 10)}${why ? ` | ${why}` : ''}`;

  return {
    eurPerUsd: eur,
    lines: lines.map(({ cartId, businessType, ...l }) => l),
    merchandise: { usd: +usd.toFixed(2), eur: +(usd * eur).toFixed(2) },
    availableCoupons: (coupons.data?.availableCoupons ?? []).map(coupon),
    unavailableCoupons: (coupons.data?.unavailableCoupons ?? []).map(coupon),
  };
})()
