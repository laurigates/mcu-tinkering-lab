---
name: aliexpress-parts
description: Price a carrier's BOM on AliExpress and read Lauri's AliExpress order history for parts already owned, from the browser console instead of screenshots. Use when comparing self-assembly against JLCPCB PCBA, checking what components are on hand, or pricing parts in bulk.
argument-hint: <project or BOM to price>
allowed-tools: Read, Write, Bash
---

# AliExpress From the Console

AliExpress pages carry their own API client, `window.lib.mtop`, signed with the
session's cookies. Calling it from the page answers in one round trip what
screenshots and clicks take dozens for. The helpers are in
[`aliexpress.js`](aliexpress.js); paste each section with the Chrome
extension's `javascript_tool` into a signed-in tab. Add to the cart only when
Lauri asks; checking out and paying are always Lauri's.

| Need | API | Runs on |
|---|---|---|
| Order history | `mtop.aliexpress.trade.buyer.order.list` (POST) | `/p/order/index.html` |
| Search | the `/w/wholesale-<query>.html` page's embedded JSON | any page (same-origin fetch) |
| Variant prices, shipping to FI | `mtop.aliexpress.pdp.pc.query` (GET) | any `/item/<id>.html` page |
| Add to cart | `mtop.aliexpress.trade.cart.add` (POST) | any `/item/<id>.html` page |
| Cart contents | `mtop.aliexpress.trade.cart.render` (GET) | `/p/shoppingcart/index.html` |

## How each was found

- **Orders:** the order-list request carries an opaque component tree
  (`params` → `data`, `linkage`, `hierarchy`). Don't rebuild it: hook
  `lib.mtop.request`, click "View orders" once, then replay the captured
  request with a different `pageIndex` and `pageSize` (50 works). Stop when the
  body component's `hasMore` is false. 2026-10-23: 4 pages, 203 order lines,
  May 2017 to Sep 2026.
- **Search:** the result page's HTML embeds each item as JSON; split on
  `"productId":"` and read `displayTitle`, `salePrice.formattedPrice` and
  `tradeDesc` (units sold). The price is the cheapest variant, often a 1- or
  2-piece pack, so price every candidate with `__item`.
- **Product:** `performance.getEntriesByType('resource')` holds the item page's
  own `pdp.pc.query` URL; its `data` parameter (ship-to country, currency,
  language) is reused with another `productId`. `SKU.skuPaths` ×
  `SKU.skuProperties` give the variant names, `PRICE.skuPriceInfoMap` their
  prices, `SHIPPING.deliveryLayoutInfo[0].bizData` the shipping fee,
  `choiceFreeShipping`, `logisticsComposeThreshold` and delivery days.
- **Cart add:** captured from the "Add to cart" button. `addItems` is a JSON
  string of `[{itemId, skuId, fulfillmentservice, quantity, attributes}]`;
  `fulfillmentservice` is the product's `bizData.deliveryOptionCode`
  (`CAINIAO_FULFILLMENT_STD` for Choice, `CAINIAO_STANDARD`,
  `AE_CN_SUPER_ECONOMY_G`). Resolve the variant to exactly one `skuId` by
  name before adding. `cart.render` lists ten lines per page with each
  line's `itemView` (item, sku text, `createTimeStamp`) and `quantityView`;
  `cart.count` returns the line count.

## Traps

1. **The tool's output filter blocks results that look like URLs, query strings
   or base64** (`[BLOCKED: Cookie/query string data]`), and truncates long
   strings. Return arrays of short strings, strip `?&=`, and never return raw
   response JSON.
2. **The order history holds every purchase, not only electronics.** Filter
   by component keywords in the page and report only matching lines; don't
   dump or quote the rest.
3. **Shipping is per store unless the item is Choice.** Choice items cost
   €2.99 alone and ship free once Choice items reach €10.00 together
   (2026-10-23). Other stores charge €1.50–7 each: the robocar carriers' BOM
   spread over ten such stores paid €41.73 in shipping on a €153.48 total.
4. **Search hits are not the part.** Titles stuff neighbouring part numbers
   ("TPS563201 TPS563208", "DMP3013SFV DMP3018SFV"): filter the variant by
   name, and check package and voltage in the variant text.
5. **Generic passives miss the datasheet numbers the carrier was designed
   to** (`power-stage-checklist.md`). An unbranded 6045 4.7 µH inductor has no
   stated Isat against the integrated carrier's 5.5 A peak, and 1812 PTCs at
   3 A are usually 6 V parts against the 16 V BSMD1812-300-16V.
6. **Solder paste ships slowly** (24–46 days on 2026-10-23) and wants
   refrigeration.
7. **Rapid cart adds trigger a slider CAPTCHA.** 25 adds about 0.6 s apart
   (2026-10-23) ended with one `TIMEOUT::接口超时` and, on the next add, a
   "drag the slider to verify" overlay. Solving it is Lauri's step. Add a few
   lines at a time, several seconds apart, in calls that finish inside the
   tool's 45 s limit. A call that times out keeps running in the page, so
   read the cart before retrying anything, or the retry adds duplicates.
8. **AliExpress prices include VAT; JLCPCB's exclude it until checkout**
   (`jlcpcb-order` § 4). Compare VAT-inclusive totals.

## Comparing with JLCPCB PCBA

Price the parts for the boards that need them, at the quantity the listing
sells, and add each store's shipping. Compare against the PCBA line in the
JLCPCB cart **minus the coupon PCBA unlocks** (the JLCONE PCBA $20 needs PCBA
in the order), and against JLCPCB's own increment for more assembled boards
(2 → 5 on the integrated carrier: +€29.73, `jlcpcb-order` § 4). Add 25.5 %
VAT to the JLCPCB figures first.

2026-10-23, both robocar carriers ×5: JLCPCB assembly €124.97 − €17.80
coupon = €107.17 ex VAT, about €134.50 with it, against €153.48 of
AliExpress parts with VAT and shipping.

Write the result to `tmp/quotes/aliexpress-bom-<date>.csv`: BOM line, need,
listing id, variant, price, shipping, delivery days, and what is already owned.

## Related

- `jlcpcb-order` skill: the JLCPCB side of the comparison
- `.claude/rules/board-layout-from-vendor-files.md`: reading a breakout's
  pinout from its AliExpress listing photos
