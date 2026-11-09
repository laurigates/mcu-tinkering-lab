---
name: aliexpress-parts
description: Price a carrier's BOM on AliExpress, check every part can be bought, and read Lauri's AliExpress order history for parts already owned, from the browser console instead of screenshots. Use when choosing parts for a design, before ordering a board with parts left for hand-soldering, comparing self-assembly against JLCPCB PCBA, or checking what is on hand.
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
| Delivered total for chosen lines | `/p/trade/confirm.html?availableProductShopcartIds=<ids>` in an iframe (`__checkout`) | any aliexpress.com page |

## How each was found

- **Orders:** the order-list request carries an opaque component tree
  (`params` → `data`, `linkage`, `hierarchy`). Don't rebuild it: hook
  `lib.mtop.request`, click "View orders" once, then replay the captured
  request as one page with `pageSize` 200, and check the body component's
  `hasMore` is false. 2026-10-30: 187 orders, 203 order lines, May 2017 to
  Sep 2026.
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
2. **Paging the order history drops orders.** Orders placed together shuffle
   between pages, so a page walk returns some orders twice and skips others,
   a different set on every run, and still ends with `hasMore` false. On
   2026-10-30, 50 per page gave 182 unique orders of 187 and 10 per page 173;
   the missing ones included the SNA-58B crimper, so it read as never bought.
   Fetch one page large enough to hold everything (200) and check its order
   ids are unique.
3. **The order history holds every purchase, not only electronics.** Filter
   by component keywords in the page and report only matching lines; don't
   dump or quote the rest.
4. **Shipping is per store unless the item is Choice.** Choice items cost
   €2.99 alone and ship free once Choice items reach €10.00 together
   (2026-10-23). Other stores charge €1.50–7 each: the robocar carriers' BOM
   spread over ten such stores paid €41.73 in shipping on a €153.48 total.
   Choice items fulfilled from the Singapore warehouse
   (`CAINIAO_FULFILLMENT_STD_SG`; SC-001 and solder paste on 2026-10-31) do
   not count toward the €10 and paid €3.19 each. Two variants of one
   global-seller listing paid shipping per line (HGD fuse, +€3.30).
5. **Search hits are not the part.** Titles stuff neighbouring part numbers
   ("TPS563201 TPS563208", "DMP3013SFV DMP3018SFV"): filter the variant by
   name, and check package and voltage in the variant text.
6. **Generic passives miss the datasheet numbers the carrier was designed
   to** (`power-stage-checklist.md`). An unbranded 6045 4.7 µH inductor has no
   stated Isat against the integrated carrier's 5.5 A peak, and 1812 PTCs at
   3 A are usually 6 V parts against the 16 V BSMD1812-300-16V.
7. **Solder paste ships slowly** (24–46 days on 2026-10-23) and wants
   refrigeration.
8. **Rapid cart adds trigger a slider CAPTCHA.** 25 adds about 0.6 s apart
   (2026-10-23) ended with one `TIMEOUT::接口超时` and, on the next add, a
   "drag the slider to verify" overlay. Solving it is Lauri's step. Add a few
   lines at a time, several seconds apart, in calls that finish inside the
   tool's 45 s limit. A call that times out keeps running in the page, so
   read the cart before retrying anything, or the retry adds duplicates.
9. **AliExpress prices include VAT; JLCPCB's exclude it until checkout**
   (`jlcpcb-order` § 4). Compare VAT-inclusive totals.
10. **A listing can refuse to ship to Finland.** The product query still
    returns variants and prices; only `SHIPPING.deliveryLayoutInfo[0].bizData`
    carries `unreachable: true`, with no fee and no delivery days. `__item`
    prints `DOES NOT SHIP HERE`. On 2026-10-30 the first 3 A 24 V 1812 PTC
    found was such a listing.
11. **The cart page renders ten lines.** The rest load only while the page
    scrolls slowly, a few hundred pixels at a time; a jump to the bottom shows
    the ten and then the recommendations. The "View more" there loads
    recommendations, not cart lines. A line can carry a different product id
    from the one it was added under (the 230-piece XH kit, added as
    33009421559, is listed as 1005006604844707), so match lines by title and
    variant, not id alone. Removing a line takes its trash icon, then the
    "Remove" button in the dialog that follows. Match a row by title **and**
    its exact variant line: the 0805 resistor title lists "10K 100K", so a
    title match for 10K hits every value.
12. **The cart's "Estimated total" leaves out import charges.** Checkout adds
    about €6.28 per tariff code per parcel (€3 EU duty + €2 handling + 25.5 %
    VAT). It shows a charge per listing, then credits the excess back as
    "AliExpress import charge reduction". The Choice lines are one parcel;
    each global-seller store is its own parcel with its own charge. On
    2026-10-31 a cart estimated at €143.88 checked out at €231.99, all of the
    gap import charges. A parcel over €150 still paid it. Rules and evidence:
    [PRD-016](../../../docs/requirements/PRD-016-build-cost-model.md) R3–R6.
    Price every plan at checkout (`__checkout`), never from the cart.
13. **The ticked selection is server state, and it moves.** Ticking a line
    saves through `mtop.aliexpress.trade.cart.async`, the same request the
    page sends to load more lines, so replaying a captured `cart.async`
    rewrites the selection: it dropped from 25 lines to 10 on 2026-10-31.
    The selection also differed between tabs and after the phone app was
    used. Never replay `cart.async`. To price or hand over a fixed set of
    lines, open `/p/trade/confirm.html?availableProductShopcartIds=<ids>`
    with explicit cart ids (`__checkout`): it renders exactly those lines
    and leaves the ticks alone. Read the ids of a ticked set from the
    checkout URL after clicking "Checkout (n)". The link stays valid while
    the lines remain in the cart.
14. **Coupon codes are not listed at checkout.** The promo field takes a
    typed code only, and the wallet (`/p/my-coupons/index.html`) can be
    empty while codes are collected. The item query's
    `COUPON_BLOCK_PC.discountPanelUrl` opens a `pdp-marketing` page that
    lists the collected platform codes with their minimum spend and dates
    (2026-10-31: NWED1108, €8 off €45, valid 1–19 Nov). Store coupons there
    need a minimum per store, which one-line global orders never reach.

## Sourcing check: can every part be bought?

Run this when choosing a part for a design and before a board is ordered with
any part left for hand-soldering (`.claude/rules/part-sourcing.md`). For each
BOM line, in order:

0. **Fitted already?** For a board JLCPCB assembled, the project's
   `output/assembly/<name>-bom-jlc-<sha7>.csv` lists what it placed
   (surface-mount only); `<sha7>` is the commit in the order's hand-off CSV.
   Order only the rest: on 2026-10-31 a 16 V / 4 A fuse for the robocar was
   added and then dropped, because F1 was on the assembled boards.
1. **Owned?** Search the order history, then the "Electronics Purchases"
   artifact (it also holds the eBay history). A matching line names the part
   or its exact rating; a kit counts only if the kit's listing names the value.
2. **Sold here?** `__scan` with the exact part number. A hit counts when:
   - the title or variant names the exact part number, or, for a passive, the
     value, package, voltage and current the design needs (trap 5);
   - the listing ships to Finland (trap 10);
   - the datasheet numbers the design depends on are stated, or the part is a
     named manufacturer's part whose datasheet states them (trap 6).
3. **A substitute?** Only through the `part-substitution` skill: both
   datasheets read and compared on the design's numbers, and the footprint
   and pin order checked against the registry entry. On 2026-10-30 the ZXMP3A16G was accepted as a
   hand-build stand-in for the unlisted ZXMP4A16G, which the design keeps: same SOT-223 pin-out, −30 V against −40 V, 45/70 mΩ against
   60/100 mΩ (Diodes DS33575 Rev. 6, DS33584 Rev. 6).
4. **None of these**: the part goes to JLCPCB, on the PCBA BOM or an LCSC
   parts order (`jlcpcb-order` § 1).

Record each line's answer in the quote CSV: owned, listing id and variant,
substitute with its reason, or JLCPCB. Write a confirmed source into the
design as `buy` on the registry entry or `hardware.toml` instance
(`aliexpress:<id>` or `owned:<what>`; a substitute is a different part and gets
no `buy` on the original). `just kicad::sourcing <project>` then lists what is
left to JLCPCB. When the parts are for several boards,
size each line to the number of PCBs ordered, so the PCBs stay the limit.

## Comparing with JLCPCB PCBA

Price the parts for the boards that need them, at the quantity the listing
sells, and add each store's shipping. Compare against the PCBA line in the
JLCPCB cart **minus the coupon PCBA unlocks** (the JLCONE PCBA $20 needs PCBA
in the order), and against JLCPCB's own increment for more assembled boards
(2 → 5 on the integrated carrier: +€29.73, `jlcpcb-order` § 4). Add 25.5 %
VAT to the JLCPCB figures first.

Report two AliExpress figures: the **cash** spent, and the **consumed**
cost, each line's price and shipping times need ÷ units bought. Listings sell
in packs (100 resistors, 10 chips), so the gap between them is stock left
over. Leftover chips only have value if another build uses them.

2026-10-23, both robocar carriers ×5: JLCPCB assembly €124.97 − €17.80
coupon = €107.17 ex VAT, about €134.50 with it. AliExpress: €150.20 cash,
€98.09 consumed, leaving 15 chips (TCA9548A, TPS563208, DMP3013 ×5 each),
270 other passives and 700 resistors worth €52.11 at the price paid. Five
of the eight chips were bought with no spare.

Write the result to `tmp/quotes/aliexpress-bom-<date>.csv`: BOM line, need,
listing id, variant, price, shipping, delivery days, and what is already owned.

## Choosing between AliExpress and EU suppliers

The flat import charge (trap 12) changed the answer for small batches of
branded parts. Decide each line by what moving it changes:

- **Moving a line out of the Choice parcel** saves its €6.28 only if no other
  line there shares its tariff code. Connectors stay together because the JST
  kits keep the connector code in the parcel anyway.
- **A global-seller line** costs its price, its own shipping and €6.28. A
  €1.26 fuse delivered at €10.83. Replace it with a Choice listing whose code
  is already in the parcel, or move it to an EU order.

Measured 2026-10-31 (`tmp/quotes/eu-supplier-basket-2026-10-31.csv`):

| Source | Result |
|---|---|
| TME (tme.eu, Poland) | Cheapest EU source for branded discretes: resistors, diodes, capacitor and ZXMP4A16GTA €35 delivered against €55 on AliExpress with import charges. €6.90 + VAT shipping, no threshold. No JST kits or screw assortments |
| Partco (Finland) | 3–9× TME on the lines it stocks (connectors 9×), and missing the MOSFET, PTCs and zener |
| Reichelt | 2–5× TME, €14.95 shipping |
| Botland (Poland) | Sells to registered businesses only |
| eBay | The same Chinese sellers, each a separate parcel; EU sellers price per piece (ZXMP4A16GTA €7.14 each) |
| LCSC | The only source of 3 A / 24 V and 4 A / 16 V 1812 PTCs; pays import charges, best added to a JLCPCB order |

Keep tools, kits, screws, solder paste and odd items on AliExpress. TME
adds to a cart only after the shopper logs in or accepts guest checkout
terms, so filling a TME cart is Lauri's step; scripted clicks on its "Add to
cart" did not register. Its search page can show "0 in stock, no prices"
before it finishes loading, so wait and re-read before recording a negative.

**Check the part, not the listing title, against the registry.** A
replacement must match the registry entry's electrical assumption: the
robocar's buzzer footprint assumes a piezo (`piezo-buzzer-12mm.toml`), and
the "passive buzzers" found at AliExpress and TME were magnetic.

## Related

- `jlcpcb-order` skill: the JLCPCB side of the comparison
- `.claude/rules/board-layout-from-vendor-files.md`: reading a breakout's
  pinout from its AliExpress listing photos
