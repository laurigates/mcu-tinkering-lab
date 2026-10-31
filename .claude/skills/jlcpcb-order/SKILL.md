---
name: jlcpcb-order
description: Quote a carrier PCB (bare or Economic PCBA) at JLCPCB through the JLCONE desktop app or the web, pick coupons and shipping, and stop at the cart for Lauri to pay. Use when ordering or re-pricing a board from `just <p>::carrier-fab`, or comparing JLCPCB with another fab.
argument-hint: <project(s) to order>
allowed-tools: Read, Grep, Glob, Bash, WebFetch, WebSearch
---

# Ordering a Carrier at JLCPCB

`carrier-fab` (the `carrier-pcb` skill, workflow step 6) makes the files; this
skill gets them into a JLCPCB cart at the lowest price that is actually
available, and **stops at the cart: Lauri checks out and pays.** Each
project's README has its own Ordering section (board options, what to check
in the preview); follow it, not a recalled copy.

Prices and limits below were observed on 2026-10-22 (task #457, PR #832) and
are a calibration, not a quote: JLCPCB changes them.

## 1. Before uploading

- **Decide who places each SMD part, and check it can be bought**
  (`.claude/rules/part-sourcing.md`). `just kicad::sourcing <project>` lists
  the parts with an LCSC number and no recorded `buy` source; for those, or to
  re-check a recorded source, run the `aliexpress-parts` skill's sourcing
  check. A part that is owned, or sold on AliExpress
  under its exact part number with its rating stated and shipping to Finland,
  may be hand-soldered. Every other part stays on the PCBA BOM. Economic PCBA
  places a partial BOM, so a board that is otherwise hand-soldered can still
  have JLCPCB fit those parts; if it ships bare, add them as an LCSC parts
  order in the same cart. The balancebot carrier went bare on 2026-10-23 and
  two of its 11 parts (ZXMP4A16G, BSMD1812-300-24V) had no usable AliExpress
  listing. Check stock for each LCSC number on the day of the order.
- **Run `just <p>::carrier-fab` from a clean tree** and read the size and
  layer count from `output/fab/*-job.gbrjob` (`.GeneralSpecs.Size`); the
  upload must detect the same.
- **Check the silkscreen apertures in the Gerber, not in board.toml.** A board
  committed before a `tools/kicad` fix does not get it until it is rebuilt:
  the integrated carrier's board predated `pcb/silk.py` by three minutes and
  shipped 0.10–0.127 mm strokes while `carrier-check` passed.

  ```sh
  grep -o '%ADD[0-9]*C,0\.1[0-4][0-9]*\*%' output/fab/*Silkscreen.g*   # must print nothing
  ```

  Control it against a board known to have thin strokes before trusting an
  empty result.
- **Panel mode decides the price.** JLCPCB counts pieces separable "by
  v-cut, millings, or drill holes" as different designs. A `mousebites` board
  quoted with Different Design 2 added €3.56 engineering + €3.61 panel, and
  with 4 designs €10.83 panel, over the single-design €3.56. A `saw` board is
  one outline and one design.
- Copy the zip, `bom-jlc.csv` and `cpl-jlc.csv` to a short path (and keep a
  copy in the repo's git-ignored `tmp/quotes/`: `/tmp` is cleared).

## 2. Where to order: the JLCONE app, not Chrome

**Use the JLCONE desktop app (`/Applications/JLCONE.app`, bundle
`com.jlcpcb.www`) for every quote, cart check and coupon, PCB and JLC3DP
alike.** Lauri's standing instruction. The app shares the account and cart
with the web, and gave one PCB per order a special offer (€3.56 → €1.78); a
second PCB in the same cart stayed at €3.56. Set currency in its gear menu
(Settings → Currency → EUR → Save).

### Drive it over CDP, not by screenshot

JLCONE is Electron 35 with no hardening fuses, and each tab is a `<webview>`
of the real site. Started with a debug port, every tab is a Chrome DevTools
Protocol target that [`cdp.mjs`](cdp.mjs) (Node 22+, no dependencies) can
read, script and upload into from Bash:

```sh
osascript -e 'quit app "JLCONE"'; open -a JLCONE --args --remote-debugging-port=9222
node .claude/skills/jlcpcb-order/cdp.mjs targets            # one line per tab
```

Quitting loses only the open tabs; the cart and quotes live on the account.
`<match>` below is a substring of the tab's URL.

| Step | Command |
|---|---|
| Open a page in a tab | `cdp.mjs nav <match> https://cart.jlcpcb.com/quote` (cart: `/shopcart/cart`) |
| Upload the Gerber zip | `cdp.mjs upload jlcpcb.com/quote 'input[type=file]' <zip>` |
| Read the detection | `cdp.mjs eval jlcpcb.com/quote 'document.body.innerText.match(/Detected[^\n]*/)?.[0]'` |
| PCB Assembly on, then to the BOM step | `eval` `document.querySelector("#smtQuoteRef .switch-btn").click()`, set the options, then click the button whose text is `NEXT` twice (quote page, then the `smt-order` PCB tab) |
| Upload BOM and CPL | `cdp.mjs upload smt-order 'input[type=file]@0' <bom.csv>`, then `@1` for the CPL; click `Process BOM & CPL` |
| Read the match | `eval smt-order 'document.body.innerText.replace(/\s+/g," ").match(/Total \d+ parts detected \d+ Parts confirmed/)?.[0]'`; per-line Lib Type is cell 6 of each `tbody tr` |
| Upload STLs to JLC3DP | `cdp.mjs nav <match> https://jlc3dp.com/3d-printing-quote`, then `upload jlc3dp 'input[type=file]' <stl>...` (one call, all files) |
| Read the models | `cdp.mjs eval jlc3dp @jlc3dp-models.js` ([`jlc3dp-models.js`](jlc3dp-models.js)): file, size, volume and price per row, and the total |
| Read the cart and its coupons | `cdp.mjs eval cart.jlcpcb @cart.js` ([`cart.js`](cart.js)): lines in USD and EUR, weight, available and unavailable coupons |
| Look at the page | `cdp.mjs shot <match> out.png`, then Read the PNG; works with the app in the background |
| Find an endpoint | `cdp.mjs net <match> 20 '/api/'` while doing the step once by hand or by `eval` click |

`<css>@N` picks the Nth match. **JLCONE vets each navigation**
(`will-frame-navigate` in its main process): `nav` from jlcpcb.com to
cart.jlcpcb.com and on to jlc3dp.com worked, but from jlc3dp.com back to
cart.jlcpcb.com it was cancelled, whether by `nav` or by setting
`location.href`, and the app hands such URLs to `open-external`. Do the PCB
work and the cart check before moving a tab to JLC3DP, or open JLC3DP from
Products in its own tab. `nav` fails after 30 s and prints where the tab is. Observed 2026-10-27 with the integrated
carrier: the zip was detected as "2 layer board of 100x94mm", matching the
job file; BOM and CPL matched 25/25; three STLs loaded in one call with
volumes matching `report.json` (tray 16.3 cm³, base 22.95 + lid 12.57 = box
35.52) and prices summing to the €4.21 total; `cart.js` read 5 goods over 4
cart lines. Each upload is also saved under Projects → Quotes. **Read the
JLC3DP rows one at a time**, as `jlc3dp-models.js` does: a regex over the
page text paired `main-tray.stl` with the lid's volume. The cart API prices in USD
(`queryRateList` gives the EUR rate; prices shown in EUR are converted in the
page). Its coupon query needs the selection's business types and one access
id per good, which `cart.js` builds the way the cart page does.

Computer-use stays the fallback when the app runs without the port:
`request_access` for `com.jlcpcb.www` (granted at full tier). Pages open as
tabs inside the app; the cart is the cart icon, other services are under
Products (JLC3DP: Products → Mechanical Services → "JLC3DP - 3D Printing").
The upload buttons open a native file picker; press Cmd+Shift+G, type the
absolute path, Return, then Open. Right-click → Inspect opens DevTools in any
tab, for a console when CDP is not set up.

Fall back to the web in Chrome only when Lauri asks for it. There the
`file_upload` tool works on the Gerber input; the PCBA BOM and CPL inputs are
hidden, so tag them first
(`document.querySelectorAll('input[type=file]').forEach((e,i)=>e.setAttribute('data-claude-file',i))`)
and `find` them by that attribute.

**The web session expires mid-flow, and Save to Cart then silently does
nothing.** The header shows "Sign In" instead of "My Account". Two quotes were
lost that way in Chrome. After every Save to Cart, in either app or browser,
confirm the line is there (`cart.js`, or open the cart; `cart.js` returns an
error when signed out). Signing in is Lauri's step, never the agent's.

## 3. The quote, step by step

1. Upload the zip. Compare "Detected N layer board of W×H" with the job file.
2. Leave the defaults the README asks for (FR-4, 1.6 mm, HASL, 1 oz,
   Different Design 1). Minimum via 0.3 mm matches the carriers' smallest
   drill; check `grep -E '^T[0-9]+C' output/fab/*.drl`.
   If `carrier-fab` printed `ORDER: select high-precision silkscreen` (both
   robocar carriers: `[silk] high_precision = true`, 0.8 mm pin labels), pick
   the high-precision silkscreen option. Standard silkscreen prints text
   under 1.0 mm tall unreadably. Record the option's price in § 4 the first
   time it is quoted.
3. Open the Gerber viewer and check what the README lists (outline, holes,
   cut lines, polarity marks).
4. PCB Assembly on → Economic → Top Side → PCBA Qty 5 or 2 (2-of-5 assembles
   two and ships five). Tooling holes: "Added by JLCPCB" unless the README
   says otherwise.
5. Upload BOM and CPL, Process. Every line must match (25/25 on the
   integrated carrier). The `Lib Type` column says Basic or Extended.
6. In the placement preview, zoom on every polarised and pin-1 part; JLCPCB's
   pink dot must land on the silkscreen mark. The rotation offsets come from
   `docs/reference/jlcpcb-rotations.toml` (`carrier-pcb` trap 32).
7. Quote & Order: Product Description is a cascading menu; choose
   "Reserch\Education\DIY\Entertainment" → "DIY Hobby Circuit Board - HS Code
   902300". Save to Cart, then verify the cart (step 2).

## 4. Where the money goes

| Charge | Observed | Lever |
|---|---|---|
| Setup + stencil | €7.33 + €1.38 per PCBA order | none |
| Extended parts | ≈€2.75 (USD 3) per Extended BOM line | Basic or Preferred parts |
| Components | per part, with JLCPCB's minimum order quantity | none for ICs |
| PCBA 2 vs 5 | components and SMT only: +€29.73 on the integrated carrier, +$3.20 on the socketed one | assemble 2 when parts are dear |
| Shipping | DHL Express €23.94 at 0.98 kg, €30.83 at 1.54 kg | Global Standard Direct Line (€19.73, 13–16 days) only up to 1.5 kg / $150 |

**Preferred Extended parts carry no loading fee** (JLCPCB PCB Assembly FAQ),
and they look like Extended in the BOM table. Check every line's type with
the parts API (curl's User-Agent, as `carrier-pcb` trap 33):

```sh
curl -s -A 'curl/8.7.1' -H 'Content-Type: application/json' -X POST https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList -d '{"keyword":"C8020","currentPage":1,"pageSize":5}' | jq '.data.componentPageInfo.list[] | {componentCode, componentLibraryType, preferredComponentFlag, stockCount}'
```

`componentLibraryType` is `base` (Basic) or `expand` (Extended);
`preferredComponentFlag: true` marks fee-free Preferred. Adding
`"componentLibraryType":"base"` to the body filters to Basic; the API ignores
a Preferred filter. On 2026-10-22 there was no Basic tantalum above 10 µF, no
Basic aluminium electrolytic, PTC fuse or power inductor, and one Basic 2 A
ferrite (C14709, 0603).

### VAT, duty and shipping appear only at checkout

**Every price before checkout excludes VAT.** The cart and quote pages show
none; the checkout summary adds it as "Customs duties & taxes". Compare
JLCPCB with a VAT-inclusive shop (AliExpress shows "Price includes VAT") only
after adding it. On 2026-10-23 the cart of €109.35 merchandise checked out as:

| Shipping (checkout, 1.78 kg) | Ship | VAT | Duty + fee | Grand total |
|---|---|---|---|---|
| IOSS DHL Express, 2–4 days | €32.29 | €36.12 | at delivery, see below | €177.76 |
| IOSS FedEx Express, 5–8 days | €44.33 | €39.19 | at delivery | €192.87 |
| IOSS UPS Express Saver, 3–6 days | €39.37 | €37.93 | at delivery | €186.65 |
| DDP DHL Express, 2–4 days | €32.78 | €39.29 | €8.96 duty + €7.36 disbursement | €197.74 |
| FCA, own freight account | — | buyer's | buyer's | — |

The VAT is 25.5 % (Finland) of goods plus shipping: €36.12 = 0.255 ×
(€109.35 + €32.29). Global Standard Direct Line was not offered at 1.78 kg.

**Since 1 July 2026 the EU charges customs duty on parcels under €150**
(JLCPCB's checkout note). IOSS options prepay only the VAT; the carrier
collects the duty and its own disbursement fee at delivery, so the IOSS
grand total understates the cost. DDP prepays all of it. Pick between them
on the delivered total, not the grand total.

Coupons are applied in the payment step, after Submit Order. Whether the
VAT is recomputed on the discounted amount was not checked.

## 5. Coupons

- **One coupon per order**, chosen in the cart's Coupons panel (picking one
  deselects the other) and again in the payment step after Submit Order.
- **A coupon only discounts its own category:** PCB, SMT (PCBA), 3DP, Stencil,
  or ALL (the New User Shipping coupon, on shipping over €13.35). A €26.70 PCB
  coupon took €5.34 off an order whose PCBs cost €5.34.
- **Splitting a cart into two orders buys a second coupon.** On 2026-10-23 the
  carriers (order A, €43.00, 1.46 kg) and the integrated board (order B,
  €79.21, 0.90 kg) cost €61.26 + €88.79 with one €8.90 coupon each, against
  €147.21 together with the €17.80 PCBA coupon. Order B over €88.99 would
  take the €17.80 one itself. The cart's ticked lines are what it prices, so
  tick each order's lines to read its shipping and eligible coupons; each
  parcel under €150 also pays EU duty and a carrier handling fee (§ 4).
- **The cart changes the selection by itself:** after a reload it picked
  "PCBA $10 - EasyEDA" and once "PCB $10 - EasyEDA" over the larger JLCONE
  PCBA $20, and with JLC3DP items in the cart a €8.90 3D-printing coupon. Re-check the panel
  immediately before checkout.
- JLCONE PCBA $20 (€17.78) needs over €88.91 excluding freight, counted over
  the whole cart; its 30 days start when claimed (Coupon Center → PCBA → Get
  Coupon). Claim it only when the order will cross the threshold.
- Third-party "promo code" sites list codes JLCPCB does not publish; the cart
  has no code field before checkout. Treat them as unverified.

## 6. Hand-off

There is no cart export; `cart.js` gives the lines. Write
`tmp/quotes/jlcpcb-cart-<date>.csv`: one row per cart line (JLC reference;
quantity; build time; EUR price; the commit each board came from; the fab
zip's sha256 prefix; BOM and CPL paths), plus summary rows for merchandise,
shipping, coupon and subtotal, and a row for the checkout's VAT once it has
been read. Then tell Lauri the total (saying whether it includes VAT), the
coupon and shipping to re-check at checkout, and anything still open on a
board.

## 7. 3D-printed cases (JLC3DP)

Cases come from `just enclosure::gen` (the `enclosure-cad` skill), which must
exit 0 first. Quote in JLCONE (Products → Mechanical Services → "JLC3DP - 3D
Printing" → Get Instant Quote); items land in the same cart under a JLC3DP
tab. The trays of 2026-10-23 were quoted in Chrome
(`https://jlc3dp.com/3d-printing-quote`), the boxes in JLCONE; the product
description and volume checks held in both.

- **In JLCONE, select every STL at once** (Cmd+A in the file picker): all 8
  box parts loaded in one go on 2026-10-23. In Chrome a multi-file selection
  registered nothing, so there upload one at a time. A quote page holds at most
  10 models; a further file is dropped without a message.
- The "Total Price" can lag the line prices while models load (it read €7.10
  against €12.86 of lines until Batch Edit was saved); check it against their
  sum before saving.
- The quoted volume should equal `report.json`'s `volume_cm3`; a mismatch means
  a stale STL.
- **Save to Cart needs a product description per item** ("Please select
  product desc"): Batch Edit → Product Desc → "Enclosure, Block, Plate,
  Cylinder" → "Plastic Enclosure - HS Code 392690". Batch Edit also holds the
  material choice (SLA resins, MJF/SLS nylon, FDM, metal) that the per-item
  editor would not open.
- The "Printing risk / Thin walls detected" flag is a heuristic: trust the
  enclosure gate's measurement, and say which flag remains.
- **The cases ride the PCB parcel**: four trays added €0.03 to DHL (1.54 →
  1.78 kg). The cart then offered "New User 3D Printing" coupons (€8.90); still
  one coupon per order, so re-select the larger PCBA one.

## Other fabs (for comparison)

Quoted the same day for the integrated carrier (5 PCBs, 2 assembled):

| Fab | Instant quote | Notes |
|---|---|---|
| JLCPCB | €74.91 all-in | shares shipping and coupon with the rest of the cart |
| NextPCB | $111.45 + $15.70 DHL | promo already applied; centroid must be zip/rar/xls(x), not csv; 11–16 working days |
| PCBWay | $5 PCB; $29 turnkey labour + parts after manual BOM review; $33.16 DHL | −$30 new-customer discount shown |
| Seeed Fusion | $9.90 PCB; PCBA needs an account | $50 off PCBA for new customers, free express shipping with PCBA |

Non-JLC sites match on manufacturer part numbers: build an MPN BOM from the
parts API's `componentModelEn` and `componentBrandEn` (blank for some parts;
fill those from the datasheet).

## Related

- `carrier-pcb` skill: generating and checking the files; traps 32 and 33
- `.claude/agents/pcb-reviewer.md`: the pre-order review
- Each carrier README's Ordering and pre-order-check sections
