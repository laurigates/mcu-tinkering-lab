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

Driving it with computer-use: `request_access` for `com.jlcpcb.www` (granted
at full tier). Pages open as tabs inside the app; the cart is the cart icon,
other services are under Products (JLC3DP: Products → Mechanical Services →
"JLC3DP - 3D Printing"). The upload buttons open a native file picker; press
Cmd+Shift+G, type the absolute path, Return, then Open.

Fall back to the web in Chrome only when Lauri asks for it. There the
`file_upload` tool works on the Gerber input; the PCBA BOM and CPL inputs are
hidden, so tag them first
(`document.querySelectorAll('input[type=file]').forEach((e,i)=>e.setAttribute('data-claude-file',i))`)
and `find` them by that attribute.

**The web session expires mid-flow, and Save to Cart then silently does
nothing.** The header shows "Sign In" instead of "My Account". Two quotes were
lost that way in Chrome. After every Save to Cart, in either app or browser,
open the cart and confirm the line is there. Signing in is Lauri's step,
never the agent's.

## 3. The quote, step by step

1. Upload the zip. Compare "Detected N layer board of W×H" with the job file.
2. Leave the defaults the README asks for (FR-4, 1.6 mm, HASL, 1 oz,
   Different Design 1). Minimum via 0.3 mm matches the carriers' smallest
   drill; check `grep -E '^T[0-9]+C' output/fab/*.drl`.
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

- **One coupon per order**, chosen in the cart's Coupons panel.
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

There is no cart export. Write `tmp/quotes/jlcpcb-cart-<date>.csv`: one row
per cart line (JLC reference as shown, truncated if the page truncates it;
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
(`https://jlc3dp.com/3d-printing-quote`); the notes below come from that run
and are not yet re-checked in the app.

- **Upload one STL at a time.** A multi-file selection registered nothing, and
  a quote page holds at most 10 models; a further file is dropped without a
  message.
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
