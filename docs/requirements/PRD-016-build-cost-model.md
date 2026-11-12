# PRD-016: Build Cost Model and Sourcing Planner

**Status**: draft
**Created**: 2026-10-31
**Confidence**: 5/10, self-assessed. The delivery-charge rules below are
measured on one JLCPCB parcel and three AliExpress checkouts, and two of those
checkouts are the data the rules were written from. Labour has no measured time
yet. Tariff codes for AliExpress listings are guessed from the part type,
because the checkout does not show them.

---

## Overview

This PRD covers a tool that answers two questions from the repo's own data:

1. **What does one balancebot or one robocar cost**, delivered to Helsinki,
   at a build quantity of N, with labour on top?
2. **Which way of ordering its parts is cheapest**: which fab, which
   AliExpress listings, which parcels, and when to order?

The tool calculates and plans. It does not order: carts, checkout and payment
stay with Lauri (`jlcpcb-order`, `aliexpress-parts`).

## Problem

Delivery charges that no quote showed added 37–61 % to the last two orders:

| Order | Quoted | Paid or due | Gap | Where the gap came from |
|---|---|---|---|---|
| JLCPCB W2026102320533303 (carriers, robocar PCBA, 9 cases) | €79.02 invoice | €108.02 | +€29.00 | €9 EU duty (4 tariff codes declared, 3 goods items charged), €20 DHL processing fee once duty passes €5 (`jlcpcb-order` § 4) |
| AliExpress cart, 25 lines (2026-10-31) | €143.88 estimated total | €231.99 at checkout | +€88.11 | net import charges only (€56.72 Choice + €31.39 global sellers, below). The €143.88 already contains the €26.77 shipping |

The cart page, the JLCPCB quote and the AliExpress product page all leave
these charges out. Comparing sources on the quoted price picks the wrong
source. Nothing in the repo puts a price on a whole robot, so there is no
figure to set against labour or a selling price
([PRD-015](PRD-015-teuvo-for-other-owners.md) route C).

## Measured cost rules

These are the rules the model has to reproduce. Each one names the order or
checkout it was measured on. Raw probes:
`tmp/quotes/aliexpress-checkout-probe-2026-10-31.csv` (git-ignored). Rows
marked *observation* record what was seen without claiming the rule behind it.

| # | Rule | Evidence |
|---|---|---|
| R1 | EU duty on a parcel of €150 or less is €3 per goods item. A goods item is lines with the same tariff code and origin. Runs 1 July 2026 to 1 July 2028 | Council Regulation (EU) 2026/382; Tulli. The JLCPCB invoice declared four codes (392690, 950300, 853400, 902300) and €9 = three items were charged; which code went uncharged is not shown (`jlcpcb-order` § 4) |
| R2 | DHL Express (Finland) adds €20 when the duty exceeds €5, so two or more charged goods items trigger it | DHL customs notice, invoice DCI0029689 |
| R3 | AliExpress collects duty at checkout: €3 duty + €2 handling fee, + 25.5 % VAT ≈ **€6.28 per tariff code per parcel**. It shows a charge per listing, then credits the excess back as "import charge reduction". Measured nets run up to €0.08 above €6.275 per code | 1 line: €6.27–6.31 for items from €1.26 to €11.65; 3 resistor values from one listing: €18.89 gross, €6.35 net; XH 2-pin + 4-pin + a separate XH kit: €6.34 net |
| R4 | Items "shipped by AliExpress" (Choice) form one parcel. Each global-seller store is a separate parcel with its own shipping and its own import charge, with no reduction shown | 20 Choice lines: net €56.72 (≈ 9 codes); 5 global stores, one line each: €31.39 ≈ 5 × €6.28 |
| R4a | *Observation.* Two variants of one global-seller listing: shipping charged per line, import charged once | HGD fuse 24 V/3 A + 16 V/4 A (2026-10-31): shipping +€3.30, net import unchanged at €56.66. Two different listings with one code from one store is unmeasured |
| R5 | *Observation.* The Choice lines plus €64.32 of tools (€158.51 of goods) paid exactly the two groups' separate net charges | €69.31 = €56.72 + €12.59. Fits both "flat rate above €150" and "split into parcels of €150 or less": open question 2 |
| R6 | Choice shipping is €2.99 for a single item and free once China-warehouse Choice items reach €10. Items fulfilled from the Singapore warehouse (`CAINIAO_FULFILLMENT_STD_SG`) paid €3.19 each on top | `aliexpress-parts` trap 4; SC-001 and solder paste, €3.19 each alone and €6.38 together in every Choice checkout of 2026-10-31. The cart's shipping line already includes it |
| R7 | JLCPCB prices exclude VAT until checkout; AliExpress prices include it. JLCPCB VAT is 25.5 % of goods plus shipping | `jlcpcb-order` § 4, `aliexpress-parts` trap 9 |
| R8 | DDP shipping at JLCPCB prepays duty: €8.96 duty + €7.36 disbursement, against IOSS's R1 duty + R2's €20 at delivery. €8.96 is not a multiple of €3, so R1 does not describe the DDP duty | JLCPCB checkout 2026-10-23, a different cart from the order |
| R9 | JLCPCB shipping is priced by weight per carrier (DHL Express €23.94 at 0.98 kg, €30.83 at 1.54 kg, €32.29 at 1.78 kg) | `jlcpcb-order` § 4 |
| R10 | JLCPCB allows one coupon per order, and a coupon discounts only its own category (PCB, SMT, 3DP, stencil, or ALL for the new-user shipping coupon) | `jlcpcb-order` § 5 |

Consequences the planner must express:

- **A part costs its price, plus its share of shipping, plus €6.28 if it adds
  a new tariff code to its parcel.** A €1.26 fuse from its own store delivered
  at €10.83.
- **Splitting an order makes it worse** unless the split removes tariff codes
  from a parcel. Each parcel pays per code again.
- **Moving a global-seller part into the Choice parcel removes its shipping and
  its €6.28** when its tariff code is already in that parcel, and the item ships
  from the China warehouse (R6).

## Goals

- A delivered cost per robot type (balancebot, robocar socketed, robocar
  integrated) at N = 1, 5, 20, split into PCB, assembly, parts, cases,
  shipping, import charges, VAT and labour. R1 and R3 cover parcels of €150 or
  less; a larger parcel (likely at N = 20) is out of the model until open
  question 2 is answered, and the tool says so instead of pricing it
- Two figures for parts bought in packs: **cash** for this order, and
  **consumed** (what one build used) (`aliexpress-parts`)
- Planning: given a list of needs and candidate sources, list the parcels and
  their delivered total. Report the cheapest plan found and the next cheapest,
  with each plan's assumptions
- Re-pricing a plan from new quotes in one command, so a cart can be iterated
- A check: a plan's predicted total within €1 of the checkout total it is
  compared with, for parcels of up to about ten tariff codes (R3 measures up to
  €0.08 over per code)

## Non-goals

- Ordering, cart edits or payment
- Live prices without a browser session. Quotes come from the existing
  console helpers and the JLCPCB cart export (`cart.js`)
- Accounting, invoicing or VAT returns. Route C of PRD-015 decides if those
  are ever needed

## Functional requirements

- **FR-044** Read each robot's needs from the tracked sources: the project's
  `hardware.toml` (on-board parts, and the modules, motors and battery it
  already declares as `parts.*`) joined with the parts registry
  (`docs/reference/parts/*.toml`), the same join `tools/hardware` and
  `tools/kicad` use. Generated files under a carrier's git-ignored `output/`
  are not an input. Parts `hardware.toml` does not declare (screws, standoffs,
  the printed case, wire) come from a new `[[consumables]]` table in the same file,
  so there is one list per robot; `tools/hardware`'s schema has to accept it
  first
- **FR-045** Hold sources as data: the registry's existing `buy` field for the
  listing, plus variant, pack size, unit price, currency, quote date, VAT
  included or not, shipping rule, lead time, parcel group (Choice warehouse,
  store id, JLCPCB order), tariff code (6-digit HS) and origin country. Mark a
  tariff code as **declared** (from an invoice) or **guessed**
- **FR-046** Calculate delivered cost per parcel with R1–R10, each rule as a
  separate, tested function with its evidence in the docstring
- **FR-047** Plan: assign needs to sources and group them into parcels to
  minimise **cash** for the order, and report consumed cost beside it. Respect
  pack sizes and stock already owned, read from the order history with the
  one-page fetch `aliexpress-parts` requires (paging drops orders, trap 2)
- **FR-048** Unit economics: cost per robot at N, plus labour as hours ×
  rate. Hours start as estimates per step (assembly, flashing, test, case) in
  `docs/reference/build-times.toml`, and measured times replace them there
- **FR-049** Fab comparison: for a carrier, JLCPCB PCBA vs bare PCB plus
  AliExpress parts plus hand assembly, at N, pricing the JLCPCB parcel with
  R1, R2 and R7–R10 for both IOSS and DDP
- **FR-050** Calibration: each order or checkout probe goes into a fixture
  with its observed total. The test suite fails for any fixture where
  `|predicted − observed| > €1`. Fixtures record the observed charged-item
  count where the documents give it (the JLCPCB parcel: three), not a count
  derived from codes. Fixtures are marked **fitted** (used to write the rules:
  the 2026-10 JLCPCB parcel, the 25-line checkout) or **held out** (the
  15-line checkout of 2026-10-31, €138.55, which a hand estimate from these
  rules missed by €6.58 because one more tariff code stayed in the Choice
  parcel than expected). Only held-out fixtures show the rules predict
- **FR-051** Output: a table per plan in the terminal, and
  `tmp/quotes/plan-<date>.csv` with one row per line: need, source (`buy`),
  variant, quantity, pack size, price, parcel, tariff code (declared or
  guessed), share of shipping, share of import charge, cash, consumed
- **FR-052** Rules carry a validity window (R1's 2026-07-01 to 2028-07-01, the
  25.5 % VAT rate, Tulli's announced handling fee) and the tool warns when a
  plan's date falls outside one

## Open questions

1. **Tariff codes for AliExpress listings.** The checkout never shows them.
   The net charge (R3) gives only the count per parcel. Guessing from the part
   type (8533 resistors, 8541 diodes and transistors, 8536 connectors, 7318
   screws) needs calibrating: the 15-line checkout kept one code more than the
   guess predicted.
2. **Parcels over €150.** R5 showed no change at €158.51 of goods. Is
   AliExpress splitting the parcel, or charging the flat rate anyway? For
   JLCPCB above €150, real duty rates apply instead of R1, and the model has no
   rule for them yet.
3. **Tulli's handling fee from November 2026.** Its amount and whether it adds
   to R3 are not yet published.
4. **Labour rate and hours.** Which rate to use, and which steps to time on
   the next build.
5. **Where it runs.** A Python package under `tools/`, with `just` recipes like
   `tools/hardware`, is the default.

## Related

- `jlcpcb-order` skill § 4–5: JLCPCB charges, coupons, VAT and the delivered
  cost
- `aliexpress-parts` skill: the console helpers that produce quotes, and
  cash versus consumed
- [PRD-013](PRD-013-rapid-hardware-prototyping.md): prototype before a fab
  order; this PRD prices that decision
- [PRD-015](PRD-015-teuvo-for-other-owners.md) route C: selling units
  needs the per-unit figure this tool produces
