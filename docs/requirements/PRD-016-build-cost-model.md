# PRD-016: Build Cost Model and Sourcing Planner

**Status**: draft
**Created**: 2026-10-31
**Confidence**: 5/10, self-assessed. The two delivery-charge rules below are
measured on one JLCPCB parcel and one AliExpress checkout. Labour has no
measured time yet. Tariff codes for AliExpress listings are guessed from the
part type, because the checkout does not show them.

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
| JLCPCB W2026102320533303 (carriers, robocar PCBA, 9 cases) | €79.02 invoice | €108.02 | +€29.00 | €9 EU duty (3 tariff codes × €3), €20 DHL processing fee once duty passes €5 (`jlcpcb-order` § 4) |
| AliExpress cart, 25 lines (2026-10-31) | €143.88 estimated total | €231.99 at checkout | +€88.11 | net import charge of about €6.28 per tariff code per parcel (below) |

The cart page, the JLCPCB quote and the AliExpress product page all leave
these charges out. Comparing sources on the quoted price picks the wrong
source. Nothing in the repo puts a price on a whole robot, so there is no
figure to set against labour or a selling price
([PRD-015](PRD-015-teuvo-for-other-owners.md) route C).

## Measured cost rules

These are the rules the model has to reproduce. Each one names the order or
checkout it was measured on. Raw probes:
`tmp/quotes/aliexpress-checkout-probe-2026-10-31.csv` (git-ignored).

| # | Rule | Evidence |
|---|---|---|
| R1 | EU duty on a parcel of €150 or less is €3 per goods item. A goods item is lines with the same tariff code and origin. Runs 1 July 2026 to 1 July 2028 | Council Regulation (EU) 2026/382; Tulli; JLCPCB parcel €9 = 3 codes |
| R2 | DHL Express (Finland) adds €20 when the duty exceeds €5, so two or more tariff codes trigger it | DHL customs notice, invoice DCI0029689 |
| R3 | AliExpress collects duty at checkout: €3 duty + €2 handling fee, + 25.5 % VAT ≈ **€6.28 per tariff code per parcel**. It shows €6.28 per listing, then credits the excess back as "import charge reduction" | 1 line: €6.27–6.31 for items from €1.26 to €11.65; 3 resistor values from one listing: €18.89 gross, €6.35 net; XH 2-pin + 4-pin + a separate XH kit: €6.34 net |
| R4 | Items "shipped by AliExpress" (Choice) form one parcel. Each global-seller store is a separate parcel with its own shipping and its own R3 charge, and gets no reduction | 20 Choice lines: net €56.72 (≈ 9 codes); 5 global stores: €31.39 = 5 × €6.28 |
| R5 | A Choice parcel over €150 still pays R3. With €64 of tools added, the net import charge was exactly the two parcels' sum | €158.51: net €69.31 = €56.72 + €12.59 |
| R6 | Choice shipping is €2.99 alone, free from €10 of Choice items. The checkout charged €6.38 on a 20-line Choice parcel that the cart showed as free | `aliexpress-parts` trap 4; checkout 2026-10-31 |
| R7 | JLCPCB prices exclude VAT until checkout; AliExpress prices include it | `jlcpcb-order` § 4, `aliexpress-parts` trap 9 |
| R8 | DDP shipping at JLCPCB prepays duty with a €7.36 disbursement, against R2's €20 | JLCPCB checkout 2026-10-23, a different cart |

Consequences the planner must express:

- **A part costs its price, plus its share of shipping, plus €6.28 if it adds
  a new tariff code to its parcel.** A €1.26 fuse from its own store delivered
  at €10.83.
- **Splitting an order makes it worse** unless the split removes tariff codes
  from a parcel. Each parcel pays per code again.
- **Moving a global-seller part into the Choice parcel removes its shipping and
  its €6.28** when its tariff code is already in that parcel.

## Goals

- A delivered cost per robot type (balancebot, robocar unified, robocar
  integrated) at N = 1, 5, 20, split into PCB, assembly, parts, cases,
  shipping, import charges, VAT and labour
- Two figures for parts bought in packs: **cash** for this order, and
  **consumed** (what one build used) (`aliexpress-parts`)
- Planning: given a list of needs and candidate sources, list the parcels and
  their delivered total. Report the cheapest plan found and the next cheapest,
  with each plan's assumptions
- Re-pricing a plan from new quotes in one command, so a cart can be iterated
- A check: a plan's predicted total within €1 of the checkout total it is
  compared with (R3 rounding gives about ±€0.05 per code)

## Non-goals

- Ordering, cart edits or payment
- Live prices without a browser session. Quotes come from the existing
  console helpers and the JLCPCB cart export (`cart.js`)
- Accounting, invoicing or VAT returns. Route C of PRD-015 decides if those
  are ever needed

## Functional requirements

- **FR-044** Read each robot's needs from the repo: the carrier BOM
  (`output/assembly/bom-jlc.csv`), the parts registry
  (`docs/reference/parts/*.toml`) and `hardware.toml`. Off-board parts
  (modules, motors, battery, screws, case) come from a per-robot list kept
  beside `hardware.toml`
- **FR-045** Hold sources as data: listing id, variant, pack size, unit price,
  VAT included or not, shipping rule, parcel group (Choice, store id, JLCPCB
  order) and tariff code (6-digit HS). Mark a tariff code as **declared**
  (from an invoice) or **guessed**
- **FR-046** Calculate delivered cost per parcel with R1–R8, each rule as a
  separate, tested function with its evidence in the docstring
- **FR-047** Plan: assign needs to sources and group them into parcels to
  minimise delivered cost. Respect pack sizes and stock already owned (order
  history, `aliexpress-parts`). Report cash and consumed cost
- **FR-048** Unit economics: cost per robot at N, plus labour as hours ×
  rate. Hours start as estimates per step (assembly, flashing, test, case)
  and are replaced by measured times
- **FR-049** Fab comparison: for a carrier, JLCPCB PCBA vs bare PCB plus
  AliExpress parts plus hand assembly, at N, using R2/R8 for the JLCPCB
  parcel
- **FR-050** Calibration: each order or checkout probe goes into a fixture
  with its observed totals. The test suite fails if a rule change moves a
  fixture's predicted total by more than €1. The 2026-10 JLCPCB parcel and
  the 2026-10-31 AliExpress checkout are the first two fixtures
- **FR-051** Output: a table per plan in the terminal, and a CSV in
  `tmp/quotes/` matching the existing cart CSVs

## Open questions

1. **Tariff codes for AliExpress listings.** The checkout never shows them.
   The net charge (R3) gives only the count per parcel. Guessing from the part
   type (8533 resistors, 8541 diodes and transistors, 8536 connectors, 7318
   screws) needs one calibration: does the guessed count reproduce the €56.72
   (≈ 9 codes)?
2. **When R3 stops applying.** R5 showed no change at €158.51. Is AliExpress
   splitting the parcel, or charging the flat rate anyway?
3. **Tulli's handling fee from November 2026.** Its amount and whether it adds
   to R3 are not yet published.
4. **Labour rate and hours.** Which rate to use, and which steps to time on
   the next build.
5. **Where it runs.** A Python package under `tools/`, with `just` recipes like
   `tools/hardware`, is the default.

## Related

- `jlcpcb-order` skill § 4: JLCPCB charges, coupons, VAT and the delivered cost
- `aliexpress-parts` skill: the console helpers that produce quotes, and
  cash versus consumed
- [PRD-013](PRD-013-rapid-hardware-prototyping.md): prototype before a fab
  order; this PRD prices that decision
- [PRD-015](PRD-015-teuvo-for-other-owners.md) route C: selling units
  needs the per-unit figure this tool produces
