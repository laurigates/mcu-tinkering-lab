---
paths:
  - 'packages/**/hardware.toml'
  - 'packages/**/hardware/**'
  - 'docs/reference/parts/**'
---
# Every Part on a Carrier Needs a Source Before the Board Is Ordered

A carrier is only as buildable as its least available part. ERC, DRC, parity
and the board check say nothing about whether a part can be bought, so a board
can pass every gate and still arrive with footprints nobody can fill.

> 2026-10-23: the balancebot carrier was ordered bare, so its 11 SMD parts were
> left to hand-sourcing. Two had no usable AliExpress listing on 2026-10-30: the ZXMP4A16G
> P-FET (Q1) is not listed at all, and every 1812 3 A PTC there is unbranded,
> most with no voltage rating (the design needs the 24 V BSMD1812-300-24V).
> Q1 became the ZXMP3A16G after a datasheet comparison on 2026-10-30, and F1 a PTC whose hold
> and trip currents nobody can state. Both parts were on JLCPCB's assembly BOM
> with LCSC numbers, and that order would have placed them.

## When choosing a part

Prefer, in this order:

1. **A part Lauri already has.** Check the AliExpress order history
   (`aliexpress-parts` skill) and the "Electronics Purchases" artifact, which
   also holds the eBay history. The XIAO boards come from Seeed and are in
   neither.
2. **A part JLCPCB can place**: in stock at LCSC, Basic if one fits the
   datasheet numbers, Extended only when no Basic part does.
3. **A part AliExpress sells under its exact part number**, from a listing that
   ships to Finland, with a variant naming the value, package and rating the
   design needs.

And for every part:

- **Current, not superseded.** The manufacturer lists it as active, not NRND,
  EOL or obsolete. For a breakout or module, design to the revision on sale
  now; an older board revision can have a different pin order
  (`board-layout-from-vendor-files.md`).
- **Bought by its datasheet numbers, not its name.** A generic listing that
  omits a rating the design depends on (a PTC's voltage and hold current, an
  inductor's saturation current) is not a source for that part
  (`aliexpress-parts` trap 6, `power-stage-checklist.md`).
- **Record the source** as `buy` on the part's registry entry or its
  `hardware.toml` instance (`aliexpress:<id>`, `lcsc:C<n>`, `owned:<what>`),
  separate from `lcsc`, which only says JLCPCB can place it. A part in a
  carrier's BOM with neither fails `kicad.sourcing` in CI
  (`docs/reference/parts/README.md` § Where a part comes from).

## Before ordering a board

`just kicad::sourcing <project>` lists the parts with an LCSC number and no
`buy`. Those are the parts only JLCPCB or LCSC can supply. For each SMD part,
decide who places it and check that it can be bought:

- **JLCPCB places it**: it is on the PCBA BOM with an LCSC number that is in
  stock today.
- **Hand-soldered**: it is owned, or a checked AliExpress listing sells it
  (exact part number, ships to Finland, rating stated).

A part with neither stays on the PCBA BOM, even when the rest of the board is
hand-soldered: Economic PCBA places a partial BOM, and the other footprints
stay empty. If the board ships bare anyway, order that part from LCSC in the
same order. Through-hole parts and connectors get the same check before the
order is paid.

## Related

- `aliexpress-parts` skill: owned parts, listing checks, the BOM sourcing check
- `part-substitution` skill: when a part has no source and another must stand in
- `jlcpcb-order` skill § 1: the sourcing gate before uploading
- `carrier-pcb` skill: adding a registry entry
- `power-stage-checklist.md`: the datasheet numbers a substitute must meet
