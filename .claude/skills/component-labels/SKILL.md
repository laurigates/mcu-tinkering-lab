---
name: component-labels
description: Print and kiss-cut sticker labels for the parts drawers and bins, from tools/labels data files, on the Epson ET-3750 and the Silhouette Portrait 2 (AutoBlade), all from the command line. Use when labelling new stock, adding labels for parts-stock items, reprinting a sheet, or tuning the cutter's depth and force.
argument-hint: <data file, or "new stock">
allowed-tools: Read, Write, Edit, Bash
---

# Component Labels: Print, Then Kiss-Cut

`tools/labels/` turns a TOML data file into A4 sticker pages with Silhouette
registration marks, and a matching cut file per page. Print on the Epson, put
the sheet on the Portrait 2's mat, and the cutter finds the marks and
kiss-cuts each sticker's outline through the face, not the liner. No Silhouette
Studio and no Inkscape GUI: the cut goes through
[inkscape-silhouette](https://github.com/fablabnbg/inkscape-silhouette)'s
`sendto_silhouette.py`, headless.

| Sticker | Size | For |
|---|---|---|
| `[[label]]` | 35 × 35 mm | divided containers: 2.5 mm inside a 40 mm face, clear of the raised rim |
| `lid = {…}` on a label | 18 × 18 mm | 25 × 30 × 20 mm flip-lid SMD boxes: 1 mm inside a 20 mm face |
| `[[strip]]` | 80 × 20 mm | bins of larger things |

## Workflow

```sh
just labels::setup                                   # once: driver + venv in ~/.cache/inkscape-silhouette
just labels::candidates > tmp/labels/new.toml        # draft labels for unlabelled parts-stock items
# review the drafts, move the ones to print into a data file under tools/labels/data/
just labels::check tools/labels/data/<file>.toml     # validate and lay out
just labels::build tools/labels/data/<file>.toml     # tmp/labels/<file>/: labels.pdf, cut-p<N>.svg, page-<N>.png
just labels::print tools/labels/data/<file>.toml     # A4, 100 %, high quality, to the Epson
just labels::test-cut 1 tools/labels/data/<file>.toml   # cut only the first sticker of page 1
just labels::cut 1 tools/labels/data/<file>.toml        # cut page 1
```

The drafts are raw material, not a data file. They carry no `schema = 1` or
`[colors]` table, so `check` rejects the file as written, and their value
and spec are the listing title, too long for a 35 mm sticker. Rewrite each
one: a short value, two spec lines, a colour, and a `render` where a kind
fits. Items whose parts-stock quantity is 0 or level `out` on a recent order
are usually still in transit; put them in a separate `<file>-incoming.toml`
and print it once the parts are in hand.

Look at `page-<N>.png` before printing (`just labels::build <file> true` draws
the cut outlines on the previews). A dry run checks a cut job with no cutter
attached: `just labels::cut 1 <file> true` writes `cut-p1.cmd`, the command
stream a Portrait 2 would receive, and `cut-p1.log`.

## Data

A data file has `schema = 1`, a `[colors]` table of named `#rrggbb` colours,
`[[label]]` entries and `[[strip]]` entries. See
`tools/labels/data/drawers-2026-10.toml`. A label:

```toml
[[label]]
cat = "MLCC capacitor"            # header band
color = "cap-blue"                # a [colors] key, or #rrggbb
value = "100 nF"
spec = ["0805 (2012 metric) · 50 V · X7R", "Unmarked"]
foot = "100 pcs · AliExpress 09/2026"
lid = { spec = "0805 · 50 V · X7R" }      # optional 18 mm lid label; value defaults to the label's
render = { kind = "chip", l = 2.0, w = 1.25, term = 0.45 }   # optional drawing (render.typ)
stock = "c0805-100n-50v-x7r"      # the parts-stock slug; candidates skips linked items
```

Render kinds and their keys are in `tools/labels/render.typ`; `labels.py`
rejects a kind it does not list. Packages with datasheet dimensions (`chip`,
`tant`, `polymer`, `sop8`, `usbc`, `dip8`, `led`) also get a 1:1 outline; set
`actual = false` to drop it. Module drawings are generic, not to scale.

## Parts-stock

`candidates` reads `~/repos/laurigates/parts-stock` (or `$PARTS_STOCK`, or
`stock=<dir>`) read-only and drafts a label for every item no label names with
`stock = "<slug>"`. Never write into parts-stock from here; it is a private repo
holding order ids and prices. This repo is public, so carry over only label
text: no order ids, listing ids or prices.

## Cutter settings

`cut` and `test-cut` take `depth` (AutoBlade, 0–10, default 1), `pressure`
(1–33, 0 = media default) and `speed` (1–10, 0 = media default). The media is
inkscape-silhouette's "Sticker Sheet" preset (134: force 20, speed 10, depth 1).
Start every new sticker stock with `test-cut`: the first sticker should peel
off cleanly with the liner unmarked. If the face does not part, raise
`pressure` before `depth`; if the liner is scored through, lower `depth`.
Record the settings that worked in the table below.

| Sticker stock | depth | pressure | speed | Result |
|---|---|---|---|---|
| (not yet cut) | | | | |

## Traps

1. **Print at 100 % on A4.** The Epson queue defaults to Letter. `labels::print`
   passes `media=A4` and `print-scaling=none`; printing from a viewer with "fit
   to page" moves the marks and every cut lands off its sticker.
2. **The marks own the corners.** The 5 mm square and the 20 mm L marks at the
   top-left, top-right and bottom-left take a 24 mm square each; the layout
   keeps stickers 2 mm clear, so the first row and the last start at x = 32.
3. **A plain driver dry run never reaches the cut.** With `--dry_run=True` the
   driver reads nothing back, and a job with marks waits for the sensor's
   "found" reply. `cut.sh` dry runs go through `dry_cut.py`, which answers that
   one reply. The AutoBlade depth (`TF`) command is absent from a dry-run
   transcript: the driver sets depth only after the real cutter reports an
   AutoBlade in holder 1.
4. **inkex from PyPI drags in PyGObject**, which needs cairo and
   gobject-introspection to build and which the driver never imports.
   `setup-cutter.sh` installs inkex with `--no-deps` and lists its real
   runtime deps, including `libusb1`, which the driver's `requirements.txt`
   omits but its macOS import needs (`brew install libusb` too).
5. **Spec lines from memory.** Some lines in the first data file were written
   from memory, not read from a listing; they carry a `# from memory` comment.
   Check those against a datasheet before reprinting them.
6. **Kits hide their values.** The film-capacitor kit's value list is only in
   the listing's images and was never read; its label shows one example code.
7. **A new sticker kind starts a new row.** Labels, then lids, then strips,
   each on fresh rows, so a few strips after a full page of labels can take
   a whole sheet alone (inventory-2026-11 page 4: three strips). Move them to
   a file with room to spare, or hold them for the next run.
8. **Text inside a drawing vanishes when the drawing is small.** `render.typ`
   draws a mark only above a size threshold, so a new kind can look fine in
   isolation and print blank on the sticker. Crop the preview
   (`magick page-<N>.png -crop …`) and read it at sticker size.
