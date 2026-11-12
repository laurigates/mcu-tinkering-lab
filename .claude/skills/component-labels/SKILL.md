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
the cut outlines on the previews). A dry run checks a cut job without touching
the cutter: `just labels::cut 1 <file> true` writes `cut-p1.cmd`, the command
stream a Portrait 2 would receive, and `cut-p1.log`. It never opens the cutter,
so the USB cable can stay plugged in (trap 3).

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
(1–33, default 10; 0 = media default) and `speed` (1–10, 0 = media default).
The media is inkscape-silhouette's "Sticker Sheet" preset (134: force 20,
speed 10, depth 1). Start every new sticker stock with `test-cut`: the first
sticker should peel off cleanly with the liner unmarked. If the face does not
part, raise `pressure` before `depth`; if the liner is scored through, lower
`depth`. Record the settings that worked in the table below.

`test-cut` cuts sticker 1 only, so a second `test-cut` on the same sheet
passes over the same outline again and says nothing about one pass. Two
no-cut probes come first and cost no sticker:

```sh
just labels::blade 1                                   # set the depth, then read the blade's window
just labels::probe-marks tools/labels/data/<file>.toml # mark search only, on the loaded sheet
```

`blade` confirms the depth command lands: the number in the AutoBlade's
window should match. `probe-marks` reports `found` or the cutter's reason
code (`top`, `left` arguments move where the search starts). Both follow `LABEL_CUTTER`;
they were used over Bluetooth LE and are untested over USB.

| Sticker stock | depth | pressure | speed | Result |
|---|---|---|---|---|
| A4 sticker sheet of the 2026-11 inventory run | 1 | 20 (the preset) | 0 (preset: 10) | Three pages cut, 2026-11-11. Stickers peel, but under some cuts the liner is cut part-way and tears when a sticker is removed: slightly too much force |
| the same stock | 1 | 10 | 0 (preset: 10) | Not yet cut. The default since that run; start the next sheet with `test-cut` |

### Where the mark search starts

The driver starts the mark search at the machine's origin and makes no
allowance for the mat's border. On the Portrait 2 the search then gave up
before the sheet had fed in to the square (reply `1`, "Couldn't find
registration marks"). `cut.sh` starts it 20 mm further in, which found the
marks, and the test cut landed on the sticker (2026-11-11, one sheet).
`LABEL_MARK_TOP` and `LABEL_MARK_LEFT` (mm) move the start along and across
the feed if a sheet sits elsewhere on the mat; they do not move the cut.

A reply of `-1` means the search started too far in. The driver's default
ends a job by feeding to the bottom of the cut and making that the new
origin, so each job left the mat further in and the second or third job on a
sheet failed, until the mat was fed again. `cut.sh` passes
`--endposition=start`, which returns the mat to where the job began; a search
straight after a cut then found the marks. If `-1` appears anyway, unload and
load the mat.

## Bluetooth

`cut` and `test-cut` use USB unless `LABEL_CUTTER` is exported. Over Bluetooth
LE, which needs no pairing:

```sh
just labels::scan                                  # every BLE device in range; find the cutter
LABEL_CUTTER=ble just labels::test-cut 1 tools/labels/data/<file>.toml
LABEL_CUTTER=ble just labels::cut 1 tools/labels/data/<file>.toml
```

The cutter is picked by advertised name, `PORTRAIT2` by default (the name the
driver's model table has for it); a substring is enough. Set
`LABEL_CUTTER_NAME` if the scan shows another name, or `LABEL_CUTTER_ID` to
the scan's local identifier when two cutters match. `LABEL_CUTTER=bluetooth`
with `LABEL_CUTTER_ADDR=<MAC>` is Bluetooth Classic, which the driver has on
Linux and Windows only.

The Portrait 2 advertises over BLE as `Portrait2-<6 hex digits>` (scan on
macOS, 2026-11-11), which the default name matches. Three pages were cut
over BLE that day, mark search included, each in about two minutes. The
cutter was paired in macOS Bluetooth settings by then; the one connection
attempted before pairing was dropped during the handshake, and whether
pairing is required was not tested.

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
   one reply. The driver also probes USB in a dry run: with a cutter plugged
   in it opens it, sends it the status and firmware queries, and reads the
   mark-search reply from it, so the dry run stops at the marks (#1104).
   `dry_cut.py` makes that probe find nothing, the driver's own path when no
   cutter is attached, so the transcript is the same plugged in or not. The
   plain driver's `--dry_run=True`, run outside `cut.sh`, still needs the
   cable unplugged.
   **The driver never sends the blade depth to a Portrait 2.** It sends the
   AutoBlade depth command (`TF`) only to the Cameo 3 and Cameo 4 lines, so
   `--depth` was dropped and the blade stayed at whatever depth it was last
   set to: the first test cut went through the liner. `autoblade.py` sends
   it, live and in a dry run, as `J1` (tool select) then `TF<depth>,1`. The
   `J1` matters: `TF1,1` on its own moved the blade but left the indicator
   on 9, then 6, while `J1 TF<d>,1` put it on 1, 5 and 8 when asked for
   each. A transcript without `J1` and `TF<depth>,1` before the mark commands
   means the depth is not being set. The cutter takes about 10 s to adjust
   the blade, and the number in the AutoBlade's window shows where it
   landed: read it after a test cut.
4. **inkex from PyPI drags in PyGObject**, which needs cairo and
   gobject-introspection to build and which the driver never imports.
   `setup-cutter.sh` installs inkex with `--no-deps` and lists its real
   runtime deps, including `libusb1`, which the driver's `requirements.txt`
   omits but its macOS import needs (`brew install libusb` too), and `bleak`
   for Bluetooth LE. A venv set up before `bleak` was added needs
   `just labels::setup` again, or the scan reports the package missing.
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
