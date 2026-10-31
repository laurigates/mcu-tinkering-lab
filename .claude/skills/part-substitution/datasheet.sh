#!/usr/bin/env bash
# Turn a datasheet PDF into text and page images you can read and quote.
#
#   .claude/skills/part-substitution/datasheet.sh <pdf> [out_dir] [pages]
#
# Writes <out_dir>/<stem>.txt (pdftotext -layout) and <out_dir>/<stem>-<n>.png
# for each page in `pages` (default 1: the pin-out drawing is usually there, and
# it is an image, not text). Then prints the document number and revision, and
# the lines that hold absolute-maximum and electrical ratings, for a first read.
# out_dir defaults to tmp/datasheets (git-ignored). Needs poppler
# (brew install poppler).
set -euo pipefail

pdf="${1:?usage: datasheet.sh <pdf> [out_dir] [pages, e.g. 1 or 1-3]}"
out="${2:-tmp/datasheets}"
pages="${3:-1}"
[[ -f "$pdf" ]] || { echo "datasheet.sh: no such file: $pdf" >&2; exit 1; }
head -c 5 "$pdf" | grep -q '^%PDF' || { echo "datasheet.sh: not a PDF: $pdf" >&2; exit 1; }
command -v pdftotext >/dev/null || { echo "datasheet.sh: pdftotext missing (brew install poppler)" >&2; exit 1; }
command -v pdftoppm >/dev/null || { echo "datasheet.sh: pdftoppm missing (brew install poppler)" >&2; exit 1; }

mkdir -p "$out"
stem="$(basename "${pdf%.*}")"
pdftotext -layout "$pdf" "$out/$stem.txt"
first="${pages%%-*}"
last="${pages##*-}"
pdftoppm -f "$first" -l "$last" -r 110 -png "$pdf" "$out/$stem"

echo "text:   $out/$stem.txt"
ls "$out/$stem"-*.png 2>/dev/null | sed 's/^/image:  /'
echo "--- document"
{ grep -oE '(Document [Nn]umber|DS[0-9]{4,})[^|]{0,40}' "$out/$stem.txt" | sed 's/ *$//' | sort -u | head -3; } || echo "(no document number found; read page 1)"
echo "--- ratings (first look; read the tables before quoting)"
grep -nE 'Voltage|Current|Resistance|R ?DS|Capacitance|Threshold|Power Dissipation|Thermal|Hold|Trip|Saturation|Tolerance' "$out/$stem.txt" | head -40 || true
