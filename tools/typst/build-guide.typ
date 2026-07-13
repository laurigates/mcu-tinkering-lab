// Shared Typst template for MCU Tinkering Lab printable build guides.
//
// Import from a project guide (compile with the repo root as --root so the
// import and any shared images resolve inside the Typst sandbox):
//
//   #import "../../../../tools/typst/build-guide.typ": guide, callout, htable, theme
//   #show: guide.with(
//     title: "robocar-unified",
//     subtitle: "Single-Board AI Robot Car",
//     tagline: "Assembly, Wiring & Firmware Build Guide",
//     version: "0.1.2",
//     meta: (
//       ("Target MCU", "ESP32-S3 (8 MB PSRAM / flash)"),
//       ("Toolchain", "ESP-IDF v5.4 (containerized)"),
//     ),
//     header-right: "XIAO ESP32-S3 Sense",
//     footer-note: [Pin data mirrors `main/pin_config.h`, which is authoritative.],
//   )
//   = 1 · Overview
//   ...
//
// Styling lives here so every guide improves together — edit this file, then
// recompile each project's guide.

// ---- Theme ------------------------------------------------------------------
#let theme = (
  accent: rgb("#1f6feb"),
  accent-soft: rgb("#e8f0fe"),
  ink: rgb("#1a1a1a"),
  muted: rgb("#6a737d"),
  rule: rgb("#d0d7de"),
  zebra: rgb("#f6f8fa"),
  // semantic callout colors
  info: rgb("#1f6feb"),
  info-soft: rgb("#e8f0fe"),
  ok: rgb("#1a7f37"),
  ok-soft: rgb("#eafbef"),
  warn: rgb("#bf8700"),
  warn-soft: rgb("#fff8e1"),
  danger: rgb("#cf222e"),
  danger-soft: rgb("#ffebe9"),
  purple: rgb("#8250df"),
  purple-soft: rgb("#f3eefc"),
)

// ---- Callout box ------------------------------------------------------------
// A soft, left-barred box. `kind` picks a semantic color pair, or pass
// explicit `bar` / `fill` to override.
#let callout(title, body, kind: "info", bar: none, fill: none) = {
  let bar-color = if bar != none { bar } else { theme.at(kind, default: theme.info) }
  let fill-color = if fill != none { fill } else { theme.at(kind + "-soft", default: theme.info-soft) }
  block(
    width: 100%,
    fill: fill-color,
    inset: (x: 10pt, y: 8pt),
    radius: 4pt,
    stroke: (left: 3pt + bar-color),
  )[
    #text(weight: "bold", fill: bar-color.darken(15%))[#title] \
    #body
  ]
}

// ---- Header-styled table ----------------------------------------------------
// White-on-accent header row, zebra body. `header` is an array of cells; each
// remaining positional arg is a row (array of cells). `aligns` optional.
#let htable(cols, header, ..rows, aligns: none) = table(
  columns: cols,
  align: if aligns == none { left } else { aligns },
  stroke: none,
  fill: (_, y) => if y == 0 { theme.accent } else if calc.odd(y) { theme.zebra } else { white },
  inset: (x: 7pt, y: 5pt),
  table.header(..header.map(c => text(fill: white, weight: "bold")[#c])),
  ..rows.pos().flatten(),
)

// ---- Document wrapper -------------------------------------------------------
// Use as `#show: guide.with(...)`. Applies page/text/heading styling and emits
// a title page + table of contents, then renders the document body.
#let guide(
  title: "",
  subtitle: "",
  tagline: "Assembly, Wiring & Firmware Build Guide",
  intro: none,
  version: none,
  meta: (),
  difficulty: none,
  header-right: none,
  footer-left: "MCU Tinkering Lab",
  footer-note: none,
  lab: "MCU TINKERING LAB",
  body,
) = {
  let accent = theme.accent
  let muted = theme.muted
  let rule = theme.rule

  set document(title: title + " Build Guide", author: "MCU Tinkering Lab")
  set page(
    paper: "a4",
    margin: (top: 2.2cm, bottom: 2cm, x: 1.9cm),
    header: context {
      if counter(page).get().first() > 1 {
        set text(9pt, fill: muted)
        grid(columns: (1fr, 1fr),
          align(left)[#title — Build Guide],
          align(right)[#{ if header-right != none [#header-right] }#{ if version != none [ · v#version] }],
        )
        line(length: 100%, stroke: 0.5pt + rule)
      }
    },
    footer: context {
      set text(9pt, fill: muted)
      grid(columns: (1fr, 1fr),
        align(left)[#footer-left],
        align(right)[#counter(page).display("1 / 1", both: true)],
      )
    },
  )

  set text(font: ("Libertinus Serif", "DejaVu Serif"), size: 10.5pt, fill: theme.ink)
  set par(justify: true, leading: 0.62em)
  show heading: set text(fill: theme.ink)
  show heading.where(level: 1): it => {
    v(0.4em)
    block(text(16pt, weight: "bold", fill: accent)[#it.body])
    v(0.15em)
    line(length: 100%, stroke: 1pt + accent)
    v(0.35em)
  }
  show heading.where(level: 2): it => {
    v(0.35em)
    block(text(12.5pt, weight: "bold")[#it.body])
    v(0.1em)
  }
  show heading.where(level: 3): it => block(text(11pt, weight: "bold", fill: accent.darken(10%))[#it.body])

  // -- Title page --
  v(3cm)
  align(center)[
    #text(11pt, fill: muted, tracking: 3pt)[#lab]
    #v(0.6cm)
    #text(30pt, weight: "bold", fill: accent)[#title]
    #v(0.15cm)
    #if subtitle != "" { text(16pt, weight: "bold")[#subtitle] }
    #v(0.3cm)
    #text(12pt, fill: muted)[#tagline]
    #if intro != none {
      v(1.2cm)
      box(width: 80%)[
        #set text(10.5pt)
        #set par(justify: false)
        #intro
      ]
    }
    #{
      let rows = meta
      if version != none { rows = (("Firmware version", strong[v#version]),) + rows }
      if difficulty != none { rows = rows + (("Difficulty", difficulty),) }
      if rows.len() > 0 {
        v(1cm)
        box(fill: theme.accent-soft, inset: 10pt, radius: 5pt)[
          #grid(columns: (auto, auto), column-gutter: 1.2cm, row-gutter: 5pt,
            ..rows.map(((k, v)) => (align(right)[#text(fill: muted)[#k]], [#v])).flatten()
          )
        ]
      }
    }
  ]
  if footer-note != none {
    v(1fr)
    align(center)[#text(9pt, fill: muted)[#footer-note]]
  }
  pagebreak()

  outline(title: [Contents], indent: 1em, depth: 2)
  pagebreak()

  body
}
