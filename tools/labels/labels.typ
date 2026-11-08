// Component sticker sheets for print-and-cut. Draws the stickers where
// tools/labels/labels.py placed them, plus the Silhouette registration marks;
// the cutter finds the marks and kiss-cuts the outline from cut-p<N>.svg.
//
//   just labels::build <data.toml>          (runs labels.py, then this)
//   typst compile --root . tools/labels/labels.typ out.pdf --input plan=/tmp/labels/<name>/plan.json
//
// --input guides=true also prints each cut outline, for cutting by hand.
// Print at 100 % (no "fit to page"): the marks and the cut file are in mm.

#import "render.typ": draw, fit, u

#let plan = json(sys.inputs.at("plan"))
#let guides = sys.inputs.at("guides", default: "false") == "true"

#set page(width: u(plan.page.w), height: u(plan.page.h), margin: 0mm)
#set text(font: ("Helvetica Neue", "Helvetica", "Arial"), size: 7pt)

// Colour bands run this far past the cut line, so a cut that lands a little off
// the print leaves no white sliver at the sticker's edge.
#let bleed = 0.8

#let lines(xs) = xs.join(linebreak())

// Lines at size `max`, shrunk (down to `min`) until the widest fits `width`, so
// a long spec line never wraps into the drawing below it.
#let fitted(xs, width, max, min, ..style) = context {
  let widest = calc.max(..xs.map(l => measure(text(size: max, ..style, l)).width))
  let size = if widest > width { calc.max(min.pt(), max.pt() * width.pt() / widest.pt()) * 1pt } else { max }
  text(size: size, ..style, lines(xs))
}

// 35 x 35 mm container label. k scales the 40 mm design it was drawn as; the
// actual-size outline stays 1:1.
#let label(c, w) = {
  let k = w / 40
  let pad = 2.2
  let head = 4.8
  place(dx: u(-bleed), dy: u(-bleed), rect(width: u(w + 2 * bleed), height: u(head + bleed), fill: rgb(c.color)))
  place(dx: u(pad), box(height: u(head), align(left + horizon,
    text(fill: white, size: 5.4pt, weight: "bold", tracking: 0.06em, upper(c.cat)))))
  place(dx: u(pad), dy: u(6.3), fitted((c.value,), u(w - 2 * pad), 15pt, 9pt, weight: "bold"))
  place(dx: u(pad), dy: u(12.8), fitted(c.spec, u(w - 2 * pad), 5.7pt, 4.4pt))
  let r = c.render
  if r != none {
    let box-h = 10.8
    let one = r.actual
    let one-w = 9.6
    let draw-w = if one { w - 2 * pad - one-w - 0.6 } else { w - 2 * pad }
    place(dx: u(pad), dy: u(19.4), box(width: u(draw-w), height: u(box-h),
      align(center + horizon, draw(r, fit(r, draw-w - 1, box-h - 1)))))
    if one {
      place(dx: u(w - pad - one-w), dy: u(19.4), box(width: u(one-w), height: u(box-h), align(center + horizon,
        stack(dir: ttb, spacing: 0.8mm, align(center, draw(r, 1, sw: 0.15pt)),
          align(center, text(size: 4.2pt, fill: luma(110), "1:1"))))))
    }
  }
  place(bottom + left, dx: u(pad), dy: u(w - 1.3) - 100%, text(size: 4.3pt, fill: luma(115), c.foot))
}

// 18 x 18 mm flip-lid label.
#let lid(c, w) = {
  let pad = 1.1
  let head = 2.7
  place(dx: u(-bleed), dy: u(-bleed), rect(width: u(w + 2 * bleed), height: u(head + bleed), fill: rgb(c.color)))
  place(dx: u(pad), box(height: u(head), align(left + horizon,
    text(fill: white, size: 3.3pt, weight: "bold", tracking: 0.04em, upper(c.cat)))))
  place(dx: u(pad), dy: u(3.5), fitted((c.value,), u(w - 2 * pad), 10pt, 6pt, weight: "bold"))
  place(dx: u(pad), dy: u(8.6), fitted(c.spec, u(w - 2 * pad), 4.4pt, 3.4pt))
  if c.render != none {
    place(dx: u(pad), dy: u(11.9), box(width: u(w - 2 * pad), height: u(4.8),
      align(left + horizon, draw(c.render, fit(c.render, w - 2 * pad - 0.6, 4.6), sw: 0.25pt))))
  }
}

// 80 x 20 mm bin strip.
#let strip(c, w, h) = {
  place(dx: u(-bleed), dy: u(-bleed), rect(width: u(5 + bleed), height: u(h + 2 * bleed), fill: rgb(c.color)))
  place(dx: 8mm, dy: 4mm, text(size: 14pt, weight: "bold", c.title))
  place(dx: 8mm, dy: 11.5mm, block(width: u(w - 10), text(size: 7pt, fill: luma(60), c.detail)))
}

#let sticker(s) = {
  let body = if s.kind == "label" { label(s.content, s.w) }
    else if s.kind == "lid" { lid(s.content, s.w) }
    else { strip(s.content, s.w, s.h) }
  place(dx: u(s.x), dy: u(s.y), box(width: u(s.w), height: u(s.h), {
    body
    if guides {
      place(rect(width: u(s.w), height: u(s.h), radius: u(s.r), stroke: 0.25pt + luma(170)))
    }
  }))
}

// inkscape-silhouette's standard marks: a filled square at the origin, L marks
// at the top-right and bottom-left. labels.py writes the same marks into the
// cut file.
#let regmarks(m) = {
  let line-stroke = u(m.stroke) + black
  place(dx: u(m.x), dy: u(m.y), rect(width: u(m.square), height: u(m.square), fill: black))
  let tr = m.x + m.w
  place(dx: u(tr - m.arm), dy: u(m.y), line(length: u(m.arm), stroke: line-stroke))
  place(dx: u(tr), dy: u(m.y), line(length: u(m.arm), angle: 90deg, stroke: line-stroke))
  let bl = m.y + m.l
  place(dx: u(m.x), dy: u(bl), line(length: u(m.arm), stroke: line-stroke))
  place(dx: u(m.x), dy: u(bl - m.arm), line(length: u(m.arm), angle: 90deg, stroke: line-stroke))
}

#for (i, page) in plan.pages.enumerate() {
  if i > 0 { pagebreak() }
  regmarks(plan.regmarks)
  for s in page { sticker(s) }
  let m = plan.regmarks
  place(dx: u(m.x + m.arm + 4), dy: u(m.y + m.l + 2), text(size: 5.5pt, fill: luma(120),
    [#plan.title · page #(i + 1) of #plan.pages.len() · print at 100 % (no "fit to page") · cut with #("cut-p" + str(i + 1) + ".svg")]))
}
