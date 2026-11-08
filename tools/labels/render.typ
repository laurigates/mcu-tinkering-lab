// Package drawings for the component stickers, top or side view.
// draw(r, s) draws render table r at s drawn mm per real mm; fit(r, w, h) is the
// largest s that fits a w x h mm box (capped at 9). r comes from a data file's
// `render` table (tools/labels/labels.py checks the kind); colours are #rrggbb.

#let u(x) = x * 1mm
#let hex(c) = rgb(c)

#let extents(r) = {
  if r.kind == "chip" or r.kind == "tant" { (r.l, r.w) }
  else if r.kind == "polymer" { (r.base, r.base) }
  else if r.kind == "sop8" { (4.9, 6.0) }
  else if r.kind == "usbc" { (r.l, r.h) }
  else if r.kind == "radial" or r.kind == "tilt" { (5, 9) }
  else if r.kind == "dip8" { (9.4, 8.0) }
  else if r.kind == "led" { (5.8, 5.8) }
  else if r.kind == "irrx" { (6.5, 9.5) }
  else if r.kind == "jack" { (12, 6) }
  else if r.kind == "trim" { (9.5, 9.5) }
  else if r.kind == "term" { (10, 7.6) }
  else if r.kind == "module" { (r.l, r.w) }
  else if r.kind == "disc" { (5, 9) }
  else if r.kind == "film" { (7, 9) }
  else if r.kind == "tact" { (6, 6) }
  else if r.kind == "card" { (8.56, 5.4) }
  else if r.kind == "xt60" { (8, 16) }
  else if r.kind == "antenna" { (20, 3) }
  else if r.kind == "matrix" { (16, 16) }
}

#let draw(r, s, sw: 0.35pt) = {
  let (W, H) = extents(r)
  let edge = sw + luma(70)
  box(width: u(W * s), height: u(H * s), {
    if r.kind == "chip" {
      let t = r.term * s
      place(rect(width: u(W * s), height: u(H * s), fill: rgb("#c8a878"), stroke: edge))
      place(rect(width: u(t), height: u(H * s), fill: rgb("#c4c8cd"), stroke: edge))
      place(dx: u(W * s - t), rect(width: u(t), height: u(H * s), fill: rgb("#c4c8cd"), stroke: edge))
    } else if r.kind == "tant" {
      let band = 0.16 * W * s
      place(rect(width: u(W * s), height: u(H * s), fill: rgb("#e3a82b"), stroke: edge, radius: u(0.15 * s)))
      place(rect(width: u(band), height: u(H * s), fill: rgb("#6b3f10"), stroke: edge))
      if s > 2 {
        place(dx: u(band), box(width: u(W * s - band), height: u(H * s),
          align(center + horizon, text(size: u(0.36 * H * s), weight: "bold", fill: rgb("#5a3510"), r.mark))))
      }
    } else if r.kind == "polymer" {
      let B = W * s
      let c = 0.2 * B
      let d = r.d * s
      place(polygon(fill: luma(45), stroke: edge,
        (u(c), u(0)), (u(B), u(0)), (u(B), u(B)), (u(c), u(B)), (u(0), u(B - c)), (u(0), u(c))))
      place(dx: u((B - d) / 2), dy: u((B - d) / 2),
        box(width: u(d), height: u(d), radius: 50%, clip: true, fill: luma(208), stroke: edge,
          place(dx: u(d * 0.72), rect(width: u(d * 0.3), height: u(d), fill: rgb("#5b21b6")))))
      if s > 2 {
        place(dx: u(0.3 * s), dy: u(B / 2 - 1.2 * s), text(size: u(1.6 * s), weight: "bold", fill: white, "+"))
      }
    } else if r.kind == "sop8" {
      let pin-l = 1.05 * s
      let body-h = 3.9 * s
      for i in range(4) {
        let px = (0.545 + i * 1.27 - 0.205) * s
        place(dx: u(px), rect(width: u(0.41 * s), height: u(pin-l), fill: luma(175), stroke: edge))
        place(dx: u(px), dy: u(pin-l + body-h), rect(width: u(0.41 * s), height: u(pin-l), fill: luma(175), stroke: edge))
      }
      place(dy: u(pin-l), rect(width: u(W * s), height: u(body-h), fill: luma(38), stroke: edge))
      place(dx: u(0.55 * s), dy: u(pin-l + body-h - 0.95 * s), circle(radius: u(0.28 * s), fill: luma(110)))
      if s > 2 {
        place(dy: u(pin-l), box(width: u(W * s), height: u(body-h),
          align(center + horizon, text(size: u(0.75 * s), weight: "bold", fill: luma(225), r.mark))))
      }
    } else if r.kind == "usbc" {
      let h = H * s
      place(rect(width: u(W * s), height: u(h), radius: u(h / 2), fill: luma(195), stroke: edge))
      place(dx: u(0.3 * s), dy: u(0.3 * s),
        rect(width: u(W * s - 0.6 * s), height: u(h - 0.6 * s), radius: u(h / 2 - 0.3 * s), fill: luma(30)))
      place(dx: u(W * s * 0.14), dy: u(h / 2 - 0.35 * s),
        rect(width: u(W * s * 0.72), height: u(0.7 * s), fill: luma(70)))
      place(dx: u(W * s * 0.18), dy: u(h / 2 - 0.12 * s),
        rect(width: u(W * s * 0.64), height: u(0.24 * s), fill: rgb("#d4a017")))
    } else if r.kind == "radial" {
      // side view: can, minus stripe on the right, long + lead on the left
      place(rect(width: u(5 * s), height: u(7 * s), radius: u(0.5 * s), fill: luma(45), stroke: edge))
      place(dx: u(3.6 * s), rect(width: u(1.1 * s), height: u(7 * s), fill: luma(170)))
      if s > 2 {
        place(dx: u(3.6 * s), dy: u(0.6 * s), box(width: u(1.1 * s), height: u(5.8 * s),
          align(center + horizon, text(size: u(1.2 * s), weight: "bold", fill: luma(45), "−"))))
      }
      place(dx: u(1.3 * s), dy: u(7 * s), rect(width: u(0.4 * s), height: u(2 * s), fill: luma(160)))
      place(dx: u(3.3 * s), dy: u(7 * s), rect(width: u(0.4 * s), height: u(1.4 * s), fill: luma(160)))
    } else if r.kind == "dip8" {
      let top = (8.0 - 6.35) / 2
      for i in range(4) {
        let px = (0.89 + i * 2.54 - 0.3) * s
        place(dx: u(px), rect(width: u(0.6 * s), height: u(top * s), fill: luma(175), stroke: edge))
        place(dx: u(px), dy: u((top + 6.35) * s), rect(width: u(0.6 * s), height: u(top * s), fill: luma(175), stroke: edge))
      }
      place(dy: u(top * s), rect(width: u(9.4 * s), height: u(6.35 * s), fill: luma(38), stroke: edge))
      place(dx: u(0.15 * s), dy: u(4 * s - 0.55 * s), circle(radius: u(0.55 * s), fill: luma(80)))
      place(dx: u(0.6 * s), dy: u((top + 6.35 - 1.3) * s), circle(radius: u(0.35 * s), fill: luma(110)))
      if s > 2 {
        place(dy: u(top * s), box(width: u(9.4 * s), height: u(6.35 * s),
          align(center + horizon, text(size: u(1.2 * s), weight: "bold", fill: luma(225), r.mark))))
      }
    } else if r.kind == "led" {
      // top view: flange with the flat on the cathode side (right)
      place(circle(radius: u(2.9 * s), fill: hex(r.fill), stroke: edge))
      place(dx: u(5.3 * s), rect(width: u(0.6 * s), height: u(5.8 * s), fill: white))
      place(dx: u(5.3 * s), dy: u(1.2 * s), line(length: u(3.4 * s), angle: 90deg, stroke: edge))
      place(dx: u(0.4 * s), dy: u(0.4 * s), circle(radius: u(2.5 * s), fill: hex(r.fill), stroke: 0.2pt + luma(150)))
    } else if r.kind == "irrx" {
      // front view, dome toward the reader, leads labelled
      place(dx: u(0.25 * s), rect(width: u(6 * s), height: u(5 * s), radius: u(0.4 * s), fill: luma(35), stroke: edge))
      place(dx: u(1.5 * s), dy: u(0.75 * s), circle(radius: u(1.75 * s), fill: luma(70), stroke: 0.3pt + luma(110)))
      for (i, n) in ("OUT", "GND", "VCC").enumerate() {
        let cx = (3.25 + (i - 1) * 2.54) * s
        place(dx: u(cx - 0.2 * s), dy: u(5 * s), rect(width: u(0.4 * s), height: u(3 * s), fill: luma(160)))
        if s > 2 {
          place(dx: u(cx - 1.2 * s), dy: u(8.1 * s), box(width: u(2.4 * s),
            align(center, text(size: u(0.9 * s), weight: "bold", n))))
        }
      }
    } else if r.kind == "tilt" {
      place(rect(width: u(5 * s), height: u(6.5 * s), radius: (top: u(1 * s)), fill: luma(195), stroke: edge))
      place(dx: u(0.3 * s), dy: u(5.6 * s), rect(width: u(4.4 * s), height: u(0.9 * s), fill: luma(40)))
      place(dx: u(1.3 * s), dy: u(6.5 * s), rect(width: u(0.4 * s), height: u(2.5 * s), fill: luma(160)))
      place(dx: u(3.3 * s), dy: u(6.5 * s), rect(width: u(0.4 * s), height: u(2.5 * s), fill: luma(160)))
    } else if r.kind == "jack" {
      place(dx: u(2 * s), rect(width: u(10 * s), height: u(6 * s), fill: luma(35), stroke: edge))
      place(dy: u(0.6 * s), rect(width: u(2.2 * s), height: u(4.8 * s), fill: luma(55), stroke: edge))
      place(dx: u(-0.2 * s), dy: u(1.25 * s), rect(width: u(1 * s), height: u(3.5 * s), radius: u(0.3 * s), fill: luma(10)))
      for (x, y) in ((4, -0.3), (8.5, -0.3), (6, 5.5), (10.5, 5.5)) {
        place(dx: u(x * s), dy: u(y * s), rect(width: u(1.2 * s), height: u(0.8 * s), fill: rgb("#c9a227")))
      }
    } else if r.kind == "trim" {
      place(rect(width: u(9.5 * s), height: u(9.5 * s), radius: u(0.6 * s), fill: rgb("#2f5fd0"), stroke: edge))
      place(dx: u(1.75 * s), dy: u(1.75 * s), circle(radius: u(3 * s), fill: rgb("#ece3cf"), stroke: edge))
      place(dx: u(3 * s), dy: u(4.75 * s), line(length: u(3.5 * s), stroke: (0.9 * s / 2) * 1pt + luma(90)))
      place(dx: u(4.75 * s), dy: u(3 * s), line(length: u(3.5 * s), angle: 90deg, stroke: (0.9 * s / 2) * 1pt + luma(90)))
    } else if r.kind == "term" {
      place(rect(width: u(10 * s), height: u(7.6 * s), radius: u(0.3 * s), fill: rgb("#2a6fdb"), stroke: edge))
      for x in (2.5, 7.5) {
        place(dx: u((x - 1.75) * s), dy: u(1 * s), circle(radius: u(1.75 * s), fill: luma(200), stroke: edge))
        place(dx: u((x - 1.2) * s), dy: u(2.75 * s), line(length: u(2.4 * s), stroke: (0.6 * s / 2) * 1pt + luma(90)))
        place(dx: u((x - 1.5) * s), dy: u(5.2 * s), rect(width: u(3 * s), height: u(1.8 * s), fill: luma(30)))
      }
    } else if r.kind == "module" {
      let (L, Wd) = (r.l * s, r.w * s)
      place(rect(width: u(L), height: u(Wd), radius: u(0.8 * s), fill: hex(r.pcb), stroke: edge))
      let cw = calc.min(r.l, r.w) * 0.38 * s
      place(dx: u(L / 2 - cw / 2), dy: u(Wd * 0.4 - cw / 2), rect(width: u(cw), height: u(cw), fill: luma(30)))
      let n = r.pins
      if n > 0 {
        let x1 = (r.l - (n - 1) * 2.54) / 2
        for i in range(n) {
          place(dx: u((x1 + i * 2.54 - 0.6) * s), dy: u((r.w - 2) * s),
            circle(radius: u(0.6 * s), fill: rgb("#d9b44a"), stroke: 0.2pt + luma(60)))
        }
      }
      // optional features: USB-C on the left edge, a screen, a lens, transducers, a BNC
      if r.at("usb", default: false) {
        place(dx: u(-0.3 * s), dy: u(Wd / 2 - 4.5 * s), rect(width: u(3.2 * s), height: u(9 * s), radius: u(1.5 * s), fill: luma(200), stroke: edge))
      }
      if r.at("screen", default: false) {
        place(dx: u(L * 0.08), dy: u(Wd * 0.12), rect(width: u(L * 0.84), height: u(Wd * 0.6), fill: luma(15), stroke: edge))
      }
      if r.at("lens", default: false) {
        place(dx: u(L * 0.62 - Wd * 0.22), dy: u(Wd * 0.5 - Wd * 0.22), circle(radius: u(Wd * 0.22), fill: luma(25), stroke: 1pt + luma(120)))
      }
      if r.at("eyes", default: false) {
        for cx in (0.25, 0.75) {
          place(dx: u(L * cx - Wd * 0.33), dy: u(Wd * 0.45 - Wd * 0.33), circle(radius: u(Wd * 0.33), fill: luma(200), stroke: edge))
          place(dx: u(L * cx - Wd * 0.2), dy: u(Wd * 0.45 - Wd * 0.2), circle(radius: u(Wd * 0.2), fill: luma(60)))
        }
      }
      if r.at("bnc", default: false) {
        place(dx: u(L - Wd * 0.45), dy: u(Wd * 0.5 - Wd * 0.3), circle(radius: u(Wd * 0.3), fill: luma(205), stroke: edge))
      }
    } else if r.kind == "disc" {
      place(dx: u(0.0), circle(radius: u(2.5 * s), fill: rgb("#e0a030"), stroke: edge))
      if s > 2 {
        place(box(width: u(5 * s), height: u(5 * s), align(center + horizon, text(size: u(1.3 * s), weight: "bold", fill: rgb("#4a2c08"), r.mark))))
      }
      place(dx: u(1.3 * s), dy: u(4.8 * s), rect(width: u(0.4 * s), height: u(4.2 * s), fill: luma(160)))
      place(dx: u(3.3 * s), dy: u(4.8 * s), rect(width: u(0.4 * s), height: u(4.2 * s), fill: luma(160)))
    } else if r.kind == "film" {
      place(rect(width: u(7 * s), height: u(6 * s), radius: u(0.8 * s), fill: rgb("#2f5fbf"), stroke: edge))
      if s > 2 {
        place(box(width: u(7 * s), height: u(6 * s), align(center + horizon, text(size: u(1.1 * s), weight: "bold", fill: white, r.mark))))
      }
      place(dx: u(1.6 * s), dy: u(6 * s), rect(width: u(0.4 * s), height: u(3 * s), fill: luma(160)))
      place(dx: u(5 * s), dy: u(6 * s), rect(width: u(0.4 * s), height: u(3 * s), fill: luma(160)))
    } else if r.kind == "tact" {
      place(rect(width: u(6 * s), height: u(6 * s), fill: luma(185), stroke: edge))
      place(dx: u(1.5 * s), dy: u(1.5 * s), circle(radius: u(1.5 * s), fill: luma(30)))
      for (x, y) in ((-0.4, 0.6), (5.6, 0.6), (-0.4, 4.6), (5.6, 4.6)) {
        place(dx: u(x * s), dy: u(y * s), rect(width: u(0.8 * s), height: u(0.8 * s), fill: luma(140)))
      }
    } else if r.kind == "card" {
      place(rect(width: u(8.56 * s), height: u(5.4 * s), radius: u(0.4 * s), fill: hex(r.at("fill", default: "#1e1e1e")), stroke: edge))
      for k in range(3) {
        let rr = (0.6 + k * 0.5) * s
        place(dx: u(4.28 * s - rr), dy: u(2.7 * s - rr), circle(radius: u(rr), stroke: (0.18 * s) * 1pt + luma(200)))
      }
    } else if r.kind == "xt60" {
      place(dy: u(2 * s), rect(width: u(8 * s), height: u(14 * s), radius: (top-left: u(2.5 * s), top-right: u(2.5 * s)), fill: rgb("#f2c400"), stroke: edge))
      place(dx: u(2 * s), rect(width: u(1.2 * s), height: u(2.5 * s), fill: rgb("#b03030")))
      place(dx: u(4.8 * s), rect(width: u(1.2 * s), height: u(2.5 * s), fill: luma(30)))
      for x in (2.6, 5.4) {
        place(dx: u((x - 1.6) * s), dy: u(7.4 * s), circle(radius: u(1.6 * s), fill: rgb("#c9a227"), stroke: edge))
      }
    } else if r.kind == "antenna" {
      place(dy: u(0.5 * s), rect(width: u(15 * s), height: u(2 * s), radius: u(1 * s), fill: luma(30), stroke: edge))
      place(dx: u(15 * s), dy: u(0.2 * s), rect(width: u(5 * s), height: u(2.6 * s), fill: rgb("#d4a017"), stroke: edge))
    } else if r.kind == "matrix" {
      place(rect(width: u(16 * s), height: u(16 * s), fill: luma(20), stroke: edge))
      for i in range(8) {
        for j in range(8) {
          place(dx: u((1 + i * 1.9) * s), dy: u((1 + j * 1.9) * s), rect(width: u(1.1 * s), height: u(1.1 * s), fill: luma(235)))
        }
      }
    }
  })
}

#let fit(r, w, h) = {
  let (W, H) = extents(r)
  calc.min(w / W, h / H, 9)
}
