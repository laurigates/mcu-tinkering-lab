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
  else if r.kind == "axial" { (14, 3) }
  else if r.kind == "to92" { (5, 10) }
  else if r.kind == "to220" { (10, 20) }
  else if r.kind == "sot223" { (6.5, 7.0) }
  else if r.kind == "header" {
    let p = r.at("pitch", default: 2.54)
    (calc.min(r.n, 10) * p, r.at("rows", default: 1) * p)
  }
  else if r.kind == "housing" {
    let p = r.at("pitch", default: 2.54)
    (r.n * p + 0.8, r.at("rows", default: 1) * p + 2.0)
  }
  else if r.kind == "barrel" { if r.at("plug", default: false) { (16, 6) } else { (9, 9) } }
  else if r.kind == "banana" { (16, 6) }
  else if r.kind == "jumper" { (20, 8) }
  else if r.kind == "spool" { (10, 10) }
  else if r.kind == "ribbon" { (14, 7) }
  else if r.kind == "coin" { if r.at("leads", default: false) { (r.d * 1.6, r.d) } else { (r.d, r.d) } }
  else if r.kind == "cell" { (r.l, r.d) }
  else if r.kind == "pouch" { (r.l, r.w + 3) }
  else if r.kind == "servo" { (32, 12.2) }
  else if r.kind == "stepper" { (42.3, 42.3) }
  else if r.kind == "dcmotor" { (33, 15) }
  else if r.kind == "speaker" { (28, 28) }
  else if r.kind == "buzzer" { (12, 12) }
  else if r.kind == "button" { (12, 18) }
  else if r.kind == "reed" { (20, 3) }
  else if r.kind == "keypad" { (14, 18) }
  else if r.kind == "ldr" { (5, 9) }
  else if r.kind == "screw" { (r.l + 3, 5.5) }
  else if r.kind == "insert" { (r.l, r.od) }
  else if r.kind == "standoff" { (r.l + if r.at("thread", default: true) { 3 } else { 0 }, 3.2) }
  else if r.kind == "heatsink" { (8.8, 8.8) }
  else if r.kind == "dht" { (12, 20) }
  else if r.kind == "laser" { (16, 6) }
  else if r.kind == "holder" { (r.l + 4, r.cells * r.d + 2) }
  else if r.kind == "magnet" { (20, 20) }
}

// A wire or lead from (x0, y0) to (x1, y1), in real mm.
#let wire(s, x0, y0, x1, y1, w, c) = place(line(
  start: (u(x0 * s), u(y0 * s)), end: (u(x1 * s), u(y1 * s)), stroke: u(w * s) + c))

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
    } else if r.kind == "axial" {
      // resistor, diode or glass fuse, leads left and right
      wire(s, 0, 1.5, 14, 1.5, 0.4, luma(160))
      if r.at("glass", default: false) {
        place(dx: u(3 * s), rect(width: u(8 * s), height: u(3 * s), fill: rgb("#e6eff3"), stroke: edge))
        wire(s, 4.2, 1.5, 9.8, 1.5, 0.12, luma(90))
        for x in (3, 9.8) {
          place(dx: u(x * s), rect(width: u(1.2 * s), height: u(3 * s), fill: luma(190), stroke: edge))
        }
      } else {
        place(dx: u(3.5 * s), rect(width: u(7 * s), height: u(3 * s), radius: u(1 * s), fill: hex(r.at("fill", default: "#d8c39a")), stroke: edge))
        for (i, b) in r.at("bands", default: ()).enumerate() {
          let x = if i == 3 { 9.0 } else { 4.6 + i * 1.1 }
          place(dx: u(x * s), rect(width: u(0.6 * s), height: u(3 * s), fill: hex(b)))
        }
        if r.at("cathode", default: false) {
          place(dx: u(8.8 * s), rect(width: u(0.8 * s), height: u(3 * s), fill: luma(200)))
        }
      }
    } else if r.kind == "to92" {
      // flat face toward the reader
      place(rect(width: u(5 * s), height: u(5 * s), radius: (top-left: u(1.6 * s), top-right: u(1.6 * s)), fill: luma(35), stroke: edge))
      for x in (1.0, 2.5, 4.0) {
        place(dx: u((x - 0.2) * s), dy: u(5 * s), rect(width: u(0.4 * s), height: u(5 * s), fill: luma(160)))
      }
      if s > 2 and r.at("mark", default: "") != "" {
        place(box(width: u(5 * s), height: u(5 * s), align(center + horizon, text(size: u(0.8 * s), weight: "bold", fill: luma(225), r.mark))))
      }
    } else if r.kind == "to220" {
      place(rect(width: u(10 * s), height: u(6.5 * s), radius: (top-left: u(0.6 * s), top-right: u(0.6 * s)), fill: luma(195), stroke: edge))
      place(dx: u(3.2 * s), dy: u(1.4 * s), circle(radius: u(1.8 * s), fill: white, stroke: edge))
      place(dy: u(6.2 * s), rect(width: u(10 * s), height: u(8.8 * s), fill: luma(35), stroke: edge))
      for x in (2.46, 5.0, 7.54) {
        place(dx: u((x - 0.4) * s), dy: u(15 * s), rect(width: u(0.8 * s), height: u(5 * s), fill: luma(175)))
      }
      if s > 2 and r.at("mark", default: "") != "" {
        place(dy: u(6.2 * s), box(width: u(10 * s), height: u(8.8 * s), align(center + horizon, text(size: u(1.3 * s), weight: "bold", fill: luma(225), r.mark))))
      }
    } else if r.kind == "sot223" {
      place(dx: u(1.75 * s), rect(width: u(3 * s), height: u(1.8 * s), fill: luma(175), stroke: edge))
      place(dy: u(1.75 * s), rect(width: u(6.5 * s), height: u(3.5 * s), fill: luma(38), stroke: edge))
      for x in (0.95, 3.25, 5.55) {
        place(dx: u((x - 0.35) * s), dy: u(5.25 * s), rect(width: u(0.7 * s), height: u(1.75 * s), fill: luma(175), stroke: edge))
      }
    } else if r.kind == "header" {
      // top view; long strips are drawn 10 pins long
      let p = r.at("pitch", default: 2.54)
      let n = calc.min(r.n, 10)
      let rows = r.at("rows", default: 1)
      let fem = r.at("female", default: false)
      place(rect(width: u(n * p * s), height: u(rows * p * s), fill: luma(25), stroke: edge))
      for i in range(n) {
        for j in range(rows) {
          let cx = (i + 0.5) * p
          let cy = (j + 0.5) * p
          if fem {
            let h = if r.at("round", default: false) { 0.5 } else { 0.45 }
            place(dx: u((cx - h * p / 1.27 * 0.6) * s), dy: u((cy - h * p / 1.27 * 0.6) * s),
              box(width: u(h * p / 1.27 * 1.2 * s), height: u(h * p / 1.27 * 1.2 * s), fill: luma(80),
                radius: if r.at("round", default: false) { 50% } else { 0pt }))
          } else if r.at("round", default: false) {
            place(dx: u((cx - 0.35) * s), dy: u((cy - 0.35) * s), circle(radius: u(0.35 * s), fill: rgb("#d9b44a")))
          } else {
            place(dx: u((cx - 0.32) * s), dy: u((cy - 0.32) * s), rect(width: u(0.64 * s), height: u(0.64 * s), fill: rgb("#d9b44a")))
          }
        }
      }
    } else if r.kind == "housing" {
      // crimp housing (JST, Dupont, IDC) face-on, with the latch on top
      let p = r.at("pitch", default: 2.54)
      let rows = r.at("rows", default: 1)
      let (W, H) = (r.n * p + 0.8, rows * p + 2.0)
      let f = hex(r.at("fill", default: "#f2efe6"))
      place(dx: u(W * 0.3 * s), rect(width: u(W * 0.4 * s), height: u(1.2 * s), fill: f, stroke: edge))
      place(dy: u(1 * s), rect(width: u(W * s), height: u((H - 1) * s), radius: u(0.3 * s), fill: f, stroke: edge))
      for i in range(r.n) {
        for j in range(rows) {
          place(dx: u((0.4 + i * p + p * 0.22) * s), dy: u((1.5 + j * p + p * 0.22) * s),
            rect(width: u(p * 0.56 * s), height: u(p * 0.56 * s), fill: luma(70)))
        }
      }
    } else if r.kind == "barrel" {
      if r.at("plug", default: false) {
        place(dy: u(1.25 * s), rect(width: u(6.5 * s), height: u(3.5 * s), fill: luma(200), stroke: edge))
        place(dx: u(6 * s), rect(width: u(10 * s), height: u(6 * s), radius: u(1 * s), fill: luma(30), stroke: edge))
        for x in (8, 10, 12, 14) {
          wire(s, x, 0.6, x, 5.4, 0.15, luma(70))
        }
      } else {
        place(rect(width: u(9 * s), height: u(9 * s), radius: u(0.6 * s), fill: hex(r.at("fill", default: "#262626")), stroke: edge))
        place(dx: u(1.3 * s), dy: u(1.3 * s), circle(radius: u(3.2 * s), fill: luma(55), stroke: edge))
        place(dx: u(3.5 * s), dy: u(3.5 * s), circle(radius: u(1 * s), fill: rgb("#d9b44a")))
      }
    } else if r.kind == "banana" {
      place(dy: u(1.5 * s), rect(width: u(6.5 * s), height: u(3 * s), radius: u(1.2 * s), fill: rgb("#d9b44a"), stroke: edge))
      wire(s, 0.8, 3, 5.5, 3, 0.2, rgb("#8a6d1c"))
      place(dx: u(6 * s), rect(width: u(10 * s), height: u(6 * s), radius: u(1.5 * s), fill: hex(r.at("fill", default: "#c62828")), stroke: edge))
    } else if r.kind == "jumper" {
      for (i, c) in ("#c62828", "#f2c400", "#2e7d32", "#1d4ed8").enumerate() {
        let y = 1 + i * 2
        wire(s, 3, y, 17, y, 0.9, rgb(c))
        place(dy: u((y - 0.8) * s), rect(width: u(3 * s), height: u(1.6 * s), fill: luma(25)))
        place(dx: u(17 * s), dy: u((y - 0.8) * s), rect(width: u(3 * s), height: u(1.6 * s), fill: luma(25)))
      }
    } else if r.kind == "spool" {
      place(dy: u(1.5 * s), rect(width: u(10 * s), height: u(7 * s), fill: hex(r.at("fill", default: "#c62828")), stroke: edge))
      for k in range(1, 14) {
        wire(s, 0, 1.5 + k * 0.5, 10, 1.5 + k * 0.5, 0.06, luma(0, 50%))
      }
      for x in (0, 8.8) {
        place(dx: u(x * s), rect(width: u(1.2 * s), height: u(10 * s), radius: u(0.3 * s), fill: luma(60), stroke: edge))
      }
    } else if r.kind == "ribbon" {
      let n = calc.min(r.at("n", default: 10), 16)
      let h = 7 / n
      for i in range(n) {
        place(dx: u(3 * s), dy: u(i * h * s), rect(width: u(11 * s), height: u(h * s), fill: if i == 0 { rgb("#c62828") } else { luma(185) }, stroke: 0.15pt + luma(120)))
      }
      if r.at("idc", default: false) {
        place(rect(width: u(3.4 * s), height: u(7 * s), radius: u(0.3 * s), fill: luma(30), stroke: edge))
      }
    } else if r.kind == "coin" {
      let d = r.d
      if r.at("leads", default: false) {
        wire(s, d * 0.8, d * 0.45, d * 1.6, d * 0.4, 0.06 * d, rgb("#c62828"))
        wire(s, d * 0.8, d * 0.6, d * 1.6, d * 0.65, 0.06 * d, luma(25))
      }
      place(circle(radius: u(d / 2 * s), fill: hex(r.at("fill", default: "#c4c8cd")), stroke: edge))
      place(dx: u(d * 0.08 * s), dy: u(d * 0.08 * s), circle(radius: u(d * 0.42 * s), stroke: 0.3pt + luma(140)))
      if 0.16 * d * s > 1.0 and r.at("mark", default: "") != "" {
        place(box(width: u(d * s), height: u(d * s), align(center + horizon,
          text(size: u(0.16 * d * s), weight: "bold", fill: luma(60), r.mark))))
      }
    } else if r.kind == "cell" {
      let (L, D) = (r.l, r.d)
      place(dx: u((L - 1.2) * s), dy: u(D * 0.32 * s), rect(width: u(1.2 * s), height: u(D * 0.36 * s), fill: luma(190), stroke: edge))
      place(rect(width: u((L - 1) * s), height: u(D * s), radius: u(0.8 * s), fill: hex(r.at("fill", default: "#f4f4f4")), stroke: edge))
      place(dx: u(L * 0.6 * s), rect(width: u(L * 0.12 * s), height: u(D * s), fill: hex(r.at("band", default: "#1d4ed8"))))
      if 0.4 * D * s > 1.0 and r.at("mark", default: "") != "" {
        place(box(width: u(L * 0.6 * s), height: u(D * s), align(center + horizon,
          text(size: u(0.4 * D * s), weight: "bold", fill: hex(r.at("band", default: "#1d4ed8")), r.mark))))
      }
    } else if r.kind == "pouch" {
      wire(s, r.l * 0.35, 0, r.l * 0.35, 3, 0.5, rgb("#c62828"))
      wire(s, r.l * 0.65, 0, r.l * 0.65, 3, 0.5, luma(25))
      place(dy: u(3 * s), rect(width: u(r.l * s), height: u(r.w * s), radius: u(0.6 * s), fill: luma(210), stroke: edge))
      place(dx: u(r.l * 0.1 * s), dy: u((3 + r.w * 0.2) * s), rect(width: u(r.l * 0.8 * s), height: u(r.w * 0.6 * s), fill: rgb("#e8eef7")))
      if 0.22 * r.w * s > 1.0 and r.at("mark", default: "") != "" {
        place(dy: u(3 * s), box(width: u(r.l * s), height: u(r.w * s), align(center + horizon,
          text(size: u(0.22 * r.w * s), weight: "bold", fill: luma(50), r.mark))))
      }
    } else if r.kind == "servo" {
      // top view: mounting ears, body, output spline with a horn
      let f = hex(r.at("fill", default: "#2b2b2b"))
      place(dy: u(3.6 * s), rect(width: u(32 * s), height: u(5 * s), radius: u(0.8 * s), fill: f, stroke: edge))
      for x in (1.4, 28.2) {
        place(dx: u(x * s), dy: u(5.2 * s), circle(radius: u(1 * s), fill: white))
      }
      place(dx: u(4.5 * s), rect(width: u(23 * s), height: u(12.2 * s), radius: u(0.6 * s), fill: f, stroke: edge))
      place(dx: u(4 * s), dy: u(4.6 * s), rect(width: u(14 * s), height: u(3 * s), radius: u(1.5 * s), fill: white, stroke: edge))
      place(dx: u(8 * s), dy: u(4.1 * s), circle(radius: u(2 * s), fill: white, stroke: edge))
      place(dx: u(9.2 * s), dy: u(5.3 * s), circle(radius: u(0.8 * s), fill: luma(150)))
    } else if r.kind == "stepper" {
      let c = 5
      let Q = 42.3
      place(polygon(fill: luma(45), stroke: edge,
        (u(c * s), u(0)), (u((Q - c) * s), u(0)), (u(Q * s), u(c * s)), (u(Q * s), u((Q - c) * s)),
        (u((Q - c) * s), u(Q * s)), (u(c * s), u(Q * s)), (u(0), u((Q - c) * s)), (u(0), u(c * s))))
      place(dx: u((Q / 2 - 11) * s), dy: u((Q / 2 - 11) * s), circle(radius: u(11 * s), fill: luma(175), stroke: edge))
      place(dx: u((Q / 2 - 2.5) * s), dy: u((Q / 2 - 2.5) * s), circle(radius: u(2.5 * s), fill: luma(225), stroke: edge))
      for (x, y) in ((-15.5, -15.5), (15.5, -15.5), (-15.5, 15.5), (15.5, 15.5)) {
        place(dx: u((Q / 2 + x - 1.5) * s), dy: u((Q / 2 + y - 1.5) * s), circle(radius: u(1.5 * s), fill: luma(15)))
      }
    } else if r.kind == "dcmotor" {
      wire(s, 0, 7.5, 9, 7.5, 2, luma(170))
      place(dx: u(8 * s), rect(width: u(20 * s), height: u(15 * s), radius: u(2 * s), fill: luma(200), stroke: edge))
      place(dx: u(27 * s), dy: u(2 * s), rect(width: u(6 * s), height: u(11 * s), radius: u(1 * s), fill: luma(245), stroke: edge))
      for y in (4.5, 10.5) {
        place(dx: u(31.5 * s), dy: u((y - 0.6) * s), rect(width: u(1.5 * s), height: u(1.2 * s), fill: rgb("#c9a227")))
      }
    } else if r.kind == "speaker" {
      place(circle(radius: u(14 * s), fill: luma(30), stroke: edge))
      for (k, rr) in (11.5, 8.5, 5.5).enumerate() {
        place(dx: u((14 - rr) * s), dy: u((14 - rr) * s), circle(radius: u(rr * s), fill: luma(55 + k * 15), stroke: 0.3pt + luma(20)))
      }
      place(dx: u(10.5 * s), dy: u(10.5 * s), circle(radius: u(3.5 * s), fill: luma(15)))
    } else if r.kind == "buzzer" {
      place(circle(radius: u(6 * s), fill: luma(25), stroke: edge))
      place(dx: u(4.8 * s), dy: u(4.8 * s), circle(radius: u(1.2 * s), fill: luma(70)))
      if s > 2 {
        place(dx: u(2 * s), dy: u(1.4 * s), text(size: u(2.4 * s), weight: "bold", fill: luma(200), "+"))
      }
    } else if r.kind == "button" {
      // side view: cap, bezel, threaded body, solder lugs
      place(dx: u(2.5 * s), rect(width: u(7 * s), height: u(4 * s), radius: (top-left: u(1 * s), top-right: u(1 * s)), fill: hex(r.at("fill", default: "#c62828")), stroke: edge))
      place(dy: u(4 * s), rect(width: u(12 * s), height: u(2 * s), radius: u(0.4 * s), fill: luma(185), stroke: edge))
      place(dx: u(2 * s), dy: u(6 * s), rect(width: u(8 * s), height: u(8 * s), fill: luma(160), stroke: edge))
      for k in range(1, 8) {
        wire(s, 2, 6 + k, 10, 6 + k, 0.12, luma(110))
      }
      for x in (3.5, 7.5) {
        place(dx: u(x * s), dy: u(14 * s), rect(width: u(1 * s), height: u(4 * s), fill: rgb("#c9a227")))
      }
    } else if r.kind == "reed" {
      wire(s, 0, 1.5, 14, 1.5, 0.35, luma(160))
      place(dx: u(4 * s), rect(width: u(12 * s), height: u(3 * s), radius: u(1.5 * s), fill: rgb("#e2edf2"), stroke: edge))
      wire(s, 4, 1.5, 10.6, 1.3, 0.3, luma(110))
      wire(s, 9.4, 1.7, 16, 1.5, 0.3, luma(110))
      wire(s, 16, 1.5, 20, 1.5, 0.35, luma(160))
    } else if r.kind == "keypad" {
      place(rect(width: u(14 * s), height: u(14 * s), radius: u(0.6 * s), fill: luma(240), stroke: edge))
      let keys = ("1", "2", "3", "A", "4", "5", "6", "B", "7", "8", "9", "C", "*", "0", "#", "D")
      for (k, t) in keys.enumerate() {
        let (i, j) = (calc.rem(k, 4), calc.quo(k, 4))
        place(dx: u((0.9 + i * 3.2) * s), dy: u((0.9 + j * 3.2) * s),
          box(width: u(2.6 * s), height: u(2.6 * s), radius: u(0.4 * s), fill: if i == 3 { rgb("#c62828") } else { rgb("#1d4ed8") },
            align(center + horizon, if s > 1.2 { text(size: u(1.4 * s), weight: "bold", fill: white, t) })))
      }
      place(dx: u(4.5 * s), dy: u(14 * s), rect(width: u(5 * s), height: u(4 * s), fill: luma(70)))
    } else if r.kind == "ldr" {
      place(circle(radius: u(2.5 * s), fill: rgb("#c4572a"), stroke: edge))
      for k in range(5) {
        let y = 1.1 + k * 0.7
        wire(s, 1.1, y, 3.9, y, 0.18, luma(235))
      }
      place(dx: u(1.3 * s), dy: u(4.8 * s), rect(width: u(0.4 * s), height: u(4.2 * s), fill: luma(160)))
      place(dx: u(3.3 * s), dy: u(4.8 * s), rect(width: u(0.4 * s), height: u(4.2 * s), fill: luma(160)))
    } else if r.kind == "screw" {
      let f = hex(r.at("fill", default: "#2b2b2b"))
      place(rect(width: u(3 * s), height: u(5.5 * s), radius: (top-left: u(0.6 * s), bottom-left: u(0.6 * s)), fill: f, stroke: edge))
      place(dx: u(3 * s), dy: u(1.25 * s), rect(width: u(r.l * s), height: u(3 * s), fill: f, stroke: edge))
      for k in range(int(r.l / 0.5)) {
        wire(s, 3.25 + k * 0.5, 1.25, 3.25 + k * 0.5 + 0.25, 4.25, 0.1, luma(140))
      }
    } else if r.kind == "insert" {
      place(rect(width: u(r.l * s), height: u(r.od * s), fill: rgb("#c9a227"), stroke: edge))
      for k in range(int(r.l / 0.5)) {
        wire(s, 0.1 + k * 0.5, 0, 0.1 + k * 0.5 + 0.35, r.od, 0.08, rgb("#7a5f12"))
      }
    } else if r.kind == "standoff" {
      let thread = r.at("thread", default: true)
      place(rect(width: u(r.l * s), height: u(3.2 * s), fill: rgb("#c9a227"), stroke: edge))
      for y in (0.8, 2.4) {
        wire(s, 0, y, r.l, y, 0.1, rgb("#8a6d1c"))
      }
      if thread {
        place(dx: u(r.l * s), dy: u(0.6 * s), rect(width: u(3 * s), height: u(2 * s), fill: rgb("#d9b44a"), stroke: edge))
        for k in range(6) {
          wire(s, r.l + 0.25 + k * 0.5, 0.6, r.l + 0.45 + k * 0.5, 2.6, 0.08, rgb("#7a5f12"))
        }
      } else {
        place(dx: u((r.l / 2 - 0.6) * s), dy: u(1 * s), circle(radius: u(0.6 * s), fill: luma(40)))
      }
    } else if r.kind == "heatsink" {
      place(rect(width: u(8.8 * s), height: u(8.8 * s), fill: luma(150), stroke: edge))
      for i in range(4) {
        for j in range(4) {
          place(dx: u((0.6 + i * 2.1) * s), dy: u((0.6 + j * 2.1) * s), rect(width: u(1.5 * s), height: u(1.5 * s), fill: luma(205)))
        }
      }
    } else if r.kind == "dht" {
      place(rect(width: u(12 * s), height: u(15.5 * s), radius: u(0.6 * s), fill: rgb("#2f6fd0"), stroke: edge))
      for i in range(4) {
        for j in range(5) {
          place(dx: u((1.4 + i * 2.5) * s), dy: u((1.2 + j * 2.6) * s), rect(width: u(1.7 * s), height: u(1.6 * s), fill: rgb("#dbe7fb")))
        }
      }
      for i in range(4) {
        place(dx: u((2.19 + i * 2.54 - 0.25) * s), dy: u(15.5 * s), rect(width: u(0.5 * s), height: u(4.5 * s), fill: luma(170)))
      }
    } else if r.kind == "laser" {
      wire(s, 10, 2.2, 16, 2.2, 0.6, rgb("#c62828"))
      wire(s, 10, 3.8, 16, 3.8, 0.6, rgb("#1d4ed8"))
      place(rect(width: u(10.5 * s), height: u(6 * s), radius: u(0.6 * s), fill: rgb("#c9a227"), stroke: edge))
      place(dy: u(0.8 * s), rect(width: u(1 * s), height: u(4.4 * s), fill: luma(50)))
    } else if r.kind == "holder" {
      let (L, D, n) = (r.l, r.d, r.cells)
      if r.at("leads", default: false) {
        wire(s, L + 1, 1.5, L + 4, 1.5, 0.6, rgb("#c62828"))
        wire(s, L + 1, n * D, L + 4, n * D, 0.6, luma(25))
      }
      place(rect(width: u((L + 2) * s), height: u((n * D + 2) * s), radius: u(1 * s), fill: luma(30), stroke: edge))
      for k in range(n) {
        place(dx: u(1 * s), dy: u((1 + k * D) * s), rect(width: u(L * s), height: u(D * s), radius: u(D / 2 * s), fill: luma(70), stroke: 0.3pt + luma(15)))
        if s * D > 4 {
          let minus = calc.rem(k, 2) == 1
          place(dx: u((if minus { L - 2 } else { 2 }) * s), dy: u((1 + k * D + D / 2 - D * 0.3) * s),
            text(size: u(D * 0.5 * s), weight: "bold", fill: luma(210), "+"))
        }
      }
    } else if r.kind == "magnet" {
      place(dx: u(8.5 * s), rect(width: u(3 * s), height: u(4 * s), fill: luma(170), stroke: edge))
      wire(s, 18, 8, 20, 6, 0.6, rgb("#c62828"))
      wire(s, 18, 10, 20, 12, 0.6, luma(25))
      place(dy: u(4 * s), rect(width: u(19 * s), height: u(15 * s), radius: u(0.8 * s), fill: luma(190), stroke: edge))
      place(dy: u(18 * s), rect(width: u(19 * s), height: u(1.5 * s), fill: luma(120)))
    }
  })
}

#let fit(r, w, h) = {
  let (W, H) = extents(r)
  calc.min(w / W, h / H, 9)
}
