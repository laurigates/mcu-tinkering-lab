# Printed Mechanisms: Use the `printed-mechanism` Skill

Parts that move or carry a servo are designed in build123d and gated by
`tools/mechanism` (`just mechanism::check`, `::showcase`). The contract, the
checklist and the traps are in the `printed-mechanism` skill
(`.claude/skills/printed-mechanism/`). Three matter at edit time:

1. **Gate over the travel, not at rest.** `just mechanism::check` sweeps every
   pose and must exit 0; read the `excused` list in `check.json`, since an
   `allow` can hide a clash.
2. **Give fasteners and cables their own groups.** Bodies in one group are
   never checked against each other, so a screw grouped with its servo, or a
   lead grouped with the part it runs along, can clash unseen.
3. **Name every estimate a part depends on, and what to measure.** Servo tab
   thickness, horn height and below-tab depth vary between makers and
   batches; a design that needs them exact needs a shim, a slot or a caliper
   reading before the order.
