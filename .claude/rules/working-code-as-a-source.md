---
paths:
  - 'packages/**/main/**'
  - 'packages/components/**'
  - 'docs/reference/**'
  - '**/hardware.toml'
  - '**/sdkconfig.defaults'
---

# Read Working Driver Code Before Choosing a Part's Operating Parameters

Datasheets for the cheap modules this repo uses often stop at the protocol and
say nothing about the numbers that make it work: the bus clock, the wait
between a command and its result, the init sequence, the retry or recovery
step. Drivers other people have running on real boards fill those gaps, and
GitHub is where they are. Read them **before** picking a value, and treat what
working code does as evidence second only to a datasheet that states it.

> 2026-11-04 (robocar-unified, PR #941): both RCWL-1601 rangefinders read
> `ERR` permanently. `sensor--rcwl-1601.md` documented the address, trigger,
> read format and conversion wait, with sources, and nothing about the clock,
> because none of its sources stated one. The firmware ran them at the bus's
> 400 kHz. At 400 kHz they ACK their address and NACK every transfer after it;
> at 100 kHz they read 5/5. Finding that took a 400/100 kHz bench comparison
> and a failed per-device attempt.

## When it applies

Before you set, for a part with no datasheet value for it:

- a bus clock (I2C, SPI, UART baud for a module protocol)
- a delay, timeout or conversion wait
- an init or reset sequence, or a register default you rely on
- a workaround for a known failure (bus lockup, brown-out, first-read garbage)

And when a part misbehaves and the datasheet does not explain why: search for
the symptom (`gh search issues`) before building a theory.

## How

Code search finds working drivers; issue search finds the failures.

```sh
gh search code "<part or chip> <protocol word>" --limit 30 --json repository,path,url --jq '.[] | "\(.repository.nameWithOwner) \(.path) \(.url)"'
gh search issues "<part> <symptom>" --limit 20 --json repository,title,url --jq '.[] | "\(.repository.nameWithOwner) \(.title) \(.url)"'
gh api repos/<owner>/<repo>/contents/<path> --jq .content | base64 -d   # read a hit
```

- **Search the chip as well as the module name.** The RCWL-1601's chip is the
  RCWL-9600, and its siblings (RCWL-9620, M5Stack Unit Sonic, RCWL-1605)
  share the protocol. Libraries for a sibling are evidence for the part.
- **Exclude this repo.** Code search returns your own files first
  (`laurigates/mcu-tinkering-lab`); they are the claim under test, not a
  source.
- **Prefer maintained libraries and code with comments explaining a value.**
  A `delay(120)` with a comment saying why outweighs ten bare copies of it.
  Note whether the code ran on comparable hardware (3.3 V, ESP32, a mux).
- **Record what each source states, separately from what it merely does.**
  "M5Stack's library calls `delay(120)`" is a fact about the library; "the
  module needs 120 ms" is an inference. The datasheet summaries under
  `docs/reference/datasheets/` already cite this way; keep doing it, with the
  repo, path and line.
- **No hit is a finding too.** Write "no driver found that sets a clock" in
  the summary rather than leaving the field out, so the next reader knows the
  value is unverified and goes to the bench.

## Then confirm on the bench

Working code elsewhere is evidence, not proof: their wiring, pull-ups, supply
and bus traffic differ. Change one variable at a time and record the result
next to the sources, as `sensor--rcwl-1601.md` does for the clock.

## Related

- `board-layout-from-vendor-files.md`: the vendor's own board file on GitHub
  for physical pin order
- `.claude/skills/board-reference/SKILL.md` § 3: where working code sits in
  the source order for a new board page
- `analog-fault-isolation.md` § "Read the datasheet; do not recall it"
