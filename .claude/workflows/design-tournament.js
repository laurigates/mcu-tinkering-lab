export const meta = {
  name: 'design-tournament',
  description: 'N designers with distinct briefs compete on one design problem; each is inspected and revised, judged through several lenses, and the winner is refined with grafts and adversarially reviewed',
  whenToUse: 'A design problem with a wide solution space (a printed mechanism, a bracket, an enclosure) where comparing independent attempts beats iterating one. Only when the user asks for a multi-agent run. Write the shared fixtures and gates first (printed-mechanism skill).',
  phases: [
    { title: 'Design', detail: 'one entry per brief' },
    { title: 'Inspect', detail: 'an independent skeptic per entry re-runs the gates' },
    { title: 'Revise', detail: 'each designer fixes blockers and majors' },
    { title: 'Judge', detail: 'one judge per lens scores every entry' },
    { title: 'Refine', detail: 'the winner, with grafts, into the final directory' },
    { title: 'Review', detail: 'adversarial review of the final design, then fixes' },
  ],
}

// args (all paths absolute):
//   problem   what is being designed and why, with every fact the designers need
//             (parts, constraints, materials, the user's asks). Shared by every agent.
//   root      directory holding common/ (shared fixtures and gates) and entries/
//   gates     the commands that define "passes", with {dir} for a design directory,
//             e.g. "just mechanism::check {dir}" and "just mechanism::showcase {dir}"
//   entrants  [{slug, brief}]: one designer each; make the briefs pull apart
//   lenses    [[key, description]]: one judge each
//   final     directory name under root for the refined winner (default "print")
//   deliverables  what an entry directory must contain (default: design.py, README.md,
//             gate outputs)
// The pan-tilt run (packages/robocar/unified/hardware/pan-tilt) is the worked example.

const A = args || {}
for (const k of ['problem', 'root', 'gates', 'entrants', 'lenses']) {
  if (!A[k]) throw new Error(`design-tournament: args.${k} is required`)
}
const ROOT = A.root
const FINAL = A.final || 'print'
const GATES = [].concat(A.gates)
const gate = dir => GATES.map(g => '`' + g.replaceAll('{dir}', dir) + '`').join(', then ')
const DELIVER = A.deliverables || `design.py (parametric; regenerates every print file), README.md (concept;
per part: process, material, colour, volume, estimated price; hardware BOM; assembly steps;
what to measure before ordering; risks), and the outputs of the gates`

const COMMON = `
## The problem (shared by every agent)
${A.problem}

## Shared fixtures and gates
${ROOT}/common/ holds the shared stand-ins and gates. Use them; do not redefine them. If one is
wrong, say so in your README and your result, and leave common/ alone.
A design passes when ${gate('<dir>')} succeed. Look at every render with the Read tool.
WebSearch and WebFetch are deferred tools: load them with ToolSearch ("select:WebSearch,WebFetch")
for process, material and part research. Lint any Python through \`pre-commit run --files\`.
Do not commit or touch git.
`

const DESIGN = {
  type: 'object',
  properties: {
    slug: { type: 'string' }, title: { type: 'string' },
    concept: { type: 'string', description: '3-6 sentences' },
    passes: { type: 'boolean', description: 'every gate succeeded on the final files' },
    fixture_issues: { type: 'array', items: { type: 'string' } },
    risks: { type: 'array', items: { type: 'string' } },
  },
  required: ['slug', 'title', 'concept', 'passes', 'risks'],
}
const ISSUES = {
  type: 'object',
  properties: {
    gates_reproduce: { type: 'boolean' },
    issues: { type: 'array', items: { type: 'object', properties: {
      severity: { type: 'string', enum: ['blocker', 'major', 'minor'] },
      what: { type: 'string' }, evidence: { type: 'string' }, fix: { type: 'string' } },
      required: ['severity', 'what', 'evidence'] } },
  },
  required: ['gates_reproduce', 'issues'],
}
const SCORES = {
  type: 'object',
  properties: { scores: { type: 'array', items: { type: 'object', properties: {
    slug: { type: 'string' }, score: { type: 'number', description: '0-10' },
    rationale: { type: 'string' },
    best_idea: { type: 'string', description: 'one idea worth grafting onto another entry' } },
    required: ['slug', 'score', 'rationale'] } } },
  required: ['scores'],
}
const serious = r => (r?.issues || []).filter(i => i.severity !== 'minor')

phase('Design')
const field = (await pipeline(
  A.entrants,
  e => agent(`You are an entrant in a ${A.entrants.length}-way design tournament.
Your slug: ${e.slug}. Your brief: ${e.brief}
${COMMON}
Deliver in ${ROOT}/entries/${e.slug}/ (create it, touch nothing else): ${DELIVER}.
Work until every gate passes and the renders look right.`,
    { label: `design:${e.slug}`, phase: 'Design', schema: DESIGN }),
  (d, e) => agent(`You are an independent, skeptical inspector of the "${e.slug}" entry. Do not trust its
README or its self-report. ${COMMON}
Inspect ${ROOT}/entries/${e.slug}/: re-run every gate yourself; then hunt for what the gates cannot
see: assembly order and tool access, load paths, cables over the whole travel, allow-excuses that
hide a clash, dependence on an estimated dimension, tolerance stacks with no take-up, and whether
the stated process and material can make each part. Write ${ROOT}/entries/${e.slug}/INSPECTION.md.
Severity: blocker = will not assemble or work; major = works badly or likely to fail; minor = polish.`,
    { label: `inspect:${e.slug}`, phase: 'Inspect', schema: ISSUES }).then(ins => ({ d, ins })),
  ({ d, ins }, e) => {
    if (!d) return null
    if (!serious(ins).length) return { slug: e.slug, design: d, inspection: ins }
    return agent(`You designed the "${e.slug}" entry (brief: ${e.brief}). An inspector found:
${JSON.stringify(serious(ins), null, 1)}
${COMMON}
Fix every blocker and major in ${ROOT}/entries/${e.slug}/ (or show concretely why one is not real),
re-run the gates, update the README, and append a "Revision" section to INSPECTION.md.`,
      { label: `revise:${e.slug}`, phase: 'Revise', schema: DESIGN })
      .then(r => ({ slug: e.slug, design: r || d, inspection: ins }))
  },
)).filter(Boolean)
const lost = A.entrants.map(e => e.slug).filter(s => !field.some(f => f.slug === s))
if (lost.length) log(`Not judged (an agent failed): ${lost.join(', ')}`)
if (!field.length) return { error: 'no entry survived design', lost }

phase('Judge')
const judged = (await parallel(A.lenses.map(([lens, desc]) => () => agent(
  `You judge a design tournament. Your lens: ${desc}.
Entries: ${ROOT}/entries/<slug>/ (design.py, README.md, INSPECTION.md, gate outputs, renders: look at
them with Read). Slugs: ${field.map(f => f.slug).join(', ')}. ${COMMON}
Read every entry yourself. Score each 0-10 through your lens only, with a concrete rationale, and
name each one's best idea worth grafting onto another. Do not edit files.`,
  { label: `judge:${lens}`, phase: 'Judge', schema: SCORES })))).filter(Boolean)
if (judged.length < A.lenses.length) log(`${A.lenses.length - judged.length} judge(s) failed; ranking on ${judged.length}`)

const totals = {}
for (const j of judged) for (const s of j.scores) totals[s.slug] = (totals[s.slug] || 0) + s.score
const ranking = Object.entries(totals).sort((a, b) => b[1] - a[1])
if (!ranking.length) return { error: 'no judge returned scores', entries: field.map(f => f.slug) }
log(`Ranking: ${ranking.map(([s, t]) => `${s} ${t.toFixed(1)}`).join(' | ')}`)
const winner = ranking[0][0]
const grafts = judged.flatMap(j => j.scores.filter(s => s.slug !== winner && s.best_idea).map(s => `${s.slug}: ${s.best_idea}`))
const notes = judged.flatMap(j => j.scores.filter(s => s.slug === winner).map(s => s.rationale))

phase('Refine')
const refined = await agent(`The tournament is decided. Winner: "${winner}" (${ROOT}/entries/${winner}/).
Ranking (sum over judges): ${JSON.stringify(ranking)}.
Judges on the winner: ${JSON.stringify(notes, null, 1)}
Grafting candidates: ${JSON.stringify(grafts, null, 1)}
${COMMON}
Build the version to order in ${ROOT}/${FINAL}/, leaving the entry untouched as the record: fix the
judges' criticisms; graft at most three ideas that clearly help (say which, and which you rejected
and why); pass every gate on ${ROOT}/${FINAL}. Write ${ROOT}/${FINAL}/README.md as the order sheet
(per part: file, process, material, colour, quantity, volume, price, orientation note; hardware BOM;
measure-before-ordering list; assembly steps) and ${ROOT}/README.md as the tournament record (each
entry with a render, the scores table, why the winner won, what was grafted, how to re-run the
gates). Return files written, grafts taken and rejected, and open risks.`,
  { label: `refine:${winner}`, phase: 'Refine' })

phase('Review')
const review = await agent(`Adversarially review ${ROOT}/${FINAL}/ before money is spent on it. Assume it is wrong
somewhere and find where. ${COMMON}
Re-run every gate. Check: each print file matches what design.py writes; every part assembles in the
stated order with tool access at each step; load paths; cables over the full travel; walls against
the stated process; hole sizes for the named screws; the order sheet against the files; every claim
in the READMEs against the geometry; the measure list covers every estimate the design depends on.
Do not edit files.`, { label: 'review:final', phase: 'Review', schema: ISSUES })

let fixed = null
if (serious(review).length) {
  fixed = await agent(`Fix these review findings on ${ROOT}/${FINAL}/:
${JSON.stringify(review.issues, null, 1)}
${COMMON}
Fix every blocker and major (minors where cheap), regenerate the print files and re-run every gate.
For a finding you judge not real, give the evidence. Return what changed per finding.`,
    { label: 'fix:final', phase: 'Review' })
}

return { ranking, winner, lost, grafts, refined, review, fixed }
