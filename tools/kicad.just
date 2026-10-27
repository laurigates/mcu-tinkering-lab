# Shared KiCad carrier-PCB recipes (tools/kicad, ADR-027).
# Import from a project justfile:  import '../../../tools/kicad.just'
#
# Every recipe takes the repo-relative project path as its argument; the
# project's public recipes pass their own project_dir. Recipes cd to the repo
# root, so they behave the same from the project directory and as a module
# (`just <project>::carrier-check`).
#
# Provides private recipes; the project defines the public ones, as with pico.just:
#   _carrier-lib, _carrier-schematic, _carrier-board, _carrier-silk, _carrier-route, _carrier-check,
#   _carrier-board-checked, _carrier-review, _carrier-measure, _carrier-fab,
#   _perfboard, _perfboard-check
# KiCad-free: _carrier-lib, _carrier-schematic, _perfboard, _perfboard-check.
# Need KiCad 10.0.6: _carrier-check, _carrier-fab (kicad-cli), _carrier-board, _carrier-silk,
# _carrier-route, _carrier-review, _carrier-measure (KiCad's bundled Python;
# _carrier-measure's `resistance` also uv, for numpy and scipy; _carrier-route and
# _carrier-board-checked also java >= 25 and the Freerouting 2.5.0 jar,
# $FREEROUTING_JAR). `just kicad::setup` installs all three.
#
# Variable names carry a _kicad_ prefix so they cannot collide with pico.just's
# or esp32*.just's (container_cmd, compose_file, tools_dir, _detected_*).

_kicad_repo := source_directory() / ".."
# KiCad's Python (it carries pcbnew): the macOS bundle's, else the system one,
# where the Linux packages install pcbnew.
_kicad_python := env("KICAD_PYTHON", if os() == "macos" { "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3" } else { "/usr/bin/python3" })
# Freerouting needs Java 25 or later; `just kicad::setup` installs a JRE here,
# and route_board.py runs the first `java` on the PATH.
_kicad_java_path := env("HOME", "") / ".cache/freerouting/jre/bin"

_carrier-lib:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.derive_xiao
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.sockets

_carrier-schematic project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.schematic {{project}}

# Rewrites the .kicad_pcb from scratch: placement only, no tracks
_carrier-board project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.boardinput {{project}}
    cd {{_kicad_repo}} && {{_kicad_python}} tools/kicad/pcb/build_board.py {{project}}/hardware/carrier/output/board-input.json

# Silkscreen only, on the routed board: board.toml [silk] positions and sizes,
# and footprint lines cut 0.15 mm back from pads. The copper is left as it is.
_carrier-silk project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.boardinput {{project}}
    cd {{_kicad_repo}} && {{_kicad_python}} tools/kicad/pcb/build_board.py --silk-only {{project}}/hardware/carrier/output/board-input.json

_carrier-route project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.boardinput {{project}}
    cd {{_kicad_repo}} && PATH="{{_kicad_java_path}}:$PATH" {{_kicad_python}} tools/kicad/pcb/route_board.py {{project}}/hardware/carrier/output/board-input.json

_carrier-check project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.check {{project}}

# Place, route and check, again until every gate passes or `tries` runs out:
# Freerouting does not route the same way twice, and a run can leave a
# connection unrouted, a track necked or a required fanout via without room.
# Stops at the first passing run, whose stamp carrier-check has written.
_carrier-board-checked project tries="5":
    #!/usr/bin/env bash
    set -uo pipefail
    cd {{_kicad_repo}}
    input={{project}}/hardware/carrier/output/board-input.json
    for i in $(seq 1 {{tries}}); do
        echo "== attempt $i of {{tries}}"
        PYTHONPATH=tools python3 -m kicad.boardinput {{project}} \
            && {{_kicad_python}} tools/kicad/pcb/build_board.py "$input" \
            && PATH="{{_kicad_java_path}}:$PATH" {{_kicad_python}} tools/kicad/pcb/route_board.py "$input" \
            && PYTHONPATH=tools python3 -m kicad.check {{project}} \
            && exit 0
    done
    echo "no passing board in {{tries}} attempt(s)" >&2
    exit 1

# The routed board's layout metrics a pre-fab review reads (vias per poured
# net, SMD ground pads without one, pour islands, narrowest power tracks,
# decoupling distances, cut-line clearances, silkscreen size). Reports; does
# not gate.
# Ends with kicad.measure's plane connectivity: pour pieces that vias and
# through-hole pads alone do not join.
_carrier-review project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.boardinput {{project}}
    cd {{_kicad_repo}} && {{_kicad_python}} tools/kicad/pcb/review.py {{project}}/hardware/carrier/output/board-input.json --dump {{project}}/hardware/carrier/output/measure.json
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.measure {{project}}/hardware/carrier/output/measure.json plane --review

# Layer-aware copper measurements (kicad.measure; pcb/dump.py writes the JSON):
# path NET A:pad B:pad, cuts NET A:pad, vias-in-pads, clearance-to-holes,
# plane, resistance NET A:pad B:pad; --json for JSON. `--ref <git-ref>`
# measures the board as committed at that ref, read with `git show` (never a
# checkout: the clone may be shared), placed with today's board.toml origin
# and filtered with today's [fanout] in_pad and [pour] regions.
# `resistance` runs under `uv run --with numpy --with scipy`; the rest is
# stdlib, as in every other kicad recipe.
[positional-arguments]
_carrier-measure project *args:
    #!/usr/bin/env bash
    set -euo pipefail
    cd {{_kicad_repo}}
    project="$1"
    # The public recipe passes its *args as one word; split it again.
    read -ra words <<< "${*:2}"
    ref=""
    argv=()
    i=0
    while [ "$i" -lt "${#words[@]}" ]; do
        w="${words[$i]}"
        case "$w" in
            --ref) ref="${words[$((i + 1))]:-}"; i=$((i + 2)); has_ref=1; continue ;;
            --ref=*) ref="${w#--ref=}"; has_ref=1 ;;
            *) argv+=("$w") ;;
        esac
        i=$((i + 1))
    done
    # A missing value would otherwise measure the working-tree board.
    if [ -n "${has_ref:-}" ] && { [ -z "$ref" ] || [ "${ref#-}" != "$ref" ]; }; then
        echo "carrier-measure: --ref needs a git ref (e.g. --ref HEAD~1), got '${ref}'" >&2
        exit 2
    fi
    if [ "${#argv[@]}" -eq 0 ]; then
        echo "usage: carrier-measure <path|cuts|vias-in-pads|clearance-to-holes|plane|resistance> [args] [--json] [--ref <git-ref>]" >&2
        exit 2
    fi
    out="$project/hardware/carrier/output"
    PYTHONPATH=tools python3 -m kicad.boardinput "$project" >&2
    pcb=()
    dumped="$out/measure.json"
    if [ -n "$ref" ]; then
        tmp="$(mktemp -d)"
        trap 'rm -rf "$tmp"' EXIT
        rel="$(python3 -c 'import json, os, sys; print(os.path.relpath(json.load(open(sys.argv[1]))["pcb"]))' "$out/board-input.json")"
        git show "$ref:$rel" > "$tmp/board.kicad_pcb"
        pcb=(--pcb "$tmp/board.kicad_pcb")
        dumped="$tmp/measure.json"
        echo "measuring $rel at $ref" >&2
    fi
    {{_kicad_python}} tools/kicad/pcb/dump.py "$out/board-input.json" "$dumped" ${pcb[@]+"${pcb[@]}"} >&2
    if [ "${argv[0]}" = resistance ]; then
        PYTHONPATH=tools uv run --quiet --no-project --python '>=3.11' --with numpy==2.5.4 --with scipy==1.18.1 python -m kicad.measure "$dumped" "${argv[@]}"
    else
        PYTHONPATH=tools python3 -m kicad.measure "$dumped" "${argv[@]}"
    fi

_carrier-fab project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.fab {{project}}

# The project's hardware/perfboard/perfboard.toml, laid out from the carrier
# netlist and the vendored footprints.
_perfboard project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.perfboard {{project}}

_perfboard-check project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.perfboard --check {{project}}
