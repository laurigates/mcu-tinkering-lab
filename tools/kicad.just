# Shared KiCad carrier-PCB recipes (tools/kicad, ADR-027).
# Import from a project justfile:  import '../../../tools/kicad.just'
#
# Every recipe takes the repo-relative project path as its argument; the
# project's public recipes pass their own project_dir. Recipes cd to the repo
# root, so they behave the same from the project directory and as a module
# (`just <project>::carrier-check`).
#
# Provides private recipes; the project defines the public ones, as with pico.just:
#   _carrier-lib, _carrier-schematic, _carrier-board, _carrier-route, _carrier-check,
#   _carrier-board-checked, _carrier-review, _carrier-fab, _perfboard, _perfboard-check
# KiCad-free: _carrier-lib, _carrier-schematic, _perfboard, _perfboard-check.
# Need KiCad 10.0.6: _carrier-check, _carrier-fab (kicad-cli), _carrier-board,
# _carrier-route, _carrier-review (KiCad's bundled Python; _carrier-route and
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
_carrier-review project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.boardinput {{project}}
    cd {{_kicad_repo}} && {{_kicad_python}} tools/kicad/pcb/review.py {{project}}/hardware/carrier/output/board-input.json

_carrier-fab project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.fab {{project}}

# The project's hardware/perfboard/perfboard.toml, laid out from the carrier
# netlist and the vendored footprints.
_perfboard project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.perfboard {{project}}

_perfboard-check project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.perfboard --check {{project}}
