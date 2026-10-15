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
#   _carrier-fab, _perfboard, _perfboard-check
# KiCad-free: _carrier-lib, _carrier-schematic, _perfboard, _perfboard-check.
# Need KiCad 10.0.6: _carrier-check, _carrier-fab (kicad-cli), _carrier-board,
# _carrier-route (KiCad's bundled Python; _carrier-route also java >= 25 and the
# Freerouting 2.5.0 jar, $FREEROUTING_JAR).
#
# Variable names carry a _kicad_ prefix so they cannot collide with pico.just's
# or esp32*.just's (container_cmd, compose_file, tools_dir, _detected_*).

_kicad_repo := source_directory() / ".."
_kicad_python := env("KICAD_PYTHON", "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3")

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
    cd {{_kicad_repo}} && {{_kicad_python}} tools/kicad/pcb/route_board.py {{project}}/hardware/carrier/output/board-input.json

_carrier-check project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.check {{project}}

_carrier-fab project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.fab {{project}}

# The project's hardware/perfboard/perfboard.toml, laid out from the carrier
# netlist and the vendored footprints.
_perfboard project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.perfboard {{project}}

_perfboard-check project:
    cd {{_kicad_repo}} && PYTHONPATH=tools python3 -m kicad.perfboard --check {{project}}
