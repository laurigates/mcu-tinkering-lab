"""3D-printable cases for the KiCad carriers, generated with build123d.

Geometry comes from the committed board, never from a second copy of it:

- board.py: footprint positions, courtyards and mounting holes from the
  committed .kicad_pcb (through the stdlib ``kicad.sexpr`` reader), part
  heights from a KiCad STEP export of the same board, and whatever the STEP
  cannot say (parts with no 3D model) from the project's enclosure.toml.
- case.py: the open tray and the closed box around one piece of the board.
- __main__.py: ``python -m enclosure <project>`` writes STEP, STL and 3MF.

This package needs build123d, so it lives outside tools/kicad: that package's
unittest discover runs on a stdlib-only Python in CI and pre-commit.
"""
