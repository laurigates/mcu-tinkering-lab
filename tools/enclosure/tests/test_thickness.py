"""The wall gate against a known artefact and a known real defect."""

import numpy as np
import pytest
import trimesh
from build123d import Align, Box, Pos
from enclosure import thickness

A = (Align.MIN,) * 3


def yoke_rim() -> tuple[trimesh.Trimesh, np.ndarray]:
    """The pan-tilt yoke's 0.01 mm reading, cut down to three facets.

    Its flange rim: a top facet (normal +z), the side facet that shares the
    rim edge and leans 0.4 degrees off vertical, and a floor 1.0 mm under the
    top. The sample point is the one seed 1 drew (rounded to 0.01 µm), 0.4 µm
    from the rim vertex. Its ray down runs along the side facet and hits it
    after 5 µm; the wall there is 1.0 mm. Coordinates are print/yoke.stl's
    (packages/robocar/unified/hardware/pan-tilt), rounded to 0.1 µm.
    """
    v = np.array(
        [
            [-0.5, 19.75, 47.1],
            [-4.7988, 21.2653, 47.1],
            [-5.8504, 21.0003, 47.1],
            [-5.8504, 21.0003, 47.1],
            [-4.7988, 21.2653, 47.1],
            [-6.2202, 20.8938, 45.1],
            [-10, 15, 46.1],
            [0, 25, 46.1],
            [0, 15, 46.1],
            [-10, 25, 46.1],
        ]
    )
    m = trimesh.Trimesh(v, [[0, 1, 2], [3, 4, 5], [6, 8, 7], [6, 7, 9]], process=False)
    return m, np.array([-5.85001, 21.00036, 47.1])


def test_a_ray_grazing_a_leaning_facet_reads_near_zero():
    """Control: the raw single-ray probe does fall for the artefact."""
    m, p = yoke_rim()
    assert thickness.ray_depth(m, p[None], m.face_normals[[0]])[0] < 0.01


def test_a_grazing_ray_is_not_reported_as_the_thinnest_wall():
    m, p = yoke_rim()
    assert thickness.corroborated_depth(m, p, 0) == pytest.approx(1.0, abs=0.01)


def test_a_real_thin_fin_still_reads_thin():
    """A 0.15 mm fin (head 1's camera-window lid had one) must not be
    smoothed away by the corroboration."""
    part = Box(20, 20, 3, align=A) + Pos(5, 0, 3) * Box(0.15, 20, 4, align=A)
    assert thickness.thinnest(part).mm == pytest.approx(0.15, abs=0.03)


def test_a_sample_beside_a_concave_corner_reads_the_wall_not_the_corner():
    """The ring must stay on the surface: a ring point past an inside corner
    sits in the material and reads near zero (the first cut of this gate read
    0.0 mm on a 1.0 mm snap-fit cradle)."""
    part = Box(20, 20, 2, align=A) + Pos(10, 0, 2) * Box(2, 20, 6, align=A)
    m = thickness.to_mesh(part)
    p = np.array([9.95, 10.0, 2.0])  # floor top, 0.05 mm from the rib's face
    face = int(m.nearest.on_surface([p])[2][0])
    assert thickness.corroborated_depth(m, p, face) == pytest.approx(2.0, abs=0.05)
