"""The thinnest wall of a printed part, measured on its mesh.

From points spread over the surface (fixed seed), a ray goes inward along the
face normal; the distance to the opposite surface is the wall there. This found
a 0.15 mm fin on a lid that JLC3DP's own upload check passed, and it is the
number case.MIN_WALL is enforced against.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import trimesh
from build123d import Part


@dataclass(frozen=True)
class Thinnest:
    mm: float
    where: tuple[float, float, float]  # a surface point at the thinnest wall


def to_mesh(part: Part, tolerance: float = 0.05) -> trimesh.Trimesh:
    verts, tris = part.tessellate(tolerance)
    return trimesh.Trimesh(
        vertices=np.array([(v.X, v.Y, v.Z) for v in verts]),
        faces=np.array(tris),
        process=True,
    )


def thinnest(part: Part, samples: int = 20000, seed: int = 1) -> Thinnest:
    mesh = to_mesh(part)
    points, faces = trimesh.sample.sample_surface(mesh, samples, seed=seed)
    inward = -mesh.face_normals[faces]
    origins = points + inward * 1e-3
    hits, rays, _ = mesh.ray.intersects_location(origins, inward, multiple_hits=False)
    depth = np.full(len(points), np.inf)
    depth[rays] = np.linalg.norm(hits - origins[rays], axis=1)
    i = int(np.argmin(depth))
    return Thinnest(float(depth[i]), tuple(round(float(c), 2) for c in points[i]))
