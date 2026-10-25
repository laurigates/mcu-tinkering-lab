"""The thinnest wall of a printed part, measured on its mesh.

From points spread over the surface (fixed seed), a ray goes inward along the
face normal; the distance to the opposite surface is the wall there. This found
a 0.15 mm fin on a lid that JLC3DP's own upload check passed, and it is the
number case.MIN_WALL is enforced against.

One ray can lie. A sample on a convex rim whose side facet the tessellator
left a few thousandths off vertical casts its ray along that facet and hits it
within micrometres: the pan-tilt yoke read 0.01 mm where the wall is 1.0 mm
(one sample in 20 000; nine other seeds read 1.0). So the thinnest few
readings are each checked against a ring of nearby points on the same face:
a reading under half the ring's median is a graze and the median stands in
for it. A real thin wall is thin under the ring too and keeps its own reading.
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


CANDIDATES = 64  # thinnest raw readings re-measured
RING_MM = 0.1  # radius of the corroborating ring, well inside any real wall
RING_POINTS = 8
GRAZE_FRACTION = 0.5  # a centre under half its ring's median is a graze


def ray_depth(
    mesh: trimesh.Trimesh, points: np.ndarray, normals: np.ndarray
) -> np.ndarray:
    """Distance from each surface point, inward along its normal, to the next
    surface; inf where the ray leaves without a hit."""
    inward = -normals
    origins = points + inward * 1e-3
    hits, rays, _ = mesh.ray.intersects_location(origins, inward, multiple_hits=False)
    depth = np.full(len(points), np.inf)
    depth[rays] = np.linalg.norm(hits - origins[rays], axis=1)
    return depth


def corroborated_depth(mesh: trimesh.Trimesh, point: np.ndarray, face: int) -> float:
    """The wall under ``point``, unless a ring of points round it on the same
    face (or a coplanar neighbour) says that reading is a graze."""
    n = mesh.face_normals[face]
    u = np.cross(n, [1.0, 0, 0] if abs(n[0]) < 0.9 else [0, 1.0, 0])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    a = np.linspace(0, 2 * np.pi, RING_POINTS, endpoint=False)
    ring = point + RING_MM * (np.outer(np.cos(a), u) + np.outer(np.sin(a), v))
    # Only ring points on a face parallel to this one: past a concave edge the
    # ring is inside the material, and past a convex one it is in the air.
    _, off, tri = trimesh.proximity.closest_point(mesh, ring)
    ring = ring[(off < 1e-4) & (mesh.face_normals[tri] @ n > 0.999)]
    centre = ray_depth(mesh, point[None], n[None])[0]
    around = ray_depth(mesh, ring, np.tile(n, (len(ring), 1))) if len(ring) else []
    around = np.asarray(around)[np.isfinite(around)]
    if not len(around):
        return float(centre)
    # A real thin wall is thin under the ring too and keeps its exact reading;
    # a graze reads a small fraction of what surrounds it and is outvoted.
    m = float(np.median(around))
    return float(centre) if centre >= GRAZE_FRACTION * m else m


def thinnest(part: Part, samples: int = 20000, seed: int = 1) -> Thinnest:
    mesh = to_mesh(part)
    points, faces = trimesh.sample.sample_surface(mesh, samples, seed=seed)
    raw = ray_depth(mesh, points, mesh.face_normals[faces])
    best, where = np.inf, points[int(np.argmin(raw))]
    for i in np.argsort(raw)[:CANDIDATES]:
        d = corroborated_depth(mesh, points[i], int(faces[i]))
        if d < best:
            best, where = d, points[i]
    if not np.isfinite(best):  # no candidate corroborated: keep the raw reading
        best = float(raw.min())
    return Thinnest(float(best), tuple(round(float(c), 2) for c in where))
