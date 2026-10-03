"""Map pointer motion through the CAD camera into a displacement in meters."""

from __future__ import annotations

import numpy as np


def screen_drag_delta(view, start, current, reference_meters, unit_scale, pixel_ratio=1.0):
    """Intersect pointer rays with the view plane through the initial body pose.

    OCC uses physical pixels and CAD model units. The document uses meters.
    Ray intersections also account for camera rotation, zoom, and perspective.
    Taking a difference preserves the offset between the click and the body.
    """
    if not np.isfinite(unit_scale) or unit_scale <= 0:
        raise ValueError("CAD unit scale must be positive and finite.")
    if not np.isfinite(pixel_ratio) or pixel_ratio <= 0:
        raise ValueError("Pixel ratio must be positive and finite.")
    normal = np.asarray(view.Proj(), dtype=float)
    normal /= np.linalg.norm(normal)
    plane_point = np.asarray(reference_meters, dtype=float) / unit_scale

    def intersect(point):
        x, y = (int(round(value * pixel_ratio)) for value in point)
        ray = np.asarray(view.ConvertWithProj(x, y), dtype=float)
        origin, direction = ray[:3], ray[3:]
        denominator = float(normal @ direction)
        if abs(denominator) < 1e-12:
            raise ValueError("Pointer ray is parallel to the drag plane.")
        distance = float(normal @ (plane_point - origin)) / denominator
        return origin + distance * direction

    delta = (intersect(current) - intersect(start)) * unit_scale
    if not np.isfinite(delta).all():
        raise ValueError("Camera produced a non-finite drag displacement.")
    return delta
