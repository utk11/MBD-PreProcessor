"""Apply committed pose changes to the viewer in one batch.

A body is redrawn when its committed origin moves by more than
``RENDER_ORIGIN_METERS`` (1e-8 m) or any rotation-matrix entry changes by
more than ``RENDER_ROTATION`` (1e-8). Those thresholds are for rendering
only. They are not the solver tolerance.

The coordinator asks the viewer to update once per batch.
"""

from __future__ import annotations

from typing import Callable, Dict, Iterable, List, Optional, Sequence

import numpy as np

from core.transforms import body_world_pose


# Documented rendering thresholds. They do not affect the kinematic step.
RENDER_ORIGIN_METERS = 1e-8
RENDER_ROTATION = 1e-8


class RendererCoordinator:
    """Push document pose changes to AIS objects and request one viewer update."""

    def __init__(self, body_renderer, display, on_body_changed: Optional[Callable[[int], None]] = None):
        self.body_renderer = body_renderer
        self.display = display
        self.on_body_changed = on_body_changed
        self._shown_origin: Dict[int, np.ndarray] = {}
        self._shown_rotation: Dict[int, np.ndarray] = {}

    def forget(self, body_ids: Iterable[int]) -> None:
        for body_id in body_ids:
            self._shown_origin.pop(int(body_id), None)
            self._shown_rotation.pop(int(body_id), None)

    def sync_baselines(self, bodies: Sequence) -> None:
        """Remember poses that are already on screen, such as a fresh import."""
        self._shown_origin.clear()
        self._shown_rotation.clear()
        for body in bodies:
            origin, rotation = body_world_pose(body)
            self._shown_origin[int(body.id)] = origin
            self._shown_rotation[int(body.id)] = rotation

    def apply(self, body_ids: Sequence[int]) -> List[int]:
        """Update bodies whose poses actually changed, then update the viewer once."""
        changed: List[int] = []
        for body_id in body_ids:
            body_id = int(body_id)
            body = self.body_renderer.bodies_dict.get(body_id)
            if body is None:
                continue
            origin, rotation = body_world_pose(body)
            if not self._differs(body_id, origin, rotation):
                continue
            self.body_renderer.update_body_transform(body_id)
            self._shown_origin[body_id] = origin
            self._shown_rotation[body_id] = rotation
            if self.on_body_changed is not None:
                self.on_body_changed(body_id)
            changed.append(body_id)
        if changed:
            self.display.Context.UpdateCurrentViewer()
            self.display.Repaint()
        return changed

    def _differs(self, body_id: int, origin: np.ndarray, rotation: np.ndarray) -> bool:
        previous_origin = self._shown_origin.get(body_id)
        previous_rotation = self._shown_rotation.get(body_id)
        if previous_origin is None or previous_rotation is None:
            return True
        if float(np.max(np.abs(origin - previous_origin))) > RENDER_ORIGIN_METERS:
            return True
        if float(np.max(np.abs(rotation - previous_rotation))) > RENDER_ROTATION:
            return True
        return False
