"""Joint-graph topology for the kinematic solver.

Bodies are nodes, joints are edges.  We use union-find to split the assembly
into connected components so the solver can:

  * solve only the component(s) that need solving (e.g. the one containing the
    dragged body), leaving unrelated bodies untouched — this is the SolveSpace
    "group decomposition" idea, and
  * report which bodies belong to which mechanism.

Ground (body id == -1) is a normal node; any component containing it is
anchored.
"""

from __future__ import annotations

from typing import Dict, Iterable, List, Optional, Set

from core.data_structures import Joint


class JointGraph:
    """Connectivity over bodies induced by joints (union-find)."""

    def __init__(self, body_ids: Iterable[int], joints: Iterable[Joint],
                 ground_id: int = -1):
        self.ground_id = ground_id
        self._parent: Dict[int, int] = {}
        self._rank: Dict[int, int] = {}

        for b in body_ids:
            self._parent[b] = b
            self._rank[b] = 0
        # Ground always exists as a node so joints-to-ground work.
        self._parent.setdefault(ground_id, ground_id)
        self._rank.setdefault(ground_id, 0)

        for j in joints:
            self._ensure_node(j.body1_id)
            self._ensure_node(j.body2_id)
            self.union(j.body1_id, j.body2_id)

    # ------------------------------------------------------------------
    def _ensure_node(self, x: int):
        if x not in self._parent:
            self._parent[x] = x
            self._rank[x] = 0

    def find(self, x: int) -> int:
        """Path-halving find."""
        root = x
        while self._parent[root] != root:
            root = self._parent[root]
        while self._parent[x] != root:
            self._parent[x], x = root, self._parent[x]
        return root

    def union(self, a: int, b: int):
        ra, rb = self.find(a), self.find(b)
        if ra == rb:
            return
        if self._rank[ra] < self._rank[rb]:
            ra, rb = rb, ra
        self._parent[rb] = ra
        if self._rank[ra] == self._rank[rb]:
            self._rank[ra] += 1

    def connected(self, a: int, b: int) -> bool:
        return self.find(a) == self.find(b)

    # ------------------------------------------------------------------
    def component_of(self, body_id: int) -> Set[int]:
        """All body ids in the same connected component as ``body_id``."""
        root = self.find(body_id)
        return {b for b in self._parent if self.find(b) == root}

    def components(self) -> List[Set[int]]:
        """All connected components as a list of body-id sets."""
        groups: Dict[int, Set[int]] = {}
        for b in self._parent:
            groups.setdefault(self.find(b), set()).add(b)
        return list(groups.values())

    def is_anchored(self, body_id: int) -> bool:
        """True if this body's component contains ground."""
        return self.connected(body_id, self.ground_id)

    def component_joints(self, joints: Iterable[Joint], body_id: int) -> List[Joint]:
        """Joints whose both endpoints lie in body_id's component."""
        comp = self.component_of(body_id)
        return [j for j in joints
                if j.body1_id in comp and j.body2_id in comp]
