"""Phase timings and the numerical trace attached to one solve request.

Timings are optional. A benchmark that wants the solver's share of time without
observer overhead leaves ``enabled`` false and measures the call from outside.
"""

from __future__ import annotations

import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Dict, Iterator, Optional


@dataclass
class PhaseTrace:
    enabled: bool = False
    phases_s: Dict[str, float] = field(default_factory=dict)
    iterations: int = 0
    accepted_steps: int = 0
    rejected_steps: int = 0
    numerical_failure: bool = False
    joint_feasible: bool = False
    pin_error: Optional[float] = None
    last_step_norm: float = 0.0
    iteration_limit: bool = False
    backend: str = ""
    formulation_version: str = ""
    grouping: str = ""
    copy_count: int = 0
    zero_copy: Optional[bool] = None
    device: str = "cpu"
    stale_revision: bool = False
    stale_model: bool = False
    compile_count: int = 0
    compile_s: float = 0.0
    linear_residual: float = 0.0
    linear_solver: str = ""
    linear_solves: int = 0
    linear_iterations: int = 0
    linear_failures: int = 0
    failure_reason: str = ""

    @contextmanager
    def phase(self, name: str) -> Iterator[None]:
        if not self.enabled:
            yield
            return
        start = time.perf_counter()
        try:
            yield
        finally:
            self.phases_s[name] = self.phases_s.get(name, 0.0) + (time.perf_counter() - start)

    def add_time(self, name: str, seconds: float) -> None:
        if self.enabled:
            self.phases_s[name] = self.phases_s.get(name, 0.0) + float(seconds)


@dataclass
class SolveOptions:
    """Compatibility defaults mirror ``KinematicSolver``."""

    max_iters: int = 50
    tol: float = 1e-9
    pin_weight: float = 100.0
    pin_orientation: bool = True
    joint_weight: float = 1e3
    analyze: bool = True
    diagnostics_tol: float = 1e-8
    trace: bool = False
    # Set by the solver session. None keeps the dense step.
    strategy: object = None
