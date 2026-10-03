"""JAX CPU evaluator for packed kinematic constraints.

The compiled functions take marker positions, marker rotations, axes, endpoint
slots, and joint types as arguments. The cache key is the executable shape:
slot count, joint count, dtype, CPU device, and formulation. Marker values do
not create a new executable. The cache is capped. Releasing a session drops
Python references to those executables; that does not promise the JAX runtime
returns all of its memory to the operating system.

The Levenberg-Marquardt loop stays in NumPy. Each evaluation copies results
back to host buffers.
"""

from __future__ import annotations

from collections import OrderedDict

import numpy as np

from core.kinematics.prepared import PreparedModel


CACHE_CAP = 8


def _load_jax():
    try:
        import jax
        import jax.numpy as jnp
    except ImportError as exc:
        raise ImportError(
            "JAX is required for the kinematic solver and is not installed. "
            "Install it in this environment with: pip install jax==0.6.2. "
            "The application will not fall back to another solver."
        ) from exc
    jax.config.update("jax_enable_x64", True)
    return jax, jnp


class JaxCpuEvaluator:
    name = "jax"
    device = "cpu"

    def __init__(self):
        self._compiled: OrderedDict = OrderedDict()
        self.compile_s = 0.0
        self.compile_count = 0
        self.materialize_s = 0.0
        self.copy_count = 0
        self.cache_cap = CACHE_CAP
        self._jax = None
        self._jnp = None
        self._bound_key = None
        self._bound_args = None

    def _runtime(self):
        if self._jax is None:
            self._jax, self._jnp = _load_jax()
        return self._jax, self._jnp

    def cache_key(self, model: PreparedModel):
        return (
            int(model.n_slots),
            int(model.n_joints),
            "float64",
            "cpu",
            str(model.formulation_version),
        )

    def bind_constants(self, model: PreparedModel):
        """Device copies of marker and topology arrays, refreshed by content signature."""
        # Re-preparing the same topology resets constant_revision. The cached
        # signature is calculated during preparation/refresh, not per call.
        key = (model.structural_signature, model.constant_signature)
        if key == self._bound_key and self._bound_args is not None:
            return self._bound_args
        jax, _jnp = self._runtime()
        cpu_device = jax.devices("cpu")[0]

        def put(values):
            return jax.device_put(np.asarray(values), cpu_device)

        self._bound_args = (
            put(np.asarray(model.slot1, dtype=np.int32)),
            put(np.asarray(model.slot2, dtype=np.int32)),
            put(np.asarray(model.joint_type, dtype=np.int32)),
            put(np.asarray(model.marker1_origin, dtype=np.float64)),
            put(np.asarray(model.marker2_origin, dtype=np.float64)),
            put(np.asarray(model.marker1_R, dtype=np.float64)),
            put(np.asarray(model.marker2_R, dtype=np.float64)),
            put(np.asarray(model.axis_local, dtype=np.float64)),
        )
        self._bound_key = key
        return self._bound_args

    def _function(self, model: PreparedModel):
        key = self.cache_key(model)
        cached = self._compiled.get(key)
        if cached is not None:
            self._compiled.move_to_end(key)
            return cached

        import time

        jax, jnp = self._runtime()
        n_joints = int(model.n_joints)
        cpu_device = jax.devices("cpu")[0]

        def skew(v):
            x, y, z = v[0], v[1], v[2]
            zero = jnp.asarray(0.0, dtype=v.dtype)
            return jnp.array([[zero, -z, y], [z, zero, -x], [-y, x, zero]])

        def log_so3(R):
            cos_theta = jnp.clip((jnp.trace(R) - 1.0) * 0.5, -1.0, 1.0)
            theta = jnp.arccos(cos_theta)
            vee = jnp.array([
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1],
            ])

            def near_identity(_):
                return 0.5 * vee

            def regular(_):
                factor = theta / (2.0 * jnp.sin(theta))
                return factor * vee

            def near_pi(_):
                A = 0.5 * (R + jnp.eye(3, dtype=R.dtype))
                axis = jnp.sqrt(jnp.maximum(jnp.diag(A), 0.0))
                axis = axis.at[1].set(jnp.where(A[0, 1] < 0.0, -axis[1], axis[1]))
                axis = axis.at[2].set(jnp.where(A[0, 2] < 0.0, -axis[2], axis[2]))
                norm = jnp.linalg.norm(axis)
                return jax.lax.cond(
                    norm < 1e-12,
                    lambda _: jnp.array([jnp.pi, 0.0, 0.0]),
                    lambda _: theta * axis / norm,
                    operand=None,
                )

            return jax.lax.cond(
                theta < 1e-9,
                near_identity,
                lambda _: jax.lax.cond(
                    jnp.abs(jnp.pi - theta) < 1e-6,
                    near_pi,
                    regular,
                    operand=None,
                ),
                operand=None,
            )

        def one_joint(origin, rotation, slot1, slot2, joint_type, marker1_o, marker2_o,
                      marker1_R, marker2_R, axes, k):
            s1, s2 = slot1[k], slot2[k]
            o1 = origin[s1] + rotation[s1] @ marker1_o[k]
            o2 = origin[s2] + rotation[s2] @ marker2_o[k]
            R1 = rotation[s1] @ marker1_R[k]
            R2 = rotation[s2] @ marker2_R[k]
            jt = joint_type[k]
            axis = axes[k]
            a1 = R1 @ axis
            a2 = R2 @ axis

            # Each joint uses the same six-row padded layout. The host packs
            # spherical joints back to three rows for the shared engine.
            point = o1 - o2
            line = jnp.cross(a1, o2 - o1)
            orientation = log_so3(R2 @ R1.T)
            axes_parallel = jnp.cross(a1, a2)
            point_active = ((jt == 0) | (jt == 1) | (jt == 4))
            line_active = (jt == 2) | (jt == 3)
            angle_log_active = (jt == 0) | (jt == 2)
            angle_axis_active = (jt == 1) | (jt == 3)
            position = jnp.where(point_active, point,
                                 jnp.where(line_active, line, jnp.zeros(3)))
            angle = jnp.where(angle_log_active, orientation,
                              jnp.where(angle_axis_active, axes_parallel, jnp.zeros(3)))
            I = jnp.eye(3, dtype=origin.dtype)
            zero_block = jnp.zeros((6, 6), dtype=origin.dtype)
            point1 = jnp.concatenate((I, -skew(o1 - origin[s1])), axis=1)
            point2 = jnp.concatenate((I, -skew(o2 - origin[s2])), axis=1)
            block1 = zero_block.at[0:3, :].set(
                jnp.where(point_active, point1, jnp.zeros((3, 6)))
            )
            block2 = zero_block.at[0:3, :].set(
                jnp.where(point_active, -point2, jnp.zeros((3, 6)))
            )

            # Keep the same compat-v1 linearization, including its constant
            # +/-I orientation blocks for fixed and prismatic joints.
            rot1 = skew(a2) @ skew(a1)
            rot2 = -skew(a1) @ skew(a2)
            block1 = block1.at[3:6, 3:6].set(jnp.where(
                angle_log_active, -I, jnp.where(angle_axis_active, rot1, jnp.zeros((3, 3)))
            ))
            block2 = block2.at[3:6, 3:6].set(jnp.where(
                angle_log_active, I, jnp.where(angle_axis_active, rot2, jnp.zeros((3, 3)))
            ))

            delta = o2 - o1
            line1 = skew(a1) @ (-point1)
            line1 = line1.at[:, 3:6].add((-skew(delta)) @ (-skew(a1)))
            line2 = skew(a1) @ point2
            block1 = block1.at[0:3, :].set(jnp.where(
                line_active, line1, jnp.where(point_active, point1, jnp.zeros((3, 6)))
            ))
            block2 = block2.at[0:3, :].set(jnp.where(
                line_active, line2, jnp.where(point_active, -point2, jnp.zeros((3, 6)))
            ))
            return jnp.concatenate((position, angle)), jnp.stack((block1, block2))

        def residual_only(origin, rotation, slot1, slot2, joint_type, marker1_o, marker2_o,
                          marker1_R, marker2_R, axes):
            indices = jnp.arange(n_joints, dtype=jnp.int32)

            def at(k):
                residual, _blocks = one_joint(
                    origin, rotation, slot1, slot2, joint_type, marker1_o, marker2_o,
                    marker1_R, marker2_R, axes, k,
                )
                return residual

            return jax.vmap(at)(indices)

        def evaluate(origin, rotation, slot1, slot2, joint_type, marker1_o, marker2_o,
                     marker1_R, marker2_R, axes):
            indices = jnp.arange(n_joints, dtype=jnp.int32)

            def at(k):
                return one_joint(
                    origin, rotation, slot1, slot2, joint_type, marker1_o, marker2_o,
                    marker1_R, marker2_R, axes, k,
                )

            residuals, blocks = jax.vmap(at)(indices)
            return residuals, blocks

        started = time.perf_counter()
        compiled_residual = jax.jit(residual_only)
        compiled_full = jax.jit(evaluate)
        dummy_origin = jax.device_put(np.zeros((model.n_slots, 3), dtype=np.float64), cpu_device)
        dummy_rotation = jax.device_put(np.tile(np.eye(3), (model.n_slots, 1, 1)), cpu_device)
        dummy_constants = self._dummy_constants(model, cpu_device)
        with jax.default_device(cpu_device):
            compiled_residual(dummy_origin, dummy_rotation, *dummy_constants).block_until_ready()
            full_residual, full_blocks = compiled_full(dummy_origin, dummy_rotation, *dummy_constants)
        full_residual.block_until_ready()
        full_blocks.block_until_ready()
        self.compile_s += time.perf_counter() - started
        self.compile_count += 2
        self._compiled[key] = (compiled_residual, compiled_full, cpu_device)
        cap = max(int(self.cache_cap), 1)
        while len(self._compiled) > cap:
            self._compiled.popitem(last=False)
        return self._compiled[key]

    def _dummy_constants(self, model: PreparedModel, cpu_device):
        jax, _jnp = self._runtime()

        def put(values):
            return jax.device_put(np.asarray(values), cpu_device)

        n_joints = int(model.n_joints)
        return (
            put(np.zeros(n_joints, dtype=np.int32)),
            put(np.zeros(n_joints, dtype=np.int32)),
            put(np.zeros(n_joints, dtype=np.int32)),
            put(np.zeros((n_joints, 3), dtype=np.float64)),
            put(np.zeros((n_joints, 3), dtype=np.float64)),
            put(np.tile(np.eye(3), (n_joints, 1, 1))),
            put(np.tile(np.eye(3), (n_joints, 1, 1))),
            put(np.zeros((n_joints, 3), dtype=np.float64)),
        )

    def prewarm(self, model: PreparedModel, origin: np.ndarray, rotation: np.ndarray) -> None:
        if model.n_joints == 0:
            return
        compiled_residual, compiled_full, cpu_device = self._function(model)
        constants = self.bind_constants(model)
        with self._jax.default_device(cpu_device):
            compiled_residual(origin, rotation, *constants).block_until_ready()
            residual, blocks = compiled_full(origin, rotation, *constants)
        residual.block_until_ready()
        blocks.block_until_ready()

    def evaluate(
        self,
        model: PreparedModel,
        origin: np.ndarray,
        rotation: np.ndarray,
        joint_indices: np.ndarray,
        residual_out: np.ndarray,
        blocks_out: np.ndarray,
        write_blocks: bool,
    ) -> int:
        if model.n_joints == 0:
            return 0
        import time

        compiled_residual, compiled_full, cpu_device = self._function(model)
        constants = self.bind_constants(model)
        started = time.perf_counter()
        with self._jax.default_device(cpu_device):
            if write_blocks:
                residuals, blocks = compiled_full(origin, rotation, *constants)
                blocks = np.asarray(blocks)
            else:
                residuals = compiled_residual(origin, rotation, *constants)
                blocks = None
        residuals = np.asarray(residuals)
        self.materialize_s += time.perf_counter() - started
        self.copy_count += 2 if write_blocks else 1
        cursor = 0
        for index in np.asarray(joint_indices, dtype=np.int32).tolist():
            width = int(model.n_rows[index])
            residual_out[cursor:cursor + width] = residuals[index, :width]
            if write_blocks:
                blocks_out[index, :, :width, :] = blocks[index, :, :width, :]
            cursor += width
        return cursor

    def release(self) -> None:
        """Drop compiled functions and device constants held by this session."""
        self._compiled.clear()
        self._bound_args = None
        self._bound_key = None
