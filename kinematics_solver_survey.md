# Kinematic Constraint Solver Library Survey

**Purpose:** choose an approach for *position-level* assembly/IK solving (solving body 6DOF poses so that revolute / prismatic / cylindrical / spherical / fixed joint constraints are satisfied), to embed in the existing PySide6 + numpy MBD-PreProcessor app (classes: `RigidBody`, `Joint`, `Frame`, `Pose` in `core/data_structures.py`).

**Scope note:** This is a *kinematic assembly* problem (solve Φ(q)=0), not a dynamics problem. Many "multibody" libraries below are dynamics-first and only touch this indirectly; that distinction drives the recommendations.

---

## 1. Library comparison

### 1.1 Constraint-solver / CAD-kernel class (solves the problem directly)

**python-solvespace (and the slvs library)**
- URL: https://github.com/solvespace/solvespace · https://pypi.org/project/python-solvespace (3.0.0, Nov 2022) · maintained Python binding: `py-slvs` https://pypi.org/project/py-slvs/
- License: **GPLv3** (both the C++ solver and the bindings) — this is the main practical obstacle for a proprietary-ish app; fine if MBD-PreProcessor is GPL-compatible.
- Maintenance: solvespace upstream is active (commits/issues through 2026); the original `python-solvespace` package is stale (2022, KmolYuan repo archived/deprecated), but `py-slvs` is a maintained wrapper of the same `slvs` core.
- Position-level solving: **Yes — this is exactly what it does.** It's a general geometric constraint solver (points, lines, circles, distances, angles, coincidence, parallelism, perpendicularity, points-on-entities, workplanes, etc.) in 2D and 3D.
- Joint types: no named "joints" — you compose revolute/prismatic/spherical mates from primitive constraints (coincident point + parallel axis ≈ revolute; coincident line ≈ prismatic/cylindrical; coincident point alone ≈ spherical; full lock ≈ fixed). All five of your joint types are expressible.
- Numerics: Newton-based nonlinear solve of constraint residuals over its own entity/parameter space, with group decomposition, redundant-constraint detection (reports over-constrained sketches), and rank-deficiency handling via SVD-ish least squares. Very battle-tested (powers SolveSpace CAD, FreeCAD assembly workbenches via `slvs`).
- Python API quality: low-level but usable (entity/constraint handles in a System object); `py-slvs` is pip-installable with wheels.
- Dependency weight: tiny (~1 compiled extension, no big deps).

**Drake (pydrake)**
- URL: https://drake.mit.edu · https://pypi.org/project/drake
- License: BSD-3. Maintenance: very active (monthly releases, TRI-backed; v1.x series through 2025-2026).
- Position-level solving: **Yes, but optimization-flavored.** `InverseKinematics` builds an NLP over generalized positions q with `AddPositionConstraint`, `AddOrientationConstraint`, `AddAngleBetweenVectorsConstraint`, etc., solved with SNOPT/IPOPT. Also does proper assembly of closed loops via its constraint system. Overkill but very capable.
- Joints: revolute, prismatic, spherical, planar, weld (fixed), 6-DOF free; no native cylindrical (compose from revolute+prismatic or add custom constraint).
- Numerics: gradient-based NLP solvers on position-level constraints (not a hand-rolled Newton on residuals; uses full mathematical-program machinery).
- Python API: excellent, `pip install drake`. Dependency weight: **very heavy** (~500MB+ wheel, Ubuntu-oriented; macOS/Windows support limited). Real friction for embedding in a PySide6 app.

### 1.2 Symbolic multibody class (derives equations; you'd still solve them yourself)

**sympy.physics.mechanics (+ joints framework)**
- URL: https://docs.sympy.org/latest/explanation/modules/physics/mechanics/joints.html
- License: BSD. Maintenance: active (sympy 1.13/1.14, 2025). Joints framework now covers Pin, Prismatic, Cylindrical, Planar, Spherical, Weld.
- Position-level solving: **Not directly** — it generates symbolic kinematics (including holonomic constraint equations Φ(q)=0 you can extract via `KanesMethod` with configuration constraints) that you then hand to `scipy.optimize` or lambdify. Workable for small mechanisms; symbolic derivation cost grows badly with model size.
- Python API: pure Python, beautiful for teaching, slow for big models. Deps: sympy/scipy only (light).

**PyDy**
- URL: https://pydy.readthedocs.io · https://github.com/pydy/pydy (0.9.4; changelog shows 0.8.0 Aug 2025)
- License: BSD. Maintenance: maintained, low tempo. Sits on sympy.mechanics; adds codegen, numerical System, visualization. Same position-level story as sympy: you get equations, you solve them. Deps: light (numpy/scipy/sympy/cython).

**pynamics**
- URL: https://pypi.org/project/pynamics/ (0.2.0, Jan 2022)
- License: permissive; maintenance: effectively stale (last release 2022, single-author Georgia Tech project). Symbolic dynamics via Kane's method. Not recommended — superseded by sympy.mechanics/PyDy.

### 1.3 Robotics IK / RBDL class (joint-space, tree-oriented)

**Pinocchio (`pin` on PyPI)**
- URL: https://github.com/stack-of-tasks/pinocchio · https://pypi.org/project/pin/
- License: BSD-2. Maintenance: very active (3.x series, v4 announced, commits through 2025-2026; conda-forge and pip wheels).
- Position-level solving: **Yes for kinematic trees** — FK + analytical Jacobians are its core; IK = Gauss-Newton/Levenberg-Marquardt on SE(3) error using those Jacobians (standard recipes, and `pink`/`pin-pink` on PyPI wraps this nicely). **Caveat:** closed loops need explicit loop-closure constraints (supported via constraints/Lagrange multipliers, but you manage them); it thinks in minimal joint coordinates q, not body 6DOF poses.
- Joints: revolute, prismatic, spherical (as composite), free-flyer, planar, helical; cylindrical via composite. URDF-driven modeling.
- Numerics: Lie-group-aware Gauss-Newton on q; CasADi binding available for AD.
- Python API: excellent; industry standard. Deps: moderate (eigenpy, hpp-fcl optional, boost-free since 3.x mostly).

**PyKDL (orocos KDL)**
- URL: https://github.com/orocos/orocos_kinematics_dynamics (python_orocos_kdl)
- License: LGPL-2.1. Maintenance: maintained but old-school (ROS-ecosystem; commits through 2025, no pip wheels — usually comes via ROS or conda robostack).
- Position-level solving: damped least-squares / Newton-Raphson IK solvers for chains (`ChainIkSolverPos_NR_JL`, `..._LMA`). Position-level, but chains/trees only — no closed loops, no general assembly.
- Joints: revolute, prismatic, fixed (continuous variants); no spherical/cylindrical primitives.
- Python API: clunky SWIG bindings. Deps: ROS-adjacent weight. For a PySide6 desktop app, Pinocchio supersedes it.

**RBDL (+ rbdl-orb python bindings)**
- URL: https://rbdl.github.io · https://github.com/rbdl/rbdl
- License: zlib (library); maintenance: slow but alive. Python bindings (`rbdl` pip via ORB fork) work.
- Position-level: has InverseKinematics addon (Newton/damped LS on constraints) but tree-oriented, joint-space, limited joint set (revolute/prismatic/spherical via multi-dof custom joints; no cylindrical). Narrower and rougher than Pinocchio.

**MuJoCo (and `mink` for IK)**
- URL: https://github.com/google-deepmind/mujoco · https://pypi.org/project/mujoco · IK: https://github.com/kevinzakka/mink
- License: Apache-2.0. Maintenance: extremely active (3.x series, DeepMind, wheels on pip).
- Position-level solving: **indirect.** MuJoCo simulates with equality constraints (connect, weld, joint limits) enforced by its constraint solver at acceleration level; `mj_kinematics` gives FK/Jacobians, and `mink` does differential IK (QP on joint velocities). There is no "assemble this mechanism" position-level solve entry point — you'd iterate differential IK to convergence, which works but is velocity-level.
- Joints: free, ball (spherical), slide (prismatic), hinge (revolute); equality constraints cover weld/connect → fixed and loop closure. Cylindrical = hinge+slide composite.
- Deps: single pip wheel, light. API: good. But impedance mismatch with "solve body poses once" is real.

### 1.4 General MBS dynamics class (position solve exists as pre-step / static solve)

**Exudyn**
- URL: https://github.com/jgerstmayr/EXUDYN · https://exudyn.readthedocs.io (v1.9.x, active 2024-2025)
- License: BSD-3. Maintenance: active (Univ. Innsbruck).
- Position-level solving: **Yes** — `SolveStatic`/`ComputeNewtonRaphson` solve nonlinear statics including pure kinematic assembly (kinematic tree → joints; also `robotics.InverseKinematicsNumerical` solving the static problem for serial arms). Joints impose position constraints with Lagrange multipliers; static solve with zero loads = constraint satisfaction. Works for closed loops.
- Joints: revolute, prismatic, spherical, generic, rigid (fixed); cylindrical available in joint library.
- Numerics: Newton-Raphson on position-level residual with analytic/numeric Jacobians, sparse solvers, augmented Lagrange for contacts.
- Python API: good, numpy-friendly, pip wheels. Deps: moderate (single extension + numpy/scipy/matplotlib optional). Realistic candidate.

**Project Chrono (pychrono)**
- URL: https://projectchrono.org/pychrono · https://github.com/projectchrono/chrono
- License: BSD-3. Maintenance: very active (9.x in 2025, NSF/ARO-funded).
- Position-level solving: has full inverse-kinematics module (`ChIKSolver`, NR and DLS variants) and assembly analysis in the MBS core; joints as bilateral constraints with Lagrange multipliers. Capable.
- Joints: revolute, prismatic, cylindrical, spherical, universal, weld — the full classic set.
- Python API: SWIG-generated, verbose C++-style; install mainly via conda (`conda install pychrono`), no plain pip wheel → **packaging friction for PySide6 app**. Deps: heavy.

**MBSim-env**
- URL: https://github.com/mbsim-env/mbsim (latest release Mar 2025)
- License: LGPL. Maintenance: active (TU München-backed). C++ core with XML/Python frontends (`mbsimgui`, `mbsimpython`); kinematics/initial-state via its own constraint machinery; full joint set. But Python integration is a niche path (fmatvec/hdf5serie build chain), aimed at Linux simulation pipelines — heavy to embed.

### 1.5 DIY class (this is what solvespace does under the hood, too)

**scipy.optimize.least_squares / root with custom Jacobian**
- Zero new deps (already have numpy/scipy). You write Φ(q) = 0 per joint type (e.g., revolute: 3 position + 2 axis-orthogonality equations = 5 constraints), stack residuals, supply analytic Jacobian, solve with `least_squares(method='trf' or 'lm')` → robust Levenberg-Marquardt with bounds and sparsity support (`trf` accepts sparse Jacobian, `x_scale`, loss functions).
- Handles over/under-constrained systems gracefully (least-squares), detects redundancy via Jacobian rank (SVD on a small system).
- All five joint types trivially expressible in body-6DOF coordinates. Full control over parameterization (e.g., quaternion + position, or incremental SE(3) updates).

**CasADi (+ Opti)**
- URL: https://web.casadi.org · license: LGPL-3.0 (with commercial option); active (3.7.x 2025). pip wheels, light deps.
- Build Φ(x)=0 symbolically → free AD Jacobian → solve with `rootfinder` (Newton/KINSOL) or as NLP least-squares with IPOPT. More machinery than scipy for this job; shines if you later want sensitivity analysis or trajectory optimization. LGPL is a consideration similar to solvespace's GPL (but CasADi is LGPL = linkable).

---

## 2. Algorithmic choices for a custom solver

### 2.1 Body-6DOF (absolute/maximal) coordinates vs minimal (joint-space) coordinates

**Body-6DOF / absolute coordinate formulation** (what ADAMS, Exudyn "generalized-alpha MBS", and your current data model naturally suggest):
- Unknowns: for each of N bodies, position (3) + orientation (quaternion 4 or rotation vector 3) → 6N-7N unknowns.
- Each joint contributes algebraic constraints Φ_j(q)=0: fixed = 6, revolute = 5 (3 coincident point + 2 axis ⊥), cylindrical = 4 (coincident-line: 2 pos + 2 dir), prismatic = 5 (2 axis-align + 2 relative-orientation + 1 point-in-plane variant), spherical = 3 (point coincidence).
- Plus N quaternion-norm constraints if using quaternions, or use incremental rotation-vector updates (local parameterization) to avoid that.
- Solve Φ(q)=0 with Newton-Raphson / LM; J = ∂Φ/∂q is sparse and has closed-form blocks. Over-constrained → least-squares with rank report (redundancy detection). Under-constrained → minimum-norm step, remaining DOFs free (that's a *feature*: mechanism mobility falls out of the null space).
- Pros: matches your `RigidBody/Joint/Frame/Pose` model 1:1; closed loops are automatic (just more rows in Φ); no graph analysis needed; trivial to add new joint types; redundancy/mobility analysis = Jacobian rank.
- Cons: bigger system than minimal coordinates; needs normalization/local parameterization care; singular configurations show up as rank loss (which you also *want* to detect).

**Minimal / joint-space (relative) coordinates** (Pinocchio/KDL/RBDL style):
- Unknowns: one scalar per joint DOF; kinematics computed by recursive FK along a spanning tree.
- Pros: tiny systems, no constraint violation drift, fast Jacobians.
- Cons: **closed loops break the tree** — you must cut loops and add closure constraints anyway (ending up back at a constrained system); requires graph/topology analysis (spanning tree, cut joints); spherical/cylindrical become composite joints; harder to map from your current "bodies + joints between arbitrary frames" data model. This is the right choice for serial robots, the wrong default for general CAD-style assemblies.

**Verdict:** for an assembly solver over arbitrary joint graphs with loops, absolute 6DOF body coordinates + Newton/LM on stacked joint residuals is the classical, robust choice (Haug, *Computer-Aided Kinematics and Dynamics of Mechanical Systems*, is the canonical reference).

### 2.2 How SolveSpace's solver works (high level)

SolveSpace's `slvs` core (src/system.cpp, solve.cpp in https://github.com/solvespace/solvespace):
1. **Parameter space:** each entity (point/line/circle/plane/normal) owns a few scalar parameters; 3D points are XYZ, orientations are quaternions (with auto-added normalization equations).
2. **Constraints → equations:** every user constraint (coincident, distance, angle, parallel, on-entity, symmetric...) generates one or more scalar residual equations f_i(params). Everything is a big square-ish nonlinear system.
3. **Group decomposition:** the param/constraint graph is split into "groups" (a sketch-in-2D group, a 3D assembly group, etc.) solved sequentially in dependency order — small systems instead of one huge one. Your solver could do the same with a union-find / condensation over the body-joint graph (solve connected rigid subassemblies first).
4. **Solve:** damped Newton (effectively LM): repeatedly compute residual vector and Jacobian (analytic, hand-written per constraint type), solve JᵀJ Δ = -Jᵀf with a diagonal damping term adapted per iteration, line-search-less but with step acceptance on ‖f‖. Converges in a handful of iterations for well-posed sketches.
5. **Rank analysis:** after converge/fail, it computes the Jacobian rank (via its sparse solver) to report *over-constrained* (and which constraints are redundant, via removal tests) vs *under-constrained* (DOF count). This "which constraint is redundant" feedback is the killer UX feature CAD users expect.
6. Handles dragged points by pinning them as extra constraints.

### 2.3 How CAD assembly mates map to this

FreeCAD's ecosystem is the direct analog:
- **A2plus / Assembly3/Assembly4-style workbenches** express mates (coincident planes, aligned axes, concentric, offset distances/angles) as constraints, and A2plus shells out to the `slvs` solver to compute part placements; Assembly4 instead builds placement hierarchies from master-sketch datums (parametric FK, not a constraint solve). The `slvs`-based ones are literally your problem: parts = bodies, mates = constraint equations, solve → 6DOF placements.
- A **revolute mate** = point-on-axis coincidence (3 eqs) + axis alignment (2 eqs) = 5 constraints; **spherical** = center coincidence (3); **cylindrical** = axis coincidence (4: 2 for a point on the axis, 2 for direction); **prismatic** = axis alignment (2) + no relative rotation about perpendicular axes (2) + point in plane (1) = 5; **fixed/weld** = full frame coincidence (6). Offsets/angles add or modify rows.
- Practical CAD-solver features worth copying: solve sequentially over the joint graph's connected components; lock "already placed" bodies; report redundancy per-constraint; return remaining DOF per body (null space of J) for animation/dragging.

---

## 3. Recommendation table (for embedding in PySide6 + numpy app)

| Rank | Option | License | Pos-level assembly | All 5 joints | Dep weight | Fit |
|---|---|---|---|---|---|---|
| 🥇 1 | **Custom Newton/LM solver on scipy** (`least_squares`, analytic J, body-6DOF coords) | BSD (scipy) | ✅ exactly | ✅ all trivially | None new | Perfect model match, no licensing risk, full redundancy/mobility reporting; ~300–500 lines of math |
| 🥈 2 | **py-slvs / python-solvespace** as the solver backend | **GPLv3** ⚠️ | ✅ purpose-built | ✅ via composed constraints | Tiny | Mathematically ideal (redundancy detection, robustness); adopt only if GPLv3 is acceptable for the app |
| 🥉 3 | **CasADi** `rootfinder`/NLP on your Φ(x)=0 | LGPL-3 | ✅ | ✅ | Light | Like (1) but with free AD Jacobians and a path to optimization later; LGPL is linkable |
| 4 | **Pinocchio** (+ closure constraints, or `pink`) | BSD-2 | ✅ trees (loops need manual closure) | ✅ (spherical/cyl. composite) | Medium | Best if the app pivots to URDF/robotics models; impedance mismatch with body-6DOF assembly data model |
| 5 | **Exudyn** static solve | BSD-3 | ✅ `SolveStatic` | ✅ | Medium | Viable if you also want dynamics later from the same model |
| 6 | **Drake** | BSD-3 | ✅ (NLP) | ✅ (no native cyl.) | **Heavy** | Great solver, bad packaging fit for desktop embedding |
| 7 | **MuJoCo + mink** | Apache-2 | ⚠️ velocity-level IK only | ✅ (composites) | Light | Wrong abstraction level for one-shot assembly |
| 8 | **pychrono** | BSD-3 | ✅ | ✅ full set | Heavy, conda-only | Packaging friction too high |
| 9 | **PyKDL / RBDL** | LGPL/zlib | ⚠️ chains only | ⚠️ partial | ROS-adjacent | Superseded by Pinocchio |
| 10 | **sympy.mechanics / PyDy / pynamics** | BSD | ⚠️ you solve it yourself | ✅ symbolic | Light | Good for deriving/validating your Φ and J symbolically, not as the runtime solver |
| 11 | **MBSim-env** | LGPL | ✅ | ✅ | Very heavy | Not designed for embedding |

**Bottom line:** write the custom solver (rank 1) in absolute 6DOF coordinates with Levenberg-Marquardt via `scipy.optimize.least_squares` and analytic per-joint Jacobians — it matches the existing `RigidBody/Joint/Frame/Pose` model exactly, handles loops and all five joint types, reports redundancy/mobility via Jacobian rank, and adds zero dependencies and zero licensing risk. Keep **py-slvs** as the fallback if robustness proves hard (subject to GPL check) and **CasADi** as the upgrade path if AD/optimization is wanted later.

---

## Key URLs
- solvespace: https://github.com/solvespace/solvespace · python-solvespace: https://pypi.org/project/python-solvespace · py-slvs: https://pypi.org/project/py-slvs/
- Pinocchio: https://github.com/stack-of-tasks/pinocchio · pink: https://pypi.org/project/pin-pink/
- Drake: https://drake.mit.edu · IK class: https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_inverse_kinematics.html
- Exudyn: https://github.com/jgerstmayr/EXUDYN · robotics IK: https://exudyn.readthedocs.io/en/v1.9.0/docs/RST/pythonUtilities/robotics.html
- Chrono: https://projectchrono.org/pychrono · MBSim: https://github.com/mbsim-env/mbsim
- MuJoCo: https://github.com/google-deepmind/mujoco · mink: https://github.com/kevinzakka/mink
- KDL: https://github.com/orocos/orocos_kinematics_dynamics · RBDL: https://rbdl.github.io
- sympy mechanics joints: https://docs.sympy.org/latest/explanation/modules/physics/mechanics/joints.html · PyDy: https://pydy.readthedocs.io · pynamics: https://pypi.org/project/pynamics/
- CasADi: https://web.casadi.org/docs
- FreeCAD Assembly4: https://wiki.freecad.org/Assembly4_Workbench
