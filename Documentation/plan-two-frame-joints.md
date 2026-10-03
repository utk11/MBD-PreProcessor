# Plan: create joints from one frame on each body

Implementation status: implemented in the current workspace. Python syntax compilation passed.

## Intended behavior

A joint is defined by two attachments: Body 1 + Frame 1, and Body 2 + Frame 2. The selected frames identify the physical connection locations even when the imported STEP bodies are separated or rotated away from their assembled positions.

For example, select a frame at the hole on one link and a frame at the pin on another link. A revolute joint requires those origins to coincide and their chosen axes to be parallel, while allowing rotation about the joint axis. The imported gap must become a constraint error to solve, rather than being baked into the joint as an offset.

Recommended workflow:

1. Create geometry frames on the desired faces, edges, or vertices, using the existing frame tools.
2. Open Create Joint and select the joint type.
3. Select Body 1 and a frame belonging to it.
4. Select Body 2 and a frame belonging to it.
5. Check the two highlighted attachments and axis directions.
6. Create the joint. Both markers remain attached to their own bodies at their current positions.
7. Run the existing Solve Assembly command to assemble the bodies. Also offer a Create & Assemble button that runs this same operation immediately after creation.

Creation records the connection; assembly changes body poses. Ground remains fixed. A free assembly can move both connected bodies unless a reference body is explicitly held fixed.

## What the current code already provides

- `gui/joint_dialog.py` offers two bodies but one `Joint Frame (Global)` selector.
- `main.py:create_joint()` obtains a flat list from `_available_world_frames()`, constructs `Joint(..., frame, axis)`, then captures both markers from that same world frame.
- `Joint.marker1` and `Joint.marker2` already store independent body-local frames. `core/kinematics/prepared.py` and the JAX backend already consume these separately.
- `AssemblyDocument` already records frame ownership and coordinate conventions through `frame_to_body`, `frames_by_body`, and `frame_coordinates`.
- `core/transforms.py` already resolves `world`, `reference_geometry`, and `body_local` frame coordinates against live body poses.
- Schema 2 project files already store both markers. JSON exports already contain both markers transformed into the current world pose.
- The joint renderer and property panel still use the single stored `joint.frame`. The motor renderer still refers to obsolete `joint.frame1` attributes.

The main work is frame selection, independent marker construction, and consistent presentation. The existing solver formulation can be reused.

## 1. Define the two attachments in the model

Keep `marker1` and `marker2` as the authoritative joint frames, expressed in the local coordinates of their corresponding bodies. Avoid introducing a second pair of independently mutable `frame1`/`frame2` fields alongside the markers.

Add a small frame-selection record carrying a selection key, display name, owner body ID, coordinate convention, and frame values. This preserves ownership information that the current flat list of world-frame copies loses. Keys must distinguish body COM frames, user frames, and Ground frames even if display names match.

Add a domain construction helper, preferably in a focused `core/joint_factory.py`, that:

- Validates the name, distinct body IDs, frame ownership, and frame matrices.
- Resolves each selected frame independently using the current body pose and its recorded coordinate convention.
- Captures each resolved world frame into its corresponding body's local coordinates.
- Copies arrays, so editing a source frame later cannot mutate an existing joint.
- Returns a complete joint with both markers populated, or a clear error before the document is modified.

For side `i`, the required transform is:

`marker_i = inverse(body_world_pose_i) * selected_frame_world_i`

Later, the solver and renderer reconstruct:

`marker_world_i = body_world_pose_i * marker_i`

For a body-local source frame, this is equivalent to copying its local transform. For a reference-geometry source, first resolve its placement through the imported reference and live pose. Use the existing transform functions to keep units and coordinate conventions consistent.

Record optional source selection labels and axis settings for display and persistence. These are provenance, not live links. Editing or deleting a source frame leaves an existing joint intact; deleting its body removes the joint through the existing document deletion path. A later explicit joint-edit operation can replace markers and increment the marker revision.

Retain `joint.frame` and the existing shared-frame capture helper for compatibility with older callers and projects. For new joints, `joint.frame` can hold a copy of side 1's creation world frame, documented as legacy metadata. Runtime geometry must use the two markers.

## 2. Replace the single selector with two body/frame pairs

Update `gui/joint_dialog.py` to display:

- Name and joint type.
- Body 1, then Frame on Body 1.
- Body 2, then Frame on Body 2.
- Axis settings for joints that use an axis.
- Create, Create & Assemble, and Cancel.

Each real body's frame list includes its COM frame and user frames whose recorded parent matches that body. Ground offers the World Frame and applicable world/Ground frames. Unparented world frames must not silently become attachments on arbitrary moving bodies.

Changing a body refreshes only that side's frame list and clears any incompatible selection. Disable creation until both selections are valid, and validate again in the domain helper when accepting the dialog. Reject selecting the same body twice. Resolve values at acceptance time so poses or selections cannot become stale while the dialog is open.

Highlight each attachment independently in the viewer, using labels or line styles to distinguish sides while preserving RGB coordinate axes. Restore the prior highlight and visibility state when the dialog closes. If only the COM frame is available, explain how to create a geometry frame for the intended connection location.

Return a structured request containing both selections instead of the current positional tuple with one frame. Use a joint-specific frame catalog; other dialogs can continue using `_available_world_frames()`.

## 3. Make orientation and allowed motion explicit

The two selected frames define the target relationship. Their present separation or orientation difference must not be captured as a rest offset by default.

| Joint | Required relationship after assembly | Allowed relative motion |
| --- | --- | --- |
| Fixed | Same origin and orientation | None |
| Revolute | Same origin; selected axes parallel | Rotation about the axis |
| Prismatic | Origins on the same axis line; same orientation | Translation along the axis |
| Cylindrical | Origins on the same axis line; selected axes parallel | Translation and rotation along/about the axis |
| Spherical | Same origin | Rotation in all directions |

For axial joints, offer a signed axis selector per attachment, defaulting to `+Z`. This lets a pin's local X axis connect to a hole's local Z axis without changing either user frame.

Normalize the selected axis of each copied marker into a deterministic right-handed basis with canonical `+Z`, then keep the existing solver's common `joint.axis = "+Z"` for new axial joints. Preserve a deterministic transverse direction and document how it sets the twist reference for prismatic joints. Older joints keep their existing axis and marker values.

Provide an explicit orientation adjustment for opposing face normals, including fixed joints. Apply flips as proper rotations of the copied marker, never as a reflection or an edit to the source frame. For prismatic/fixed joints, preview the full orientation relationship; spherical joints need no axis controls.

The current revolute/cylindrical cross-product residual accepts both parallel and antiparallel axes. Preserve and document that line-axis convention in the initial change; positive motion direction follows side 1's canonical axis. Enforcing directed alignment would be a separate residual change requiring matching updates to the reference evaluator and JAX backend.

## 4. Integrate creation and assembly through the controller

Move construction into a controller command invoked by `main.py:create_joint()`. Capture both markers before any solve or pose change. Add the joint only after complete validation, then publish the document topology/marker revisions and update the tree and viewer. Do not catch a marker-capture error and continue adding an incomplete joint, as the current creation path can do.

Create keeps body poses unchanged and shows separated markers when the assembly is unsatisfied. Create & Assemble adds the same joint and schedules assembly through `ApplicationController` and `SolveScheduler`, using their existing revision checks and worker isolation.

For Create & Assemble:

- Ground is always fixed. If the user selects a reference body for an ungrounded assembly, pass it as a temporary locked body for that solve; do not create a permanent fixed joint.
- Run the whole affected constraint system so previously created joints remain respected.
- Treat feasibility as a separate requirement from a finite, current worker result. The existing acceptance check permits finite partial results; this new action must not commit an infeasible trial pose as a successful assembly.
- On failure, keep the newly defined joint and original body poses, show the unresolved attachments, and provide the solver's failure details.
- Ignore stale results if bodies, markers, or the project changed during the solve.

Test large relative rotations, including perpendicular axes and near-180-degree frame differences. The current local iterative solve must not be assumed to converge from every disassembled arrangement. If those tests expose a convergence problem, add a joint-aware initial pose estimate in worker-owned trial data, then solve all constraints before committing. Avoid directly snapping one live body when it already participates in other joints. Any seed must preserve each joint's free rotation/translation rather than adding constraints.

## 5. Update graphics, properties, save/load, and export

Replace single-frame joint rendering with the live world transforms of both markers. Give them distinct names and side labels, and draw a finite dashed connector when their origins differ. For prismatic and cylindrical joints, origin separation along the common axis is valid travel; it must not be labeled as an unsatisfied constraint. Use the actual joint residual for that status.

Refresh affected joints as either body moves and after assembly. Use `joints_by_body` to limit updates and batch viewer redraws through the existing coordinator. Propagate the assembly unit scale to the joint frame renderer and connector. Remove both marker displays and the connector on joint deletion.

Change the property panel from one Joint Frame field to two attachment fields with their body names, source labels, and current placement. Update motor visualization to use marker 1 transformed through the live body pose, replacing obsolete `joint.frame1` accesses. Motor direction follows the canonical axis; implement no new motor physics in this change.

Keep schema 2's existing numeric marker representation. Optional source-selection/axis metadata can be added without changing that representation or requiring a schema bump. Validate both markers together for newly authored two-frame joints. Preserve the existing legacy migration path for markerless older joints; never recapture a saved marker pair from the current world pose.

Old projects may not identify two source frames. Display legacy attachment labels rather than guessing ownership from a frame name. Schema 1 user frames remain world frames until explicitly attached to a body.

Continue exporting `marker1_world` and `marker2_world` using live poses. Add local marker transforms and optional source labels so downstream tools can reconstruct attachments directly. Retain `frame_world` as documented legacy creation metadata; consumers must use the two markers to define the connection. Update the README workflow and export description accordingly.

## 6. Verification and implementation order

Implement in these increments:

1. Frame catalog and domain construction helper, including coordinate conversion and copied-marker tests.
2. Two-selector dialog and controller integration; creation alone must work for separated bodies.
3. Dual-marker graphics, properties, and motor display updates.
4. Create & Assemble integration, feasibility handling, and any initial-pose support justified by displaced-body tests.
5. Metadata persistence, export, legacy compatibility, and documentation.

Required acceptance checks:

- Separate and rotate two bodies, select physical attachment frames on each, and verify their independently captured local markers reproduce both selected locations before solving.
- Solve a feasible displaced pair for each of the five supported joint types and verify both constrained and free directions from the table above. Include arbitrary-axis and opposite-normal cases.
- Cover Ground on either side, an ungrounded pair, and an explicit temporary reference lock.
- Repeat capture after a body was dragged, for `reference_geometry`, `body_local`, and `world` sources, with nonidentity body reference rotations.
- Verify meter/millimeter scaling in both numeric construction and graphics.
- Move either body and verify its joint marker follows it; properties, motor indicators, and exports must agree with the live marker transforms.
- Reject invalid/missing frames, wrong owners, same-body pairs, duplicate names, non-finite values, and invalid rotation matrices before publishing a joint.
- Verify source-frame edits/deletion do not change copied markers, while body deletion removes dependent joints.
- Save/load distinct markers, source labels, axes, and motors without recapture; load schema 1 and existing schema 2 projects with the same joint behavior as before.
- Exercise incompatible constraints and stale worker results; failed Create & Assemble must preserve the pre-solve poses and report the unresolved joint.
- Check an existing assembled mechanism and a displaced closed loop, including a clear failure outcome when assembly is infeasible or does not converge.

Update the older joint tests to the supported model: `tests/test_joint_structure.py` still uses removed `frame1`/`frame2` keywords, and `tests/test_joint_renderer.py` passes a second frame into the current axis argument. Reuse the kinematics, architecture, and refactor regression suites for compatibility checks, then perform a viewer smoke test with the supplied STEP assembly.

Completion means a user can define a physical joint from two different attachment locations without first moving the STEP bodies into their final assembly positions, and the two markers remain consistent through assembly, dragging, save/load, and export.
