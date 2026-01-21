# Multi-Body Dynamics Preprocessor - Incremental Implementation Guide

## Philosophy
Build in small, testable increments. Each increment should:
1. Produce a working, demonstrable feature
2. Be fully testable before moving to the next increment
3. Build upon the previous working increment
4. Take 1-3 days maximum to complete

---

## Increment 1: Minimal GUI + STEP File Display

### Goal
Display a STEP file in a 3D viewer - the absolute minimum to verify the stack works.

### Implementation Steps
1. Create `main.py` with basic PySide2 window
2. Add pythonocc 3D viewer widget to window
3. Load a STEP file (hardcoded path initially)
4. Display all shapes in the viewer

### Files to Create
```
mbd_preprocessor/
├── main.py                 # ~50 lines: window + viewer
└── requirements.txt        # dependencies list
```

### Testing Criteria
- [ ] Window opens without errors
- [ ] Can see 3D viewport
- [ ] STEP file geometry displays correctly
- [ ] Can rotate/pan/zoom the view with mouse

### Debugging Checklist
- Verify pythonocc-core installation
- Check STEP file is valid
- Confirm OpenGL drivers are working

### Expected Output
A window with a 3D viewer showing your STEP file geometry.

---

## Increment 2: File Menu + Open Dialog

### Goal
Add ability to open any STEP file through a file dialog.

### Implementation Steps
1. Add menu bar to main window
2. Add "File > Open" menu item
3. Create file dialog for STEP files (.step, .stp)
4. Clear previous geometry and load new file
5. Add error handling for invalid files

### Files to Modify/Create
```
mbd_preprocessor/
├── main.py                 # Add menu bar and file dialog
└── core/
    └── step_parser.py      # Extract STEP loading logic
```

### Testing Criteria
- [ ] Menu appears in window
- [ ] File dialog opens when clicking "File > Open"
- [ ] Can select and load different STEP files
- [ ] Previous geometry is cleared when loading new file
- [ ] Error message shows for invalid files

### Debugging Checklist
- Test with multiple STEP files
- Try loading non-STEP files (should show error)
- Check memory cleanup (no leaks)

---

## Increment 3: Extract Individual Bodies

### Goal
Separate compound STEP assemblies into individual solid bodies.

### Implementation Steps
1. Implement `extract_bodies_from_compound()` function
2. Parse STEP file and identify all solid bodies
3. Display count of bodies found
4. Assign each body a temporary ID and name

### Files to Create/Modify
```
mbd_preprocessor/
└── core/
    ├── step_parser.py          # Body extraction logic
    └── data_structures.py      # RigidBody class (minimal)
```

### Testing Criteria
- [ ] Correctly counts number of bodies in assembly
- [ ] Each body has unique ID
- [ ] Body names generated (Body_0, Body_1, etc.)
- [ ] All bodies still visible in viewer

### Test Cases
- Single body STEP file → 1 body
- Assembly with 5 parts → 5 bodies
- Nested assembly → flattened list of all solids

---

## Increment 4: Body List Widget (Read-Only)

### Goal
Display list of bodies in a tree widget on the left side.

### Implementation Steps
1. Create left panel with QTreeWidget
2. Populate tree with body names when file loads
3. Add body count label
4. Simple layout: left panel (30%) + viewer (70%)

### Files to Create/Modify
```
mbd_preprocessor/
├── main.py                     # Add splitter layout
└── gui/
    └── body_tree_widget.py     # Tree widget implementation
```

### Testing Criteria
- [ ] Tree widget appears on left side
- [ ] All bodies listed with names
- [ ] Body count displayed
- [ ] Viewer still works on right side
- [ ] Splitter can resize panels

### Debugging Checklist
- Verify tree updates when new file loaded
- Check layout doesn't break on window resize

---

## Increment 5: Body Selection & Highlighting

### Goal
Click a body in the list to highlight it in the 3D viewer.

### Implementation Steps
1. Connect tree item click to selection handler
2. Implement body highlighting in viewer (change color/outline)
3. Store mapping between tree items and body IDs
4. Add visual feedback (selected body turns yellow/highlighted)

### Files to Modify
```
mbd_preprocessor/
├── gui/
│   └── body_tree_widget.py     # Add selection signal
└── visualization/
    └── body_renderer.py        # Highlighting logic (new)
```

### Testing Criteria
- [ ] Clicking body in list highlights it in viewer
- [ ] Only one body highlighted at a time
- [ ] Highlighting is clearly visible
- [ ] Clicking another body switches highlighting

### Test Cases
- Select each body individually
- Select body, load new file (selection clears)
- Rapid clicking between bodies (no lag)

---

## Increment 6: Simple Property Display

### Goal
Show basic body information when selected (name, ID only - no calculations yet).

### Implementation Steps
1. Create right panel with property display
2. Show selected body name and ID
3. Update display when selection changes
4. Layout: left panel (20%) + viewer (50%) + right panel (30%)

### Files to Create
```
mbd_preprocessor/
└── gui/
    └── property_panel.py       # Property display widget
```

### Testing Criteria
- [ ] Right panel appears
- [ ] Shows "No body selected" when nothing selected
- [ ] Displays name and ID when body selected
- [ ] Updates when different body selected

---

## Increment 7: Volume Calculation

### Goal
Calculate and display body volume using OCC GProp.

### Implementation Steps
1. Implement `calculate_volume()` function
2. Compute volume for each body when file loads
3. Store volume in RigidBody object
4. Display volume in property panel

### Files to Create/Modify
```
mbd_preprocessor/
└── core/
    ├── data_structures.py      # Add volume field to RigidBody
    └── physics_calculator.py   # Volume calculation (new)
```

### Testing Criteria
- [ ] Volume calculated for each body
- [ ] Volume displayed in property panel (in m³)
- [ ] Non-zero volume for solid bodies
- [ ] Calculation completes quickly (< 1 second per body)

### Debugging Checklist
- Verify volume is positive
- Test with known geometry (e.g., 10cm cube = 0.001 m³)
- Check units are correct

---

## Increment 8: Center of Mass Calculation

### Goal
Calculate and display center of mass for each body.

### Implementation Steps
1. Implement `calculate_center_of_mass()` function
2. Compute COM for each body when file loads
3. Store COM coordinates in RigidBody
4. Display COM in property panel

### Files to Modify
```
mbd_preprocessor/
└── core/
    ├── data_structures.py      # Add center_of_mass field
    └── physics_calculator.py   # COM calculation
```

### Testing Criteria
- [ ] COM calculated for each body
- [ ] COM displayed as [x, y, z] coordinates
- [ ] COM appears reasonable (inside body geometry)

### Test Cases
- Symmetric body → COM at geometric center
- Asymmetric body → COM offset from geometric center

---

## Increment 9: Visualize Center of Mass

### Goal
Show COM as a small sphere in the 3D viewer.

### Implementation Steps
1. Create sphere geometry at COM location
2. Render sphere when body is selected
3. Use distinct color (e.g., red) for COM marker
4. Toggle visibility with checkbox in property panel

### Files to Modify
```
mbd_preprocessor/
└── visualization/
    └── body_renderer.py        # Add COM sphere rendering
```

### Testing Criteria
- [ ] Small red sphere appears at COM when body selected
- [ ] Sphere moves with body if body transforms
- [ ] Checkbox toggles COM visibility
- [ ] COM hidden when body deselected

---

## Increment 10: Inertia Tensor Calculation

### Goal
Calculate normalized inertia tensor about the center of mass.

### Implementation Steps
1. Implement `calculate_inertia_normalized()` function
2. Compute inertia tensor for each body
3. Store as 3×3 matrix in RigidBody
4. Display in property panel (formatted matrix)

### Files to Modify
```
mbd_preprocessor/
└── core/
    ├── data_structures.py          # Add inertia_tensor field
    └── physics_calculator.py       # Inertia calculation
```

### Testing Criteria
- [ ] Inertia tensor calculated for each body
- [ ] Tensor is 3×3 symmetric matrix
- [ ] Diagonal values are positive
- [ ] Displayed in readable format in property panel

### Debugging Checklist
- Verify tensor is about COM, not origin
- Check symmetry: I[i,j] == I[j,i]
- Principal moments should be positive

---

## Increment 11: Frame Data Structure & Rendering

### Goal
Create Frame class and visualize as RGB axes (X=Red, Y=Green, Z=Blue).

### Implementation Steps
1. Implement Frame class with origin and rotation matrix
2. Create test frame at world origin [0, 0, 0]
3. Implement frame rendering (three colored arrows)
4. Add "Show World Frame" checkbox to toggle visibility

### Files to Create
```
mbd_preprocessor/
├── core/
│   └── data_structures.py      # Add Frame class
└── visualization/
    └── frame_renderer.py       # Frame visualization (new)
```

### Testing Criteria
- [ ] Frame class stores origin and orientation
- [ ] World frame displays as RGB axes at origin
- [ ] X-axis is red, Y-axis is green, Z-axis is blue
- [ ] Checkbox toggles frame visibility
- [ ] Axes scale appropriately with model size

### Test Cases
- Create frame at [0,0,0] with identity rotation
- Create frame at [1,0,0] with 90° rotation
- Verify axes are orthogonal

---

## Increment 12: Body Local Frame

### Goal
Each body has its own local coordinate frame (initially at COM).

### Implementation Steps
1. Add local_frame field to RigidBody
2. Initialize local frame at COM with identity rotation
3. Display local frame when body is selected
4. Add checkbox to toggle local frame visibility

### Files to Modify
```
mbd_preprocessor/
└── core/
    └── data_structures.py      # Add local_frame to RigidBody
```

### Testing Criteria
- [ ] Each body has a local frame
- [ ] Local frame appears when body selected
- [ ] Frame origin at body COM
- [ ] Frame axes aligned with world axes initially

---

## Increment 13: Mouse Click Selection in Viewer

### Goal
Click directly on bodies in the 3D viewer to select them.

### Implementation Steps
1. Implement mouse click detection in viewer
2. Use OCC selection API to identify clicked body
3. Update tree selection when body clicked in viewer
4. Update property panel with clicked body info

### Files to Create
```
mbd_preprocessor/
└── gui/
    └── viewer_3d.py            # Enhanced viewer with selection
```

### Testing Criteria
- [ ] Clicking body in viewer selects it
- [ ] Tree selection updates to match
- [ ] Property panel updates
- [ ] Highlighting works same as tree selection

### Debugging Checklist
- Test clicking on different parts of same body
- Test clicking on overlapping bodies
- Test clicking on empty space (deselects)

---

## Increment 14: Face Selection Mode

### Goal
Switch to face selection mode and highlight individual faces.

### Implementation Steps
1. Add "Selection Mode" dropdown (Body / Face / Edge)
2. Implement face selection in viewer
3. Highlight selected face (different color)
4. Display face info in property panel (area, center, normal)

### Files to Create/Modify
```
mbd_preprocessor/
└── core/
    └── geometry_utils.py       # Face geometry extraction (new)
```

### Testing Criteria
- [ ] Can switch between Body and Face selection modes
- [ ] Clicking face highlights only that face
- [ ] Face normal vector calculated correctly
- [ ] Face center point calculated correctly
- [ ] Property panel shows face information

### Test Cases
- Select flat face → normal perpendicular to surface
- Select curved face → normal at center point
- Select multiple faces in sequence

---

## Increment 15: Create Frame from Face

### Goal
Create a coordinate frame at selected face (origin at center, Z-axis along normal).

### Implementation Steps
1. Add "Create Frame from Face" button (only active in face mode)
2. Extract face center and normal vector
3. Create Frame with Z-axis = normal, X and Y perpendicular
4. Display created frame in viewer
5. Store frame in Assembly.frames dictionary

### Files to Modify
```
mbd_preprocessor/
└── core/
    ├── data_structures.py      # Frame creation methods
    └── geometry_utils.py       # Face-to-frame conversion
```

### Testing Criteria
- [ ] Button only enabled when face selected
- [ ] Click button creates frame at face center
- [ ] Frame Z-axis (blue) points along face normal
- [ ] Frame persists in viewer
- [ ] Can create multiple frames

### Debugging Checklist
- Verify normal direction (should point outward)
- Check X and Y axes are orthogonal to Z
- Ensure right-handed coordinate system

---

## Increment 16: Edge Selection Mode

### Goal
Select edges and get edge direction and midpoint.

### Implementation Steps
1. Add "Edge" option to selection mode dropdown
2. Implement edge selection in viewer
3. Highlight selected edge
4. Calculate edge midpoint and direction vector
5. Display edge info in property panel

### Files to Modify
```
mbd_preprocessor/
└── core/
    └── geometry_utils.py       # Edge geometry extraction
```

### Testing Criteria
- [ ] Can switch to Edge selection mode
- [ ] Clicking edge highlights it
- [ ] Edge midpoint calculated correctly
- [ ] Edge direction vector calculated correctly
- [ ] Property panel shows edge information

### Test Cases
- Select straight edge → direction is consistent
- Select curved edge → direction at midpoint tangent
- Select very short edge → no numerical issues

---

## Increment 17: Create Frame from Edge

### Goal
Create frame at selected edge (origin at midpoint, Z-axis along edge).

### Implementation Steps
1. Add "Create Frame from Edge" button (only active in edge mode)
2. Extract edge midpoint and direction
3. Create Frame with Z-axis = edge direction, X and Y perpendicular
4. Display created frame in viewer
5. Store in Assembly.frames dictionary

### Files to Modify
```
mbd_preprocessor/
└── core/
    ├── data_structures.py      # Edge-to-frame method
    └── geometry_utils.py       # Edge direction calculation
```

### Testing Criteria
- [ ] Button only enabled when edge selected
- [ ] Creates frame at edge midpoint
- [ ] Frame Z-axis (blue) along edge direction
- [ ] Can create frames from multiple edges

---

## Increment 18: Frame List Management

### Goal
Display list of all created frames and allow selection/deletion.

### Implementation Steps
1. Add "Frames" section to tree widget
2. List all created frames with names (Frame_0, Frame_1, etc.)
3. Click frame in list to highlight in viewer
4. Right-click menu to delete frames
5. Show frame properties in property panel when selected

### Files to Modify
```
mbd_preprocessor/
└── gui/
    └── body_tree_widget.py     # Add frames section
```

### Testing Criteria
- [ ] All created frames appear in list
- [ ] Clicking frame in list highlights it in viewer
- [ ] Can delete frames via right-click menu
- [ ] Frame count updates when frames added/removed

---

## Increment 19: Manual Frame Translation

### Goal
Adjust frame position using numeric input fields.

### Implementation Steps
1. Add frame adjustment section to property panel
2. Add X, Y, Z position spinboxes
3. Update frame position when values change
4. Update viewer in real-time as values change

### Files to Modify
```
mbd_preprocessor/
└── gui/
    └── property_panel.py       # Add frame adjustment controls
```

### Testing Criteria
- [ ] Position spinboxes appear when frame selected
- [ ] Changing X value moves frame along X-axis
- [ ] Changing Y value moves frame along Y-axis
- [ ] Changing Z value moves frame along Z-axis
- [ ] Updates visible immediately in viewer
- [ ] Can input negative values

### Test Cases
- Move frame by small increments (0.001 m)
- Move frame by large distances (1 m)
- Enter negative coordinates

---

## Increment 20: Manual Frame Rotation

### Goal
Rotate frame about its local axes using angle inputs.

### Implementation Steps
1. Add rotation angle spinboxes (about X, Y, Z axes)
2. Implement frame rotation methods
3. Update rotation matrix when angles change
4. Display rotation angles in degrees
5. Real-time update in viewer

### Files to Modify
```
mbd_preprocessor/
├── core/
│   └── data_structures.py      # Frame rotation methods
└── gui/
    └── property_panel.py       # Add rotation controls
```

### Testing Criteria
- [ ] Rotation spinboxes appear when frame selected
- [ ] Rotating about X axis rolls the frame
- [ ] Rotating about Y axis pitches the frame
- [ ] Rotating about Z axis yaws the frame
- [ ] Updates visible immediately in viewer
- [ ] Angles wrap correctly (-180° to +180°)

### Debugging Checklist
- Verify rotation order (XYZ Euler angles)
- Check gimbal lock scenarios
- Ensure rotation matrix remains orthonormal

---

## Increment 21: Joint Data Structure

### Goal
Define Joint class and basic joint types.

### Implementation Steps
1. Create JointType enum (Fixed, Hinge, Prismatic, Cylindrical)
2. Implement Joint class with body references and frames
3. Add joints collection to Assembly
4. Create simple test joint manually (no UI yet)

### Files to Modify
```
mbd_preprocessor/
└── core/
    └── data_structures.py      # Add Joint class and JointType
```

### Testing Criteria
- [ ] Joint class stores two body IDs
- [ ] Joint stores frame for each body (local coords)
- [ ] Joint has a type (enum)
- [ ] Can create joint programmatically
- [ ] Joint stored in Assembly.joints dict

### Test Cases
- Create Fixed joint between two bodies
- Create Hinge joint with rotation axis
- Verify frame references are correct

---

## Increment 22: Joint Visualization

### Goal
Display joints as frames with connection lines between bodies.

### Implementation Steps
1. Create JointRenderer class
2. Display both joint frames (different colors)
3. Draw line connecting the two joint frame origins
4. Add joint-specific visual hints (e.g., cylinder for hinge axis)

### Files to Create
```
mbd_preprocessor/
└── visualization/
    └── joint_renderer.py       # Joint visualization (new)
```

### Testing Criteria
- [ ] Joint frames visible in viewer
- [ ] Line connects the two joint origins
- [ ] Joint frames have distinct color from body frames
- [ ] Can toggle joint visibility

### Test Cases
- Create joint, verify frames appear
- Move body, verify joint frames move with it
- Multiple joints all visible simultaneously

---

## Increment 23: Joint Creation Wizard - Part 1 (UI)

### Goal
Create dialog for joint creation with step-by-step interface.

### Implementation Steps
1. Create JointCreationWizard dialog
2. Add joint type selection (radio buttons or dropdown)
3. Add "Next" and "Cancel" buttons
4. Display current step indicator (Step 1 of 10)
5. No functionality yet - just UI shell

### Files to Create
```
mbd_preprocessor/
└── gui/
    └── joint_creation_dialog.py    # Wizard dialog (new)
```

### Testing Criteria
- [ ] Dialog opens from menu or button
- [ ] All joint types listed
- [ ] Can select joint type
- [ ] Step indicator visible
- [ ] Cancel button closes dialog
- [ ] Next button advances to next step (empty for now)

---

## Increment 24: Joint Creation Wizard - Part 2 (Body Selection)

### Goal
Implement body selection steps in the wizard.

### Implementation Steps
1. Step 2: "Select first body" - show instructions
2. Enable body selection in main viewer
3. Wizard dialog shows selected body name
4. "Next" button advances to Step 3
5. Step 3: "Select second body" - same process

### Files to Modify
```
mbd_preprocessor/
└── gui/
    └── joint_creation_dialog.py    # Add body selection steps
```

### Testing Criteria
- [ ] Step 2 shows "Select first body" instruction
- [ ] Can click body in viewer to select
- [ ] Selected body name appears in wizard
- [ ] Next advances to step 3
- [ ] Step 3 selects second body
- [ ] Cannot select same body twice

### Debugging Checklist
- Verify bodies are distinct
- Check selection works while dialog is open
- Handle case where user closes dialog mid-process

---

## Increment 25: Joint Creation Wizard - Part 3 (Geometry Selection)

### Goal
Select face/edge on each body for frame placement.

### Implementation Steps
1. Step 4: "Select geometry on Body 1" - switch to face/edge mode
2. User clicks face or edge on first body
3. Wizard shows selected geometry info
4. Step 6: Repeat for second body
5. Store GeometrySelection objects

### Files to Modify
```
mbd_preprocessor/
└── gui/
    └── joint_creation_dialog.py    # Add geometry selection steps
```

### Testing Criteria
- [ ] Step 4 enables face/edge selection
- [ ] Can select face on first body
- [ ] Selected geometry info displayed in wizard
- [ ] Step 6 selects geometry on second body
- [ ] Both selections stored correctly

---

## Increment 26: Joint Creation Wizard - Part 4 (Frame Creation)

### Goal
Automatically create frames from selected geometry.

### Implementation Steps
1. Step 5: Create frame from Body 1 geometry selection
2. Display preview of created frame in viewer
3. Step 7: Create frame from Body 2 geometry selection
4. Store frames in joint definition
5. Show frame preview with "looks good?" confirmation

### Files to Modify
```
mbd_preprocessor/
└── gui/
    └── joint_creation_dialog.py    # Add frame creation steps
```

### Testing Criteria
- [ ] Frame automatically created from face selection
- [ ] Frame automatically created from edge selection
- [ ] Preview frames visible in viewer
- [ ] User can see frames before confirming
- [ ] "Previous" button allows going back

---

## Increment 27: Joint Creation Wizard - Part 5 (Completion)

### Goal
Finalize joint creation and add to assembly.

### Implementation Steps
1. Step 10: Show summary of joint configuration
2. Display: joint type, bodies, frames
3. "Create Joint" button finalizes
4. Add joint to Assembly.joints
5. Close wizard and show joint in viewer

### Files to Modify
```
mbd_preprocessor/
└── gui/
    └── joint_creation_dialog.py    # Add completion step
```

### Testing Criteria
- [ ] Summary shows all joint information
- [ ] Create Joint button works
- [ ] Joint added to assembly
- [ ] Joint visible in viewer
- [ ] Joint appears in tree widget
- [ ] Wizard closes after creation

### Test Cases
- Create Fixed joint - complete workflow
- Create Hinge joint - complete workflow
- Cancel mid-process - no joint created
- Create multiple joints in sequence

---

## Increment 28: Body Isolation Mode

### Goal
Add isolation mode to hide/show bodies for easier geometry selection when bodies occlude each other.

### Implementation Steps
1. Add context menu to body items in tree widget
2. Implement "Isolate Body" option to hide all other bodies
3. Implement "Exit Isolation" option to restore all body visibility
4. Add isolation state tracking (active flag and isolated body ID)
5. Create isolation handler methods in MainWindow
6. Connect signals from tree widget to handlers

### Files to Modify
```
mbd_preprocessor/
├── gui/
│   └── body_tree_widget.py     # Add context menu and signals
└── main.py                     # Add isolation handlers
```

### Testing Criteria
- [x] Right-click on body shows context menu
- [x] "Isolate Body" hides all other bodies
- [x] Only selected body is visible in isolation mode
- [x] "Exit Isolation" restores all bodies to visible
- [x] Can isolate different bodies sequentially
- [x] Isolation mode persists until explicitly exited
- [x] Clear user feedback (notification dialog)
- [x] No errors when switching isolation on/off

### Use Cases
- Selecting occluded faces/edges for frame creation
- Focusing on specific body without distractions
- Creating joints between bodies with complex geometry
- Inspecting internal features

### Debugging Checklist
- Verify body_renderer.set_body_visibility() works correctly
- Check that isolation state is properly tracked
- Test with 2+ bodies to verify hiding/showing
- Ensure viewer updates correctly after isolation changes

---

## Increment 29: Force Application

### Goal
Add ability to apply forces to bodies at specific frames with magnitude and direction.

### Implementation Steps
1. Create `Force` data structure in `core/data_structures.py`
   - Store force name, body ID, frame reference, magnitude (N), direction vector
   - Auto-normalize direction vectors
2. Create `ForceDialog` in `gui/force_dialog.py`
   - Body selection (including Ground)
   - Frame selection (world frame, body local frames, user-created frames)
   - Magnitude input (positive float)
   - Direction specification:
     * **Frame Axis Mode**: Choose from +X, -X, +Y, -Y, +Z, -Z relative to selected frame
     * **Custom Vector Mode**: Specify arbitrary direction (x, y, z components)
   - Transform frame axis directions to world coordinates using frame rotation matrix
3. Create `ForceRenderer` in `visualization/force_renderer.py`
   - Render forces as 3D arrows (line shaft + cone head)
   - Blue color for force arrows
   - Logarithmic scaling: length = base_length × (1 + log₁₀(magnitude))
   - Base length: 1.0 meters, adjustable with force_scale parameter
   - Arrow components: BRepBuilderAPI_MakeEdge for shaft, BRepPrimAPI_MakeCone for head
   - Thick lines (width 5.0) for visibility
4. Add force storage to MainWindow
   - `forces: Dict[str, Force]` dictionary
   - `add_force()` method with ForceDialog integration
   - `on_force_selected()` handler for tree selection
   - `on_force_deleted()` handler for deletion
5. Integrate forces into tree widget
   - Add "Forces" group with expand/collapse
   - List all forces with names
   - Right-click context menu with "Delete Force" option
   - Selection signal to show force properties
6. Add force property panel section
   - Display force name, magnitude, direction vector
   - Show associated body and frame
   - Update when force selected in tree

### Files Created/Modified
```
mbd_preprocessor/
├── core/
│   └── data_structures.py          # Force class
├── gui/
│   ├── force_dialog.py             # NEW: Force creation dialog
│   ├── body_tree_widget.py         # Add forces group
│   └── property_panel.py           # Add force_group UI
├── visualization/
│   └── force_renderer.py           # NEW: Force arrow renderer
└── main.py                         # Force management methods
```

### Force Data Structure
```python
class Force:
    def __init__(self, name: str, body_id: int, frame: Frame, 
                 magnitude: float, direction: np.ndarray):
        self.name = name
        self.body_id = body_id
        self.frame = frame
        self.magnitude = magnitude
        self.direction = direction / np.linalg.norm(direction)  # Auto-normalize
```

### Testing Criteria
- [x] Can create force via Edit > Add Force menu
- [x] Force dialog validates all inputs (name, magnitude > 0, direction non-zero)
- [x] Force appears in tree widget under "Forces" group
- [x] Force arrow visible in 3D viewer (blue arrow)
- [x] Arrow length scales with magnitude (logarithmic)
- [x] Arrow points in correct direction
- [x] Frame axis mode transforms direction correctly to world coordinates
- [x] Custom vector mode accepts arbitrary directions
- [x] Selecting force in tree shows properties in panel
- [x] Can delete force via right-click context menu
- [x] Force properties display correctly (body, frame, magnitude, direction)
- [x] Multiple forces can coexist without interference
- [x] Forces persist in tree when switching selections

### Use Cases
- Apply gravitational force to body (magnitude = mass × 9.81 N, direction = -Z)
- Apply thrust force at attachment point (custom magnitude and direction)
- Define external loads for simulation setup
- Visualize force magnitudes and directions before export
- Test different force configurations

### Debugging Checklist
- Verify arrow scaling makes forces visible (base_length = 1.0m works well)
- Check direction vector transformation for frame axes
- Test with wide range of magnitudes (1 N to 10000 N)
- Verify arrow color is distinct from bodies and frames (blue)
- Ensure forces render on top of geometry (no z-fighting)
- Test force at world origin vs. body-attached frames
- Verify force deletion removes arrow from viewer

### Known Issues & Solutions
- **Arrow too small**: Increase `base_length` in ForceRenderer (default: 1.0m)
- **Direction incorrect**: Check frame rotation_matrix transformation in ForceDialog
- **Arrow not visible**: Verify Display() and UpdateCurrentViewer() are called
- **Material error**: Use SetColor() only, avoid SetMaterial() with incompatible types

---

## Increment 28: Project Save/Load (COMPLETED ✓)

### Goal
Implement project file format to save and restore work sessions including frames and joints.

### Implementation Steps
1. Add `current_step_file` tracking variable to MainWindow
2. Create `save_project()` method to serialize project data to JSON
3. Create `load_project()` method to deserialize and restore project state
4. Add "Save Project" and "Load Project" menu items with keyboard shortcuts
5. Implement smart STEP file path handling (absolute path, fallback to relative, manual locate)
6. Add error handling for missing files and corrupted project data
7. Store all user-created frames with positions and orientations
8. Store all joints with complete frame information

### Files to Modify
```
mbd_preprocessor/
└── main.py                     # Add save/load methods and menu items
```

### Project File Format (.mbdp)
```json
{
  "version": "1.0",
  "step_file": "/absolute/path/to/assembly.step",
  "unit_scale": 0.001,
  "frames": [
    {
      "name": "Frame_B0_F2",
      "origin": [0.1, 0.2, 0.3],
      "rotation_matrix": [[1,0,0], [0,1,0], [0,0,1]]
    }
  ],
  "joints": [
    {
      "name": "Joint_Revolute",
      "type": "REVOLUTE",
      "body1_id": 0,
      "body2_id": 1,
      "frame1_name": "Frame_B0_F2",
      "frame1_origin": [0.1, 0.2, 0.3],
      "frame1_rotation": [[1,0,0], [0,1,0], [0,0,1]],
      "frame2_name": "Frame_B1_F5",
      "frame2_origin": [0.4, 0.5, 0.6],
      "frame2_rotation": [[1,0,0], [0,1,0], [0,0,1]],
      "axis": "+Z"
    }
  ]
}
```

### Testing Criteria
- [x] Can save project with Ctrl+S
- [x] Save dialog shows .mbdp file filter
- [x] All frames are saved to project file
- [x] All joints are saved to project file
- [x] STEP file path is saved as absolute path
- [x] Can load project with Ctrl+L
- [x] STEP file is automatically loaded
- [x] All frames are restored with correct positions/orientations
- [x] All joints are restored with correct connections
- [x] Missing STEP file prompts user to locate manually
- [x] Invalid project file shows clear error message
- [x] Can save, close app, reopen, and load project successfully
- [x] Project file is human-readable JSON format

### Use Cases
- Save work-in-progress to continue later
- Share projects with collaborators
- Version control for assembly configurations
- Backup before making major changes
- Quick switching between different joint configurations

### Debugging Checklist
- Verify JSON serialization works for all frame types
- Test with moved/missing STEP files
- Verify frame positions are in correct units
- Check joint frame references are properly restored
- Test with projects containing many frames/joints
- Verify no data loss on save/load cycle

---

## Increment 30: Torque Application

### Goal
Add ability to apply torques (moments) to bodies at specific frames with magnitude and rotation axis.

### Implementation Steps
1. Create `Torque` data structure in `core/data_structures.py`
   - Store torque name, body ID, frame reference, magnitude (N·m), rotation axis vector
   - Auto-normalize axis vectors
2. Create `TorqueDialog` in `gui/torque_dialog.py`
   - Body selection (including Ground)
   - Frame selection (world frame, body local frames, user-created frames)
   - Magnitude input (positive float)
   - Rotation axis specification:
     * **Frame Axis Mode**: Choose from +X, -X, +Y, -Y, +Z, -Z relative to selected frame
     * **Custom Vector Mode**: Specify arbitrary axis (x, y, z components)
   - Transform frame axis directions to world coordinates using frame rotation matrix
3. Create `TorqueRenderer` in `visualization/torque_renderer.py`
   - Render torques as 3D circular arrows (arc + cone arrowhead)
   - Magenta color for torque visualization
   - Logarithmic scaling: radius = base_radius × (1 + log₁₀(magnitude))
   - Base radius: 0.5 meters, adjustable with torque_scale parameter
   - Arc: 270° circular arc using GC_MakeArcOfCircle
   - Arrow components: BRepBuilderAPI_MakeEdge for arc, BRepPrimAPI_MakeCone for arrowhead
   - Thick lines (width 5.0) for visibility
4. Add torque storage to MainWindow
   - `torques: Dict[str, Torque]` dictionary
   - `add_torque()` method with TorqueDialog integration
   - `on_torque_selected()` handler for tree selection
   - `on_torque_deleted()` handler for deletion
5. Integrate torques into tree widget
   - Add "Torques" group with expand/collapse
   - List all torques with names
   - Right-click context menu with "Delete Torque" option
   - Selection signal to show torque properties
6. Add torque property panel section
   - Display torque name, magnitude, rotation axis vector
   - Show associated body and frame
   - Update when torque selected in tree

### Files Created/Modified
```
mbd_preprocessor/
├── core/
│   └── data_structures.py          # Torque class
├── gui/
│   ├── torque_dialog.py            # NEW: Torque creation dialog
│   ├── body_tree_widget.py         # Add torques group
│   └── property_panel.py           # Add torque_group UI
├── visualization/
│   └── torque_renderer.py          # NEW: Torque circular arrow renderer
└── main.py                         # Torque management methods
```

### Torque Data Structure
```python
class Torque:
    def __init__(self, name: str, body_id: int, frame: Frame, 
                 magnitude: float, axis: np.ndarray):
        self.name = name
        self.body_id = body_id
        self.frame = frame
        self.magnitude = magnitude
        self.axis = axis / np.linalg.norm(axis)  # Auto-normalize
```

### Testing Criteria
- [x] Can create torque via Edit > Add Torque menu
- [x] Torque dialog validates all inputs (name, magnitude > 0, axis non-zero)
- [x] Torque appears in tree widget under "Torques" group
- [x] Torque circular arrow visible in 3D viewer (magenta)
- [x] Arc radius scales with magnitude (logarithmic)
- [x] Arrow points around correct rotation axis
- [x] Frame axis mode transforms axis correctly to world coordinates
- [x] Custom vector mode accepts arbitrary rotation axes
- [x] Selecting torque in tree shows properties in panel
- [x] Can delete torque via right-click context menu
- [x] Torque properties display correctly (body, frame, magnitude, axis)
- [x] Multiple torques can coexist without interference
- [x] Torques persist in tree when switching selections

### Use Cases
- Apply motor torque at joint (magnitude and axis of rotation)
- Define external moments for simulation setup
- Visualize torque magnitudes and rotation axes before export
- Test different torque configurations
- Model rotational actuators and motors

### Debugging Checklist
- Verify circular arc rendering (270° arc with proper orientation)
- Check rotation axis transformation for frame axes
- Test with wide range of magnitudes (1 N·m to 10000 N·m)
- Verify arc color is distinct from forces and bodies (magenta)
- Ensure arrowhead points in correct tangential direction
- Test torque at world origin vs. body-attached frames
- Verify torque deletion removes arc from viewer

### Known Issues & Solutions
- **Arc too small**: Increase `base_radius` in TorqueRenderer (default: 0.5m)
- **Axis incorrect**: Check frame rotation_matrix transformation in TorqueDialog
- **Arc not visible**: Verify GC_MakeArcOfCircle succeeded and Display() called
- **Wrong rotation direction**: Check right-hand rule for axis orientation

---

## Increment 31: Joint Limits

### Goal
Add ability to define limits for revolute and prismatic joints.

### Implementation Steps
1. Add "Joints" section to tree widget
2. List all joints with names (Joint_0, Joint_1, etc.)
3. Show joint type and connected bodies
4. Right-click menu to delete joints
5. Click to select and show in property panel

### Files to Modify
```
mbd_preprocessor/
└── gui/
    └── body_tree_widget.py     # Add joints section
```

### Testing Criteria
- [ ] All joints appear in tree
- [ ] Joint names are descriptive (type + bodies)
- [ ] Can delete joints via right-click
- [ ] Selecting joint shows properties
- [ ] Selecting joint highlights it in viewer

---

## Increment 30: Convex Hull Generation

### Goal
Generate simplified convex hull for collision detection.

### Implementation Steps
1. Implement convex hull generation using scipy
2. Sample points from body surface
3. Compute convex hull vertices
4. Store hull in RigidBody.collision_hull
5. Add checkbox to visualize hull in viewer

### Files to Create
```
mbd_preprocessor/
└── core/
    └── geometry_utils.py       # Convex hull generation
```

### Testing Criteria
- [ ] Convex hull generated for each body
- [ ] Hull encloses original geometry
- [ ] Hull is simplified (fewer vertices than mesh)
- [ ] Checkbox toggles hull visualization
- [ ] Hull vertices listed in property panel

### Test Cases
- Simple cube → 8 vertices
- Cylinder → circular hull
- Complex shape → reasonable simplification

---

## Increment 30: Mesh Tessellation for Export

### Goal
Generate high-quality triangulated mesh for visualization export.

### Implementation Steps
1. Implement mesh tessellation with quality parameter
2. Generate mesh for each body
3. Store mesh data (vertices, faces) in RigidBody
4. Display mesh statistics in property panel
5. Add mesh quality slider (low/medium/high)

### Files to Modify
```
mbd_preprocessor/
└── core/
    └── geometry_utils.py       # Mesh tessellation
```

### Testing Criteria
- [ ] Mesh generated with adjustable quality
- [ ] Higher quality = more triangles
- [ ] Mesh captures geometry details
- [ ] Mesh statistics shown (vertex count, face count)
- [ ] Tessellation completes in reasonable time

---

## Increment 31: JSON Export - Metadata

### Goal
Export assembly metadata to JSON file.

### Implementation Steps
1. Create JSONExporter class
2. Export metadata: version, date, units, coordinate system
3. Add "Export > Export JSON" menu item
4. Save JSON file to user-selected location
5. Display success message

### Files to Create
```
mbd_preprocessor/
└── export/
    └── json_exporter.py        # JSON export (new)
```

### Testing Criteria
- [ ] Export menu item works
- [ ] File dialog for save location
- [ ] JSON file created with metadata section
- [ ] Metadata contains all required fields
- [ ] JSON is valid (can be parsed)

### Test JSON Structure
```json
{
  "metadata": {
    "preprocessor_version": "0.1.0",
    "export_date": "2026-01-12T10:30:00Z",
    "units": {"length": "meters", "mass": "kilograms"},
    "coordinate_system": "right_handed_z_up"
  }
}
```

---

## Increment 32: JSON Export - Bodies

### Goal
Export body properties to JSON.

### Implementation Steps
1. Add bodies array to JSON export
2. Export for each body: id, name, volume, COM, inertia
3. Include local frame data
4. Include collision hull vertices
5. Verify JSON format matches specification

### Files to Modify
```
mbd_preprocessor/
└── export/
    └── json_exporter.py        # Add body export
```

### Testing Criteria
- [ ] All bodies exported to JSON
- [ ] Each body has all required fields
- [ ] Inertia tensor exported as 3×3 array
- [ ] Collision hull vertices exported
- [ ] Can load and parse exported JSON

### Debugging Checklist
- Verify array shapes (3×3 for inertia, Nx3 for hull)
- Check number precision (6-8 decimal places)
- Validate JSON schema

---

## Increment 33: JSON Export - Joints

### Goal
Export joint definitions to JSON.

### Implementation Steps
1. Add joints array to JSON export
2. Export for each joint: id, name, type, body IDs
3. Export frame_body_1 and frame_body_2 (local frames)
4. Export frame_global (computed from current positions)
5. Include joint constraints (limits)

### Files to Modify
```
mbd_preprocessor/
└── export/
    └── json_exporter.py        # Add joint export
```

### Testing Criteria
- [ ] All joints exported to JSON
- [ ] Each joint has all required fields
- [ ] Frames exported as origin + rotation matrix
- [ ] Joint type exported correctly
- [ ] Constraints included (if defined)

---

## Increment 34: OBJ Mesh Export

### Goal
Export visualization meshes to OBJ files.

### Implementation Steps
1. Implement OBJ file writer
2. Export each body mesh to separate OBJ file
3. Create meshes/ subdirectory
4. Name files: body_0_visual.obj, body_1_visual.obj, etc.
5. Reference OBJ paths in JSON export

### Files to Modify
```
mbd_preprocessor/
└── export/
    └── json_exporter.py        # Add OBJ export
```

### Testing Criteria
- [ ] OBJ files created in meshes/ folder
- [ ] Each OBJ file contains valid mesh
- [ ] OBJ files can be opened in external viewer
- [ ] JSON references correct OBJ paths
- [ ] Mesh coordinates match body coordinates

### Test Cases
- Export single body → 1 OBJ file
- Export 5-body assembly → 5 OBJ files
- Load OBJ in MeshLab/Blender to verify

---

## Increment 35: Project Save/Load

### Goal
Save entire project to .mbd file for later editing.

### Implementation Steps
1. Implement project serialization
2. Save all bodies, joints, frames, settings
3. Add "File > Save Project" and "File > Load Project"
4. Use JSON format with .mbd extension
5. Preserve all user-created data

### Files to Modify
```
mbd_preprocessor/
└── export/
    └── json_exporter.py        # Add project save/load
```

### Testing Criteria
- [ ] Can save project to .mbd file
- [ ] Can load previously saved project
- [ ] All bodies restored correctly
- [ ] All joints restored with frames
- [ ] User-created frames preserved
- [ ] Property values maintained

### Test Cases
- Create assembly, save, close, reopen
- Verify all data intact after reload
- Test with complex assemblies (10+ bodies, 5+ joints)

---

## Increment 36: Data Validation

### Goal
Validate assembly before export with helpful error messages.

### Implementation Steps
1. Check all bodies have valid geometry
2. Check all joints reference existing bodies
3. Verify frames are properly defined
4. Warn about bodies with no joints (floating bodies)
5. Show validation report before export

### Files to Create
```
mbd_preprocessor/
└── core/
    └── validator.py            # Validation logic (new)
```

### Testing Criteria
- [ ] Validation runs before export
- [ ] Errors prevent export
- [ ] Warnings shown but allow export
- [ ] Clear error messages guide user to fix issues
- [ ] Validation report shows all problems

### Validation Checks
- Bodies with zero volume → Error
- Joints with missing body references → Error
- Floating bodies (no joints) → Warning
- Identical joint frames → Warning

---

## Increment 37: Joint Constraints UI

### Goal
Add UI for defining joint limits and constraints.

### Implementation Steps
1. Add constraints section to property panel (when joint selected)
2. For Hinge: rotation limits (min/max angles)
3. For Prismatic: translation limits (min/max distance)
4. For Cylindrical: both rotation and translation limits
5. Store constraints in Joint object

### Files to Modify
```
mbd_preprocessor/
├── core/
│   └── data_structures.py      # Add constraints to Joint
└── gui/
    └── property_panel.py       # Add constraint controls
```

### Testing Criteria
- [ ] Constraint controls appear for appropriate joint types
- [ ] Can set min/max rotation limits for Hinge
- [ ] Can set min/max translation limits for Prismatic
- [ ] Limits validated (min < max)
- [ ] Constraints saved with joint
- [ ] Constraints exported to JSON

---

## Increment 38: Error Handling & User Feedback

### Goal
Robust error handling with clear user feedback.

### Implementation Steps
1. Add try-except blocks around file operations
2. Show error dialogs with helpful messages
3. Add progress bars for long operations
4. Add status bar with operation feedback
5. Log errors to file for debugging

### Files to Modify
```
mbd_preprocessor/
├── main.py                     # Add status bar
└── core/
    └── logger.py               # Logging setup (new)
```

### Testing Criteria
- [ ] Error dialogs show for file load failures
- [ ] Progress bar appears for export operations
- [ ] Status bar shows current operation
- [ ] Errors logged to file
- [ ] User never sees raw exceptions/tracebacks

### Test Error Scenarios
- Load corrupted STEP file
- Load non-STEP file
- Export to read-only directory
- Export with validation errors

---

## Increment 39: UI Polish & Tooltips

### Goal
Improve user experience with polish and documentation.

### Implementation Steps
1. Add tooltips to all buttons and controls
2. Add keyboard shortcuts (Ctrl+O, Ctrl+S, etc.)
3. Add icons to menu items and buttons
4. Improve layout spacing and alignment
5. Add "Help > User Guide" with basic instructions

### Files to Modify
```
mbd_preprocessor/
└── gui/
    └── main_window.py          # Add polish elements
```

### Testing Criteria
- [ ] All buttons have descriptive tooltips
- [ ] Keyboard shortcuts work
- [ ] Icons are clear and intuitive
- [ ] Layout is clean and professional
- [ ] Help menu provides useful information

---

## Increment 40: Testing & Documentation

### Goal
Final testing and create user documentation.

### Implementation Steps
1. Test with variety of STEP files (simple to complex)
2. Test all joint types
3. Test export and reimport
4. Write README.md with installation and usage
5. Create example project with sample STEP file

### Files to Create
```
mbd_preprocessor/
├── README.md                   # User documentation
├── examples/
│   ├── simple_robot.step
│   └── simple_robot.mbd
└── tests/
    └── test_data/              # Test STEP files
```

### Testing Checklist
- [ ] Load 10+ different STEP files
- [ ] Create Fixed, Hinge, Prismatic, Cylindrical joints
- [ ] Export and validate JSON format
- [ ] Save and load project files
- [ ] Test on fresh installation (clean environment)
- [ ] Documentation is clear and complete

---

## Progress Tracking

Use this checklist to track overall progress:

### Foundation (Increments 1-6)
- [ ] Inc 1: Minimal GUI + STEP display
- [ ] Inc 2: File menu + open dialog
- [ ] Inc 3: Extract individual bodies
- [ ] Inc 4: Body list widget
- [ ] Inc 5: Body selection & highlighting
- [ ] Inc 6: Simple property display

### Physics (Increments 7-10)
- [ ] Inc 7: Volume calculation
- [ ] Inc 8: Center of mass
- [ ] Inc 9: Visualize COM
- [ ] Inc 10: Inertia tensor

### Frames (Increments 11-20)
- [ ] Inc 11: Frame structure & rendering
- [ ] Inc 12: Body local frames
- [ ] Inc 13: Mouse click selection
- [ ] Inc 14: Face selection mode
- [ ] Inc 15: Create frame from face
- [ ] Inc 16: Edge selection mode
- [ ] Inc 17: Create frame from edge
- [ ] Inc 18: Frame list management
- [ ] Inc 19: Manual frame translation
- [ ] Inc 20: Manual frame rotation

### Joints (Increments 21-28)
- [ ] Inc 21: Joint data structure
- [ ] Inc 22: Joint visualization
- [ ] Inc 23: Wizard UI
- [ ] Inc 24: Wizard body selection
- [ ] Inc 25: Wizard geometry selection
- [ ] Inc 26: Wizard frame creation
- [ ] Inc 27: Wizard completion
- [ ] Inc 28: Joint list in tree

### Export (Increments 29-35)
- [ ] Inc 29: Convex hull generation
- [ ] Inc 30: Mesh tessellation
- [ ] Inc 31: JSON export - metadata
- [ ] Inc 32: JSON export - bodies
- [ ] Inc 33: JSON export - joints
- [ ] Inc 34: OBJ mesh export
- [ ] Inc 35: Project save/load

### Polish (Increments 36-40)
- [ ] Inc 36: Data validation
- [ ] Inc 37: Joint constraints UI
- [ ] Inc 38: Error handling
- [ ] Inc 39: UI polish & tooltips
- [ ] Inc 40: Testing & documentation

---

## Daily Workflow

For each increment:

1. **Read** the increment description carefully
2. **Plan** the specific code changes needed
3. **Implement** the increment
4. **Test** all testing criteria
5. **Debug** any issues before moving on
6. **Commit** working code (if using version control)
7. **Move** to next increment only when current one is perfect

**NEVER** skip ahead. Each increment builds on the previous one.

---

## Debugging Strategy

When things don't work:

1. **Isolate** - Which increment broke?
2. **Test** - Does previous increment still work?
3. **Print** - Add debug prints to see values
4. **Simplify** - Test with simplest possible input
5. **Revert** - Go back to last working state if needed

---

## Success Metrics

You'll know an increment is complete when:
- ✅ All testing criteria pass
- ✅ No errors or warnings in console
- ✅ Feature works with multiple test cases
- ✅ Code is clean and commented
- ✅ You can demonstrate it to someone else

**Only then** move to the next increment.
