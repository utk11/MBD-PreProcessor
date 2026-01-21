# Multi-Body Dynamics Preprocessor

## Current Status: ✅ Increments 1-30 Complete (Motor System)

### Completed Features
- ✅ **Increment 1:** Minimal GUI + STEP File Display - TESTED ✓
- ✅ **Increment 2:** File Menu + Open Dialog - TESTED ✓
- ✅ **Increment 3:** Extract Individual Bodies - TESTED ✓
- ✅ **Increment 4:** Body List Widget - TESTED ✓
- ✅ **Increment 5:** Body Selection & Highlighting - TESTED ✓
- ✅ **Increment 6:** Simple Property Display - TESTED ✓
- ✅ **Increment 7:** Volume Calculation (WITH UNIT CONVERSION) - TESTED ✓
- ✅ **Increment 8:** Center of Mass Calculation - TESTED ✓
- ✅ **Increment 9:** Visualize Center of Mass - TESTED ✓
- ✅ **Increment 10:** Inertia Tensor Calculation - TESTED ✓
- ✅ **Increment 11:** Frame Data Structure & Rendering - TESTED ✓
- ✅ **Increment 12:** Body Local Frame - TESTED ✓
- ✅ **Increment 13:** Mouse Click Selection in Viewer - TESTED ✓
- ✅ **Increment 14:** Face Selection Mode - TESTED ✓
- ✅ **Increment 15:** Create Frame from Face - TESTED ✓
- ✅ **Increment 16:** Edge Selection Mode - TESTED ✓
- ✅ **Increment 17:** Create Frame from Edge - TESTED ✓
- ✅ **Increment 27:** Vertex Selection Mode - TESTED ✓
- ✅ **Increment 18:** Frame List Management - TESTED ✓
- ✅ **Increment 19:** Manual Frame Translation - TESTED ✓
- ✅ **Increment 20:** Manual Frame Rotation - TESTED ✓
- ✅ **Increment 21:** Joint Data Structure - TESTED ✓
- ✅ **Increment 22:** Joint Visualization - TESTED ✓
- ✅ **Increment 23:** Joint Creation UI - TESTED ✓
- ✅ **Increment 24:** Joint Properties Panel - TESTED ✓
- ✅ **Increment 25:** Joint Axis Specification & Orientation Triad - TESTED ✓
- ✅ **Increment 26:** Assembly Export (JSON & OBJ) - TESTED ✓
- ✅ **Increment 27:** Body Isolation Mode - TESTED ✓
- ✅ **Increment 28:** Project Save/Load - TESTED ✓
- ✅ **Increment 29:** Force Application - TESTED ✓
- ✅ **Increment 30:** Motor System - TESTED ✓
- ✅ **Body Deletion Feature:** Delete bodies with automatic cleanup - TESTED ✓

### What Works Now

- **Motor System (NEW!)**
    - **Add Motors to Joints**: Convert revolute or prismatic joints into motorized actuators
    - **Three Motor Types**:
      - **Velocity Motor**: Speed control (rad/s for revolute, m/s for prismatic)
      - **Torque Motor**: Force/torque control (N·m for revolute, N for prismatic)
      - **Position Motor**: Position control (rad for revolute, m for prismatic)
    - **Motor Dialog**: User-friendly interface to add motors with automatic unit display
    - **Visual Indicators**: 
      - Green curved arrows for velocity motors
      - Orange straight arrows for torque motors
      - Purple crosshair for position motors
    - **Tree Widget Integration**: Motorized joints show ⚡ lightning bolt indicator
    - **Property Panel**: Displays motor type, value, and units for selected joints
    - **Validation**: Only revolute and prismatic joints can be motorized (one motor per joint)
    - **Motor Management**: Add or remove motors from joints via Assembly menu
    - **JSON Export**: Motor specifications included in assembly export with units
    - **One Motor Per Joint**: Clean design prevents multiple motors on same joint
- **Torque Application**
    - **Create Torque**: Apply torques (moments) to bodies or ground via Edit menu
    - **Torque Dialog**: Define torque name, magnitude (N·m), and rotation axis
    - **Axis Modes**:
      - **Frame Axis**: Select rotation axis along frame axes (+X, -X, +Y, -Y, +Z, -Z)
      - **Custom Vector**: Specify arbitrary rotation axis (x, y, z components)
    - **Circular Arrow Visualization**: Torques rendered as magenta circular arrows (3/4 circle with arrowhead)
    - **Logarithmic Scaling**: Arc radius scales with torque magnitude
    - **Frame Attachment**: Torques applied at specific frames (world frame, body local frames, or user-created frames)
    - **Property Display**: View torque properties (body, frame, magnitude, axis) in property panel
    - **Tree Management**: Torques organized in tree widget under "Torques" group
    - **Torque Deletion**: Right-click context menu to delete torques
    - **Body Association**: Torques linked to bodies (or ground) with automatic coordinate transformations
- **Force Application**
    - **Create Force**: Apply forces to bodies or ground via Edit menu or toolbar
    - **Force Dialog**: Define force name, magnitude (N), and direction
    - **Direction Modes**: 
      - **Frame Axis**: Select force direction along frame axes (+X, -X, +Y, -Y, +Z, -Z)
      - **Custom Vector**: Specify arbitrary direction vector (x, y, z components)
    - **Arrow Visualization**: Forces rendered as blue arrows with length proportional to magnitude
    - **Logarithmic Scaling**: Handles wide range of force magnitudes gracefully
    - **Frame Attachment**: Forces applied at specific frames (world frame, body local frames, or user-created frames)
    - **Property Display**: View force properties (body, frame, magnitude, direction) in property panel
    - **Tree Management**: Forces organized in tree widget under "Forces" group
    - **Force Deletion**: Right-click context menu to delete forces
    - **Body Association**: Forces linked to bodies (or ground) with automatic coordinate transformations
- **Project Save/Load**
    - **Save Project** (Ctrl+S): Save your work to a `.mbdp` project file
    - **Load Project** (Ctrl+L): Restore a previously saved project
    - **JSON Format**: Human-readable project files for easy inspection
    - **Smart Loading**: Automatically locates STEP file or prompts if moved
    - **Complete State**: Saves all frames, joints, and settings
    - **Version Tracking**: Future-proof format with version information
    - **Error Handling**: Clear feedback if files are missing or corrupted
    - **Workflow**: Create frames/joints, save project, close app, reload later
- **Body Deletion**
    - **Context Menu**: Right-click any body in the tree widget to access delete option
    - **Delete Body**: Remove a body from the assembly with confirmation dialog
    - **Automatic Cleanup**: Automatically removes associated joints, frames, and properties
    - **Safe Deletion**: Confirmation dialog prevents accidental deletions
    - **GUI Update**: Tree widget, property panel, and 3D viewer all update automatically
    - **Use Case**: Remove unwanted bodies from imported assemblies before export
- **Body Isolation Mode**
    - **Context Menu**: Right-click any body in the tree widget to access isolation options
    - **Isolate Body**: Hide all bodies except the selected one for unobstructed viewing
    - **Exit Isolation**: Restore visibility of all bodies
    - **Use Case**: Perfect for selecting faces/edges on bodies that are occluded by other bodies
    - **Workflow Integration**: Simplifies frame creation on hidden surfaces
    - **State Tracking**: System remembers which body is isolated and provides clear feedback
- **Assembly Export**
    - **JSON Export**: Export complete assembly to JSON format (Ctrl+E)
      - All body properties (ID, name, volume, COM, inertia tensor)
      - All joint definitions with frames transformed to world coordinates
      - All user-created frames
      - Metadata including units and coordinate system
      - All coordinates in global world frame for consistency
    - **OBJ Mesh Export**: Export body geometries as .obj files (Ctrl+Shift+E)
      - Each body saved as individual .obj file ({body_name}_{body_id}.obj)
      - Triangulated mesh with vertices in world coordinates
      - Standard OBJ format compatible with 3D software
      - Batch export to user-selected directory
- **Kinematic Joints**
    - **Joint Data Model**: Support for Fixed, Revolute, Prismatic, Cylindrical, and Spherical joints.
    - **Visualization**: Joints rendered as dashed lines connecting attachment frames.
    - **Creation Dialog**: Modal dialog to create joints between bodies (or Ground).
    - **Axis Selection**: Specify rotation/translation axis (+X, -X, +Y, -Y, +Z, -Z).
    - **Joint Properties**: View and verify joint type, connected bodies, axis, and full frame details in the property panel.
    - **Deletion**: Context menu in tree widget to delete joints.
- **Orientation Triad**
    - **View Cube**: Large coordinate frame (triedron) in the bottom-left corner showing global X, Y, Z orientation.
    - **Visibility**: Significance increased (3x default size) to ensure clear axis convention reference.
- **Ground Body**
    - **Default Ground**: "Ground" body (ID -1) always available as a reference.
    - **Integration**: Can be selected as a base for joints.

- Basic project structure with core/, gui/, visualization/, and export/ directories

- Working PySide6 GUI with 3D viewer
- File menu with Open/Exit options (Ctrl+O, Ctrl+Q shortcuts)
- File dialog to load STEP files (.step, .stp)
- Error handling for invalid files with user-friendly dialogs
- Display clearing when loading new files
- **Body extraction from STEP assemblies**
- **Unique ID and name assignment for each body (body_0, body_1, etc.)**
- **Body count and names displayed in console**
- **All bodies visible in 3D viewer simultaneously**
- **Tree widget on left side showing all bodies**
- **Body count label (e.g., "Bodies (3)")**
- **Resizable splitter between tree and viewer (30% / 70%)**
- **Tree automatically updates when new file loads**
- **Click bodies in tree to select them**
- **Selected bodies highlighted in yellow in 3D viewer**
- **Only one body highlighted at a time**
- **Previous highlight clears when selecting new body**
- **Highlighting clears when loading new file**
- **Right panel showing body properties**
- **Property panel displays selected body name and ID**
- **Shows 'No Selection' when nothing selected**
- **Property panel updates when selection changes**
- **"Visible" checkbox to toggle body visibility (Hide/Show)**
- **Three-panel layout: 20% left tree + 50% viewer + 30% right properties**
- **Volume calculation for all bodies using OCC GProp**
- **Volume displayed in property panel with smart formatting**
- **Volumes calculated automatically when STEP file loads**
- **Scientific notation for very small or large volumes**
- **Center of mass calculation for all bodies using OCC GProp**
- **COM displayed in property panel as [x, y, z] coordinates in meters**
- **COM calculated automatically when STEP file loads**
- **Unit-aware COM calculation (respects STEP file units)**
- **COM visualization as red sphere marker in 3D viewer**
- **COM marker appears when body is selected**
- **Checkbox to toggle COM marker visibility ("Show Center of Mass")**
- **Smooth COM marker updates when selecting different bodies**
- **Inertia tensor calculation for all bodies using OCC GProp**
- **Inertia tensor displayed in property panel as 3×3 matrix**
- **Unit-aware inertia calculation (respects STEP file units)**
- **Inertia tensor normalized about center of mass**
- **Symmetric matrix with positive diagonal values**
- **Scientific notation display for inertia values**
- **Frame class with origin and rotation matrix (3×3)**
- **Frame rendering as RGB axes (X=Red, Y=Green, Z=Blue)**
- **World frame at origin [0, 0, 0] with identity rotation (visible by default)**
- **"Show World Frame" checkbox to toggle visibility**
- **Frame axes scale automatically with model size (20% of bounding box)**
- **Orthogonal axes with arrow heads for clear visualization**
- **Each body has a local coordinate frame initialized at its COM**
- **Local frames created automatically after COM calculation**
- **Local frame aligned with world axes initially (identity rotation)**
- **"Show Local Frame" checkbox in property panel (disabled when no body selected)**
- **Local frame displays when body is selected**
- **Local frame updates automatically when different body selected**
- **Previous body's local frame hidden when selecting new body**
- **Mouse click selection in 3D viewer**
- **Click directly on bodies to select them (left-click)**
- **Tree selection automatically updates when body clicked in viewer**
- **Property panel updates when body clicked in viewer**
- **Highlighting, COM marker, and local frame work with both tree and viewer selection**
- **Click empty space or non-body objects doesn't cause errors**
- **Selection mode dropdown (Body / Face / Edge / Vertex) in property panel**
- **Face extraction from body geometry with indices**
- **Face area calculation (in m²)**
- **Face center point calculation**
- **Face normal vector calculation**
- **Edge extraction from body geometry with indices**
- **Edge length calculation (in m)**
- **Edge midpoint calculation**
- **Edge direction vector calculation**
- **Vertex extraction from body geometry with indices**
- **Vertex coordinate calculation (X, Y, Z in meters)**
- **Switch between body, face, edge, and vertex selection modes**
- **Face information display in property panel (index, area, center, normal)**
- **Edge information display in property panel (index, length, midpoint, direction)**
- **Vertex information display in property panel (index, X, Y, Z coordinates)**
- **Automatic extraction of all faces, edges, and vertices when STEP file loads**
- **Face, edge, and vertex properties indexed for later frame creation**
- **Button to create a frame from the currently selected face (origin at face center, Z along face normal)**
- **Button to create a frame from the currently selected edge (origin at midpoint, Z along edge direction)**
- **Button to create a frame from the currently selected vertex (origin at vertex location)**
- **Vertex highlighting with cyan sphere marker (8mm radius) in 3D viewer**
- **Frame list in tree widget**: "Frames" group separately from bodies
- **List of user-created frames** displayed in the tree
- **Select frames** from the tree to view their properties
- **Delete frames** via right-click context menu
- **Manual Frame Translation**: Adjust frame (X, Y, Z) position using spinboxes
- **Manual Frame Rotation**: Rotate frame using X, Y, Z Euler angle spinboxes (degrees)
- **Robust selection in 3D viewer with tuned pixel tolerance (4 px)**
- **Improved AIS object identification for sub-shape selection**

### Test Results
When you load a STEP file, the console displays:
```
Loading STEP file: your_file.step
Found 3 body/bodies in the assembly:
  - Body_0 (ID: 0)
  - Body_1 (ID: 1)
  - Body_2 (ID: 2)
Body tree widget initialized
Tree widget updated with 3 bodies
STEP file loaded successfully! Total bodies: 3
```

When you load a file, units are detected and volumes are calculated:
```
Loading STEP file: your_file.step
STEP file declares units: MILLIMETRE
Using unit scale: 0.001 m/unit (millimeters)
Volume scale factor: 1e-09 (to convert to m³)

Calculating volumes...
Unit scale: 0.001 m/unit
Volume scale: 1e-09 (model_units³ to m³)
Body 0 (Body_0): Volume = 1.234567e+03 model³ = 1.234567e-06 m³
Body 1 (Body_1): Volume = 2.345678e+02 model³ = 2.345678e-07 m³
Body 2 (Body_2): Volume = 5.678901e+03 model³ = 5.678901e-06 m³
Volume calculation complete.

Calculating centers of mass...
Unit scale: 0.001 m/unit
Body 0 (Body_0): COM = [-0.019151, 0.089888, 0.001500] m
Body 1 (Body_1): COM = [-0.010848, 0.072450, 0.004500] m
Body 2 (Body_2): COM = [0.016015, 0.042184, 0.004500] m
Center of mass calculation complete.

Calculating inertia tensors...
Unit scale: 0.001 m/unit
Inertia scale: 1e-15 (model_units^5 to kg·m²)
Body 0 (Body_0): Inertia tensor calculated
  Diagonal: [1.234567e-10, 2.345678e-10, 3.456789e-10] kg·m²
Body 1 (Body_1): Inertia tensor calculated
  Diagonal: [4.567890e-11, 5.678901e-11, 6.789012e-11] kg·m²
Body 2 (Body_2): Inertia tensor calculated
  Diagonal: [7.890123e-11, 8.901234e-11, 9.012345e-11] kg·m²
Inertia tensor calculation complete.

Initializing local frames...
Body 0 (Body_0): Local frame created at COM [-0.019151, 0.089888, 0.001500]
Body 1 (Body_1): Local frame created at COM [-0.010848, 0.072450, 0.004500]
Body 2 (Body_2): Local frame created at COM [0.016015, 0.042184, 0.004500]
Local frame initialization complete.
```

When you click a body in the tree:
```
Body selected in tree: Body_1 (ID: 1)
Highlighting body 1 in viewer
Body 1 highlighted in viewer
COM marker displayed at [-0.010848, 0.072450, 0.004500] m
Frame 'Body_1_LocalFrame' rendered at [-0.010848, 0.072450, 0.004500] m (visible: True)
Property panel updated for body: Body_1 (ID: 1)
```

When you click a body directly in the 3D viewer:
```
Body clicked in viewer: Body_1 (ID: 1)
Body selected in tree: Body_1 (ID: 1)
Highlighting body 1 in viewer
Body 1 highlighted in viewer
COM marker displayed at [-0.010848, 0.072450, 0.004500] m
Frame 'Body_1_LocalFrame' rendered at [-0.010848, 0.072450, 0.004500] m (visible: True)
Property panel updated for body: Body_1 (ID: 1)
```

And the GUI shows:
- **Left panel:** Tree widget with "Bodies (3)" label and list of all bodies
- **Middle panel:** 3D viewer with all bodies displayed
- **Right panel:** Property panel showing selected body's name, ID, **volume in m³**, **COM coordinates**, and **inertia tensor as 3×3 matrix**
- **Splitter handles:** Draggable dividers between panels (20%/50%/30% ratio)
- **Selection:** Clicked body in tree is highlighted in bright yellow in the viewer
- **Click Selection:** Can also click directly on bodies in the 3D viewer to select them
- **Tree Sync:** Tree selection automatically updates when body clicked in viewer
- **COM Marker:** Small red sphere appears at the center of mass location
- **Local Frame:** RGB axes appear at body's COM (X=Red, Y=Green, Z=Blue)
- **Visualization Options:** 
  - Checkbox to toggle COM marker visibility
  - Checkbox to toggle World frame visibility
  - Checkbox to toggle Local frame visibility (enabled when body selected)
- **Clear visual feedback:** Easy to see which body is selected, where its COM is, and its local coordinate frame
- **Property display:** Shows "Name: Body_1", "ID: 1", "Volume: 2.345678e-07 m³", "Center of Mass: [-0.010848, 0.072450, 0.004500] m", and inertia tensor matrix

### Quick Start

1. **Install Dependencies:**
   ```powershell
   pip install -r requirements.txt
   ```
   Or with conda:
   ```powershell
   conda install -c conda-forge pythonocc-core
   pip install PySide6 numpy scipy trimesh
   ```

2. **Run the Application:**
   ```powershell
   python main.py
   ```

3. **Load a STEP File:**
   - Click **File > Open** (or press Ctrl+O)
   - Select your STEP file
   - View all bodies in the 3D model
   - Check console output for body count, names, and volumes
   - See body list in left panel tree widget

3b. **Or Load a Saved Project:**
   - Click **File > Load Project** (or press Ctrl+L)
   - Select your `.mbdp` project file
   - STEP file, frames, and joints are automatically restored
   - If STEP file has moved, you'll be prompted to locate it

4. **Interact with the UI:**
   - **Click any body in the left tree widget to select it**
   - **Or click directly on a body in the 3D viewer to select it (left-click)**
   - **Right-click any body in the tree for context menu options:**
     - Select "Isolate Body" to hide all other bodies
     - Select "Exit Isolation" to show all bodies again
     - Select "Delete Body" to remove the body (with confirmation)
     - Useful for accessing hidden faces/edges or removing unwanted bodies
   - Selected body will highlight in yellow in the 3D viewer
   - Tree selection updates automatically when clicking in viewer
   - Red sphere appears at center of mass location
   - RGB axes (local frame) appear at body's COM
   - Right panel shows body name, ID, volume, COM coordinates, and inertia tensor
   - Use "Show Center of Mass" checkbox to toggle COM marker visibility
   - Use "Show World Frame" checkbox to toggle world coordinate frame display (visible by default)
   - Use "Show Local Frame" checkbox to toggle body's local frame display (enabled when body selected)
   - World frame displays as RGB axes: X=Red, Y=Green, Z=Blue
   - Local frames also use RGB axes at each body's COM
   - Click different bodies to see highlighting, COM marker, and local frame update
   - Drag the splitter handles to resize panels
   - Rotate view in 3D viewer (right mouse drag)
   - Pan view (Shift + right mouse drag)
   - Zoom (mouse wheel)

5. **Export Assembly Data:**
   - **Export as JSON**: Click **Export > Export Assembly as JSON...** (or press Ctrl+E)
     - Select output file location
     - Exports complete assembly with all bodies, joints, frames in world coordinates
   - **Export Body Meshes**: Click **Export > Export Body Meshes as OBJ...** (or press Ctrl+Shift+E)
     - Select output directory
     - Exports all bodies as individual .obj mesh files
     - Files named as {body_name}_{body_id}.obj

6. **Save Your Work:**
   - Click **File > Save Project** (or press Ctrl+S)
   - Choose location and name for your `.mbdp` project file
   - All frames and joints are saved
   - Reopen later with **File > Load Project** (Ctrl+L)

### Testing Criteria (Completed ✓)

**Increment 1:**
- ✓ Window opens without errors
- ✓ Can see 3D viewport (gray background)
- ✓ Can rotate view (right mouse button drag)
- ✓ Can pan view (Shift + right mouse button)
- ✓ Can zoom view (mouse wheel)

**Increment 2:**
- ✓ Menu bar appears in window
- ✓ File dialog opens when clicking "File > Open"
- ✓ Can select and load different STEP files
- ✓ Previous geometry is cleared when loading new file
- ✓ Error messages show for invalid files

**Increment 3:**
- ✓ Correctly counts number of bodies in assembly
- ✓ Each body has unique ID (body_0, body_1, etc.)
- ✓ Body names generated (Body_0, Body_1, etc.)
- ✓ All bodies still visible in viewer
- ✓ Console shows body count and names
- ✓ Works with single body files (1 body)
- ✓ Works with multi-body assemblies (multiple bodies)
- ✓ Handles nested assemblies correctly

**Increment 4:**
- ✓ Tree widget appears on left side
- ✓ All bodies listed with names in tree
- ✓ Body count displayed at top ("Bodies (N)")
- ✓ Viewer still works on right side
- ✓ Splitter can resize panels (drag handle)
- ✓ Tree updates when new file loaded
- ✓ Layout doesn't break on window resize

**Increment 5:**
- ✓ Clicking body in tree highlights it in viewer
- ✓ Only one body highlighted at a time
- ✓ Highlighting is clearly visible (bright yellow)
- ✓ Clicking another body switches highlighting
- ✓ Rapid clicking between bodies works smoothly
- ✓ Selection clears when loading new file
- ✓ Console shows selection feedback

**Increment 6:**
- ✓ Right panel appears with property display
- ✓ Shows "No Selection" when nothing selected
- ✓ Displays body name when body selected
- ✓ Displays body ID when body selected
- ✓ Updates when different body selected
- ✓ Three-panel layout works (20% + 50% + 30%)
- ✓ Splitter handles resize panels correctly
- ✓ Property panel clears when loading new file
**Increment 7:**
- ✓ Volume calculated for all bodies automatically
- ✓ Volume displayed in property panel (in m³)
- ✓ **Unit-aware: Extracts units from STEP file (mm, cm, m, inch, etc.)**
- ✓ **Automatic conversion to m³ based on file units**
- ✓ Uses OCC.Core.STEPControl for proper unit handling
- ✓ Scientific notation for very small/large volumes
- ✓ Non-zero volumes for solid bodies verified
- ✓ Calculation completes quickly (< 1 second per body)
- ✓ Shows "Not calculated" if volume calculation fails

**Increment 8:**
- ✓ COM calculated for all bodies automatically
- ✓ COM displayed in property panel as [x, y, z] in meters
- ✓ **Unit-aware: Respects STEP file units for coordinate conversion**
- ✓ Uses OCC.Core.GProp for accurate COM calculation
- ✓ COM values are reasonable (inside body geometry)
- ✓ Calculation completes quickly (< 1 second per body)
- ✓ Shows "Not calculated" if COM calculation fails
- ✓ Display shows 6 decimal places precision

**Increment 9:**
- ✓ Small red sphere appears at COM location when body selected
- ✓ COM marker uses distinct red color for visibility
- ✓ Checkbox in property panel toggles COM marker visibility
- ✓ COM marker updates when different body selected
- ✓ COM marker hidden when body deselected
- ✓ Sphere size appropriate for typical models (5mm radius)
- ✓ Smooth transitions between body selections

**Increment 10:**
- ✓ Inertia tensor calculated for all bodies automatically
- ✓ Tensor displayed in property panel as 3×3 matrix
- ✓ **Unit-aware: Respects STEP file units for scaling (model_units^5 to kg·m²)**
- ✓ Uses OCC.Core.GProp for accurate inertia calculation
- ✓ Tensor normalized about center of mass (not origin)
- ✓ Matrix is symmetric (I[i,j] == I[j,i])
- ✓ Diagonal values are positive
- ✓ Scientific notation for clear display
- ✓ Calculation completes quickly (< 1 second per body)
- ✓ Shows "Not calculated" if inertia calculation fails

**Increment 11:**
- ✓ Frame class stores origin and rotation matrix
- ✓ World frame displays as RGB axes at origin [0, 0, 0] (visible by default)
- ✓ X-axis is red, Y-axis is green, Z-axis is blue
- ✓ Checkbox toggles frame visibility ("Show World Frame")
- ✓ Axes scale appropriately with model size (20% of bounding box)
- ✓ Frame axes are orthogonal with arrow heads
- ✓ Frame visibility persists when loading new files
- ✓ Bounding box computed correctly using OCC.Core.Bnd methods

**Increment 12:**
- ✓ Each body has local_frame field in RigidBody class
- ✓ Local frame initialized at body COM with identity rotation
- ✓ Local frame created automatically after COM calculation
- ✓ "Show Local Frame" checkbox in property panel
- ✓ Checkbox disabled when no body selected
- ✓ Checkbox enabled when body is selected
- ✓ Local frame displays when body selected
- ✓ Local frame hidden when different body selected
- ✓ Frame origin at body COM, aligned with world axes initially
- ✓ Frame updates smoothly when selection changes

**Increment 13:**
- ✓ Mouse click detection implemented in 3D viewer
- ✓ Left-click on body selects it in viewer
- ✓ OCC selection API used to identify clicked body
- ✓ Tree widget selection updates when body clicked in viewer
- ✓ Property panel updates with clicked body info
- ✓ Highlighting works same as tree selection
- ✓ COM marker and local frame display on click
- ✓ Clicking empty space doesn't cause errors
- ✓ Clicking on frames/markers doesn't interfere with body selection
- ✓ Seamless integration between tree and viewer selection
- ✓ Frame origin at body COM, aligned with world axes initially
- ✓ Frame updates smoothly when selection changes

**Increment 14:**
- ✓ Selection mode dropdown implemented (Body / Face / Edge)
- ✓ Face extraction from body geometry working
- ✓ Face properties calculated: index, area, center, normal
- ✓ Edge extraction from body geometry working
- ✓ Edge properties calculated: index, length, midpoint, direction
- ✓ Faces and edges extracted automatically when STEP file loads
- ✓ Face information displayed in property panel
- ✓ Edge information displayed in property panel
- ✓ Property groups visibility updated based on selection mode
- ✓ Face and edge indices computed for all bodies
- ✓ Units properly handled (m² for area, m for lengths)
- ✓ All 20 test bodies processed successfully (up to 1092 edges per body)

### Mouse Controls

- **Select Body:** Left mouse button click on body
- **Rotate:** Right mouse button drag
- **Pan:** Shift + right mouse button drag
- **Zoom:** Mouse wheel

---

## Project Structure

```
NF/
├── main.py                     # ✅ Application entry point with three-panel layout
├── requirements.txt            # ✅ Dependencies (PySide6, pythonocc-core, etc.)
├── README.md                   # ✅ This file - updated with Increment 26 status
├── Idea.md                     # Original concept document
├── plan.md                     # Overall development plan
├── implementation.md           # 40-increment implementation guide
├── core/                       # ✅ Core logic
│   ├── __init__.py            # ✅ Module exports
│   ├── data_structures.py     # ✅ RigidBody, Frame, Joint, and Assembly classes
│   ├── step_parser.py         # ✅ STEP file loading and body extraction
│   ├── physics_calculator.py  # ✅ Volume, COM, and inertia calculations
│   └── geometry_utils.py      # ✅ Face and edge property extraction
├── gui/                        # ✅ GUI components
│   ├── __init__.py            # ✅ Module exports
│   ├── body_tree_widget.py    # ✅ Tree widget with bodies, frames, and joints
│   ├── property_panel.py      # ✅ Property display with controls and frame creation
│   ├── viewer_3d.py           # ✅ Enhanced 3D viewer with body/face/edge selection
│   └── joint_dialog.py        # ✅ Joint creation dialog
├── visualization/              # ✅ Rendering code
│   ├── __init__.py            # ✅ Module exports
│   ├── body_renderer.py       # ✅ Body highlighting, rendering, and COM visualization
│   ├── frame_renderer.py      # ✅ Frame visualization as RGB axes
│   ├── face_renderer.py       # ✅ Face highlighting overlay
│   ├── edge_renderer.py       # ✅ Edge highlighting with thick red lines
│   └── joint_renderer.py      # ✅ Joint connection line rendering
└── export/                     # ✅ Export functionality
    ├── __init__.py            # ✅ Module exports
    └── exporter.py            # ✅ JSON and OBJ export (world frame coordinates)
```

---

---

## Troubleshooting

**If pythonocc-core fails to install:**
- Windows: Use conda: `conda install -c conda-forge pythonocc-core`
- Or download pre-built wheels from the pythonOCC website

**If PySide6 installation issues:**
- Make sure you're using Python 3.8 or higher
- Try: `pip install PySide6 --upgrade`

**If OpenGL errors occur:**
- Update your graphics drivers
- Check that your system supports OpenGL 2.1 or higher

**If File > Open doesn't show files:**
- Make sure your STEP files have .step or .stp extensions
- Try selecting "All Files (*.*)" in the file dialog

---

## Development Progress

### Completed & Tested ✅
1. **Increment 1:** Basic GUI with 3D viewer ✓
2. **Increment 2:** File menu and STEP file loading ✓
3. **Increment 3:** Extract individual bodies from assemblies ✓
4. **Increment 4:** Body list widget ✓
5. **Increment 5:** Body selection & highlighting ✓
6. **Increment 6:** Simple property display panel ✓
7. **Increment 7:** Volume calculation ✓
8. **Increment 8:** Center of mass calculation ✓
9. **Increment 9:** Visualize center of mass ✓
10. **Increment 10:** Inertia tensor calculation ✓
11. **Increment 11:** Frame data structure & rendering ✓
12. **Increment 12:** Body local frame ✓
13. **Increment 13:** Mouse click selection in viewer ✓
14. **Increment 14:** Face selection mode ✓
15. **Increment 15:** Create frame from face ✓
16. **Increment 16:** Edge selection mode ✓
17. **Increment 17:** Create frame from edge ✓
18. **Increment 18:** Frame list management ✓
19. **Increment 19:** Manual frame translation ✓
20. **Increment 20:** Manual frame rotation ✓
21. **Increment 21:** Joint data structure ✓
22. **Increment 22:** Joint visualization ✓
23. **Increment 23:** Joint creation UI ✓
24. **Increment 24:** Joint properties panel ✓
25. **Increment 25:** Joint axis specification ✓
26. **Increment 26:** Assembly export (JSON & OBJ) ✓
27. **Increment 27:** Body isolation mode ✓
28. **Increment 28:** Project save/load ✓

### Ready to Implement 🔨
29. **Increment 29:** URDF Export Format (NEXT)

### Upcoming 🚀
30. **Increment 30:** Material Properties
31. **Increment 31:** Contact Detection
32. **Increment 32:** Animation & Simulation Preview

### Summary
- **28 of 40 increments complete** (70% done)
- **Export** functionality implemented with world frame coordinates
- Bodies, Frames, Joints, and Export fully implemented
- Ready for advanced export formats (URDF, etc.)
- Steady, incremental development approach working perfectly

See [implementation.md](implementation.md) for the complete 40-increment roadmap.

---

---

## Testing Your Installation

If you don't have a STEP file handy, create a simple test file:

```python
# test_create_step.py
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Extend.DataExchange import write_step_file

# Create a simple box
box = BRepPrimAPI_MakeBox(100, 100, 100).Shape()

# Save as STEP file
write_step_file(box, "test_box.step")
print("Created test_box.step - Load this in the application!")
```

Run: `python test_create_step.py`, then open `test_box.step` in the application.

---

## Contributing

This project follows an incremental development approach. Each feature is built and tested before moving to the next. See [implementation.md](implementation.md) for the detailed roadmap.

When contributing:
1. Complete the current increment fully before moving to the next
2. Test all criteria for each increment
3. Update this README when increments are completed

---

## License

Multi-Body Dynamics Preprocessor  
For research and educational purposes.

