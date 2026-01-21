# Multi-Body Dynamics Preprocessor - Complete Technical Documentation

**Version:** 2.0 (Increment 34 Complete - Multi-Selection, Multi-Delete & Contact Detection)  
**Last Updated:** January 19, 2026  
**Author:** Engineering Team

---

## Recent Updates (Increment 34)

**Major Improvements:**
- ✅ **Multi-Selection for Bodies**: Body tree widget now supports ExtendedSelection mode, allowing users to select multiple bodies using Ctrl+Click (toggle individual items) and Shift+Click (range selection).
- ✅ **Multi-Delete with Dependency Cleanup**: Users can now delete multiple bodies at once. The system automatically identifies and removes all dependent frames and joints, showing a confirmation dialog with the full impact analysis.
- ✅ **Contact Detection Configuration**: Added `contact_enabled` boolean attribute to RigidBody class (fully implemented), allowing users to control whether contact detection should be enabled for each body in the solver. This is exposed as a checkbox in the property panel and exported in the JSON file. Default value: True (enabled).

**Previous Updates (Increment 33):**
- ✅ **Local Frame Mesh Export**: Body meshes are now exported in the body's local coordinate frame (COM-centered) instead of global frame. This simplifies integration with physics engines that work with body-local coordinates.
- ✅ **Single Frame Joint Definition**: Joints now use a single frame expressed in global coordinates instead of two separate frames. This simplifies joint definition and export.
- ✅ **Updated Joint Serialization**: JSON export now includes `frame_world` instead of `frame1_world` and `frame2_world`.
- ✅ **UI Simplification**: Joint creation dialog now only requires selecting one frame location instead of two.

**Previous Updates (Increment 32):**
- ✅ **Dynamic Visualization Scaling**: Markers for forces, torques, and frames now automatically scale based on the model's bounding box size (Forces: 30%, Torques: 20%, Frames: 20% of max model dimension).
- ✅ **Unit-Correct Rendering**: Fixed a critical issue where forces and torques (stored in SI Meters) were rendered incorrectly in Model Units (e.g., Millimeters). Renderers now apply inverse unit scaling.
- ✅ **Enhanced Vertex Visibility**: Vertex selection highlighting now adjusts its size dynamically to remain visible regardless of the model's unit system (targeting ~5mm visual size).
- ✅ **Torque Renderer Upgrade**: Added specialized scaling control to `TorqueRenderer` for better visibility of circular arrows.

---

## Table of Contents

1. [Overview](#overview)
2. [Recent Updates](#recent-updates-increment-31)
3. [Architecture](#architecture)
4. [Technology Stack & Frameworks](#technology-stack--frameworks)
5. [Module Structure](#module-structure)
6. [Data Structures](#data-structures)
7. [Core Modules](#core-modules)
8. [GUI Modules](#gui-modules)
9. [Visualization Modules](#visualization-modules)
10. [Export Modules](#export-modules)
11. [Application Workflow](#application-workflow)
12. [API Reference](#api-reference)
13. [Code Quality Improvements](#code-quality-improvements)

---

## Overview

The Multi-Body Dynamics Preprocessor is a desktop application for loading, analyzing, and preprocessing STEP CAD files for multi-body dynamics simulations. It provides:

- **3D Visualization**: Interactive 3D viewer with mouse-based selection
- **Physical Properties Calculation**: Volume, center of mass, and inertia tensors
- **Geometric Analysis**: Face and edge property extraction
- **Coordinate Frame Management**: World frame, local frames, and custom frames
- **Joint Definition**: Create kinematic joints between bodies with single frame in global coordinates
- **Motor Actuation**: Add motors to joints for velocity, torque, or position control
- **Assembly Export**: Export complete assembly to JSON with frames in global coordinates
- **Mesh Export**: Export body geometries as OBJ mesh files in body-local coordinates (COM-centered)
- **Body Selection**: Multiple selection modes (body, face, edge, vertex) with distinct visual feedback
- **User Interface**: Three-panel layout with body tree, 3D viewer, and property panel

**Key Features:**
- Load and parse STEP files (.step, .stp)
- Extract individual solid bodies from assemblies
- Calculate physical properties with proper unit conversion
- Interactive body, face, edge, and vertex selection
- **Robust Selection Highlighting**: Custom rendering for bodies (yellow), faces (yellow overlay), edges (red thick line), and vertices (cyan sphere)
- Visualize coordinate frames and centers of mass
- Create custom frames from face, edge, and vertex geometry
- Define kinematic joints with frame attachments
- **Add motors to revolute and prismatic joints** for actuation
- **Three motor types**: Velocity, torque/force, and position control
- **Visual motor indicators** in 3D view with color coding
- **Multi-selection support** for bodies with Ctrl+Click and Shift+Click
- **Multi-delete with dependency cleanup** of associated frames and joints
- **Contact detection configuration** per body for solver integration
- **Export assembly data to JSON format (all coordinates in global world frame)**
- **Export body meshes as OBJ files for external use**

---

## Architecture

### High-Level Architecture

The application follows a **modular MVC-inspired architecture** with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────────┐
│                     main.py (MainWindow)                    │
│                  Application Controller/Orchestrator         │
└────────────┬────────────────────────────────┬───────────────┘
             │                                │
    ┌────────▼────────┐              ┌───────▼────────┐
    │   GUI Modules   │              │  Core Modules  │
    │  (User Interface)│              │  (Business Logic)│
    └────────┬────────┘              └───────┬────────┘
             │                                │
    ┌────────▼────────┐              ┌───────▼────────┐
    │ Visualization   │              │ Data Structures │
    │   Modules       │              │  (Model Layer)  │
    └─────────────────┘              └─────────────────┘
```

**Architecture Layers:**

1. **Presentation Layer** (`gui/`):
   - Body tree widget for hierarchical display
   - Property panel for detail viewing
   - 3D viewer for interactive visualization

2. **Business Logic Layer** (`core/`):
   - STEP file parsing and loading
   - Physical property calculations
   - Geometry analysis utilities

3. **Visualization Layer** (`visualization/`):
   - Body rendering and highlighting
   - **Face rendering and highlighting**
   - **Edge rendering and highlighting**
   - **Vertex rendering and highlighting**
   - Frame coordinate system rendering

4. **Data Layer** (`core/data_structures.py`):
   - RigidBody and Frame data models

5. **Controller Layer** (`main.py`):
   - Event handling and coordination
   - Signal/slot connections
   - Application lifecycle management

---

## Technology Stack & Frameworks

### Core Technologies

#### 1. **PythonOCC (pythonocc-core)**
- **Purpose**: CAD kernel and 3D geometry processing
- **Version**: Latest compatible with Python 3.x
- **Key Components Used**:
  - `OCC.Core.TopoDS`: Topology data structures (shapes, faces, edges, solids)
  - `OCC.Core.STEPControl`: STEP file I/O
  - `OCC.Core.GProp`: Geometric property calculations (volume, center of mass, inertia)
  - `OCC.Core.BRepGProp`: Boundary representation geometric properties
  - `OCC.Core.AIS`: Application Interactive Services for 3D display
  - `OCC.Display`: 3D visualization backend
  - `OCC.Core.BRepPrimAPI`: Primitive shape creation (spheres, cylinders, cones)
  - `OCC.Core.Quantity`: Color and material definitions

#### 2. **PySide6**
- **Purpose**: Qt-based GUI framework
- **Version**: >= 6.0.0
- **Key Components Used**:
  - `QMainWindow`: Main application window
  - `QTreeWidget`: Hierarchical body list
  - `QSplitter`: Resizable panel layout
  - `QFileDialog`: File selection dialogs
  - `QMenuBar`: Menu system
  - `Signal/Slot`: Event-driven communication
  - `QWidget`, `QVBoxLayout`, `QFormLayout`: UI layout management

#### 3. **NumPy**
- **Purpose**: Numerical computing and array operations
- **Version**: >= 1.21.0
- **Usage**:
  - Coordinate frame mathematics (rotation matrices, vectors)
  - Inertia tensor calculations
  - Center of mass computations
  - Linear algebra operations

### Development Stack

- **Language**: Python 3.x
- **Build System**: pip (requirements.txt)
- **Dependencies**: Listed in `requirements.txt`

---

## Module Structure

### Directory Organization

```
Multi-Body Dynamics Preprocessor/
│
├── main.py                      # Application entry point
├── requirements.txt             # Python dependencies
├── README.md                    # Project overview
├── test.step                    # Sample STEP file for testing
│
├── core/                        # Core business logic
│   ├── __init__.py
│   ├── data_structures.py       # Data models (RigidBody, Frame, Joint, Assembly)
│   ├── step_parser.py           # STEP file I/O
│   ├── physics_calculator.py    # Physical property calculations
│   └── geometry_utils.py        # Geometry analysis utilities
│
├── gui/                         # User interface components
│   ├── __init__.py
│   ├── body_tree_widget.py      # Body/frame/joint tree widget
│   ├── property_panel.py        # Property display panel
│   ├── viewer_3d.py             # Enhanced 3D viewer with robust selection
│   ├── joint_dialog.py          # Joint creation dialog
│   ├── force_dialog.py          # Force creation dialog
│   ├── torque_dialog.py         # Torque creation dialog
│   └── motor_dialog.py          # Motor creation dialog
│
├── visualization/               # 3D rendering components
│   ├── __init__.py
│   ├── body_renderer.py         # Body display and highlighting
│   ├── face_renderer.py         # Face highlighting
│   ├── edge_renderer.py         # Edge highlighting
│   ├── vertex_renderer.py       # Vertex highlighting
│   ├── frame_renderer.py        # Coordinate frame rendering
│   ├── joint_renderer.py        # Joint connection line rendering
│   ├── force_renderer.py        # Force arrow rendering
│   ├── torque_renderer.py       # Torque circular arrow rendering
│   └── motor_renderer.py        # Motor actuation indicators
│
├── export/                      # Export functionality
│   ├── __init__.py
│   └── exporter.py              # JSON and OBJ export (world frame coordinates)
│
├── Documentation/               # Project documentation
│   ├── documentation.md         # This file
│   ├── Idea.md                  # Conceptual documentation
│   └── implementation.md        # Implementation notes
│
├── progress_files/              # Development tracking
│   ├── plan.md                  # Development plan
│   └── INCREMENT_*.md           # Increment completion records
│
├── tests/                       # Test scripts and files
│   ├── test_*.py                # Unit and integration tests
│   └── test_mm.step             # Test STEP files
└── __pycache__/                 # Python bytecode cache
```

---

## Data Structures

### 1. Frame Class

**File:** `core/data_structures.py`

**Purpose:** Represents a coordinate frame (reference frame) with origin position and orientation.

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `name` | `str` | Descriptive name for the frame (e.g., "World Frame", "Body_0_LocalFrame") |
| `origin` | `np.ndarray` | 3D position vector [x, y, z] in meters |
| `rotation_matrix` | `np.ndarray` | 3×3 rotation matrix defining frame orientation relative to world |

**Methods:**

```python
__init__(self, origin: np.ndarray = None, rotation_matrix: np.ndarray = None, name: str = "Frame")
```
Initialize a coordinate frame with optional origin and rotation. Defaults to world origin [0, 0, 0] with identity rotation.

```python
get_x_axis(self) -> np.ndarray
```
Returns the X-axis direction vector (first column of rotation matrix).

```python
get_y_axis(self) -> np.ndarray
```
Returns the Y-axis direction vector (second column of rotation matrix).

```python
get_z_axis(self) -> np.ndarray
```
Returns the Z-axis direction vector (third column of rotation matrix).

```python
get_euler_angles(self) -> np.ndarray
```
Returns rotation angles in degrees [x, y, z] (Extrinsic XYZ convention).

```python
set_rotation_from_euler(self, angles_deg: np.ndarray) -> None
```
Sets rotation matrix from Euler angles in degrees [x, y, z] (Extrinsic XYZ convention).

**Usage Example:**
```python
# Create world frame
world_frame = Frame(name="World Frame")

# Create custom frame
custom_origin = np.array([1.0, 2.0, 3.0])
custom_rotation = np.eye(3)  # Identity rotation
custom_frame = Frame(origin=custom_origin, rotation_matrix=custom_rotation, name="Custom")
```

---


### 2. RigidBody Class

**File:** `core/data_structures.py`

**Purpose:** Represents a rigid body extracted from a STEP assembly with associated geometry and physical properties.

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `id` | `int` | Unique identifier (0, 1, 2... for bodies, -1 for Ground) |
| `shape` | `TopoDS_Shape` | OpenCASCADE geometry representation |
| `name` | `str` | Display name (e.g., "Body_0", "Ground") |
| `volume` | `float` | Volume in cubic meters (m³) |
| `center_of_mass` | `Optional[List[float]]` | Center of mass [x, y, z] in meters |
| `inertia_tensor` | `Optional[np.ndarray]` | 3×3 inertia tensor matrix in kg·m² (about COM, unit density) |
| `local_frame` | `Optional[Frame]` | Local coordinate frame at center of mass |
| `visible` | `bool` | Visibility state of the body (True/False) |
| `contact_enabled` | `bool` | Contact detection enabled for this body (True/False, default: True) |

**Methods:**

```python
__init__(self, body_id: int, shape: TopoDS_Shape, name: Optional[str] = None)
```
Initialize a rigid body with ID and geometry. Name defaults to "Body_{id}" if not provided.

**Usage Example:**
```python
# Create a rigid body (typically done by StepParser)
body = RigidBody(body_id=0, shape=solid_shape, name="Chassis")

# After calculation
body.volume = 0.00123  # m³
body.center_of_mass = [0.5, 0.3, 0.1]  # meters
body.inertia_tensor = np.array([[...], [...], [...]])  # kg·m²
body.contact_enabled = True  # Enable contact detection (default)
```

---


### 3. FaceProperties Class

**File:** `core/geometry_utils.py`

**Purpose:** Container for geometric properties of a face (planar or curved surface).

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `face` | `TopoDS_Face` | OpenCASCADE face object |
| `face_index` | `int` | Index of face in parent body (0, 1, 2, ...) |
| `area` | `float` | Surface area in m² |
| `center` | `np.ndarray` | Geometric center [x, y, z] in meters |
| `normal` | `np.ndarray` | Surface normal vector [nx, ny, nz] (normalized) |

**Methods:**

```python
__init__(self, face: TopoDS_Face, face_index: int)
```
Initialize and automatically calculate face properties using `_calculate_properties()`.

```python
_calculate_properties(self) -> None
```
Internal method to compute area, center, and normal using OCC BRepGProp and surface evaluation.

```python
_calculate_normal(self) -> None
```
Internal method to compute surface normal at face center using parametric surface evaluation.

---


### 4. EdgeProperties Class

**File:** `core/geometry_utils.py`

**Purpose:** Container for geometric properties of an edge (curve segment).

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `edge` | `TopoDS_Edge` | OpenCASCADE edge object |
| `edge_index` | `int` | Index of edge in parent body (0, 1, 2, ...) |
| `length` | `float` | Edge length in meters |
| `midpoint` | `np.ndarray` | Midpoint [x, y, z] in meters |
| `direction` | `np.ndarray` | Direction vector [dx, dy, dz] (normalized) |

**Methods:**

```python
__init__(self, edge: TopoDS_Edge, edge_index: int)
```
Initialize and automatically calculate edge properties using `_calculate_properties()`.

```python
_calculate_properties(self) -> None
```
Internal method to compute length, midpoint, and direction using curve adaptor.

---


### 5. Joint Class

**File:** `core/data_structures.py`

**Purpose:** Represents a kinematic joint between two bodies with optional motor actuation.

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `name` | `str` | Unique name for the joint |
| `joint_type` | `JointType` | Joint type (FIXED, REVOLUTE, PRISMATIC, etc.) |
| `body1_id` | `int` | ID of the first body |
| `body2_id` | `int` | ID of the second body |
| `frame` | `Frame` | Joint frame in global world coordinates |
| `axis` | `str` | Axis of rotation/translation (+Z, -Z, +X, -X, etc.) |
| `is_motorized` | `bool` | Whether joint has a motor (default: False) |
| `motor_type` | `Optional[MotorType]` | Type of motor if motorized (VELOCITY, TORQUE, POSITION) |
| `motor_value` | `float` | Motor value (default: 0.0) |

**Methods:**

```python
__init__(self, name: str, joint_type: JointType, body1_id: int, body2_id: int, frame: Frame = None, axis: str = "+Z")
```
Initialize a joint connecting two bodies with a single frame in global coordinates and an optional axis specification.

**Note:** The frame is stored in world coordinates since this is a preprocessor where bodies don't move. No local-to-global conversion is needed.

```python
add_motor(self, motor_type: MotorType, value: float) -> None
```
Add motor actuation to this joint. Only works for REVOLUTE and PRISMATIC joints. Raises ValueError if joint type is invalid or if already motorized.

**Parameters:**
- `motor_type` (MotorType): Type of motor (VELOCITY, TORQUE, POSITION)
- `value` (float): Motor value (rad/s, N·m, rad for revolute; m/s, N, m for prismatic)

```python
remove_motor(self) -> None
```
Remove motor actuation from this joint.

```python
get_motor_description(self) -> str
```
Get a human-readable description of the motor with appropriate units.

**Returns:** String like "VELOCITY: 10.5 rad/s" or "No motor"

---


### 6. MotorType Enum

**File:** `core/data_structures.py`

**Purpose:** Enumeration of motor actuation types.

**Values:**

| Value | Description | Units (Revolute) | Units (Prismatic) |
|-------|-------------|------------------|-------------------|
| `VELOCITY` | Speed control | rad/s | m/s |
| `TORQUE` | Torque/force control | N·m | N |
| `POSITION` | Position control | rad | m |

**Usage Example:**
```python
from core.data_structures import MotorType

# Add velocity motor to revolute joint
joint.add_motor(MotorType.VELOCITY, 10.5)  # 10.5 rad/s

# Add force motor to prismatic joint
joint.add_motor(MotorType.TORQUE, 100.0)  # 100 N
```

---


### 7. Assembly Class

**File:** `core/data_structures.py`

**Purpose:** Container for the entire multi-body system.

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `bodies` | `Dict[int, RigidBody]` | Dictionary mapping ID to RigidBody |
| `joints` | `Dict[str, Joint]` | Dictionary mapping name to Joint |
| `frames` | `Dict[str, Frame]` | Dictionary mapping name to user-defined Frame |

---


## Core Modules

### 1. step_parser.py

**Purpose:** STEP file loading, parsing, and body extraction.

**Class:** `StepParser`

**Static Methods:**

#### `load_step_file(filepath: str) -> Tuple[Optional[TopoDS_Shape], float]`

Load a STEP file and extract unit information.

**Parameters:**
- `filepath` (str): Path to STEP file (.step or .stp)

**Returns:**
- `Tuple[TopoDS_Shape, float]`:
  - Loaded shape (or None if failed)
  - Unit scale factor (meters per model unit)
    - 1.0 for meters
    - 0.001 for millimeters
    - 0.01 for centimeters
    - 0.0254 for inches

**Raises:**
- `FileNotFoundError`: If file doesn't exist
- `Exception`: For loading errors

**Algorithm:**
1. Create `STEPControl_Reader`
2. Read file with `ReadFile()`
3. Extract unit declarations using `FileUnits()`
4. Map unit names to scale factors
5. Transfer roots and return `OneShape()`
6. Print unit information to console

**Usage:**
```python
shape, unit_scale = StepParser.load_step_file("model.step")
if shape:
    print(f"Loaded with unit scale: {unit_scale}")
```

#### `extract_bodies_from_compound(shape: TopoDS_Shape) -> List[RigidBody]`

Extract individual solid bodies from a compound shape.

**Parameters:**
- `shape` (TopoDS_Shape): Compound or single solid shape

**Returns:**
- `List[RigidBody]`: List of extracted bodies with auto-assigned IDs

**Algorithm:**
1. Use `TopExp_Explorer` to traverse `TopAbs_SOLID` shapes
2. Create `RigidBody` for each solid with sequential ID
3. If no solids found, treat entire shape as one body

**Usage:**
```python
bodies = StepParser.extract_bodies_from_compound(shape)
print(f"Found {len(bodies)} bodies")
```

#### `validate_step_file(filepath: str) -> bool`

Check if a file has a valid STEP extension.

**Parameters:**
- `filepath` (str): Path to check

**Returns:**
- `bool`: True if extension is .step, .stp, .STEP, or .STP

---


### 2. physics_calculator.py

**Purpose:** Physical property calculations (volume, center of mass, inertia).

**Class:** `PhysicsCalculator`

**Static Methods:**

#### `calculate_volume(shape: TopoDS_Shape) -> Optional[float]`

Calculate volume of a solid using OpenCASCADE GProp.

**Parameters:**
- `shape` (TopoDS_Shape): Solid shape

**Returns:**
- `float`: Volume in cubic model units (must be scaled to m³), or None if calculation fails

**Algorithm:**
1. Create `GProp_GProps` object
2. Call `brepgprop.VolumeProperties(shape, props)`
3. Extract volume with `props.Mass()`
4. Verify volume > 0

**Notes:**
- Returns volume in **model units³** (not m³)
- Must multiply by `unit_scale³` to convert to m³

#### `calculate_volumes_for_bodies(bodies: List[RigidBody], unit_scale: float = 1.0) -> None`

Calculate and store volumes for all bodies with unit conversion.

**Parameters:**
- `bodies` (List[RigidBody]): List of bodies to process
- `unit_scale` (float): Scale factor (meters per model unit)

**Side Effects:**
- Sets `body.volume` attribute in m³
- Prints volume information to console

**Algorithm:**
1. Compute volume scale: `volume_scale = unit_scale³`
2. For each body:
   - Calculate volume in model units
   - Convert to m³: `volume_m3 = volume * volume_scale`
   - Store in `body.volume`

#### `calculate_center_of_mass(shape: TopoDS_Shape, unit_scale: float = 1.0) -> Optional[List[float]]`

Calculate center of mass (geometric centroid for uniform density).

**Parameters:**
- `shape` (TopoDS_Shape): Solid shape
- `unit_scale` (float): Scale factor

**Returns:**
- `List[float]`: [x, y, z] in meters, or None if failed

**Algorithm:**
1. Create `GProp_GProps` object
2. Call `brepgprop.VolumeProperties()`
3. Extract COM with `props.CentreOfMass()`
4. Scale to meters: `[x * unit_scale, y * unit_scale, z * unit_scale]`

#### `calculate_centers_of_mass_for_bodies(bodies: List[RigidBody], unit_scale: float = 1.0) -> None`

Calculate and store centers of mass for all bodies.

**Side Effects:**
- Sets `body.center_of_mass` attribute
- Prints COM coordinates

#### `calculate_inertia_tensor(shape: TopoDS_Shape, unit_scale: float = 1.0) -> Optional[np.ndarray]`

Calculate inertia tensor about center of mass (unit density assumption).

**Parameters:**
- `shape` (TopoDS_Shape): Solid shape
- `unit_scale` (float): Scale factor

**Returns:**
- `np.ndarray`: 3×3 symmetric inertia tensor in kg·m² (unit density), or None if failed

**Algorithm:**
1. Calculate volume properties using `GProp_GProps` to get mass properties.
2. Get the **central inertia tensor** (about COM) directly using `props.MatrixOfInertia()`.
   _Note: OpenCASCADE's MatrixOfInertia returns the inertia tensor relative to the center of mass._
3. Extract inertia matrix components: `Ixx`, `Iyy`, `Izz`, `Ixy`, `Ixz`, `Iyz`.
4. Scale to SI units: `I_SI = I_model * unit_scale⁵`
   _Geometric inertia scales as length⁵. Mass inertia = density * geometric_inertia. For unit density (1 kg/m³), we just scale dimensions._
5. Construct symmetric 3×3 matrix.

**Notes:**
- Assumes **unit density** (1 kg/m³)
- For actual inertia, multiply by real density: `I_real = density * I_calculated`
- Geometric inertia scales as **length⁵**

#### `calculate_inertia_tensors_for_bodies(bodies: List[RigidBody], unit_scale: float = 1.0) -> None`

Calculate and store inertia tensors for all bodies.

**Side Effects:**
- Sets `body.inertia_tensor` attribute
- Prints diagonal components

#### `initialize_local_frames(bodies: List[RigidBody]) -> None`

Create local coordinate frames at each body's center of mass.

**Parameters:**
- `bodies` (List[RigidBody]): Bodies with calculated COM

**Side Effects:**
- Sets `body.local_frame` attribute
- Frame origin = COM
- Frame rotation = identity (aligned with world axes)

**Usage:**
```python
PhysicsCalculator.initialize_local_frames(bodies)
```

---


### 3. geometry_utils.py

**Purpose:** Geometry analysis and face/edge property extraction.

**Class:** `GeometryUtils`

**Static Methods:**

#### `extract_faces(shape: TopoDS_Shape) -> List[FaceProperties]`

Extract all faces from a shape with computed properties.

**Parameters:**
- `shape` (TopoDS_Shape): Body shape

**Returns:**
- `List[FaceProperties]`: List of face property objects with indices

**Algorithm:**
1. Use `TopExp_Explorer` with `TopAbs_FACE`
2. For each face, create `FaceProperties` object
3. Properties are auto-calculated on construction

#### `extract_edges(shape: TopoDS_Shape) -> List[EdgeProperties]`

Extract all edges from a shape with computed properties.

**Parameters:**
- `shape` (TopoDS_Shape): Body shape

**Returns:**
- `List[EdgeProperties]`: List of edge property objects with indices

#### `get_face_by_index(shape: TopoDS_Shape, face_index: int) -> Optional[TopoDS_Face]`

Retrieve a specific face by index.

**Parameters:**
- `shape` (TopoDS_Shape): Body shape
- `face_index` (int): Zero-based face index

**Returns:**
- `TopoDS_Face`: Face object, or None if index out of range

#### `get_edge_by_index(shape: TopoDS_Shape, edge_index: int) -> Optional[TopoDS_Edge]`

Retrieve a specific edge by index.

#### `frame_from_face(face_props: FaceProperties, name: str = "Face Frame", unit_scale: float = 1.0) -> Frame`

Create a coordinate frame from a face.

**Parameters:**
- `face_props` (FaceProperties): Face properties
- `name` (str): Frame name
- `unit_scale` (float): Scale factor to convert model units to meters (default: 1.0)

**Returns:**
- `Frame`: New frame with:
  - Origin = face center (scaled to meters)
  - Z-axis = face normal
  - X-axis = orthogonal to normal (computed)
  - Y-axis = Z × X (right-hand rule)

**Algorithm:**
1. Normalize face normal → Z-axis
2. **Strategy:** Align X-axis with Global X where possible.
3. Determine Target Vector (Global X). If Normal is parallel to Global X, switch Target to Global Y.
4. Project Target onto plane perpendicular to Normal: `X = Target - (Target · Z) * Z` (normalized).
5. Compute Y-axis: `Y = Z × X` (Right-hand rule).
6. Build rotation matrix: `[X | Y | Z]` (column vectors).
7. **Scale origin to meters**: `origin = face_center * unit_scale`.

**Usage:**
```python
face_props = GeometryUtils.extract_faces(body.shape)[0]
frame = GeometryUtils.frame_from_face(face_props, "FaceFrame_0", unit_scale=0.001)  # mm to m
```

#### `frame_from_edge(edge_props: EdgeProperties, name: str = "Edge Frame", unit_scale: float = 1.0) -> Frame`

Create a coordinate frame from an edge.

**Parameters:**
- `edge_props` (EdgeProperties): Edge properties
- `name` (str): Frame name
- `unit_scale` (float): Scale factor to convert model units to meters (default: 1.0)

**Returns:**
- `Frame`: New frame with:
  - Origin = edge midpoint (scaled to meters)
  - Z-axis = edge direction
  - X-axis = orthogonal to direction (computed)
  - Y-axis = Z × X (right-hand rule)

**Algorithm:**
1. Normalize edge direction → Z-axis
2. **Strategy:** Align X-axis with Global X where possible.
3. Determine Target Vector (Global X). If Direction is parallel to Global X, switch Target to Global Y.
4. Project Target onto plane perpendicular to Direction: `X = Target - (Target · Z) * Z` (normalized).
5. Compute Y-axis: `Y = Z × X` (Right-hand rule).
6. Build rotation matrix: `[X | Y | Z]` (column vectors).
7. **Scale origin to meters**: `origin = edge_midpoint * unit_scale`.

**Usage:**
```python
edge_props = GeometryUtils.extract_edges(body.shape)[0]
frame = GeometryUtils.frame_from_edge(edge_props, "EdgeFrame_0", unit_scale=0.001)  # mm to m
```

---


## GUI Modules

### 1. body_tree_widget.py

**Purpose:** Display hierarchical list of bodies and frames in a tree widget with multi-selection support.

**Class:** `BodyTreeWidget` (inherits `QWidget`)

**Selection Mode:** ExtendedSelection (supports Ctrl+Click for toggle selection and Shift+Click for range selection)

**Signals:**

```python
body_selected = Signal(int)             # Emitted when user selects a body (body_id)
frame_selected = Signal(str)            # Emitted when user selects a frame (frame_name)
joint_selected = Signal(str)            # Emitted when user selects a joint (joint_name)
delete_frame_requested = Signal(str)    # Emitted when user requests frame deletion
delete_body_requested = Signal(int)     # Emitted when user requests single body deletion
delete_bodies_requested = Signal(list)  # Emitted when user requests multi-body deletion (list of body_ids)
joint_selected = Signal(str)            # Emitted when user selects a joint (joint_name)
delete_joint_requested = Signal(str)    # Emitted when user requests joint deletion
isolate_body_requested = Signal(int)    # Emitted when body isolation is requested
exit_isolation_requested = Signal()     # Emitted when exiting isolation mode
```

**Methods:**

#### `__init__(self, parent=None)`

Create tree widget with body count label and tree view.

#### `update_bodies(self, bodies: List[RigidBody]) -> None`

Populate tree with body list.

**Parameters:**
- `bodies` (List[RigidBody]): Bodies to display

**Side Effects:**
- Clears existing body items
- Updates count label: "Bodies (N)"
- Adds tree items under "Bodies" group

#### `update_frames(self, frames: Iterable[Frame]) -> None`

Populate frame group with frame list.

#### `update_joints_list(self, joints: Iterable[Joint]) -> None`

Populate joint group with joint list.

**Parameters:**
- `joints` (Iterable[Joint]): Joints to display

**Parameters:**
- `frames` (Iterable[Frame]): Frames to display

#### `clear(self) -> None`

Clear all tree items (bodies and frames) and reset count.

#### `select_body(self, body_id: int) -> None`

Programmatically select a body in the tree.

**Parameters:**
- `body_id` (int): ID of body to select

#### `select_frame(self, frame_name: str) -> None`

Programmatically select a frame in the tree.

**Parameters:**
- `frame_name` (str): Name of frame to select

#### `_on_item_clicked(self, item: QTreeWidgetItem, column: int) -> None`

Internal handler for item clicks. Emits `body_selected` or `frame_selected` signal.

#### `_show_context_menu(self, position) -> None`

Show context menu (Delete Frame) for frame items.

---


### 2. property_panel.py

**Purpose:** Display properties of selected bodies, faces, edges, and frames.

**Class:** `PropertyPanel` (inherits `QWidget`)

**Signals:**

```python
com_visibility_changed = Signal(bool)            # COM marker visibility toggle
world_frame_visibility_changed = Signal(bool)    # World frame visibility
local_frame_visibility_changed = Signal(bool)    # Local frame visibility
selection_mode_changed = Signal(str)             # Mode: "Body", "Face", "Edge", "Vertex"
create_frame_from_face = Signal()                # Request frame creation
create_frame_from_edge = Signal()                # Request frame creation
create_frame_from_vertex = Signal()              # Request frame creation from vertex
frame_position_changed = Signal(str, tuple)      # Manual position update
frame_rotation_changed = Signal(str, tuple)      # Manual rotation update
body_visibility_changed = Signal(int, bool)      # Body visibility toggle (body_id, visible)
contact_detection_changed = Signal(int, bool)    # Contact detection toggle (body_id, contact_enabled)
```

**UI Components:**

- **Selection Mode Dropdown**: Choose Body/Face/Edge/Vertex selection
- **Body Information Group**: Name, ID, volume, COM, inertia tensor, visibility checkbox, contact detection checkbox
- **Face Information Group**: Face index, area, center, normal, frame creation button
- **Edge Information Group**: Edge index, length, midpoint, direction, frame creation button
- **Vertex Information Group**: Vertex index, X/Y/Z coordinates, frame creation button
- **Frame Information Group**: Name, Position XYZ (spinboxes), Rotation XYZ (spinboxes), axis vectors
- **Visualization Options Group**: Checkboxes for COM, world frame, local frame visibility

**Methods:**

#### `get_selection_mode(self) -> str`

Returns current selection mode ("Body", "Face", "Edge", or "Vertex").

#### `set_selection_mode(self, mode: str) -> None`

Set selection mode programmatically.

#### `show_body_properties(self, body: RigidBody) -> None`

Display body properties in the panel.

**Formatting:**
- Volume: Scientific notation if < 0.001 m³ or > 1000 m³
- COM: 6 decimal places in meters
- Inertia: 4 decimal scientific notation in kg·m²

#### `show_face_properties(self, face_properties: FaceProperties) -> None`

Display face properties in the panel.

#### `show_edge_properties(self, edge_properties: EdgeProperties) -> None`

Display edge properties in the panel.

#### `show_vertex_properties(self, vertex_properties: VertexProperties) -> None`

Display vertex properties in the panel (index and X/Y/Z coordinates).

#### `show_frame_properties(self, frame: Frame) -> None`

Display frame properties (origin and rotation matrix). Update spinbox values.

#### `show_joint_properties(self, joint: Joint) -> None`

Display joint properties (name, type, connected bodies, frames).

#### `show_no_selection(self) -> None`

Clear all properties and show "No Selection" message.

#### `clear(self) -> None`

Clear all displayed properties (calls `show_no_selection()`)

---


### 3. viewer_3d.py

**Purpose:** Enhanced 3D viewer with interactive body, face, and edge selection.

**Class:** `SelectableViewer3d` (inherits `qtViewer3d` from PythonOCC)

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `body_ais_map` | `Dict[int, AIS_Shape]` | Map body_id → AIS_Shape |
| `ais_body_map` | `Dict[AIS_Shape, int]` | Reverse map AIS_Shape → body_id |
| `bodies_dict` | `Dict[int, RigidBody]` | Map body_id → RigidBody |
| `selection_mode` | `str` | Current mode: "Body", "Face", "Edge", "Vertex" |
| `on_body_clicked` | `Callable[[int], None]` | Callback for body selection |
| `on_face_clicked` | `Callable[[int, int], None]` | Callback for face selection (body_id, face_index) |
| `on_edge_clicked` | `Callable[[int, int], None]` | Callback for edge selection (body_id, edge_index) |
| `on_vertex_clicked` | `Callable[[int, int], None]` | Callback for vertex selection (body_id, vertex_index) |

**Methods:**

#### `__init__(self, parent=None)`

Initialize viewer with high-DPI selection tolerance.

**Configuration:**
- Sets pixel tolerance to 12 for easier selection on high-DPI displays

#### `set_selection_mode(self, mode: str) -> None`

Set selection mode and activate corresponding OCC selection mode.

**Parameters:**
- `mode` (str): "Body", "Face", "Edge", or "Vertex"

**OCC Activation Modes:**
- Body → Mode 0 (whole shape)
- Face → Mode 4 (faces)
- Edge → Mode 2 (edges)
- Vertex → Mode 1 (vertices)

#### `orientation_triad()`

Displays a stationary orientation triad (XYZ reference axes) in the bottom-left corner of the viewer. The triad helps users understand the global orientation of the model.

- **Scale**: Increased to 0.30 (3x default size) for optimal visibility.
- **Axes**: X (Red), Y (Green), Z (Blue).
- **Label Color**: Black/White (depending on background).

#### `set_body_mapping(self, body_ais_map: Dict[int, AIS_Shape], bodies_dict: Dict[int, RigidBody] = None) -> None`

Set body-to-AIS mapping for selection.

**Parameters:**
- `body_ais_map`: Map of body IDs to AIS shapes
- `bodies_dict`: Map of body IDs to RigidBody objects (for face/edge selection)

#### `mouseReleaseEvent(self, event) -> None`

Override Qt mouse event handler for left-click selection.

**Algorithm:**
1. Check if left-click with no modifiers
2. Call `_select_at_position()` for custom selection
3. For other events, delegate to parent implementation

#### `_select_at_position(self, x: int, y: int) -> None`

Perform selection at screen coordinates using OCC selection API.

**Parameters:**
- `x` (int): Screen X coordinate (pixels)
- `y` (int): Screen Y coordinate (pixels)

**Algorithm:**
1. Account for high-DPI: `px = x * devicePixelRatio`, `py = y * devicePixelRatio`
2. Call `Context.MoveTo(px, py)` to detect objects under cursor
3. If detected, `Context.Select()` to confirm selection
4. Otherwise, `Context.Select(px, py, px, py)` for point selection
5. Check number of selected objects with `Context.NbSelected()`
6. Extract selected AIS shape
7. Lookup body ID from `ais_body_map`
8. For Body mode: invoke `on_body_clicked(body_id)`
9. For Face mode: extract face index and invoke `on_face_clicked(body_id, face_index)`
10. For Edge mode: extract edge index and invoke `on_edge_clicked(body_id, edge_index)`
11. For Vertex mode: extract vertex index and invoke `on_vertex_clicked(body_id, vertex_index)`

#### `_extract_sub_shape_index(self, owner, sub_shape_type) -> int`

Extract face or edge index from OCC selection owner with **robust matching logic**.

**Parameters:**
- `owner`: OCC selection owner object
- `sub_shape_type`: `TopAbs_FACE` or `TopAbs_EDGE`

**Returns:**
- `int`: Index of selected sub-shape, or -1 if not found

**Matching Algorithm:**
Iterate through all sub-shapes (faces or edges) of the parent body and check for equality using three levels of comparison:
1. **`IsEqual()`**: Checks TShape and Orientation (Most strict).
2. **`TShape()` pointer**: Checks underlying geometry reference.
3. **`hash()`**: Python's built-in hash function for shape comparison (Least strict).

**Challenges:**
- OpenCASCADE shape comparisons can vary based on context (selection vs. explorer).
- The three-level check ensures robust identification across different scenarios.
- Note: Older `HashCode()` method was removed in PythonOCC 7.7+; use Python's `hash()` instead.

#### `clear_mappings(self) -> None`

Clear all body-to-AIS mappings.

---


### 4. joint_dialog.py

**Purpose:** Modal dialog for creating new joints between bodies.

**Class:** `JointCreationDialog` (inherits `QDialog`)

**Input Fields:**
- **Name**: String input for joint name
- **Type**: Dropdown (Fixed, Revolute, Prismatic, Cylindrical, Spherical)
- **Body 1**: Dropdown of available bodies (including Ground)
- **Body 2**: Dropdown of available bodies (including Ground)
- **Frame 1**: Dropdown of available frames on Body 1
- **Frame 2**: Dropdown of available frames on Body 2

**Methods:**

#### `get_data(self) -> Dict`
Returns a dictionary containing the user's choices if the dialog is accepted.

---


## Visualization Modules

### 1. body_renderer.py

**Purpose:** Render and highlight bodies in the 3D viewer with COM markers.

**Class:** `BodyRenderer`

**Color Constants:**

```python
NORMAL_COLOR = Quantity_Color(0.8, 0.8, 0.8, Quantity_TOC_RGB)      # Light gray
HIGHLIGHT_COLOR = Quantity_Color(1.0, 0.9, 0.0, Quantity_TOC_RGB)   # Yellow
COM_COLOR = Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB)         # Red
```

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `display` | `Viewer3d` | OCC display object |
| `body_ais_shapes` | `Dict[int, AIS_Shape]` | Map body_id → AIS_Shape |
| `currently_highlighted_id` | `Optional[int]` | ID of currently highlighted body |
| `com_marker_ais` | `Optional[AIS_Shape]` | AIS shape for COM sphere marker |
| `com_marker_visible` | `bool` | COM visibility state |
| `bodies_dict` | `Dict[int, RigidBody]` | Map body_id → RigidBody |

**Methods:**

#### `display_bodies(self, bodies: List[RigidBody]) -> None`

Display all bodies in the viewer.

**Parameters:**
- `bodies` (List[RigidBody]): Bodies to display

**Algorithm:**
1. Clear previous AIS shapes and mappings
2. For each body:
   - Create `AIS_Shape(body.shape)`
   - Display with `Context.Display()`
   - **Activate selection mode 0** (critical for click selection)
   - Set default light gray color
   - Store in `body_ais_shapes` map
3. Update viewer

**Notes:**
- Selection mode activation is **critical** for mouse picking

#### `highlight_body(self, body_id: int) -> None`

Highlight a body by changing its color to yellow.

**Parameters:**
- `body_id` (int): ID of body to highlight

**Side Effects:**
- Unhighlights previously highlighted body
- Sets body color to HIGHLIGHT_COLOR
- Updates COM marker position

#### `clear_highlight(self) -> None`

Remove highlighting from all bodies.

#### `clear_all(self) -> None`

Clear all rendered bodies from display.

#### `set_com_visibility(self, visible: bool) -> None`

Toggle COM marker visibility.

**Parameters:**
- `visible` (bool): True to show, False to hide

#### `_update_com_marker(self, body_id: int) -> None`

Update COM marker position for specified body.

**Algorithm:**
1. Clear existing COM marker
2. Get body's center of mass
3. Create sphere at COM using `BRepPrimAPI_MakeSphere()` with 5mm radius
4. Create `AIS_Shape` for sphere
5. Display with red color

#### `_clear_com_marker(self) -> None`

Remove COM marker from display.

---


### 2. face_renderer.py

**Purpose:** Render and highlight faces in the 3D viewer.

**Class:** `FaceRenderer`

**Color Constants:**
```python
HIGHLIGHT_COLOR = Quantity_Color(1.0, 0.9, 0.0, Quantity_TOC_RGB)  # Yellow
```

**Methods:**

#### `highlight_face(self, face: TopoDS_Face) -> None`
Creates a new AIS_Shape for the specific face and displays it with a yellow color overlay.

#### `clear_highlight(self) -> None`
Removes the face highlight overlay.

---


### 3. edge_renderer.py

**Purpose:** Render and highlight edges in the 3D viewer.

**Class:** `EdgeRenderer`

**Color Constants:**
```python
HIGHLIGHT_COLOR = Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_RGB)  # Red
```

**Methods:**

#### `highlight_edge(self, edge: TopoDS_Edge) -> None`
Creates a new AIS_Shape for the specific edge and displays it with a red color and increased linewidth (3.0).

#### `clear_highlight(self) -> None`
Removes the edge highlight overlay.

---


### 4. frame_renderer.py

**Purpose:** Render coordinate frames as RGB axes (X=red, Y=green, Z=blue).

**Class:** `FrameRenderer`

**Default Parameters:**

```python
axis_length = 0.1        # Default 10% of model size
axis_radius = 0.002      # Cylinder radius
arrow_length = 0.015     # Cone length
arrow_radius = 0.005     # Cone base radius
```

**Attributes:**

| Attribute | Type | Description |
|-----------|------|-------------|
| `display` | `Viewer3d` | OCC display object |
| `frame_shapes` | `Dict[str, List[AIS_Shape]]` | Map frame_name → list of AIS shapes (cylinders + cones) |
| `frame_visible` | `Dict[str, bool]` | Map frame_name → visibility state |

**Methods:**

#### `set_axis_scale(self, scale: float) -> None`

Set axis length scale based on model size.

**Parameters:**
- `scale` (float): Axis length in model units

**Recommended Usage:**
```python
max_dimension = calculate_bounding_box_size()
frame_renderer.set_axis_scale(max_dimension * 0.2)  # 20% of model size
```

#### `render_frame(self, frame: Frame, visible: bool = True) -> None`

Render a frame as three colored axes.

**Parameters:**
- `frame` (Frame): Frame to render
- `visible` (bool): Initial visibility

**Algorithm:**
1. Remove existing frame if already rendered
2. Create X-axis (red) cylinder + cone
3. Create Y-axis (green) cylinder + cone
4. Create Z-axis (blue) cylinder + cone
5. Store shapes in `frame_shapes[frame.name]`
6. Display if `visible == True`

**Axis Construction:**
- Cylinder: From origin to `origin + direction * axis_length`
- Cone: At end of cylinder pointing in direction

#### `_create_axis(self, origin: np.ndarray, direction: np.ndarray, color: Quantity_Color) -> List[AIS_Shape]`

Create cylinder and cone for a single axis.

**Returns:**
- `List[AIS_Shape]`: [cylinder_ais, cone_ais]

**Implementation:**
```python
# Cylinder
cylinder_origin = gp_Pnt(origin[0], origin[1], origin[2])
cylinder_dir = gp_Dir(direction[0], direction[1], direction[2])
cylinder_ax = gp_Ax2(cylinder_origin, cylinder_dir)
cylinder = BRepPrimAPI_MakeCylinder(cylinder_ax, radius, length).Shape()

# Cone (arrow head)
cone_origin = origin + direction * axis_length
cone = BRepPrimAPI_MakeCone(cone_ax, arrow_radius, 0.0, arrow_length).Shape()
```

#### `set_frame_visibility(self, frame_name: str, visible: bool) -> None`

Toggle frame visibility.

**Parameters:**
- `frame_name` (str): Name of frame
- `visible` (bool): True to show, False to hide

#### `remove_frame(self, frame_name: str) -> None`

Remove frame from display.

#### `clear_all_frames(self) -> None`

Remove all frames from display.

### 6. JointRenderer Class

**File:** `visualization/joint_renderer.py`

**Purpose:** Renders kinematic joints with a coordinate frame showing the joint location and orientation.

**Methods:**

#### `render_joint(self, joint: Joint, visible: bool = True) -> None`

Render a joint visualization with a single coordinate frame.
- Renders the joint's `frame` using `FrameRenderer` (with unique temporary name).
- The frame shows the location and orientation of the joint in global coordinates.

**Parameters:**
- `joint` (Joint): Joint object to visualize
- `visible` (bool): Visibility state

#### `remove_joint(self, joint_name: str) -> None`

Remove visualization for a specific joint.

#### `clear(self) -> None`

Remove all rendered joints.

---

### 7. ForceRenderer Class

**File:** `visualization/force_renderer.py`

**Purpose:** Render and highlight applied forces as arrows.

**Methods:**

#### `set_force_scale(self, scale: float) -> None`

Set the visual length scale for force arrows.

**Parameters:**
- `scale` (float): Arrow length in model units

#### `set_unit_scale(self, scale: float) -> None`

Set the unit conversion scale (Meters per Model Unit) to ensure forces defined in SI meters appear correctly in the model's coordinate space.

**Parameters:**
- `scale` (float): Unit scale factor (e.g., 0.001 for mm)

#### `render_force(self, force: Force, visible: bool = True) -> None`

Render a force arrow.
- Origin is scaled by `1/unit_scale` to convert from Meters back to Model Units.
- Arrow length is calculated logarithmically based on magnitude and `force_scale`.

---


### 8. TorqueRenderer Class

**File:** `visualization/torque_renderer.py`

**Purpose:** Render and highlight applied torques as circular arrows.

**Methods:**

#### `set_torque_scale(self, scale: float) -> None`

Set the visual radius scale for torque markers.

**Parameters:**
- `scale` (float): Base radius in model units

#### `set_unit_scale(self, scale: float) -> None`

Set the unit conversion scale (Meters per Model Unit).

#### `render_torque(self, torque: Torque, visible: bool = True) -> None`

Render a torque circular arrow.
- Origin is scaled by `1/unit_scale` to convert from Meters back to Model Units.
- Arc radius is calculated logarithmically based on magnitude and `torque_scale`.

---


### 9. VertexRenderer Class

**File:** `visualization/vertex_renderer.py`

**Purpose:** Highlight selected vertices.

**Methods:**

#### `set_unit_scale(self, scale: float) -> None`

Set unit scale to adjust the vertex highlight sphere size.
- Calculates `highlight_size` to be approximately 5mm in the model's units.
- Formula: `highlight_size = 0.005 / scale`

#### `highlight_vertex(self, vertex: TopoDS_Vertex) -> None`

Render a sphere at the vertex position using the dynamically calculated `highlight_size`.

---


## Export Modules

### 1. exporter.py

**Purpose:** Export assembly data to JSON format and body geometries to OBJ mesh files with all coordinates in global world frame.

**Class:** `AssemblyExporter`

**Static Methods:**

#### `export_assembly_to_json(bodies, joints, frames, ground_body, unit_scale, output_path) -> bool`

Export complete assembly to JSON format with all coordinates in global world frame.

**Parameters:**
- `bodies` (List[RigidBody]): List of rigid bodies
- `joints` (Dict[str, Joint]): Dictionary of joints (name → Joint)
- `frames` (Dict[str, Frame]): Dictionary of user-created frames (name → Frame)
- `ground_body` (RigidBody): The ground body reference
- `unit_scale` (float): Unit scale factor from STEP file
- `output_path` (str): Path to save JSON file

**Returns:**
- `bool`: True if export successful, False otherwise

**JSON Structure:**
```json
{
  "metadata": {
    "version": "1.0",
    "description": "Multi-Body Dynamics Assembly Export",
    "coordinate_system": "global_world_frame",
    "unit_scale": 0.001,
    "units": "meters"
  },
  "ground_body": {
    "id": -1,
    "name": "Ground",
    "center_of_mass": [0.0, 0.0, 0.0],
    "local_frame": { ... }
  },
  "bodies": [
    {
      "id": 0,
      "name": "Body_0",
      "volume": 1.23e-06,
      "center_of_mass": [0.1, 0.2, 0.3],
      "inertia_tensor": [[...], [...], [...]],
      "contact_enabled": true,
      "local_frame": {
        "name": "Body_0_LocalFrame",
        "origin": [0.1, 0.2, 0.3],
        "rotation_matrix": [[...], [...], [...]],
        "euler_angles_deg": [0.0, 0.0, 0.0],
        "x_axis": [1.0, 0.0, 0.0],
        "y_axis": [0.0, 1.0, 0.0],
        "z_axis": [0.0, 0.0, 1.0]
      }
    }
  ],
  "joints": [
    {
      "name": "Joint_1",
      "type": "REVOLUTE",
      "axis": "+Z",
      "body1_id": -1,
      "body1_name": "Ground",
      "body2_id": 0,
      "body2_name": "Body_0",
      "motorized": false,
      "frame_world": {
        "name": "Joint_1_Frame",
        "origin": [0.1, 0.2, 0.3],
        "rotation_matrix": [[...], [...], [...]],
        "euler_angles_deg": [0.0, 0.0, 0.0],
        "x_axis": [1.0, 0.0, 0.0],
        "y_axis": [0.0, 1.0, 0.0],
        "z_axis": [0.0, 0.0, 1.0]
      }
    }
  ],
  "frames": {
    "Frame_B0_F1": {
      "name": "Frame_B0_F1",
      "origin": [0.5, 0.6, 0.7],
      "rotation_matrix": [[...], [...], [...]],
      ...
    }
  }
}
```

**Algorithm:**
1. Build metadata with unit information
2. Serialize ground body with `_serialize_body()`
3. Serialize all bodies with physical properties
4. Serialize all joints with single frame in world coordinates using `_serialize_joint()`
5. Serialize all user-created frames
6. Write JSON with pretty formatting (indent=2)

**Key Feature:**
- Joint frames are **already stored in world coordinates** (since this is a preprocessor where bodies don't move), so no transformation is needed during export.

#### `_serialize_body(body: RigidBody) -> Dict`

Serialize a rigid body to dictionary with all coordinates in world frame.

**Returns:**
- Dictionary with: id, name, volume, center_of_mass (world), inertia_tensor (world orientation), local_frame (world), contact_enabled

#### `_serialize_frame(frame: Frame) -> Dict`

Serialize a frame to dictionary in world frame coordinates.

**Returns:**
- Dictionary with: name, origin, rotation_matrix, euler_angles_deg, x_axis, y_axis, z_axis

#### `_serialize_joint(joint: Joint, bodies, ground_body) -> Dict`

Serialize a joint with single frame in world coordinates.

**Algorithm:**
1. Find body1 and body2 from IDs
2. Serialize basic joint properties (name, type, axis, body IDs)
3. Serialize the joint's frame directly (already in world coordinates)
4. Store frame under key `frame_world`
5. Add motor information if joint is motorized

**Returns:**
- Dictionary with joint properties and frame in world coordinates

**Note:** The `_transform_frame_to_world()` method is no longer used since joints now use a single frame that is already in global coordinates.

#### `export_body_meshes_to_obj(bodies, output_dir, unit_scale) -> bool`

Export all body geometries as OBJ mesh files with vertices in **body-local frame** (COM-centered).

**Parameters:**
- `bodies` (List[RigidBody]): Bodies to export
- `output_dir` (str): Directory to save OBJ files
- `unit_scale` (float): Unit scale factor to convert model units to meters

**Returns:**
- `bool`: True if export successful, False otherwise

**Algorithm:**
1. Create output directory if it doesn't exist
2. For each body:
   - Generate safe filename: `{body_name}_{body_id}.obj`
   - Call `_export_shape_to_obj()` to tessellate and write mesh in body-local coordinates
3. Print summary of exported files

**Output:**
- Individual `.obj` files for each body (e.g., `Body_0_0.obj`, `Body_1_1.obj`)
- Vertices are in body-local coordinates (centered at body's COM)

#### `_export_shape_to_obj(shape: TopoDS_Shape, filepath: str, name: str, scale_factor: float, body: RigidBody) -> bool`

Export a TopoDS_Shape to OBJ format with vertices in **body-local frame** (COM-centered).

**Parameters:**
- `shape` (TopoDS_Shape): The shape to export
- `filepath` (str): Path to save OBJ file
- `name` (str): Name for the object in OBJ file
- `scale_factor` (float): Unit scale factor to convert to meters
- `body` (RigidBody): Body object (used to transform to local frame)

**Returns:**
- `bool`: True if successful, False otherwise

**Algorithm:**
1. Tessellate shape using `BRepMesh_IncrementalMesh(shape, deflection=0.1)`
2. For each face in shape:
   - Get triangulation with `BRep_Tool.Triangulation()`
   - Extract nodes (vertices) and transform to world coordinates
   - **Transform to body-local frame:**
     - Translate: `translated = world_coords - body.center_of_mass`
     - Rotate: `local_coords = body.local_frame.rotation_matrix.T @ translated`
   - Build vertex map to avoid duplicates (using 6 decimal precision)
   - Extract triangles and map to global vertex indices
3. Write OBJ file:
   - Header with metadata (indicating "Body local frame (COM-centered)")
   - Object name: `o {name}`
   - Vertices: `v x y z` (body-local coordinates)
   - Faces: `f v1 v2 v3` (1-based indexing)

**OBJ Format Example:**
```obj
# OBJ file exported from Multi-Body Dynamics Preprocessor
# Object: Body_0
# Vertices: 245
# Faces: 486
# Coordinate system: Global world frame

o Body_0

v 0.100000 0.200000 0.300000
v 0.105000 0.205000 0.305000
...

f 1 2 3
f 2 4 3
...
```

**Usage Example:**
```python
from export.exporter import AssemblyExporter

# Export assembly to JSON
success = AssemblyExporter.export_assembly_to_json(
    bodies=bodies,
    joints=joints,
    frames=created_frames,
    ground_body=ground_body,
    unit_scale=unit_scale,
    output_path="assembly.json"
)

# Export body meshes to OBJ
success = AssemblyExporter.export_body_meshes_to_obj(
    bodies=bodies,
    output_dir="mesh_output",
    unit_scale=unit_scale
)
```

---


## Application Workflow

### 1. Application Startup

**Sequence:**

1. **main.py**: `main()` function creates `QApplication`
2. **MainWindow.__init__()** executes:
   - Create menu bar (File → Open, Exit)
   - Create three-panel layout:
     - Left: `BodyTreeWidget` (20%)
     - Center: `SelectableViewer3d` (65%)
     - Right: `PropertyPanel` (20%)
   - Initialize renderers:
     - `BodyRenderer`
     - `FaceRenderer`
     - `EdgeRenderer`
     - `FrameRenderer`
     - `JointRenderer`
   - Create and render world frame at origin
   - Initialize fixed **Ground Body** (ID -1) linked to world frame
   - Connect signals and slots:
     - `body_tree.body_selected` → `on_body_selected()`
     - `viewer_3d.on_body_clicked` → `on_body_clicked_in_viewer()`
     - `property_panel.com_visibility_changed` → `on_com_visibility_changed()`
     - `property_panel.selection_mode_changed` → `on_selection_mode_changed()`
     - And more...
3. **Show window** and start Qt event loop

---


### 2. Loading a STEP File

**User Action:** File → Open STEP File...

**Workflow:**

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. User clicks "File > Open"                                    │
│    → open_step_file() shows QFileDialog                         │
│    → User selects .step/.stp file                               │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. load_step_file(filepath) begins                              │
│    → Clear previous display and data                            │
│    → body_renderer.clear_all()
│    → face_renderer.clear_highlight()
│    → edge_renderer.clear_highlight()
│    → body_tree.clear()
│    → property_panel.clear()
│    → frame_renderer.clear_all_frames()
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. StepParser.load_step_file(filepath)                          │
│    → Read STEP file                                             │
│    → Extract unit declarations (mm, m, in, etc.)                │
│    → Return (shape, unit_scale)                                 │
│    → Print unit info to console                                 │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. StepParser.extract_bodies_from_compound(shape)               │
│    → Traverse TopAbs_SOLID shapes                               │
│    → Create RigidBody for each solid                            │
│    → Assign sequential IDs (0, 1, 2, ...)                       │
│    → Return List[RigidBody]                                     │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. PhysicsCalculator.calculate_volumes_for_bodies()             │
│    → For each body:                                             │
│      → brepgprop.VolumeProperties()                             │
│      → volume_m3 = volume * unit_scale³                         │
│      → body.volume = volume_m3                                  │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. PhysicsCalculator.calculate_centers_of_mass_for_bodies()     │
│    → For each body:                                             │
│      → Get COM from GProp                                       │
│      → Scale to meters                                          │
│      → body.center_of_mass = [x, y, z]                          │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 7. PhysicsCalculator.calculate_inertia_tensors_for_bodies()     │
│    → For each body:                                             │
│      → Get inertia at origin                                    │
│      → Apply parallel axis theorem → inertia at COM             │
│      → Scale to kg·m²                                           │
│      → body.inertia_tensor = 3×3 matrix                         │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 8. PhysicsCalculator.initialize_local_frames()                  │
│    → For each body:                                             │
│      → Create Frame at COM with identity rotation               │
│      → body.local_frame = Frame(...)                            │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 9. GeometryUtils.extract_faces() and extract_edges()            │
│    → For each body:                                             │
│      → Extract all faces with properties                        │
│      → Extract all edges with properties                        │
│      → Store in face_properties_map and edge_properties_map     │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 10. Update GUI                                                  │
│     → body_tree.update_bodies(bodies)                           │
│     → body_renderer.display_bodies(bodies)                      │
│     → viewer_3d.set_body_mapping()                              │
│     → display.FitAll()                                          │
│     → Calculate bounding box                                    │
│     → Scale world frame axes to 20% of model size               │
│     → Re-render world frame                                     │
└─────────────────────────────────────────────────────────────────┘
```

**Result:**
- Bodies displayed in 3D viewer (light gray)
- Body tree populated with names
- Property panel shows "No Selection"
- World frame rendered at origin
- All physical properties calculated and stored

---


### 3. Selecting a Body

**User Action:** Click body in tree or 3D viewer

**Workflow:**

```
┌─────────────────────────────────────────────────────────────────┐
│ Option A: User clicks body in tree widget                       │
│    → body_tree._on_item_clicked()
│    → body_selected signal emitted with body_id                  │
│    → on_body_selected(body_id)                                  │
└─────────────────────────────────────────────────────────────────┘
                              OR
┌─────────────────────────────────────────────────────────────────┐
│ Option B: User clicks body in 3D viewer                         │
│    → viewer_3d.mouseReleaseEvent()
│    → _select_at_position()
│    → on_body_clicked callback invoked                           │
│    → on_body_clicked_in_viewer(body_id)                         │
│    → body_tree.select_body(body_id)  [syncs tree selection]    │
│    → on_body_selected(body_id)  [triggered by tree signal]     │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ on_body_selected(body_id):                                      │
│  1. Hide previous body's local frame (if any)                   │
│  2. body_renderer.highlight_body(body_id)                       │
│     → Unhighlight previous body (restore gray)                  │
│     → Highlight new body (yellow)                               │
│     → Update COM marker position                                │
│  3. property_panel.show_body_properties(body)                   │
│     → Display name, ID, volume, COM, inertia                    │
│  4. Render local frame at COM (if checkbox enabled)             │
│     → frame_renderer.render_frame(body.local_frame)             │
└─────────────────────────────────────────────────────────────────┘
```

**Result:**
- Selected body turns yellow in viewer
- Property panel shows body details
- Red COM marker sphere appears at center of mass (if enabled)
- Local frame axes rendered at COM (if enabled)
- Previous selection unhighlighted

---


### 4. Selecting a Face

**Prerequisite:** User changes selection mode to "Face" in property panel

**Workflow:**

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. User selects "Face" from dropdown                            │
│    → property_panel.selection_mode_changed.emit("Face")         │
│    → on_selection_mode_changed("Face")                          │
│    → viewer_3d.set_selection_mode("Face")                       │
│      → Activate OCC selection mode 2 (faces) for all bodies     │
│    → Update property panel visibility (hide body group, show    │
│      face group)                                                │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. User clicks a face in 3D viewer                              │
│    → viewer_3d.mouseReleaseEvent()
│    → _select_at_position()
│    → Detect face selection in OCC context                       │
│    → _extract_sub_shape_index(owner, TopAbs_FACE)               │
│      → Match selected face to index in parent body (IsEqual)    │
│    → on_face_clicked(body_id, face_index)                       │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. on_face_clicked_in_viewer(body_id, face_index)               │
│    → Lookup face_props from face_properties_map                 │
│    → property_panel.show_face_properties(face_props)            │
│      → Display face index, area, center, normal                 │
│      → Enable "Create Frame from Face" button                   │
│    → face_renderer.highlight_face(face_props.face)              │
│      → Display yellow overlay on face                           │
│    → Store (body_id, face_index) in last_face_selection         │
└─────────────────────────────────────────────────────────────────┘
```

**Result:**
- Face properties displayed in property panel
- **Face highlighted with a yellow overlay**
- "Create Frame from Face" button enabled

---


### 5. Selecting an Edge

**Prerequisite:** User changes selection mode to "Edge" in property panel

**Workflow:**

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. User selects "Edge" from dropdown                            │
│    → property_panel.selection_mode_changed.emit("Edge")         │
│    → on_selection_mode_changed("Edge")                          │
│    → viewer_3d.set_selection_mode("Edge")                       │
│      → Activate OCC selection mode 2 (edges) for all bodies     │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. User clicks an edge in 3D viewer                             │
│    → viewer_3d.mouseReleaseEvent()
│    → _select_at_position()
│    → Detect edge selection in OCC context                       │
│    → _extract_sub_shape_index(owner, TopAbs_EDGE)               │
│      → Match selected edge to index in parent body (IsEqual)    │
│    → on_edge_clicked(body_id, edge_index)                       │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. on_edge_clicked_in_viewer(body_id, edge_index)               │
│    → Lookup edge_props from edge_properties_map                 │
│    → property_panel.show_edge_properties(edge_props)            │
│      → Display edge index, length, midpoint, direction          │
│      → Enable "Create Frame from Edge" button                   │
│    → edge_renderer.highlight_edge(edge_props.edge)              │
│      → Display thick red line overlay on edge                   │
│    → Store (body_id, edge_index) in last_edge_selection         │
└─────────────────────────────────────────────────────────────────┘
```

**Result:**
- Edge properties displayed in property panel
- **Edge highlighted with a thick red line**
- "Create Frame from Edge" button enabled

---


### 6. Creating a Frame from Face

**User Action:** Click "Create Frame from Face" button after selecting a face

**Workflow:**

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. User clicks "Create Frame from Face"                         │
│    → property_panel.create_frame_from_face.emit()               │
│    → on_create_frame_from_face()                                │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. Check last_face_selection exists                             │
│    → If None, show error message and return                     │
│    → Extract (body_id, face_index)                              │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. Get face_props from face_properties_map                      │
│    → face_props = face_properties_map[body_id][face_index]      │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. Generate unique frame name                                   │
│    → base_name = f"Frame_B{body_id}_F{face_index}"              │
│    → If name exists, append suffix: "_1", "_2", etc.            │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. GeometryUtils.frame_from_face(face_props, frame_name)        │
│    → Create Frame with:                                         │
│      → origin = face center                                     │
│      → Z-axis = face normal                                     │
│      → X-axis orthogonal to normal                              │
│      → Y-axis = Z × X (right-hand rule)                         │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. Store and render frame                                       │
│    → created_frames[frame_name] = frame                         │
│    → frame_renderer.render_frame(frame, visible=True)           │
│    → property_panel.show_frame_properties(frame)                │
│    → Print confirmation to console                              │
└─────────────────────────────────────────────────────────────────┘
```

**Result:**
- New frame rendered in 3D viewer at face center
- Frame axes aligned with face geometry:
  - Z-axis (blue) = face normal
  - X-axis (red) = tangent direction
  - Y-axis (green) = perpendicular to both
- Frame properties shown in property panel
- Frame stored in `created_frames` dictionary

---


### 7. Deleting a Body

**User Action:** Right-click body in tree and select "Delete Body"

**Workflow:**

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. User right-clicks body in tree widget                        │
│    → _show_context_menu() displays context menu                 │
│    → User selects "Delete Body"                                 │
│    → _request_delete_body() emits delete_body_requested signal  │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. on_body_deleted(body_id) receives signal                     │
│    → Find body in bodies list                                   │
│    → If not found, return with error message                    │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. Show confirmation dialog                                     │
│    → QMessageBox with Yes/No buttons                            │
│    → Message lists what will be deleted:                        │
│      - Body name                                                │
│      - Associated frames                                        │
│      - Connected joints                                         │
│    → If user clicks No, return without changes                  │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. Delete connected joints                                      │
│    → Find all joints where body1_id or body2_id = body_id       │
│    → For each joint:                                            │
│      → joint_renderer.remove_joint(joint_name)                  │
│      → Delete from joints dictionary                            │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. Delete associated frames                                     │
│    → Find frames with naming pattern "_B{body_id}_"             │
│    → For each frame:                                            │
│      → frame_renderer.remove_frame(frame_name)                  │
│      → Delete from created_frames dictionary                    │
│    → Remove body's local frame                                  │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. Remove from renderers and viewer                             │
│    → body_renderer.remove_body(body_id)                         │
│      → Erase from 3D display                                    │
│      → Clear from body_ais_shapes mapping                       │
│      → Clear highlight and COM marker if needed                 │
│    → viewer_3d.remove_body_from_mapping(body_id)                │
│      → Remove from selection mappings                           │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 7. Clean up data structures                                     │
│    → Remove from face_properties_map                            │
│    → Remove from edge_properties_map                            │
│    → Remove from bodies list                                    │
│    → Clear selection if deleted body was selected               │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 8. Update all GUI components                                    │
│    → body_tree.update_bodies(bodies)                            │
│    → body_tree.update_frames(created_frames.values())           │
│    → body_tree.update_joints_list(joints.values())              │
│    → property_panel.show_no_selection() if body was selected    │
│    → display.Repaint()                                          │
└─────────────────────────────────────────────────────────────────┘
```

**Result:**
- Body completely removed from 3D viewer
- All connected joints deleted and visualizations cleared
- Associated frames removed (based on naming convention)
- Body's local frame removed
- Face and edge properties cleaned up
- Tree widget updated with remaining bodies
- Property panel cleared if deleted body was selected
- Console prints deletion progress and completion

---


### 8. Multi-Deleting Bodies

**User Action:** Select multiple bodies (Ctrl+Click or Shift+Click) and press Delete key or right-click and select "Delete Bodies"

**Workflow:**

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. User selects multiple bodies in tree widget                  │
│    → Use Ctrl+Click to toggle individual bodies                 │
│    → Use Shift+Click for range selection                        │
│    → Right-click and select "Delete Bodies" or press Delete key │
│    → delete_bodies_requested signal emitted with list of IDs    │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. on_bodies_deleted(body_ids: List[int]) receives signal       │
│    → Validate all body IDs exist                                │
│    → Collect body names for display                             │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. Analyze dependencies                                         │
│    → Find all joints connected to any of the bodies             │
│    → Find all frames associated with any of the bodies          │
│    → Count total deletions: bodies + joints + frames            │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. Show confirmation dialog with full impact analysis           │
│    → Display list of bodies to be deleted                       │
│    → Display count of joints that will be removed               │
│    → Display count of frames that will be removed               │
│    → QMessageBox with Yes/No buttons                            │
│    → If user clicks No, return without changes                  │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. Delete all connected joints                                  │
│    → For each body_id:                                          │
│      → Find joints where body1_id or body2_id = body_id         │
│      → joint_renderer.remove_joint(joint_name)                  │
│      → Delete from joints dictionary                            │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. Delete all associated frames                                 │
│    → For each body_id:                                          │
│      → Find frames with pattern "_B{body_id}_"                  │
│      → frame_renderer.remove_frame(frame_name)                  │
│      → Delete from created_frames dictionary                    │
│      → Remove body's local frame                                │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 7. Remove all bodies from renderers and viewer                  │
│    → For each body_id:                                          │
│      → body_renderer.remove_body(body_id)                       │
│      → viewer_3d.remove_body_from_mapping(body_id)              │
│      → Remove from face_properties_map                          │
│      → Remove from edge_properties_map                          │
│      → Remove from bodies list                                  │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 8. Update all GUI components                                    │
│    → body_tree.update_bodies(bodies)                            │
│    → body_tree.update_frames(created_frames.values())           │
│    → body_tree.update_joints_list(joints.values())              │
│    → property_panel.show_no_selection() if any deleted          │
│    → display.Repaint()                                          │
│    → Print success message with count                           │
└─────────────────────────────────────────────────────────────────┘
```

**Result:**
- All selected bodies removed from 3D viewer
- All connected joints deleted with visualizations cleared
- All associated frames removed for all bodies
- All bodies' local frames removed
- Face and edge properties cleaned up for all bodies
- Tree widget updated with remaining bodies
- Property panel cleared
- Console prints deletion summary with counts

---


### 9. Creating a Frame from Edge

**User Action:** Click "Create Frame from Edge" button after selecting an edge

**Workflow:**

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. User clicks "Create Frame from Edge"                         │
│    → property_panel.create_frame_from_edge.emit()               │
│    → on_create_frame_from_edge()                                │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. Check last_edge_selection exists                             │
│    → If None, show error message and return                     │
│    → Extract (body_id, edge_index)                              │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. Get edge_props from edge_properties_map                      │
│    → edge_props = edge_properties_map[body_id][edge_index]      │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. Generate unique frame name                                   │
│    → base_name = f"Frame_B{body_id}_E{edge_index}"              │
│    → If name exists, append suffix: "_1", "_2", etc.            │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. GeometryUtils.frame_from_edge(edge_props, frame_name)        │
│    → Create Frame with:                                         │
│      → origin = edge midpoint                                   │
│      → Z-axis = edge direction                                  │
│      → X-axis orthogonal to direction                           │
│      → Y-axis = Z × X (right-hand rule)                         │
└────────────┬────────────────────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 6. Store and render frame                                       │
│    → created_frames[frame_name] = frame                         │
│    → frame_renderer.render_frame(frame, visible=True)           │
│    → property_panel.show_frame_properties(frame)                │
│    → Print confirmation to console                              │
└─────────────────────────────────────────────────────────────────┘
```

**Result:**
- New frame rendered in 3D viewer at edge midpoint
- Frame axes aligned with edge geometry:
  - Z-axis (blue) = edge direction
  - X-axis (red) = tangent direction
  - Y-axis (green) = perpendicular to both
- Frame properties shown in property panel
- Frame stored in `created_frames` dictionary

---


## API Reference

### Quick Reference Tables

#### Core Module Functions

| Function | Module | Description |
|----------|--------|-------------|
| `StepParser.load_step_file()` | step_parser | Load STEP file with unit extraction |
| `StepParser.extract_bodies_from_compound()` | step_parser | Extract solids from assembly |
| `PhysicsCalculator.calculate_volume()` | physics_calculator | Calculate body volume |
| `PhysicsCalculator.calculate_center_of_mass()` | physics_calculator | Calculate COM |
| `PhysicsCalculator.calculate_inertia_tensor()` | physics_calculator | Calculate inertia about COM |
| `GeometryUtils.extract_faces()` | geometry_utils | Get all faces with properties |
| `GeometryUtils.extract_edges()` | geometry_utils | Get all edges with properties |
| `GeometryUtils.frame_from_face()` | geometry_utils | Create frame from face |

#### GUI Signals

| Signal | Widget | Parameters | Description |
|--------|--------|------------|-------------|
| `body_selected` | BodyTreeWidget | `int` (body_id) | Emitted when body selected in tree |
| `com_visibility_changed` | PropertyPanel | `bool` (visible) | COM marker visibility toggled |
| `world_frame_visibility_changed` | PropertyPanel | `bool` (visible) | World frame visibility toggled |
| `local_frame_visibility_changed` | PropertyPanel | `bool` (visible) | Local frame visibility toggled |
| `selection_mode_changed` | PropertyPanel | `str` (mode) | Selection mode changed |
| `create_frame_from_face` | PropertyPanel | None | Request frame creation from face |

#### Visualization Methods

| Method | Class | Description |
|--------|-------|-------------|
| `display_bodies()` | BodyRenderer | Display all bodies in viewer |
| `highlight_body()` | BodyRenderer | Highlight body in yellow |
| `set_com_visibility()` | BodyRenderer | Toggle COM marker |
| `highlight_face()` | FaceRenderer | Highlight face with yellow overlay |
| `highlight_edge()` | EdgeRenderer | Highlight edge with thick red line |
| `render_frame()` | FrameRenderer | Render frame as RGB axes |
| `set_frame_visibility()` | FrameRenderer | Toggle frame visibility |
| `set_axis_scale()` | FrameRenderer | Scale axis length |

---


## Usage Examples

### Example 1: Load STEP File Programmatically

```python
from core.step_parser import StepParser
from core.physics_calculator import PhysicsCalculator

# Load STEP file
shape, unit_scale = StepParser.load_step_file("model.step")

# Extract bodies
bodies = StepParser.extract_bodies_from_compound(shape)

# Calculate physical properties
PhysicsCalculator.calculate_volumes_for_bodies(bodies, unit_scale)
PhysicsCalculator.calculate_centers_of_mass_for_bodies(bodies, unit_scale)
PhysicsCalculator.calculate_inertia_tensors_for_bodies(bodies, unit_scale)
PhysicsCalculator.initialize_local_frames(bodies)

# Access properties
for body in bodies:
    print(f"Body: {body.name}")
    print(f"  Volume: {body.volume:.6e} m³")
    print(f"  COM: {body.center_of_mass}")
    print(f"  Inertia:\n{body.inertia_tensor}")
```

### Example 2: Create Custom Frame from Face

```python
from core.geometry_utils import GeometryUtils

# Extract faces from a body
faces = GeometryUtils.extract_faces(body.shape)

# Get first face
face_props = faces[0]

# Create frame aligned with face
frame = GeometryUtils.frame_from_face(face_props, "MyFrame")

# Access frame properties
print(f"Frame origin: {frame.origin}")
print(f"Frame Z-axis: {frame.get_z_axis()}")
```

### Example 3: Manual Body Rendering

```python
from visualization.body_renderer import BodyRenderer
from OCC.Display.qtDisplay import qtViewer3d

# Create viewer and renderer
viewer = qtViewer3d()
renderer = BodyRenderer(viewer._display)

# Display bodies
renderer.display_bodies(bodies)

# Highlight a specific body
renderer.highlight_body(body_id=0)

# Toggle COM visibility
renderer.set_com_visibility(visible=True)
```

---


## Appendix

### Unit Conversion Table

| File Unit | Scale Factor | Description |
|-----------|--------------|-------------|
| METRE, METER, M | 1.0 | Meters (SI base unit) |
| MILLIMETRE, MILLIMETER, MM | 0.001 | Millimeters → meters |
| CENTIMETRE, CENTIMETER, CM | 0.01 | Centimeters → meters |
| INCH, IN | 0.0254 | Inches → meters |
| FOOT, FT | 0.3048 | Feet → meters |

**Volume Scaling:** `volume_m³ = volume_model * (unit_scale)³`

**Inertia Scaling:** `inertia_SI = inertia_model * (unit_scale)⁵` (assuming unit density)

### Color Codes

| Element | Color | RGB | Purpose |
|---------|-------|-----|---------|
| Normal Body | Light Gray | (0.8, 0.8, 0.8) | Default body color |
| Highlighted Body | Yellow | (1.0, 0.9, 0.0) | Selected body |
| Highlighted Face | Yellow (Overlay) | (1.0, 0.9, 0.0) | Selected face |
| Highlighted Edge | Red (Line) | (1.0, 0.0, 0.0) | Selected edge |
| Highlighted Vertex | Cyan (Sphere) | (0.0, 0.8, 1.0) | Selected vertex |
| COM Marker | Red | (1.0, 0.0, 0.0) | Center of mass sphere |
| X-Axis | Red | (1.0, 0.0, 0.0) | Frame X direction |
| Y-Axis | Green | (0.0, 1.0, 0.0) | Frame Y direction |
| Z-Axis | Blue | (0.0, 0.0, 1.0) | Frame Z direction |

### File Extensions

**Supported STEP Extensions:**
- `.step`
- `.stp`
- `.STEP`
- `.STP`

### Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| Ctrl+O | Open STEP File |
| Ctrl+Q | Exit Application |

### Mouse Controls (3D Viewer)

| Action | Control |
|--------|---------|
| Rotate View | Right-click + drag |
| Pan View | Shift + Right-click + drag |
| Zoom | Mouse wheel |
| Select Body/Face/Edge/Vertex | Left-click (depends on mode) |

---


## Troubleshooting

### Issue: Bodies not selectable in viewer

**Solution:** Ensure selection mode is activated:
```python
for body_id, ais_shape in body_ais_map.items():
    display.Context.Activate(ais_shape, 0, False)  # Mode 0 for bodies
```

### Issue: Face selection index mismatch

**Solution:** Ensure face properties are extracted **before** setting body mapping:
```python
face_properties_map[body.id] = GeometryUtils.extract_faces(body.shape)
viewer_3d.set_body_mapping(body_ais_map, bodies_dict)
```

### Issue: COM marker not visible

**Solution:** Check COM visibility checkbox is enabled and body has calculated COM:
```python
if body.center_of_mass is not None:
    body_renderer.set_com_visibility(True)
```

### Issue: Vertex properties not visible when vertex is selected

**Symptom:** When switching to Vertex selection mode or clicking on a vertex, the vertex property panel (with X/Y/Z coordinates) doesn't appear on the right panel.

**Root Cause:** The `_update_group_visibility()` method in `property_panel.py` was missing a case for "Vertex" mode, only handling "Body", "Face", and "Edge" modes.

**Solution (Fixed in v1.6):** Added "Vertex" case to `_update_group_visibility()`:
```python
elif mode == "Vertex":
    self.body_group.setVisible(False)
    self.face_group.setVisible(False)
    self.edge_group.setVisible(False)
    self.vertex_group.setVisible(True)
    self.face_frame_button.setEnabled(False)
    self.edge_frame_button.setEnabled(False)
    self.vertex_frame_button.setEnabled(self.vertex_index_label.text() != "—")
```

**Verification:** 
1. Select "Vertex" from the Selection Mode dropdown
2. Click on any vertex in the 3D viewer
3. Vertex Information panel should appear showing vertex index and X/Y/Z coordinates
4. A cyan sphere should highlight the selected vertex

---

## Code Quality Improvements

### Increment 31: Robustness & Consistency Enhancements

This increment focused on improving code reliability, consistency, and maintainability through targeted bug fixes and architectural improvements.

### 1. Unit Conversion Consistency

**Issue:** Frame origins returned in inconsistent units across geometry utilities

**Solution:** Standardized all frame creation methods to handle unit conversion transparently

**Modified Methods:**
- `GeometryUtils.frame_from_face(face_props, name, unit_scale)` 
- `GeometryUtils.frame_from_edge(edge_props, name, unit_scale)`

**Changes:**
- Added `unit_scale` parameter (default: 1.0) to both methods
- Origin coordinates are now automatically scaled to **meters**
- Consistent with `frame_from_vertex()` which already scaled coordinates
- Eliminated need for manual scaling in calling code

**Before:**
```python
frame = GeometryUtils.frame_from_face(face_props, "Frame_0")
frame.origin = frame.origin * self.unit_scale  # Manual scaling required
```

**After:**
```python
frame = GeometryUtils.frame_from_face(face_props, "Frame_0", unit_scale=self.unit_scale)
# Origin is automatically in meters - no manual conversion needed
```

**Impact:** Improves code maintainability and eliminates fragile manual conversions

---

### 2. Frame-to-Body Association Tracking

**Issue:** Frames deleted via fragile string pattern matching (`f"_B{body_id}_"`)

**Solution:** Explicit dictionary-based association tracking

**Changes:**
- Added `frame_to_body_map: Dict[str, int]` to MainWindow
- All frame creation methods register frame-to-body associations
- Frame deletion uses O(1) dictionary lookup instead of O(n) string search
- Properly handles user-renamed frames

**Before:**
```python
# Fragile string matching - would miss renamed frames
frames_to_delete = []
for frame_name, frame in self.created_frames.items():
    if f"_B{body_id}_" in frame_name:  # Breaks if user renames frame
        frames_to_delete.append(frame_name)
```

**After:**
```python
# Robust dictionary lookup - works regardless of frame names
frames_to_delete = [name for name, bid in self.frame_to_body_map.items() if bid == body_id]
for frame_name in frames_to_delete:
    del self.frame_to_body_map[frame_name]  # Clean up association
```

**Frame Registration in Creation Methods:**
```python
# on_create_frame_from_face()
self.frame_to_body_map[frame_name] = body_id

# on_create_frame_from_edge()
self.frame_to_body_map[frame_name] = body_id

# on_create_frame_from_vertex()
if self.selected_body_id is not None:
    self.frame_to_body_map[frame_name] = self.selected_body_id
```

**Impact:** More reliable frame management and better UX

---

### 3. Selection Mode Reset on File Load

**Issue:** Selection mode (Body/Face/Edge/Vertex) persisted across file loads

**Solution:** Reset selection mode to "Body" when loading new files

**Implementation:**
```python
def load_step_file(self, filepath):
    # ... clear previous data ...
    self.frame_to_body_map.clear()  # Clear associations
    self.property_panel.set_selection_mode("Body")  # Reset to default
    # ... load new file ...
```

**Impact:** Better user experience - consistent behavior across file loads

---

### 4. Null Shape Validation

**Issue:** Loading empty or corrupted STEP files could cause crashes

**Solution:** Validate shape immediately after loading

**Implementation in `step_parser.py`:**
```python
# Transfer the roots to get the shape
reader.TransferRoots()
shape = reader.OneShape()

# Validate that the shape is not null/empty
if shape.IsNull():
    raise Exception("STEP file is empty or invalid - no solid shapes found")
```

**Error Message:**
```
STEP file is empty or invalid - no solid shapes found
```

**Impact:** Early detection of invalid files with clear error messages

---

### 5. Enhanced Export Error Handling

**Issue:** Export failures provided generic error messages and no validation

**Solution:** Comprehensive path validation and specific error handling

**Implementation in `exporter.py`:**
```python
try:
    # Validate output path
    output_path_obj = Path(output_path)
    output_dir = output_path_obj.parent
    
    # Create output directory if needed
    if not output_dir.exists():
        output_dir.mkdir(parents=True, exist_ok=True)
    
    # Verify it's actually a directory
    if not output_dir.is_dir():
        raise ValueError(f"Output path parent is not a directory: {output_dir}")
    
    # Check write permissions
    if not os.access(output_dir, os.W_OK):
        raise PermissionError(f"No write permission for directory: {output_dir}")
    
    # Proceed with export...
    
except PermissionError as e:
    print(f"Permission Error: Cannot write to {output_path}")
except ValueError as e:
    print(f"Invalid path: {e}")
except Exception as e:
    print(f"Error exporting assembly: {e}")
```

**Benefits:**
- Creates directories automatically when needed
- Validates write permissions before attempting export
- Specific error messages for different failure modes
- Prevents incomplete/corrupted file writes

**Impact:** More robust export with informative error messages

---

### Summary of Changes by File

| File | Changes | Reason |
|------|---------|--------|
| `core/geometry_utils.py` | Added `unit_scale` parameter to frame creation methods | Unit conversion consistency |
| `core/step_parser.py` | Added null shape validation | File validation & error handling |
| `main.py` | Added `frame_to_body_map`, updated frame operations, reset selection mode | Frame tracking & UX improvement |
| `export/exporter.py` | Enhanced path validation & error handling | Robust export with clear errors |

### Testing Recommendations

**Unit Tests:**
1. Test frame creation with different unit scales (mm=0.001, m=1.0, in=0.0254)
2. Verify frame-to-body associations on creation and deletion
3. Test empty STEP file rejection
4. Test export with invalid/read-only directories

**Integration Tests:**
1. Load STEP file, create frames, delete body → verify proper cleanup
2. Load file, change selection mode, load another file → verify reset
3. Export to non-existent directory → verify creation
4. Export with permission errors → verify informative message

**Manual Testing:**
1. Create frames from face/edge/vertex and verify origins are in meters
2. Rename frames and delete parent body → verify all cleaned up
3. Change to Vertex mode, load new file → verify mode resets
4. Export assembly with various directory scenarios

---


## Future Enhancements

**Planned Features:**
- Motor control laws (PID, trajectory following)
- Motor limits (velocity, torque saturation)
- Cylindrical joint dual actuation (rotation + translation)
- Export to simulation formats (URDF, MuJoCo, etc.)
- Collision geometry generation
- Contact point detection
- Assembly constraint analysis
- Material property assignment
- Mass distribution visualization
- Gear ratios and transmission modeling

---

**End of Documentation**

```