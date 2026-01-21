# Multi-Body Dynamics Preprocessor - Refined Implementation Plan

Perfect! Your clarifications make the scope much clearer. Let me create a concrete implementation plan.

---

## Core Data Structures (Refined)

```python
class Frame:
    """Represents a coordinate frame (origin + orientation)"""
    - origin: np.array([x, y, z])
    - orientation: 3x3 rotation matrix (or quaternion)
    - x_axis, y_axis, z_axis: direction vectors
    
    Methods:
    - to_transform_matrix() → 4x4 homogeneous transform
    - transform_to_global(parent_frame) → Frame in global coords
    - render() → create visual representation (3 colored arrows)
    - from_point_and_normal(point, normal) → create frame
    - from_edge_direction(point, direction) → create frame

class RigidBody:
    - id: str
    - name: str
    - geometry: TopoDS_Shape (original CAD)
    - visualization_mesh: tessellated mesh
    - collision_hull: simplified convex hull vertices
    - volume: float (m³)
    - center_of_mass: np.array([x, y, z])
    - inertia_tensor_normalized: 3x3 matrix (divide by density to get actual)
    - local_frame: Frame (body's local coordinate system)
    - color: RGB for visualization
    - material_density: float (optional, for user reference)

class Joint:
    - id: str
    - name: str
    - type: JointType enum
    - body_1_id: str
    - body_2_id: str
    - frame_body_1: Frame (joint frame in body 1's local coords)
    - frame_body_2: Frame (joint frame in body 2's local coords)
    - frame_global: Frame (computed from current body positions)
    
    Methods:
    - compute_global_frame(body1, body2) → updates frame_global
    - set_limits(lower, upper) → for joint constraints
    - render() → visualize joint frame and connection

class Assembly:
    - name: str
    - bodies: dict[str, RigidBody]
    - joints: dict[str, Joint]
    - selection_manager: handles current selections
    - frames: dict[str, Frame] (user-created reference frames)
```

---

## Detailed Phase Breakdown

### **Phase 1: Project Setup & Basic Infrastructure**

#### 1.1 Project Structure
```
mbd_preprocessor/
├── main.py                          # Application entry point
├── requirements.txt
├── core/
│   ├── __init__.py
│   ├── data_structures.py          # Frame, RigidBody, Joint, Assembly
│   ├── step_parser.py              # STEP file loading
│   ├── geometry_utils.py           # Mesh, hull, volume calculations
│   └── physics_calculator.py       # Volume, COM, inertia calculations
├── gui/
│   ├── __init__.py
│   ├── main_window.py              # Main application window
│   ├── viewer_3d.py                # OCC 3D viewport widget
│   ├── body_tree_widget.py         # Left panel body/joint tree
│   ├── property_panel.py           # Right panel properties
│   └── joint_creation_dialog.py    # Joint creation wizard
├── visualization/
│   ├── __init__.py
│   ├── frame_renderer.py           # Render Frame objects
│   ├── body_renderer.py            # Render bodies with selection
│   └── joint_renderer.py           # Render joint visualizations
├── export/
│   ├── __init__.py
│   └── json_exporter.py            # Export to JSON format
└── tests/
    └── ...
```

#### 1.2 Dependencies
```
pythonocc-core>=7.7.0
PySide2>=5.15.0
numpy>=1.21.0
scipy>=1.7.0  # for convex hull generation
trimesh>=3.9.0  # for mesh operations
```

---

### **Phase 2: STEP Loading & Body Management**

#### 2.1 STEP Parser Implementation

**Key Functions:**
```python
def load_step_file(filepath: str) -> List[TopoDS_Shape]:
    """Load STEP file and extract all solid bodies"""
    
def extract_bodies_from_compound(shape: TopoDS_Shape) -> List[TopoDS_Shape]:
    """Separate individual solids from compound/assembly"""
    
def generate_body_name(shape: TopoDS_Shape, index: int) -> str:
    """Try to extract name from STEP metadata, fallback to Body_N"""
```

#### 2.2 Geometry Processing

```python
def tessellate_shape(shape: TopoDS_Shape, quality: float = 0.5) -> MeshData:
    """Convert CAD geometry to triangulated mesh for visualization"""
    
def generate_convex_hull(shape: TopoDS_Shape) -> np.ndarray:
    """Generate simplified convex hull vertices for collision"""
    # Use scipy.spatial.ConvexHull on sampled points from surface
    
def calculate_volume(shape: TopoDS_Shape) -> float:
    """Calculate body volume using OCC GProp"""
    
def calculate_inertia_normalized(shape: TopoDS_Shape, com: np.array) -> np.ndarray:
    """Calculate inertia tensor / unit density about COM"""
    # Returns tensor such that: actual_inertia = tensor * density
```

 For the normalized inertia tensor
- Store it about the center of mass?

---

### **Phase 3: Interactive 3D Viewer**

#### 3.1 Selection System

This is critical for joint creation. Here's the approach:

```python
class SelectionManager:
    """Manages geometry selection in 3D viewport"""
    
    def __init__(self, ais_context):
        self.context = ais_context
        self.selection_mode = SelectionMode.BODY  # or FACE, EDGE, VERTEX
        
    def select_body(self, screen_x, screen_y) -> Optional[str]:
        """Returns body_id if body clicked"""
        
    def select_face(self, screen_x, screen_y) -> Optional[GeometrySelection]:
        """Returns face and center point"""
        
    def select_edge(self, screen_x, screen_y) -> Optional[GeometrySelection]:
        """Returns edge, midpoint, and direction vector"""

class GeometrySelection:
    body_id: str
    geometry_type: str  # "FACE", "EDGE"
    shape: TopoDS_Shape  # The selected face/edge
    point: np.array  # Center of face or midpoint of edge
    normal: Optional[np.array]  # For faces
    direction: Optional[np.array]  # For edges
```

#### 3.2 Frame Visualization

```python
class FrameRenderer:
    """Renders coordinate frames as colored arrows"""
    
    def render_frame(frame: Frame, scale: float = 1.0, label: str = ""):
        """
        Creates AIS objects for frame:
        - Red arrow for X-axis
        - Green arrow for Y-axis  
        - Blue arrow for Z-axis
        - Optional text label at origin
        """
```

---

### **Phase 4: Joint Creation Workflow**

This is the most complex part. Here's the detailed workflow:

#### 4.1 Joint Creation State Machine

```python
class JointCreationWizard:
    """Multi-step wizard for creating joints"""
    
    States:
    1. SELECT_JOINT_TYPE → user picks from dropdown
    2. SELECT_BODY_1 → highlight bodies on hover, click to select
    3. SELECT_GEOMETRY_1 → switch to face/edge selection mode
    4. DEFINE_FRAME_1 → create Frame from selection, show preview
    5. ADJUST_FRAME_1 → (optional) manual adjustment of frame orientation
    6. SELECT_BODY_2 → same as step 2
    7. SELECT_GEOMETRY_2 → same as step 3
    8. DEFINE_FRAME_2 → same as step 4
    9. ADJUST_FRAME_2 → (optional) manual adjustment
    10. CONFIRM → create Joint object, add to assembly
```

#### 4.2 Frame Creation from Geometry

```python
def create_frame_from_face(face: TopoDS_Face, point: np.array) -> Frame:
    """
    Create frame with:
    - Origin at point (face center)
    - Z-axis along face normal
    - X and Y axes in plane (arbitrary but orthogonal)
    """
    
def create_frame_from_edge(edge: TopoDS_Edge, point: np.array) -> Frame:
    """
    Create frame with:
    - Origin at point (edge midpoint)
    - Z-axis along edge direction
    - X and Y perpendicular (arbitrary)
    """
```

#### 4.3 Manual Frame Adjustment Widget

```python
class FrameAdjustmentPanel(QWidget):
    """
    Allow user to adjust frame:
    - Translate origin (X, Y, Z spinboxes)
    - Rotate about axes (angle spinboxes or sliders)
    - Visual feedback in 3D viewport
    - Reset to initial state
    """
```

For frame adjustment,  we provide:

- Or full 6-DOF adjustment (translation + rotation)?

---

### **Phase 5: Joint Types & Constraints**

#### 5.1 Initial Joint Types to Implement

```python
class JointType(Enum):
    FIXED = "fixed"              # 0 DOF - rigid connection
    HINGE = "hinge"              # 1 DOF - rotation about Z-axis
    CYLINDRICAL = "cylindrical"  # 2 DOF - rotation + translation along Z
    PRISMATIC = "prismatic"      # 1 DOF - translation along Z
    # Add later: SPHERICAL, UNIVERSAL, PLANAR
```

#### 5.2 Joint Constraints

```python
class JointConstraints:
    """Stores joint-specific parameters"""
    
    # For HINGE, CYLINDRICAL
    rotation_limits: Tuple[float, float]  # radians
    
    # For PRISMATIC, CYLINDRICAL
    translation_limits: Tuple[float, float]  # meters
    
    # Optional: damping, stiffness for compliant joints
    damping: float
    stiffness: float
```



---

### **Phase 6: Export System**

#### 6.1 JSON Output Format (Refined)

```json
{
  "metadata": {
    "preprocessor_version": "0.1.0",
    "export_date": "2026-01-12T10:30:00Z",
    "units": {
      "length": "meters",
      "mass": "kilograms",
      "angle": "radians"
    },
    "coordinate_system": "right_handed_z_up"
  },
  
  "bodies": [
    {
      "id": "body_0",
      "name": "base_link",
      "volume": 0.00125,
      "center_of_mass": [0.0, 0.0, 0.05],
      "inertia_tensor_normalized": [
        [2.083e-5, 0.0, 0.0],
        [0.0, 2.083e-5, 0.0],
        [0.0, 0.0, 4.167e-6]
      ],
      "local_frame": {
        "origin": [0.0, 0.0, 0.0],
        "rotation_matrix": [[1,0,0], [0,1,0], [0,0,1]]
      },
      "visualization": {
        "mesh_format": "OBJ",
        "mesh_path": "./meshes/body_0_visual.obj"
      },
      "collision": {
        "type": "convex_hull",
        "vertices": [[...], [...], ...],
        "num_vertices": 156
      }
    }
  ],
  
  "joints": [
    {
      "id": "joint_0",
      "name": "base_to_link1",
      "type": "hinge",
      "body_1": "body_0",
      "body_2": "body_1",
      
      "frame_body_1": {
        "origin": [0.0, 0.0, 0.1],
        "rotation_matrix": [[1,0,0], [0,1,0], [0,0,1]]
      },
      
      "frame_body_2": {
        "origin": [0.0, 0.0, -0.05],
        "rotation_matrix": [[1,0,0], [0,1,0], [0,0,1]]
      },
      
      "frame_global": {
        "origin": [0.0, 0.0, 0.1],
        "axis": [0.0, 0.0, 1.0],
        "rotation_matrix": [[1,0,0], [0,1,0], [0,0,1]]
      },
      
      "constraints": {
        "rotation_limits": [-3.14159, 3.14159]
      }
    }
  ]
}
```

---

## Implementation Timeline

### **Sprint 1 (Week 1-2): Foundation**
- [ ] Set up project structure
- [ ] Implement Frame, RigidBody, Joint, Assembly classes
- [ ] Create basic PySide2 main window
- [ ] Integrate pythonocc 3D viewer
- [ ] STEP file loading (single file → display all bodies)

### **Sprint 2 (Week 3-4): Body Management**
- [ ] Body tree widget (list all bodies)
- [ ] Body selection and highlighting
- [ ] Calculate volume, COM, normalized inertia
- [ ] Property panel showing body properties
- [ ] Generate and display convex hull

### **Sprint 3 (Week 5-6): Frame System**
- [ ] Implement Frame rendering (colored axes)
- [ ] Face/edge selection in viewer
- [ ] Create frames from selected geometry
- [ ] Frame adjustment panel (translation + rotation)

### **Sprint 4 (Week 7-9): Joint Creation**
- [ ] Joint creation wizard UI
- [ ] State machine for joint creation workflow
- [ ] Implement FIXED, HINGE, PRISMATIC joints
- [ ] Joint visualization in viewport
- [ ] Joint list in tree widget

### **Sprint 5 (Week 10-11): Export & Polish**
- [ ] JSON export functionality
- [ ] Mesh export (OBJ files for visualization)
- [ ] Data validation before export
- [ ] Save/load project files (.mbd format)
- [ ] Error handling and user feedback

---

## Technical Decisions Needed

Coordinate system convention:
 Right-handed, Z-up (common in robotics)

---

## Next Steps

I'm ready to create:
1. **Initial project skeleton** with folder structure
2. **Core data structure classes** (Frame, RigidBody, Joint, Assembly)
3. **Basic GUI** with 3D viewer integration
4. **STEP loader prototype**

