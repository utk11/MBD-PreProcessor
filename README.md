# Multi-Body Dynamics Preprocessor

A desktop application for preparing CAD assemblies for multi-body dynamics simulations. Load STEP files, define rigid bodies, create joints, and export to simulation-ready formats.
Here is a demo :

https://www.linkedin.com/posts/utkarsh-kulkarni-737a21143_python-mbd-simulation-activity-7417465950595358720-Bytp?utm_source=share&utm_medium=member_desktop&rcm=ACoAACLbVNsBFxFMNDgH9pOH3QkAD194cmSTt6s

![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)
![PySide6](https://img.shields.io/badge/GUI-PySide6-green.svg)
![License](https://img.shields.io/badge/License-GPL--3.0-blue.svg)

## Features

- **STEP File Import**: Load CAD assemblies from STEP files and automatically extract individual bodies
- **3D Visualization**: Interactive 3D viewer with pan, zoom, rotate, and selection capabilities
- **Physics Properties**: Automatic calculation of volume, center of mass, and inertia tensors
- **Joint Creation**: Define joints between bodies by selecting faces, edges, or vertices
  - Supported joint types: Fixed, Revolute (Hinge), Prismatic, Cylindrical, Spherical, Universal, Planar
- **Forces & Torques**: Add external forces and torques to bodies
- **Motors**: Attach motors to joints with position, velocity, or torque control
- **Export**: Export assembly data to JSON format for use in dynamics solvers

## Installation

### Prerequisites

- Python 3.8 or higher
- [Conda](https://docs.conda.io/en/latest/) (recommended for pythonocc-core)

### Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/utk11/MBD-PreProcessor.git
   cd MBD-PreProcessor
   ```

2. Create a conda environment and install pythonocc-core:
   ```bash
   conda create -n mbd_preproc python=3.10
   conda activate mbd_preproc
   conda install -c conda-forge pythonocc-core
   ```

3. Install remaining dependencies, including JAX:
   ```bash
   pip install -r requirements.txt
   ```

   JAX evaluates joint residuals and Jacobians. This environment is Python 3.10,
   so the pin is `jax==0.6.2` (`jax==0.11.2` requires Python 3.12). The app
   exits with an install message if JAX is missing. It does not fall back to
   another evaluator. The shared Levenberg-Marquardt loop uses NumPy; its
   selectable linear methods use NumPy and SciPy.

## Usage

Run the application:
```bash
python main.py
```

Use the **Linear solver** dropdown above the viewer to choose **Dense (default)**,
**SuperLU**, **Conjugate Gradient (CG)**, or **LSMR**. Your selection applies to
both body dragging and **Assembly → Solve Assembly** (`Ctrl+K`). Switching keeps
the current body poses and the warmed JAX evaluator; any unfinished solve with
the previous method is discarded. The selection lasts for the current app
session, including when you load another project.

### Basic Workflow

1. **File → Open STEP**: Load a CAD assembly file
2. **Select bodies** in the left panel tree view
3. **Create frames** on the faces, edges, or vertices that form each connection
4. **Create joints** via Edit → Create Joint:
   - Select joint type and a frame on each connected body
   - Choose an axis on each frame for revolute, prismatic, and cylindrical joints, and flip either frame orientation when needed
   - Choose **Create & Assemble** to solve the assembly, optionally keeping Body 1 fixed, or choose **Create** and run Solve Assembly later
5. **Add forces/torques** if needed
6. **File → Export**: Save to JSON for your dynamics solver

## Project Structure

```
├── main.py                 # Application entry point
├── core/                   # Core logic
│   ├── data_structures.py  # RigidBody, Joint, Frame classes
│   ├── assembly_document.py # Bodies, joints, loads, and revisions
│   ├── project_store.py    # Versioned .mbdp save and load
│   ├── transforms.py       # World pose and reference frame
│   ├── kinematics/         # JAX residual/Jacobian and the LM solver
│   ├── step_parser.py      # STEP file loading
│   ├── geometry_utils.py   # Mesh and hull calculations
│   └── physics_calculator.py
├── gui/                    # User interface
│   ├── viewer_3d.py        # 3D viewport
│   ├── solve_scheduler.py  # One worker, one pending drag target
│   ├── application_controller.py
│   ├── body_tree_widget.py # Body/joint tree panel
│   ├── property_panel.py   # Properties editor
│   └── *_dialog.py         # Creation dialogs
├── visualization/          # Rendering
│   └── *_renderer.py       # Body, joint, force renderers
├── export/                 # Export functionality
│   └── exporter.py         # JSON export
└── tests/                  # Unit tests
```

## Export Format

The exported JSON includes:
- **Bodies**: Volume, center of mass, inertia tensor, collision hull vertices.
  `world_pose` is the live placement. `local_frame` is the reference
  center-of-mass frame from import. Mesh vertices are written in that
  reference frame.
- **Joints**: Type and connected bodies, both body-local attachments
  (`marker1_local`, `marker2_local`), and both attachments transformed by the
  live body poses (`marker1_world`, `marker2_world`). `frame_world` remains as
  the legacy creation-frame field.
- **Forces/Torques**: Magnitude, direction, application point
- **Motors**: Control type, target values

Project files (`.mbdp`) use schema 2. They store poses, markers, frame
attachments, motors, forces, torques, and a STEP fingerprint. Schema 1 files
still open. Poses, markers, motors, and frame parents that version 1 never
saved are not invented; the load dialog lists those limits.

## Dependencies

Optional application control through MCP is documented in
[MCP control setup](Documentation/mcp-control.md). Start with `main.py --enable-mcp`
to expose the local control bridge; normal startup leaves control disabled.

- [pythonocc-core](https://github.com/tpaviot/pythonocc-core) - CAD kernel (OpenCASCADE wrapper)
- [PySide6](https://wiki.qt.io/Qt_for_Python) - GUI framework
- [NumPy](https://numpy.org/) - Numerical computing
- [SciPy](https://scipy.org/) - Convex hull generation and sparse linear prototypes
- [JAX](https://github.com/jax-ml/jax) - Required kinematic residual and Jacobian evaluation (`jax==0.6.2` on Python 3.10)
- [trimesh](https://trimesh.org/) - Mesh operations

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## License

This project is licensed under the GPL-3.0 License - see the [LICENSE](LICENSE) file for details.
