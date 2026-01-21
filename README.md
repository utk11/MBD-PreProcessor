# Multi-Body Dynamics Preprocessor

A desktop application for preparing CAD assemblies for multi-body dynamics simulations. Load STEP files, define rigid bodies, create joints, and export to simulation-ready formats.

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
   conda create -n mbd python=3.10
   conda activate mbd
   conda install -c conda-forge pythonocc-core
   ```

3. Install remaining dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

Run the application:
```bash
python main.py
```

### Basic Workflow

1. **File → Open STEP**: Load a CAD assembly file
2. **Select bodies** in the left panel tree view
3. **Create joints** via Edit → Create Joint:
   - Select joint type
   - Pick two bodies
   - Select geometry (face/edge) on each body to define joint frames
4. **Add forces/torques** if needed
5. **File → Export**: Save to JSON for your dynamics solver

## Project Structure

```
├── main.py                 # Application entry point
├── core/                   # Core logic
│   ├── data_structures.py  # RigidBody, Joint, Frame classes
│   ├── step_parser.py      # STEP file loading
│   ├── geometry_utils.py   # Mesh and hull calculations
│   └── physics_calculator.py
├── gui/                    # User interface
│   ├── viewer_3d.py        # 3D viewport
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
- **Bodies**: Volume, center of mass, inertia tensor, collision hull vertices
- **Joints**: Type, connected bodies, joint frames in local and global coordinates
- **Forces/Torques**: Magnitude, direction, application point
- **Motors**: Control type, target values

## Dependencies

- [pythonocc-core](https://github.com/tpaviot/pythonocc-core) - CAD kernel (OpenCASCADE wrapper)
- [PySide6](https://wiki.qt.io/Qt_for_Python) - GUI framework
- [NumPy](https://numpy.org/) - Numerical computing
- [SciPy](https://scipy.org/) - Convex hull generation
- [trimesh](https://trimesh.org/) - Mesh operations

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## License

This project is licensed under the GPL-3.0 License - see the [LICENSE](LICENSE) file for details.