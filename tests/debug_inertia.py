
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepGProp import brepgprop
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Trsf
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
import numpy as np

def calculate_inertia_tensor(shape):
    props = GProp_GProps()
    brepgprop.VolumeProperties(shape, props)
    
    com = props.CentreOfMass()
    matrix_origin = props.MatrixOfInertia()
    
    Ixx_origin = matrix_origin.Value(1, 1)
    Iyy_origin = matrix_origin.Value(2, 2)
    Izz_origin = matrix_origin.Value(3, 3)
    Ixy_origin = matrix_origin.Value(1, 2)
    Ixz_origin = matrix_origin.Value(1, 3)
    Iyz_origin = matrix_origin.Value(2, 3)
    
    mass = props.Mass()
    
    cx = com.X()
    cy = com.Y()
    cz = com.Z()
    
    print(f"Mass: {mass}")
    print(f"COM: {cx}, {cy}, {cz}")
    print(f"Origin Inertia Tensor from OCC:")
    print(f"[[{Ixx_origin}, {Ixy_origin}, {Ixz_origin}],")
    print(f" [{Ixy_origin}, {Iyy_origin}, {Iyz_origin}],")
    print(f" [{Ixz_origin}, {Iyz_origin}, {Izz_origin}]]")

    # Manual Parallel Axis Theorem
    Ixx_com = Ixx_origin - mass * (cy*cy + cz*cz)
    Iyy_com = Iyy_origin - mass * (cx*cx + cz*cz)
    Izz_com = Izz_origin - mass * (cx*cx + cy*cy)
    Ixy_com = Ixy_origin + mass * cx * cy
    Ixz_com = Ixz_origin + mass * cx * cz
    Iyz_com = Iyz_origin + mass * cy * cz

    print(f"Calculated COM Inertia components:")
    print(f"Ixx_com: {Ixx_com}")
    print(f"Iyy_com: {Iyy_com}")
    print(f"Izz_com: {Izz_com}")
    print(f"Ixy_com: {Ixy_com}")
    
    # Original code construction
    inertia_tensor_orig = np.array([
        [Ixx_com, -Ixy_com, -Ixz_com],
        [-Ixy_com, Iyy_com, -Iyz_com],
        [-Ixz_com, -Iyz_com, Izz_com]
    ])
    
    print("Inertia Tensor (Original Code logic):")
    print(inertia_tensor_orig)

    # Check against theoretical box inertia
    # Box size 10, 20, 30 centered at 100, 100, 100
    # Ixx_com = m/12 * (y^2 + z^2)
    
    return inertia_tensor_orig

# Create a box at origin
box = BRepPrimAPI_MakeBox(10., 20., 30.).Shape()

# Move it
trsf = gp_Trsf()
trsf.SetTranslation(gp_Vec(100., 100., 100.))
transform = BRepBuilderAPI_Transform(box, trsf, True)
moved_box = transform.Shape()

print("--- Running Test ---")
calculate_inertia_tensor(moved_box)
