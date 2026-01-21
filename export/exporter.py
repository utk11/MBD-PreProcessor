"""
Assembly Export Module
Exports assembly data to JSON and body meshes to OBJ format

All coordinates are in global world frame.
Note: Since this is a preprocessor (not a simulator), all frames are stored 
in world coordinates and exported directly without transformation.
"""

import json
import os
import numpy as np
from typing import List, Dict, Optional
from pathlib import Path
from OCC.Core.TopoDS import TopoDS_Shape
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.BRep import BRep_Tool
from OCC.Core.gp import gp_Pnt

from core.data_structures import RigidBody, Joint, Frame


class AssemblyExporter:
    """Handles exporting assembly data to various formats"""
    
    @staticmethod
    def export_assembly_to_json(
        bodies: List[RigidBody],
        joints: Dict[str, Joint],
        frames: Dict[str, Frame],
        ground_body: RigidBody,
        unit_scale: float,
        output_path: str,
        export_meshes: bool = True
    ) -> bool:
        """
        Export complete assembly to JSON format with all coordinates in global world frame
        
        Args:
            bodies: List of rigid bodies
            joints: Dictionary of joints (name -> Joint)
            frames: Dictionary of user-created frames (name -> Frame)
            ground_body: The ground body reference
            unit_scale: Unit scale factor from STEP file
            output_path: Path to save JSON file
            export_meshes: Whether to export OBJ meshes alongside JSON
            
        Returns:
            True if export successful, False otherwise
        """
        try:
            # Validate output path
            output_path_obj = Path(output_path)
            output_dir = output_path_obj.parent
            
            # Create output directory if it doesn't exist
            if not output_dir.exists():
                output_dir.mkdir(parents=True, exist_ok=True)
                print(f"Created output directory: {output_dir}")
            
            if not output_dir.is_dir():
                raise ValueError(f"Output path parent is not a directory: {output_dir}")
            
            if not os.access(output_dir, os.W_OK):
                raise PermissionError(f"No write permission for directory: {output_dir}")
            
            # Export meshes if requested
            mesh_rel_dir = "meshes"
            if export_meshes:
                meshes_dir = output_dir / mesh_rel_dir
                print(f"Exporting meshes to {meshes_dir}")
                AssemblyExporter.export_body_meshes_to_obj(bodies, str(meshes_dir), unit_scale)

            # Serialize bodies with mesh paths
            serialized_bodies = []
            for body in bodies:
                mesh_uri = None
                if export_meshes and body.shape is not None:
                     mesh_uri = f"{mesh_rel_dir}/{AssemblyExporter._get_mesh_filename(body)}"
                
                serialized_bodies.append(AssemblyExporter._serialize_body(body, mesh_uri))

            # Build the assembly data structure
            assembly_data = {
                "metadata": {
                    "version": "1.0",
                    "description": "Multi-Body Dynamics Assembly Export",
                    "coordinate_system": "global_world_frame",
                    "unit_scale": 1.0,  # All geometry and physics data normalized to meters
                    "original_unit_scale": unit_scale,
                    "units": "meters"
                },
                "ground_body": AssemblyExporter._serialize_body(ground_body),
                "bodies": serialized_bodies,
                "joints": [AssemblyExporter._serialize_joint(joint, bodies, ground_body) for joint in joints.values()],
                "frames": {name: AssemblyExporter._serialize_frame(frame) for name, frame in frames.items()}
            }
            
            # Write to file with pretty formatting
            with open(output_path, 'w') as f:
                json.dump(assembly_data, f, indent=2)
            
            print(f"Assembly exported successfully to: {output_path}")
            return True
            
        except PermissionError as e:
            print(f"Permission Error: Cannot write to {output_path}")
            print(f"Details: {e}")
            import traceback
            traceback.print_exc()
            return False
        except ValueError as e:
            print(f"Invalid path: {e}")
            import traceback
            traceback.print_exc()
            return False
        except Exception as e:
            print(f"Error exporting assembly to JSON: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    @staticmethod
    def _serialize_body(body: RigidBody, mesh_uri: Optional[str] = None) -> Dict:
        """Serialize a rigid body to dictionary (all coordinates in world frame)"""
        body_data = {
            "id": body.id,
            "name": body.name,
            "volume": float(body.volume) if body.volume else 0.0,
            "contact_enabled": body.contact_enabled,
        }

        if mesh_uri:
            body_data["mesh_file"] = mesh_uri
        
        # Center of mass (already in world frame)
        if body.center_of_mass is not None:
            body_data["center_of_mass"] = [float(x) for x in body.center_of_mass]
        else:
            body_data["center_of_mass"] = [0.0, 0.0, 0.0]
        
        # Inertia tensor (about COM, in world frame orientation)
        if body.inertia_tensor is not None:
            body_data["inertia_tensor"] = [
                [float(x) for x in row] for row in body.inertia_tensor
            ]
        else:
            body_data["inertia_tensor"] = [
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0]
            ]
        
        # Local frame (in world frame coordinates)
        if body.local_frame is not None:
            body_data["local_frame"] = AssemblyExporter._serialize_frame(body.local_frame)
        else:
            body_data["local_frame"] = None
        
        return body_data
    
    @staticmethod
    def _serialize_frame(frame: Frame) -> Dict:
        """Serialize a frame to dictionary (in world frame coordinates)"""
        return {
            "name": frame.name,
            "origin": [float(x) for x in frame.origin],
            "rotation_matrix": [[float(x) for x in row] for row in frame.rotation_matrix],
            "euler_angles_deg": [float(x) for x in frame.get_euler_angles()],
            "x_axis": [float(x) for x in frame.get_x_axis()],
            "y_axis": [float(x) for x in frame.get_y_axis()],
            "z_axis": [float(x) for x in frame.get_z_axis()]
        }
    
    @staticmethod
    def _serialize_joint(joint: Joint, bodies: List[RigidBody], ground_body: RigidBody) -> Dict:
        """
        Serialize a joint to dictionary with frames in world coordinates
        
        Transforms joint frames from body-local coordinates to world coordinates
        """
        # Find the bodies this joint connects
        body1 = None
        body2 = None
        
        if joint.body1_id == -1:
            body1 = ground_body
        else:
            body1 = next((b for b in bodies if b.id == joint.body1_id), None)
        
        if joint.body2_id == -1:
            body2 = ground_body
        else:
            body2 = next((b for b in bodies if b.id == joint.body2_id), None)
        
        joint_data = {
            "name": joint.name,
            "type": joint.joint_type.name,
            "axis": joint.axis,
            "body1_id": joint.body1_id,
            "body1_name": body1.name if body1 else "Unknown",
            "body2_id": joint.body2_id,
            "body2_name": body2.name if body2 else "Unknown",
        }
        
        # Add motor information if motorized
        if joint.is_motorized:
            joint_data["motorized"] = True
            joint_data["motor_type"] = joint.motor_type.name
            joint_data["motor_value"] = joint.motor_value
            
            # Add appropriate units based on motor type and joint type
            if joint.motor_type.name == "VELOCITY":
                joint_data["motor_units"] = "rad/s" if joint.joint_type.name == "REVOLUTE" else "m/s"
            elif joint.motor_type.name == "TORQUE":
                joint_data["motor_units"] = "N·m" if joint.joint_type.name == "REVOLUTE" else "N"
            elif joint.motor_type.name == "POSITION":
                joint_data["motor_units"] = "rad" if joint.joint_type.name == "REVOLUTE" else "m"
        else:
            joint_data["motorized"] = False
        
        # Frame is already stored in world coordinates (this is a preprocessor, bodies don't move)
        # No transformation needed - just serialize it directly
        joint_data["frame_world"] = AssemblyExporter._serialize_frame(joint.frame) if joint.frame else None
        
        return joint_data
    
    @staticmethod
    def _get_mesh_filename(body: RigidBody) -> str:
        """Generate a consistent mesh filename for a body"""
        return f"{body.name}_{body.id}.obj"

    @staticmethod
    def export_body_meshes_to_obj(
        bodies: List[RigidBody],
        output_dir: str,
        unit_scale: float = 1.0
    ) -> bool:
        """
        Export all body geometries as OBJ mesh files (in world frame coordinates)
        
        Args:
            bodies: List of rigid bodies to export
            output_dir: Directory to save OBJ files
            unit_scale: Unit scale factor to convert meshes to meters
            
        Returns:
            True if export successful, False otherwise
        """
        try:
            output_path = Path(output_dir)
            output_path.mkdir(parents=True, exist_ok=True)
            
            for body in bodies:
                if body.shape is None:
                    print(f"Skipping body {body.name} (no geometry)")
                    continue
                
                # Create safe filename
                filename = AssemblyExporter._get_mesh_filename(body)
                filepath = output_path / filename
                
                # Export this body to OBJ (in body's local frame)
                success = AssemblyExporter._export_shape_to_obj(
                    body.shape, str(filepath), body.name, unit_scale, body
                )
                
                if success:
                    print(f"Exported {body.name} to {filename}")
                else:
                    print(f"Failed to export {body.name}")
            
            print(f"\nAll body meshes exported to: {output_dir}")
            return True
            
        except Exception as e:
            print(f"Error exporting body meshes: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    @staticmethod
    def _export_shape_to_obj(shape: TopoDS_Shape, filepath: str, name: str, scale_factor: float = 1.0, body: RigidBody = None) -> bool:
        """
        Export a TopoDS_Shape to OBJ format (vertices in body's local frame)
        
        Args:
            shape: The shape to export
            filepath: Path to save OBJ file
            name: Name for the object in OBJ file
            scale_factor: Factor to scale vertices to meters
            body: RigidBody object (used to transform to local frame)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            # Tessellate the shape (create triangular mesh)
            # Use a reasonable deflection value for mesh quality
            mesh = BRepMesh_IncrementalMesh(shape, 0.1, False, 0.1, True)
            mesh.Perform()
            
            if not mesh.IsDone():
                print(f"Mesh tessellation failed for {name}")
                return False
            
            # Collect vertices and faces
            vertices = []
            faces = []
            vertex_map = {}  # Map (x,y,z) to vertex index
            vertex_index = 1  # OBJ uses 1-based indexing
            
            # Explore all faces in the shape
            face_explorer = TopExp_Explorer(shape, TopAbs_FACE)
            
            while face_explorer.More():
                face = face_explorer.Current()
                location = face.Location()
                
                # Get triangulation for this face
                face_triangulation = BRep_Tool.Triangulation(face, location)
                
                if face_triangulation is not None:
                    # Get transformation
                    transform = location.Transformation()
                    
                    # Get nodes (vertices)
                    num_nodes = face_triangulation.NbNodes()
                    
                    # Map local node indices to global vertex indices
                    local_to_global = {}
                    
                    for i in range(1, num_nodes + 1):
                        # Get vertex and transform to world coordinates
                        pnt = face_triangulation.Node(i)
                        pnt.Transform(transform)
                        
                        # Get coordinates in world frame and scale to meters
                        world_coords = np.array([pnt.X() * scale_factor, pnt.Y() * scale_factor, pnt.Z() * scale_factor])
                        
                        # Transform to body's local frame (COM-centered) if body is provided
                        if body and body.local_frame:
                            # Translate to COM-centered coordinates
                            translated = world_coords - np.array(body.center_of_mass)
                            # Rotate to local frame (inverse rotation = transpose)
                            local_coords = body.local_frame.rotation_matrix.T @ translated
                            coords = tuple(local_coords)
                        else:
                            coords = tuple(world_coords)
                        
                        # Check if this vertex already exists (with tolerance)
                        vertex_key = tuple(round(c, 6) for c in coords)
                        
                        if vertex_key not in vertex_map:
                            vertices.append(coords)
                            vertex_map[vertex_key] = vertex_index
                            local_to_global[i] = vertex_index
                            vertex_index += 1
                        else:
                            local_to_global[i] = vertex_map[vertex_key]
                    
                    # Get triangles (faces)
                    num_triangles = face_triangulation.NbTriangles()
                    
                    for i in range(1, num_triangles + 1):
                        triangle = face_triangulation.Triangle(i)
                        n1, n2, n3 = triangle.Get()
                        
                        # Map to global indices
                        v1 = local_to_global[n1]
                        v2 = local_to_global[n2]
                        v3 = local_to_global[n3]
                        
                        # OBJ face format: indices are 1-based
                        faces.append((v1, v2, v3))
                
                face_explorer.Next()
            
            # Write OBJ file
            with open(filepath, 'w') as f:
                f.write(f"# OBJ file exported from Multi-Body Dynamics Preprocessor\n")
                f.write(f"# Object: {name}\n")
                f.write(f"# Vertices: {len(vertices)}\n")
                f.write(f"# Faces: {len(faces)}\n")
                coord_system = "Body local frame (COM-centered)" if body else "Global world frame"
                f.write(f"# Coordinate system: {coord_system}\n\n")
                
                f.write(f"o {name}\n\n")
                
                # Write vertices
                for v in vertices:
                    f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
                
                f.write("\n")
                
                # Write faces
                for face in faces:
                    f.write(f"f {face[0]} {face[1]} {face[2]}\n")
            
            return True
            
        except Exception as e:
            print(f"Error exporting shape to OBJ: {e}")
            import traceback
            traceback.print_exc()
            return False
