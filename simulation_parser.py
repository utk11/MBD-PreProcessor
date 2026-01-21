import pychrono as chrono
import pychrono.irrlicht as chronoirr
import json
import os
import argparse
import sys
import math

class SimulationParser:
    def __init__(self, json_path):
        self.json_path = json_path
        self.base_dir = os.path.dirname(os.path.abspath(json_path))
        self.system = chrono.ChSystemNSC()
        self.system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))  # Assuming Z-up
        
        # Ground and Body ID mapping
        self.bodies = {} # id -> ChBody
        
        # Default material for contact
        self.material = chrono.ChContactMaterialNSC()
        self.material.SetFriction(0.5)
        self.material.SetDampingF(0.2)
        self.material.SetCompliance(0.0000001)
        self.material.SetComplianceT(0.0000001)

    def load_and_create(self):
        with open(self.json_path, 'r') as f:
            data = json.load(f)
            
        print(f"Loaded assembly: {data.get('metadata', {}).get('description', 'Unknown')}")
        
        # 1. Create Ground
        self.parse_ground(data['ground_body'])
        
        # 2. Create Bodies
        self.parse_bodies(data['bodies'])
        
        # 3. Create Joints and Motors
        self.parse_joints(data['joints'])
        self.parse_motors(data['joints'])
        
        # 4. Create Forces and Torques
        self.parse_forces(data.get('forces', []))
        self.parse_torques(data.get('torques', []))
        
        return self.system

    def _create_matrix_from_list(self, mat_list):
        """Convert 3x3 list to ChMatrix33d"""
        # Create rotation matrix from 3x3 list using direct constructor
        # ChMatrix33d can be constructed from 9 values (row-major)
        return chrono.ChMatrix33d(
            chrono.ChVector3d(mat_list[0][0], mat_list[0][1], mat_list[0][2]),
            chrono.ChVector3d(mat_list[1][0], mat_list[1][1], mat_list[1][2]),
            chrono.ChVector3d(mat_list[2][0], mat_list[2][1], mat_list[2][2])
        )

    def _create_vector_from_list(self, vec_list):
        return chrono.ChVector3d(vec_list[0], vec_list[1], vec_list[2])

    def _get_coordsys_from_frame_data(self, frame_data):
        pos = self._create_vector_from_list(frame_data['origin'])
        rot_mat = self._create_matrix_from_list(frame_data['rotation_matrix'])
        rot = rot_mat.GetQuaternion()
        return chrono.ChCoordsysd(pos, rot)
    
    def _create_body_aux_ref(self, body_data, is_fixed=False):
        body = chrono.ChBodyAuxRef()
        
        # Properties
        name = body_data.get('name', 'Body')
        body_id = body_data.get('id', -1)
        body.SetName(name)
        body.SetFixed(is_fixed)
        
        # Parse Mass properties
        # JSON has volume. Approx mass = volume * density (e.g. Steel 7800 kg/m3)
        # Or just use volume as mass if density is 1.0
        volume = body_data.get('volume', 0.001)
        mass = volume * 1000.0 if volume > 0 else 1.0 # Default density 1000 kg/m3
        if is_fixed:
            mass = 1.0
            
        com_list = body_data.get('center_of_mass', [0,0,0])
        inertia_list = body_data.get('inertia_tensor', [[1,0,0],[0,1,0],[0,0,1]])
        
        com_vec = self._create_vector_from_list(com_list)
        inertia_mat = self._create_matrix_from_list(inertia_list)
        
        # Set Configuration
        # ChBodyAuxRef allows separate frames for Reference (Visual/Collision) and COG (Physics)
        # We assume the exported OBJ and coordinates are in WORLD Frame.
        # So we place the Reference Frame at World Origin (0,0,0) with Identity rotation.
        # And we set the COG Frame at the COM location (relative to REF, which is world).
        
        # 1. Set REF frame to (0,0,0) Identity (This places the "body" at global origin)
        body.SetFrameRefToAbs(chrono.ChFramed(chrono.ChVector3d(0,0,0), chrono.ChQuaterniond(1,0,0,0)))
        
        # 2. Set COG frame. Since REF is at (0,0,0) Identity, COG pos is just Global COM.
        # Orientation of COG frame: Exporter says 'inertia_tensor' is in World Frame Orientation.
        # So COG frame aligns with World Frame.
        body.SetFrameCOMToAbs(chrono.ChFramed(com_vec, chrono.ChQuaterniond(1,0,0,0)))
        
        # 3. Set Mass/Inertia
        body.SetMass(mass)
        body.SetInertia(inertia_mat) 
        
        # Visuals and Collision
        mesh_rel_path = body_data.get('mesh_file')
        if mesh_rel_path:
            mesh_path = os.path.join(self.base_dir, mesh_rel_path)
            if os.path.exists(mesh_path):
                # Load mesh
                try:
                    mesh = chrono.ChTriangleMeshConnected()
                    mesh.LoadWavefrontMesh(mesh_path, True, True)
                    
                    # Visualization
                    vis_shape = chrono.ChVisualShapeTriangleMesh()
                    vis_shape.SetMesh(mesh)
                    vis_shape.SetName(name + "_vis")
                    body.AddVisualShape(vis_shape)
                    
                    # Collision - enabled by default as user requested contact simulation
                    col_shape = chrono.ChCollisionShapeTriangleMesh(
                        self.material,
                        mesh,
                        False,  # is_static
                        False,  # is_convex
                        0.005   # sphere_sweep_radius
                    )
                    body.AddCollisionShape(col_shape)
                    body.EnableCollision(True)
                    
                except Exception as e:
                    print(f"Warning: Failed to load mesh for {name}: {e}")
        
        return body

    def parse_ground(self, ground_data):
        print("Creating Ground...")
        body = self._create_body_aux_ref(ground_data, is_fixed=True)
        self.system.Add(body)
        self.bodies[-1] = body # Standard ground ID or implicit?
        # If ground has an ID in json, map it too
        if 'id' in ground_data:
            self.bodies[ground_data['id']] = body

    def parse_bodies(self, bodies_data):
        print(f"Creating {len(bodies_data)} bodies...")
        for b_data in bodies_data:
            body = self._create_body_aux_ref(b_data)
            self.system.Add(body)
            self.bodies[b_data['id']] = body

    def parse_joints(self, joints_data):
        # Filter for passive joints only
        passive_joints = [j for j in joints_data if not j.get('motorized', False)]
        print(f"Creating {len(passive_joints)} passive joints...")
        
        for j_data in passive_joints:
            name = j_data.get('name', 'Joint')
            type_ = j_data.get('type')
            b1_id = j_data.get('body1_id')
            b2_id = j_data.get('body2_id')
            
            if b1_id not in self.bodies or b2_id not in self.bodies:
                print(f"Skipping joint {name}: Body IDs not found ({b1_id}, {b2_id})")
                continue
                
            body1 = self.bodies[b1_id]
            body2 = self.bodies[b2_id]
            
            # Location and orientation in World Frame
            frame_data = j_data.get('frame1_world')
            coords = self._get_coordsys_from_frame_data(frame_data)
            
            link = None
            
            # Standard Passive Joint
            if type_ == 'REVOLUTE':
                link = chrono.ChLinkLockRevolute()
            elif type_ == 'PRISMATIC':
                link = chrono.ChLinkLockPrismatic()
            elif type_ == 'SPHERICAL':
                link = chrono.ChLinkLockSpherical()
            elif type_ == 'FIXED':
                link = chrono.ChLinkLockLock()
            else:
                print(f"  Unknown joint type: {type_}")
                continue
                
            link.Initialize(body1, body2, chrono.ChFramed(coords))
            print(f"  Created Joint {name} ({type_})")

            if link:
                link.SetName(name)
                self.system.Add(link)

    def parse_motors(self, joints_data):
        # Filter for motorized joints
        motors = [j for j in joints_data if j.get('motorized', False)]
        print(f"Creating {len(motors)} motors...")
        
        for j_data in motors:
            name = j_data.get('name', 'Motor')
            type_ = j_data.get('type') # REVOLUTE or PRISMATIC usually
            b1_id = j_data.get('body1_id')
            b2_id = j_data.get('body2_id')
            
            if b1_id not in self.bodies or b2_id not in self.bodies:
                print(f"Skipping motor {name}: Body IDs not found")
                continue
                
            body1 = self.bodies[b1_id]
            body2 = self.bodies[b2_id]
            
            frame_data = j_data.get('frame1_world')
            coords = self._get_coordsys_from_frame_data(frame_data)
            frame = chrono.ChFramed(coords)
            
            motor_type = j_data.get('motor_type')
            motor_val = j_data.get('motor_value', 0.0)
            
            link = None
            
            if motor_type == 'VELOCITY':
                # Speed motor
                if type_ == 'REVOLUTE':
                    link = chrono.ChLinkMotorRotationSpeed()
                    link.Initialize(body1, body2, frame)
                    fun = chrono.ChFunctionConst(float(motor_val))
                    link.SetSpeedFunction(fun)
                elif type_ == 'PRISMATIC':
                    link = chrono.ChLinkMotorLinearSpeed()
                    link.Initialize(body1, body2, frame)
                    fun = chrono.ChFunctionConst(float(motor_val))
                    link.SetSpeedFunction(fun)
            elif motor_type == 'POSITION':
                if type_ == 'REVOLUTE':
                    link = chrono.ChLinkMotorRotationAngle()
                    link.Initialize(body1, body2, frame)
                    fun = chrono.ChFunctionConst(float(motor_val))
                    link.SetAngleFunction(fun)
                elif type_ == 'PRISMATIC':
                    link = chrono.ChLinkMotorLinearPosition()
                    link.Initialize(body1, body2, frame)
                    fun = chrono.ChFunctionConst(float(motor_val))
                    link.SetMotionFunction(fun)
            elif motor_type == 'TORQUE':
                if type_ == 'REVOLUTE':
                    link = chrono.ChLinkMotorRotationTorque()
                    link.Initialize(body1, body2, frame)
                    fun = chrono.ChFunctionConst(float(motor_val))
                    link.SetTorqueFunction(fun)
                elif type_ == 'PRISMATIC':
                    link = chrono.ChLinkMotorLinearForce()
                    link.Initialize(body1, body2, frame)
                    fun = chrono.ChFunctionConst(float(motor_val))
                    link.SetForceFunction(fun)
            
            if link:
                print(f"  Created Motor {name} ({motor_type} on {type_})")
                link.SetName(name)
                self.system.Add(link)
            else:
                print(f"  Warning: Could not create motor {name} (Type: {motor_type}, Joint: {type_})")

    def parse_forces(self, forces_data):
        if not forces_data:
            return
            
        print(f"Creating {len(forces_data)} forces...")
        for f_data in forces_data:
            name = f_data.get('name', 'Force')
            body_id = f_data.get('body_id')
            if body_id not in self.bodies:
                print(f"Skipping force {name}: Body ID {body_id} not found")
                continue
            
            body = self.bodies[body_id]
            magnitude = f_data.get('magnitude', 0.0)
            
            # Direction in World Frame
            direction_list = f_data.get('direction', [0,0,1])
            world_dir = self._create_vector_from_list(direction_list)
            world_dir.Normalize()
            
            # Application Point in World Frame
            # Assume frame_world has the origin. If not present, default to COM
            frame_data = f_data.get('frame_world')
            if frame_data and 'origin' in frame_data:
                world_pos = self._create_vector_from_list(frame_data['origin'])
            else:
                world_pos = body.GetPos()
            
            # Create force using ChLoadBodyForce (modern API)
            # Transform world position to body-relative
            local_pos = body.TransformPointParentToLocal(world_pos)
            
            force = chrono.ChForce()
            force.SetMode(chrono.ChForce.FORCE)
            force.SetName(name)
            force.SetVrelpoint(local_pos)
            force.SetDir(world_dir)  # Apply in absolute direction
            force.SetVpoint(world_pos)
            force.SetMforce(magnitude)
            force.SetFrame(chrono.ChForce.WORLD)  # Force direction in world frame
            force.SetAlign(chrono.ChForce.WORLD_DIR)
            
            body.AddForce(force)
            print(f"  Created Force {name} on body {body_id}")

    def parse_torques(self, torques_data):
        if not torques_data:
            return
            
        print(f"Creating {len(torques_data)} torques...")
        for t_data in torques_data:
            name = t_data.get('name', 'Torque')
            body_id = t_data.get('body_id')
            if body_id not in self.bodies:
                print(f"Skipping torque {name}: Body ID {body_id} not found")
                continue
                
            body = self.bodies[body_id]
            magnitude = t_data.get('magnitude', 0.0)
            
            # Axis in World Frame
            axis_list = t_data.get('axis', [0,0,1])
            world_axis = self._create_vector_from_list(axis_list)
            world_axis.Normalize()
            
            torque = chrono.ChForce()
            torque.SetMode(chrono.ChForce.TORQUE)
            torque.SetName(name)
            torque.SetDir(world_axis)
            torque.SetMforce(magnitude)
            torque.SetFrame(chrono.ChForce.WORLD)
            torque.SetAlign(chrono.ChForce.WORLD_DIR)
            
            body.AddForce(torque)
            print(f"  Created Torque {name} on body {body_id}")

    def run_visualization(self):
        # Create the Irrlicht visualization
        vis = chronoirr.ChVisualSystemIrrlicht()
        vis.AttachSystem(self.system)
        vis.SetWindowSize(1024, 768)
        vis.SetWindowTitle('MBD Simulation')
        vis.Initialize()
        vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
        vis.AddSkyBox()
        vis.AddCamera(chrono.ChVector3d(1, 1, 1))
        vis.AddTypicalLights()
        vis.EnableCollisionShapeDrawing(True)
        # vis.EnableBodyFrameDrawing(True)
        
        timestep = 0.01
        
        print("Starting Simulation...")
        while vis.Run():
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            self.system.DoStepDynamics(timestep)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run PyChrono simulation from JSON export")
    parser.add_argument("json_file", help="Path to the JSON file")
    args = parser.parse_args()
    
    if not os.path.exists(args.json_file):
        print(f"Error: File not found: {args.json_file}")
        sys.exit(1)
        
    sim = SimulationParser(args.json_file)
    sim.load_and_create()
    sim.run_visualization()
