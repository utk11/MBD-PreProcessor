#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;

int main(int argc, char* argv[]) {
    // Create the physical system
    ChSystemNSC sys;
    sys.SetGravitationalAcceleration(ChVector3d(0,-9.81,0));

    // Set collision system to BULLET (MUST be set BEFORE adding bodies)
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Set solver to PSOR with 500 iterations
    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.GetSolver()->AsIterative()->SetMaxIterations(500);

    // Set collision envelope and margin for robust contact detection
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0001);

    // Create a shared contact material
    auto contact_material = chrono_types::make_shared<ChContactMaterialNSC>();
    contact_material->SetFriction(0.5f);
    contact_material->SetRestitution(0.0f);

    // Material density (steel)
    double density = 7850.0; // kg/m^3

    // ============= GROUND BODY (Fixed) =============
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->SetName("Ground");
    sys.Add(ground);

    // ============= BODY 0 =============
    auto body_0 = chrono_types::make_shared<ChBody>();
    body_0->SetName("Body_0");

    double volume_0 = 1.5399397375184605e-06;
    double mass_0 = volume_0 * density;
    body_0->SetMass(mass_0);

    ChVector3d com_0(-0.01915120645210125, 0.0898881411885598, 0.0015000000000000002);
    body_0->SetPos(com_0);

    ChMatrix33<> inertia_0;
    inertia_0(0, 0) = 4.3598877674019536e-10;
    inertia_0(0, 1) = -6.278790346079749e-11;
    inertia_0(0, 2) = 1.2732925824820996e-26;
    inertia_0(1, 0) = -6.278790346079749e-11;
    inertia_0(1, 1) = 2.483004590366861e-11;
    inertia_0(1, 2) = 5.820766091346741e-26;
    inertia_0(2, 0) = 1.2732925824820996e-26;
    inertia_0(2, 1) = 5.820766091346741e-26;
    inertia_0(2, 2) = 4.585089130375865e-10;
    inertia_0 *= density;
    body_0->SetInertia(inertia_0);

    // UPDATED: Mesh vertices are now in BODY-LOCAL frame, so NO offset needed
    auto mesh_0 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_0_0.obj", false, true);
    auto mesh_shape_0 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape_0->SetMesh(mesh_0);
    mesh_shape_0->SetName("Body_0_Mesh");
    body_0->AddVisualShape(mesh_shape_0, ChFrame<>(VNULL, QUNIT));

    sys.Add(body_0);

    // ============= BODY 1 =============
    auto body_1 = chrono_types::make_shared<ChBody>();
    body_1->SetName("Body_1");

    double volume_1 = 4.064810985344584e-07;
    double mass_1 = volume_1 * density;
    body_1->SetMass(mass_1);

    ChVector3d com_1(-0.010848374273086617, 0.07244971178210596, 0.004500000000000048);
    body_1->SetPos(com_1);

    ChMatrix33<> inertia_1;
    inertia_1(0, 0) = 4.962968599753999e-11;
    inertia_1(0, 1) = 1.8815927281657833e-11;
    inertia_1(0, 2) = -4.5474735088646415e-28;
    inertia_1(1, 0) = 1.8815927281657833e-11;
    inertia_1(1, 1) = 8.456117135611476e-12;
    inertia_1(1, 2) = -1.8189894035458566e-27;
    inertia_1(2, 0) = -4.5474735088646415e-28;
    inertia_1(2, 1) = -1.8189894035458566e-27;
    inertia_1(2, 2) = 5.747608148534975e-11;
    inertia_1 *= density;
    body_1->SetInertia(inertia_1);

    auto mesh_1 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_1_1.obj", false, true);
    auto mesh_shape_1 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape_1->SetMesh(mesh_1);
    mesh_shape_1->SetName("Body_1_Mesh");
    body_1->AddVisualShape(mesh_shape_1, ChFrame<>(VNULL, QUNIT));

    sys.Add(body_1);

    // ============= BODY 2 =============
    auto body_2 = chrono_types::make_shared<ChBody>();
    body_2->SetName("Body_2");

    double volume_2 = 2.1433209318342035e-06;
    double mass_2 = volume_2 * density;
    body_2->SetMass(mass_2);

    ChVector3d com_2(0.016014721911644964, 0.04218430478670574, 0.0045000000000000534);
    body_2->SetPos(com_2);

    ChMatrix33<> inertia_2;
    inertia_2(0, 0) = 2.492652623070956e-10;
    inertia_2(0, 1) = -9.209816075267251e-11;
    inertia_2(0, 2) = 1.4915713109076024e-25;
    inertia_2(1, 0) = -9.209816075267251e-11;
    inertia_2(1, 1) = 1.3192803841293825e-10;
    inertia_2(1, 2) = 2.7102942112833265e-25;
    inertia_2(2, 0) = 1.4915713109076024e-25;
    inertia_2(2, 1) = 2.7102942112833265e-25;
    inertia_2(2, 2) = 3.7797793186214796e-10;
    inertia_2 *= density;
    body_2->SetInertia(inertia_2);

    auto mesh_2 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_2_2.obj", false, true);
    auto mesh_shape_2 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape_2->SetMesh(mesh_2);
    mesh_shape_2->SetName("Body_2_Mesh");
    body_2->AddVisualShape(mesh_shape_2, ChFrame<>(VNULL, QUNIT));

    // Enable collision - mesh is already in local frame, no transformation needed
    body_2->EnableCollision(true);
    auto collision_mesh_2 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_2_2.obj", false, true);
    auto collision_shape_2 = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
        contact_material, collision_mesh_2, false, false);
    body_2->AddCollisionShape(collision_shape_2, ChFrame<>(VNULL, QUNIT));

    sys.Add(body_2);

    // ============= BODY 3 (Base - Fixed to Ground) =============
    auto body_3 = chrono_types::make_shared<ChBody>();
    body_3->SetName("Body_3");

    double volume_3 = 3.955261456849713e-06;
    double mass_3 = volume_3 * density;
    body_3->SetMass(mass_3);

    ChVector3d com_3(-2.40939754652445e-05, 0.031943609382802676, 0.0015000000000000002);
    body_3->SetPos(com_3);

    ChMatrix33<> inertia_3;
    inertia_3(0, 0) = 1.3329774312480623e-09;
    inertia_3(0, 1) = -4.458917120351132e-12;
    inertia_3(0, 2) = -5.542233338928782e-27;
    inertia_3(1, 0) = -4.458917120351132e-12;
    inertia_3(1, 1) = 6.318292122951644e-10;
    inertia_3(1, 2) = 1.4551915228366854e-25;
    inertia_3(2, 0) = -5.542233338928782e-27;
    inertia_3(2, 1) = 1.4551915228366854e-25;
    inertia_3(2, 2) = 1.958850084122062e-09;
    inertia_3 *= density;
    body_3->SetInertia(inertia_3);

    auto mesh_3 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_3_3.obj", false, true);
    auto mesh_shape_3 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape_3->SetMesh(mesh_3);
    mesh_shape_3->SetName("Body_3_Mesh");
    body_3->AddVisualShape(mesh_shape_3, ChFrame<>(VNULL, QUNIT));

    sys.Add(body_3);

    // ============= BODY 4 =============
    auto body_4 = chrono_types::make_shared<ChBody>();
    body_4->SetName("Body_4");

    double volume_4 = 2.1433209318342026e-06;
    double mass_4 = volume_4 * density;
    body_4->SetMass(mass_4);

    ChVector3d com_4(-0.01538917678827482, 0.04247550225910225, 0.0044999999999999684);
    body_4->SetPos(com_4);

    ChMatrix33<> inertia_4;
    inertia_4(0, 0) = 2.7127290442741074e-10;
    inertia_4(0, 1) = 7.358953234988835e-11;
    inertia_4(0, 2) = 9.822542779147626e-26;
    inertia_4(1, 0) = 7.358953234988835e-11;
    inertia_4(1, 1) = 1.0992039629262273e-10;
    inertia_4(1, 2) = -2.2009771782904866e-25;
    inertia_4(2, 0) = 9.822542779147626e-26;
    inertia_4(2, 1) = -2.2009771782904866e-25;
    inertia_4(2, 2) = 3.7797793186214745e-10;
    inertia_4 *= density;
    body_4->SetInertia(inertia_4);

    auto mesh_4 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_4_4.obj", false, true);
    auto mesh_shape_4 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape_4->SetMesh(mesh_4);
    mesh_shape_4->SetName("Body_4_Mesh");
    body_4->AddVisualShape(mesh_shape_4, ChFrame<>(VNULL, QUNIT));

    // Enable collision
    body_4->EnableCollision(true);
    auto collision_mesh_4 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_4_4.obj", false, true);
    auto collision_shape_4 = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
        contact_material, collision_mesh_4, false, false);
    body_4->AddCollisionShape(collision_shape_4, ChFrame<>(VNULL, QUNIT));

    sys.Add(body_4);

    // ============= BODY 5 =============
    auto body_5 = chrono_types::make_shared<ChBody>();
    body_5->SetName("Body_5");

    double volume_5 = 1.5399397375184603e-06;
    double mass_5 = volume_5 * density;
    body_5->SetMass(mass_5);

    ChVector3d com_5(0.022857057173923216, 0.08816060482900764, 0.0014999999999999916);
    body_5->SetPos(com_5);

    ChMatrix33<> inertia_5;
    inertia_5(0, 0) = 4.35975835785619e-10;
    inertia_5(0, 1) = 6.283025895214332e-11;
    inertia_5(0, 2) = -9.094947017729284e-27;
    inertia_5(1, 0) = 6.283025895214332e-11;
    inertia_5(1, 1) = 2.4842986858244608e-11;
    inertia_5(1, 2) = 8.003553375601769e-26;
    inertia_5(2, 0) = -9.094947017729284e-27;
    inertia_5(2, 1) = 8.003553375601769e-26;
    inertia_5(2, 2) = 4.585089130375864e-10;
    inertia_5 *= density;
    body_5->SetInertia(inertia_5);

    auto mesh_5 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_5_5.obj", false, true);
    auto mesh_shape_5 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape_5->SetMesh(mesh_5);
    mesh_shape_5->SetName("Body_5_Mesh");
    body_5->AddVisualShape(mesh_shape_5, ChFrame<>(VNULL, QUNIT));

    sys.Add(body_5);

    // ============= BODY 6 =============
    auto body_6 = chrono_types::make_shared<ChBody>();
    body_6->SetName("Body_6");

    double volume_6 = 4.064810985344583e-07;
    double mass_6 = volume_6 * density;
    body_6->SetMass(mass_6);

    ChVector3d com_6(0.01270145358727158, 0.07158580483021891, 0.004500000000000028);
    body_6->SetPos(com_6);

    ChMatrix33<> inertia_6;
    inertia_6(0, 0) = 4.4006384062498056e-11;
    inertia_6(0, 1) = -2.353612126646734e-11;
    inertia_6(0, 2) = 2.0463630789890888e-27;
    inertia_6(1, 0) = -2.353612126646734e-11;
    inertia_6(1, 1) = 1.4079419070653497e-11;
    inertia_6(1, 2) = 3.637978807091713e-27;
    inertia_6(2, 0) = 2.0463630789890888e-27;
    inertia_6(2, 1) = 3.637978807091713e-27;
    inertia_6(2, 2) = 5.747608148534978e-11;
    inertia_6 *= density;
    body_6->SetInertia(inertia_6);

    auto mesh_6 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_6_6.obj", false, true);
    auto mesh_shape_6 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape_6->SetMesh(mesh_6);
    mesh_shape_6->SetName("Body_6_Mesh");
    body_6->AddVisualShape(mesh_shape_6, ChFrame<>(VNULL, QUNIT));

    sys.Add(body_6);

    // ============= BODY 19 =============
    auto body_19 = chrono_types::make_shared<ChBody>();
    body_19->SetName("Body_19");

    double volume_19 = 1.0560530685032682e-06;
    double mass_19 = volume_19 * density;
    body_19->SetMass(mass_19);

    ChVector3d com_19(0.0075000003680338965, 0.01759971820688802, 0.0026633790479687106);
    body_19->SetPos(com_19);

    ChMatrix33<> inertia_19;
    inertia_19(0, 0) = 2.031614041546674e-11;
    inertia_19(0, 1) = -2.916324164543851e-20;
    inertia_19(0, 2) = -7.633507050416725e-19;
    inertia_19(1, 0) = -2.916324164543851e-20;
    inertia_19(1, 1) = 2.0316140328813944e-11;
    inertia_19(1, 2) = -4.726928720978248e-18;
    inertia_19(2, 0) = -7.633507050416725e-19;
    inertia_19(2, 1) = -4.726928720978248e-18;
    inertia_19(2, 2) = 3.4207523027220655e-11;
    inertia_19 *= density;
    body_19->SetInertia(inertia_19);

    auto mesh_19 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_19_19.obj", false, true);
    auto mesh_shape_19 = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    mesh_shape_19->SetMesh(mesh_19);
    mesh_shape_19->SetName("Body_19_Mesh");
    body_19->AddVisualShape(mesh_shape_19, ChFrame<>(VNULL, QUNIT));

    // Enable collision
    body_19->EnableCollision(true);
    auto collision_mesh_19 = ChTriangleMeshConnected::CreateFromWavefrontFile("meshes/Body_19_19.obj", false, true);
    auto collision_shape_19 = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
        contact_material, collision_mesh_19, false, false);
    body_19->AddCollisionShape(collision_shape_19, ChFrame<>(VNULL, QUNIT));

    sys.Add(body_19);

    // ============= JOINTS =============

    // Joint 1: FIXED joint between Ground and Body_3
    ChVector3d joint_fixed_pos(-2.40939754652445e-05, 0.031943609382802676, 0.0015000000000000002);
    ChQuaternion<> joint_fixed_quat(1, 0, 0, 0);

    auto joint_fixed = chrono_types::make_shared<ChLinkMateFix>();
    joint_fixed->Initialize(ground, body_3, ChFrame<>(joint_fixed_pos, joint_fixed_quat));
    sys.Add(joint_fixed);
    
    // Joint 2: REVOLUTE (MOTORIZED) between Body_3 and Body_19
    ChVector3d joint_motor_pos(0.007500000000000009, 0.017599715906798, 0.0015);
    ChQuaternion<> joint_motor_quat(1, 0, 0, 0);

    auto motor_19 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor_19->Initialize(body_3, body_19, ChFrame<>(joint_motor_pos, joint_motor_quat));
    auto angle_function = chrono_types::make_shared<ChFunctionRamp>(0.0, CH_PI);
    motor_19->SetAngleFunction(angle_function);
    sys.Add(motor_19);

    // Joint 3: REVOLUTE between Body_3 and Body_4
    ChVector3d joint_rev2_pos(-0.0135, 0.037599715906798, 0.0015000000000000002);
    ChQuaternion<> joint_rev2_quat(1, 0, 0, 0);

    auto joint_rev2 = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rev2->Initialize(body_3, body_4, ChFrame<>(joint_rev2_pos, joint_rev2_quat));
    sys.Add(joint_rev2);

    // Joint 4: REVOLUTE between Body_3 and Body_1
    ChVector3d joint_rev3_pos(-0.00523982460190923, 0.058, 0.0015000000000000002);
    ChQuaternion<> joint_rev3_quat(1, 0, 0, 0);

    auto joint_rev3 = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rev3->Initialize(body_3, body_1, ChFrame<>(joint_rev3_pos, joint_rev3_quat));
    sys.Add(joint_rev3);

    // Joint 5: REVOLUTE between Body_3 and Body_6
    ChVector3d joint_rev4_pos(0.00523982460190923, 0.058, 0.0015000000000000002);
    ChQuaternion<> joint_rev4_quat(1, 0, 0, 0);

    auto joint_rev4 = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rev4->Initialize(body_3, body_6, ChFrame<>(joint_rev4_pos, joint_rev4_quat));
    sys.Add(joint_rev4);

    // Joint 6: REVOLUTE between Body_3 and Body_2
    ChVector3d joint_rev5_pos(0.0135, 0.037599715906798, 0.0015000000000000002);
    ChQuaternion<> joint_rev5_quat(1, 0, 0, 0);

    auto joint_rev5 = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rev5->Initialize(body_3, body_2, ChFrame<>(joint_rev5_pos, joint_rev5_quat));
    sys.Add(joint_rev5);

    // Joint 7: REVOLUTE between Body_0 and Body_4
    ChVector3d joint_rev6_pos(-0.0247046632450216, 0.0665039633726223, 0.0015000000000000035);
    ChQuaternion<> joint_rev6_quat(1, 0, 0, 0);

    auto joint_rev6 = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rev6->Initialize(body_0, body_4, ChFrame<>(joint_rev6_pos, joint_rev6_quat));
    sys.Add(joint_rev6);

    // Joint 8: REVOLUTE between Body_0 and Body_1
    ChVector3d joint_rev7_pos(-0.016456923944263912, 0.08689942356421242, 0.0015);
    ChQuaternion<> joint_rev7_quat(1, 0, 0, 0);

    auto joint_rev7 = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rev7->Initialize(body_0, body_1, ChFrame<>(joint_rev7_pos, joint_rev7_quat));
    sys.Add(joint_rev7);

    // Joint 9: REVOLUTE between Body_2 and Body_5
    ChVector3d joint_rev8_pos(0.0284129229328218, 0.06477699924416859, 0.0014999999999999953);
    ChQuaternion<> joint_rev8_quat(1, 0, 0, 0);

    auto joint_rev8 = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rev8->Initialize(body_2, body_5, ChFrame<>(joint_rev8_pos, joint_rev8_quat));
    sys.Add(joint_rev8);

    // Joint 10: REVOLUTE between Body_5 and Body_6
    ChVector3d joint_rev9_pos(0.02016308257263391, 0.08517160966043825, 0.0014999999999999916);
    ChQuaternion<> joint_rev9_quat(1, 0, 0, 0);

    auto joint_rev9 = chrono_types::make_shared<ChLinkLockRevolute>();
    joint_rev9->Initialize(body_5, body_6, ChFrame<>(joint_rev9_pos, joint_rev9_quat));
    sys.Add(joint_rev9);

    // ============= VISUALIZATION =============
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("Multi-Body Linkage Mechanism");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(0, 0, 0.3));
    vis->AddTypicalLights();

    // ============= SIMULATION LOOP =============
    double time_step = 0.001;
    double end_time = 1.0;

    while (vis->Run() && sys.GetChTime() < end_time) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();
        sys.DoStepDynamics(time_step);

        // Contact debugging output
        if (sys.GetNumContacts() > 0) {
            std::cout << "Time: " << sys.GetChTime()
                << " - Contacts: " << sys.GetNumContacts() << std::endl;
        }
    }

    return 0;
}