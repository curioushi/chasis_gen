from omni.isaac.kit import SimulationApp
import numpy as np

simulation_app = SimulationApp({"headless": False})

import omni
import omni.kit
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.physxcommands import SetRigidBodyCommand, CreateJointCommand
from omni.physxcommands import ApplyAPISchemaCommand
from omni.usd.commands.usd_commands import MovePrimCommand

success = open_stage(usd_path='RangerMini2.usda')
world = World(stage_units_in_meters=1.0)

world.scene.add_default_ground_plane()

stage = omni.usd.get_context().get_stage()

joints = stage.DefinePrim('/RangerMini2/joints', 'Xform')
robot = stage.GetPrimAtPath('/RangerMini2')
chasis = stage.GetPrimAtPath('/RangerMini2/chasis')
steer_left_front = stage.GetPrimAtPath('/RangerMini2/steer_left_front')
steer_left_back = stage.GetPrimAtPath('/RangerMini2/steer_left_back')
steer_right_front = stage.GetPrimAtPath('/RangerMini2/steer_right_front')
steer_right_back = stage.GetPrimAtPath('/RangerMini2/steer_right_back')
wheel_left_front = stage.GetPrimAtPath('/RangerMini2/wheel_left_front')
wheel_left_back = stage.GetPrimAtPath('/RangerMini2/wheel_left_back')
wheel_right_front = stage.GetPrimAtPath('/RangerMini2/wheel_right_front')
wheel_right_back = stage.GetPrimAtPath('/RangerMini2/wheel_right_back')
material = stage.GetPrimAtPath('/_materials')

# setup rigid body and collision
for obj in [chasis, steer_left_front, steer_left_back, steer_right_front, steer_right_back, wheel_left_front, wheel_left_back, wheel_right_front, wheel_right_back]:
    obj_path = obj.GetPath()
    obj_name = obj.GetName()
    SetRigidBodyCommand(obj_path).do()
    ApplyAPISchemaCommand(UsdPhysics.MassAPI, obj).do()
    # mesh = stage.GetPrimAtPath(obj_path.AppendPath(obj_name + '_mesh'))
    # mesh.GetAttribute('physics:approximation').Set('none')

chasis.GetProperty('physics:mass').Set(40)
for steer in [steer_left_front, steer_left_back, steer_right_front, steer_right_back]:
    steer.GetProperty('physics:mass').Set(1)
for wheel in [wheel_left_front, wheel_left_back, wheel_right_front, wheel_right_back]:
    wheel.GetProperty('physics:mass').Set(2)

# steering joints
def create_steer_joint(name, parent, child):
    global stage, joints
    joint = CreateJointCommand(stage, 'Revolute', parent, child).do()
    joint.GetProperty('physics:axis').Set('Y')
    MovePrimCommand(joint.GetPath(), joints.GetPath().AppendPath(name)).do()
    joint = stage.GetPrimAtPath(joints.GetPath().AppendPath(name))
    UsdPhysics.DriveAPI.Apply(joint, UsdPhysics.Tokens.angular)
    joint.GetProperty('drive:angular:physics:type').Set('acceleration')
    joint.GetProperty('drive:angular:physics:stiffness').Set(350)
    joint.GetProperty('drive:angular:physics:damping').Set(20)
    joint.CreateAttribute('drive:angular:physics:targetPosition', Sdf.ValueTypeNames.Float).Set(0)
    PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "angular")
    return joint

joint_steer_left_front = create_steer_joint('joint_steer_left_front', chasis, steer_left_front)
joint_steer_left_back = create_steer_joint('joint_steer_left_back', chasis, steer_left_back)
joint_steer_right_front = create_steer_joint('joint_steer_right_front', chasis, steer_right_front)
joint_steer_right_back = create_steer_joint('joint_steer_right_back', chasis, steer_right_back)

# wheel joints
def create_wheel_joint(name, parent, child):
    global stage, joints
    joint = CreateJointCommand(stage, 'Revolute', parent, child).do()
    joint.GetProperty('physics:axis').Set('Z')
    MovePrimCommand(joint.GetPath(), joints.GetPath().AppendPath(name)).do()
    joint = stage.GetPrimAtPath(joints.GetPath().AppendPath(name))
    UsdPhysics.DriveAPI.Apply(joint, UsdPhysics.Tokens.angular)
    joint.GetProperty('drive:angular:physics:type').Set('acceleration')
    joint.GetProperty('drive:angular:physics:damping').Set(10)
    # joint.CreateAttribute('physxJoint:maxJointVelocity', Sdf.ValueTypeNames.Float).Set(10)
    joint.CreateAttribute('drive:angular:physics:targetVelocity', Sdf.ValueTypeNames.Float).Set(0)
    PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "angular")
    return joint

joint_wheel_left_front = create_wheel_joint('joint_wheel_left_front', steer_left_front, wheel_left_front)
joint_wheel_left_back = create_wheel_joint('joint_wheel_left_back', steer_left_back, wheel_left_back)
joint_wheel_right_front = create_wheel_joint('joint_wheel_right_front', steer_right_front, wheel_right_front)
joint_wheel_right_back = create_wheel_joint('joint_wheel_right_back', steer_right_back, wheel_right_back)

ApplyAPISchemaCommand(UsdPhysics.ArticulationRootAPI, robot).do()
ApplyAPISchemaCommand(PhysxSchema.PhysxArticulationAPI, robot).do()
robot.GetAttribute('physxArticulation:enabledSelfCollisions').Set(False)

MovePrimCommand(material.GetPath(), robot.GetPath().AppendPath('_materials')).do()

# set up physics material
wheel_mat_path = robot.GetPath().AppendPath('wheel_material')
omni.kit.commands.execute("AddRigidBodyMaterialCommand", stage=stage, path=str(wheel_mat_path))
wheel_mat = stage.GetPrimAtPath(wheel_mat_path)
wheel_mat.GetAttribute('physics:dynamicFriction').Set(1)
wheel_mat.GetAttribute('physics:staticFriction').Set(1)

for wheel, wheel_name in zip([wheel_left_front, wheel_left_back, wheel_right_front, wheel_right_back],
                 ['wheel_left_front', 'wheel_left_back', 'wheel_right_front', 'wheel_right_back']):
    wheel_mesh = stage.GetPrimAtPath(wheel.GetPath().AppendPath(wheel_name + '_mesh'))
    rel = wheel_mesh.CreateRelationship('material:binding:physics', False)
    rel.SetTargets([str(wheel_mat_path)])

stage.Export('RangerMini2_physics.usda')

robot.GetAttribute('xformOp:translate').Set((0, 0, 0.5))
# robot.GetAttribute('xformOp:orient').Set(Gf.Quatf(0, 1, 0, 0))

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0

    def step(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class ModDerivative:
    def __init__(self, mod):
        self.last_value = 0
        self.mod = mod

    def step(self, value, dt):
        diff = value - self.last_value
        self.last_value = value
        diff = diff % self.mod
        if diff > self.mod / 2:
            diff -= self.mod
        return diff / dt

wheel_speed_controllers = [PIDController(0, 1, 0) for _ in range(4)]
wheel_speed_measurers = [ModDerivative(360) for _ in range(4)]
global_time = 0

def on_physics_step(step_size):
    global global_time
    global_time += step_size
    target_speed = 360
    wheel_joints = [joint_wheel_left_front, joint_wheel_left_back, joint_wheel_right_front, joint_wheel_right_back]
    for wheel_joint, wheel_speed_controller, wheel_speed_measurer in zip(wheel_joints, wheel_speed_controllers, wheel_speed_measurers):
        encoder_reading = wheel_joint.GetAttribute('state:angular:physics:position').Get()
        measure_speed = wheel_speed_measurer.step(encoder_reading, step_size)
        error = target_speed - measure_speed
        actuator = wheel_speed_controller.step(error, step_size)
        wheel_joint.GetAttribute('drive:angular:physics:targetVelocity').Set(actuator)
    
    steer_joints = [joint_steer_left_front, joint_steer_right_front]
    for steer_joint in steer_joints:
        if (global_time // 2) % 2 == 0:
            steer_joint.GetAttribute('drive:angular:physics:targetPosition').Set(20)
        else:
            steer_joint.GetAttribute('drive:angular:physics:targetPosition').Set(-20)

world.add_physics_callback('controller', callback_fn=on_physics_step)
world.reset()

while simulation_app.is_running():
    world.step(render=True)
