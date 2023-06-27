from omni.isaac.kit import SimulationApp
import numpy as np

simulation_app = SimulationApp({"headless": False})

import omni
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf
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
    mesh = stage.GetPrimAtPath(obj_path.AppendPath(obj_name + '_mesh'))
    mesh.GetAttribute('physics:approximation').Set('none')

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
    joint.GetProperty('drive:angular:physics:damping').Set(100)
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
    joint.GetProperty('drive:angular:physics:damping').Set(1)
    joint.GetProperty('drive:angular:physics:stiffness').Set(20)
    joint.CreateAttribute('physxJoint:maxJointVelocity', Sdf.ValueTypeNames.Float).Set(10)
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

stage.Export('RangerMini2_physics.usda')

while True:
    world.step(render=True)
