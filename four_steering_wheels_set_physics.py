from omni.isaac.kit import SimulationApp
import numpy as np

simulation_app = SimulationApp({"headless": False})

import omni
from pxr import Usd, UsdGeom, UsdPhysics
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.physxcommands import SetRigidBodyCommand, CreateJointCommand
from omni.physxcommands import ApplyAPISchemaCommand

success = open_stage(usd_path='RangerMini2.usda')
world = World(stage_units_in_meters=1.0)

world.scene.add_default_ground_plane()

stage = omni.usd.get_context().get_stage()

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

# setup rigid body and collision
for obj in [chasis, wheel_left_front, wheel_left_back, wheel_right_front, wheel_right_back]:
    obj_path = obj.GetPath()
    obj_name = obj.GetName()
    SetRigidBodyCommand(obj_path).do()
    ApplyAPISchemaCommand(UsdPhysics.MassAPI, obj).do()
    mesh = stage.GetPrimAtPath(obj_path.AppendPath(obj_name + '_mesh'))
    mesh.GetAttribute('physics:approximation').Set('none')

joint = CreateJointCommand(stage, 'Revolute', chasis, steer_left_front).do()
joint.GetProperty('physics:axis').Set('Z')
joint = CreateJointCommand(stage, 'Revolute', chasis, steer_left_back).do()
joint.GetProperty('physics:axis').Set('Z')
joint = CreateJointCommand(stage, 'Revolute', chasis, steer_right_front).do()
joint.GetProperty('physics:axis').Set('Z')
joint = CreateJointCommand(stage, 'Revolute', chasis, steer_right_back).do()
joint.GetProperty('physics:axis').Set('Z')

joint = CreateJointCommand(stage, 'Revolute', steer_left_front, wheel_left_front).do()
joint.GetProperty('physics:axis').Set('Z')
joint = CreateJointCommand(stage, 'Revolute', steer_left_back, wheel_left_back).do()
joint.GetProperty('physics:axis').Set('Z')
joint = CreateJointCommand(stage, 'Revolute', steer_right_front, wheel_right_front).do()
joint.GetProperty('physics:axis').Set('Z')
joint = CreateJointCommand(stage, 'Revolute', steer_right_back, wheel_right_back).do()
joint.GetProperty('physics:axis').Set('Z')

stage.Export('RangerMini2_physics.usda')

while True:
    world.step(render=True)
