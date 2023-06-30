from omni.isaac.kit import SimulationApp
import numpy as np

simulation_app = SimulationApp({"headless": True})

import argparse
import yaml
import omni
import omni.kit
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Sdf, Gf
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.physxcommands import SetRigidBodyCommand, CreateJointCommand
from omni.physxcommands import ApplyAPISchemaCommand
from omni.usd.commands.usd_commands import MovePrimCommand

parser = argparse.ArgumentParser()
parser.add_argument('input', help='Path to the input file')
parser.add_argument('output', help='Path to the output file')
parser.add_argument('-c', '--config', help='Path to the YAML config file', default='configs/RangerMini2.yaml')
args = parser.parse_args()

with open(args.config) as f:
    config = yaml.load(f, Loader=yaml.FullLoader)
    print('Using config file: ' + args.config)
    chasis_name = config['chasis_name']
    chasis_mass = config['chasis_mass']
    steer_mass = config['steer_mass']
    steer_stiffness = config['steer_stiffness']
    steer_damping = config['steer_damping']
    wheel_mass = config['wheel_mass']
    wheel_damping = config['wheel_damping']
    wheel_friction = config['wheel_friction']

open_stage(usd_path=args.input)
stage = omni.usd.get_context().get_stage()

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

joints = stage.DefinePrim(f'/{chasis_name}/joints', 'Xform')
robot = stage.GetPrimAtPath(f'/{chasis_name}')
chasis = stage.GetPrimAtPath(f'/{chasis_name}/chasis')
steer_left_front = stage.GetPrimAtPath(f'/{chasis_name}/steer_left_front')
steer_left_back = stage.GetPrimAtPath(f'/{chasis_name}/steer_left_back')
steer_right_front = stage.GetPrimAtPath(f'/{chasis_name}/steer_right_front')
steer_right_back = stage.GetPrimAtPath(f'/{chasis_name}/steer_right_back')
wheel_left_front = stage.GetPrimAtPath(f'/{chasis_name}/wheel_left_front')
wheel_left_back = stage.GetPrimAtPath(f'/{chasis_name}/wheel_left_back')
wheel_right_front = stage.GetPrimAtPath(f'/{chasis_name}/wheel_right_front')
wheel_right_back = stage.GetPrimAtPath(f'/{chasis_name}/wheel_right_back')
material = stage.GetPrimAtPath('/_materials')

# setup rigid body and collision
for obj in [chasis, steer_left_front, steer_left_back, steer_right_front, steer_right_back, wheel_left_front, wheel_left_back, wheel_right_front, wheel_right_back]:
    obj_path = obj.GetPath()
    obj_name = obj.GetName()
    SetRigidBodyCommand(obj_path).do()
    ApplyAPISchemaCommand(UsdPhysics.MassAPI, obj).do()
    # mesh = stage.GetPrimAtPath(obj_path.AppendPath(obj_name + '_mesh'))
    # mesh.GetAttribute('physics:approximation').Set('none')

chasis.GetProperty('physics:mass').Set(chasis_mass)
for steer in [steer_left_front, steer_left_back, steer_right_front, steer_right_back]:
    steer.GetProperty('physics:mass').Set(steer_mass)
for wheel in [wheel_left_front, wheel_left_back, wheel_right_front, wheel_right_back]:
    wheel.GetProperty('physics:mass').Set(wheel_mass)

# steering joints
def create_steer_joint(name, parent, child):
    global stage, joints
    joint = CreateJointCommand(stage, 'Revolute', parent, child).do()
    joint.GetProperty('physics:axis').Set('Y')
    MovePrimCommand(joint.GetPath(), joints.GetPath().AppendPath(name)).do()
    joint = stage.GetPrimAtPath(joints.GetPath().AppendPath(name))
    UsdPhysics.DriveAPI.Apply(joint, UsdPhysics.Tokens.angular)
    joint.GetProperty('drive:angular:physics:type').Set('acceleration')
    joint.GetProperty('drive:angular:physics:stiffness').Set(steer_stiffness)
    joint.GetProperty('drive:angular:physics:damping').Set(steer_damping)
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
    joint.GetProperty('drive:angular:physics:damping').Set(wheel_damping)
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
wheel_mat.GetAttribute('physics:dynamicFriction').Set(wheel_friction)
wheel_mat.GetAttribute('physics:staticFriction').Set(wheel_friction)

for wheel, wheel_name in zip([wheel_left_front, wheel_left_back, wheel_right_front, wheel_right_back],
                 ['wheel_left_front', 'wheel_left_back', 'wheel_right_front', 'wheel_right_back']):
    wheel_mesh = stage.GetPrimAtPath(wheel.GetPath().AppendPath(wheel_name + '_mesh'))
    rel = wheel_mesh.CreateRelationship('material:binding:physics', False)
    rel.SetTargets([str(wheel_mat_path)])

stage.Export(args.output)

simulation_app.close()