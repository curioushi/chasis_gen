from omni.isaac.kit import SimulationApp
import numpy as np

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros_bridge")

import argparse
import rospy
from std_msgs.msg import Float64
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
parser.add_argument('-c', '--config', help='Path to the YAML config file', default='configs/RangerMini2.yaml')
args = parser.parse_args()


with open(args.config) as f:
    config = yaml.load(f, Loader=yaml.FullLoader)
    print('Using config file: ' + args.config)
    chasis_name = config['chasis_name']

open_stage(usd_path=args.input)
stage = omni.usd.get_context().get_stage()

world = World(stage_units_in_meters=1.0)

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
joint_steer_left_front = stage.GetPrimAtPath(f'/{chasis_name}/joints/joint_steer_left_front')
joint_steer_left_back = stage.GetPrimAtPath(f'/{chasis_name}/joints/joint_steer_left_back')
joint_steer_right_front = stage.GetPrimAtPath(f'/{chasis_name}/joints/joint_steer_right_front')
joint_steer_right_back = stage.GetPrimAtPath(f'/{chasis_name}/joints/joint_steer_right_back')
joint_wheel_left_front = stage.GetPrimAtPath(f'/{chasis_name}/joints/joint_wheel_left_front')
joint_wheel_left_back = stage.GetPrimAtPath(f'/{chasis_name}/joints/joint_wheel_left_back')
joint_wheel_right_front = stage.GetPrimAtPath(f'/{chasis_name}/joints/joint_wheel_right_front')
joint_wheel_right_back = stage.GetPrimAtPath(f'/{chasis_name}/joints/joint_wheel_right_back')
wheel_joints = [joint_wheel_left_front, joint_wheel_left_back, joint_wheel_right_front, joint_wheel_right_back]
steer_joints = [joint_steer_left_front, joint_steer_left_back, joint_steer_right_front, joint_steer_right_back]

robot.GetAttribute('xformOp:translate').Set(Gf.Vec3d(0, 0, 0.5))
# robot.GetAttribute('xformOp:orient').Set(Gf.Quatf(0, 1, 0, 0))

def clamp(x, min, max):
    return min if x < min else max if x > max else x

class PIDController:
    def __init__(self, kp, ki, kd, min, max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
        self.min = min
        self.max = max

    def step(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return clamp(self.kp * error + self.ki * self.integral + self.kd * derivative, self.min, self.max)

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

class SpeedLimiter:
    def __init__(self, max_speed):
        self.max_speed = max_speed

    def step(self, current, target, dt):
        diff = target - current
        if diff > self.max_speed * dt:
            return current + self.max_speed * dt
        elif diff < -self.max_speed * dt:
            return current - self.max_speed * dt
        else:
            return target


def square_wave(t, min, max, duration):
    t = t % duration
    if t < duration / 2:
        return min
    else:
        return max

def triangle_wave(t, min, max, duration):
    t = t % duration
    if t < duration / 2:
        return min + (max - min) / (duration / 2) * t
    else:
        return max - (max - min) / (duration / 2) * (t - duration / 2)

def sin_wave(t, min, max, duration):
    t = t % duration
    return min + (max - min) / 2 * (1 + np.sin(2 * np.pi * t / duration))

wheel_speed_controllers = [PIDController(0.75, 1.25, 0, -1440, 1440) for _ in range(4)]
wheel_speed_measurers = [ModDerivative(360) for _ in range(4)]
steer_position_controllers = [SpeedLimiter(400) for _ in range(4)]
global_time = 0

rospy.init_node('controller', anonymous=True)
wheel_speed_pubs = [rospy.Publisher('wheel_left_front_speed', Float64, queue_size=10),
                    rospy.Publisher('wheel_left_back_speed', Float64, queue_size=10),
                    rospy.Publisher('wheel_right_front_speed', Float64, queue_size=10),
                    rospy.Publisher('wheel_right_back_speed', Float64, queue_size=10)]

wheel_target_speed_pubs = [rospy.Publisher('wheel_left_front_target_speed', Float64, queue_size=10),
                            rospy.Publisher('wheel_left_back_target_speed', Float64, queue_size=10),
                            rospy.Publisher('wheel_right_front_target_speed', Float64, queue_size=10),
                            rospy.Publisher('wheel_right_back_target_speed', Float64, queue_size=10)]

steer_position_pubs = [rospy.Publisher('steer_left_front_position', Float64, queue_size=10),
                        rospy.Publisher('steer_left_back_position', Float64, queue_size=10),
                        rospy.Publisher('steer_right_front_position', Float64, queue_size=10),
                        rospy.Publisher('steer_right_back_position', Float64, queue_size=10)]

steer_target_position_pubs = [rospy.Publisher('steer_left_front_target_position', Float64, queue_size=10),
                                rospy.Publisher('steer_left_back_target_position', Float64, queue_size=10),
                                rospy.Publisher('steer_right_front_target_position', Float64, queue_size=10),
                                rospy.Publisher('steer_right_back_target_position', Float64, queue_size=10)]


def on_physics_step(step_size):
    global global_time
    global_time += step_size
    target_speed = square_wave(global_time, -860, 860, 20)
    target_position = triangle_wave(global_time, -45, 45, 2)

    for wheel_joint, wheel_speed_controller, wheel_speed_measurer, wheel_speed_pub, wheel_target_speed_pub \
          in zip(wheel_joints, wheel_speed_controllers, wheel_speed_measurers, wheel_speed_pubs, wheel_target_speed_pubs):
        encoder_reading = wheel_joint.GetAttribute('state:angular:physics:position').Get()
        measure_speed = wheel_speed_measurer.step(encoder_reading, step_size)
        error = target_speed - measure_speed
        actuator = wheel_speed_controller.step(error, step_size)
        wheel_joint.GetAttribute('drive:angular:physics:targetVelocity').Set(actuator)
        wheel_speed_pub.publish(measure_speed)
        wheel_target_speed_pub.publish(target_speed)
    
    for steer_joint, steer_position_controller, steer_position_pub, steer_target_position_pub \
            in zip(steer_joints, steer_position_controllers, steer_position_pubs, steer_target_position_pubs):
        encoder_reading = steer_joint.GetAttribute('state:angular:physics:position').Get()
        actuator = steer_position_controller.step(encoder_reading, target_position, step_size)
        steer_joint.GetAttribute('drive:angular:physics:targetPosition').Set(actuator)
        steer_position_pub.publish(encoder_reading)
        steer_target_position_pub.publish(target_position)
    
    

world.add_physics_callback('controller', callback_fn=on_physics_step)
world.reset()

while simulation_app.is_running():
    world.step(render=True)
