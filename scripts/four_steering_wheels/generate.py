import os
import shutil
import yaml
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('-c', '--config', help='Path to the YAML config file', default='configs/RangerMini2.yaml')
parser.add_argument('-p', '--python_isaac', help='Path to the python_isaac executable', default='/home/shq/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh')
args = parser.parse_args()

with open(args.config) as f:
    config = yaml.load(f, Loader=yaml.FullLoader)
    chasis_name = config['chasis_name']

output_dir = os.path.normpath(os.path.abspath(f'output/{chasis_name}'))
shutil.rmtree(output_dir, ignore_errors=True)
os.makedirs(output_dir, exist_ok=True)
temp_usd = os.path.join(output_dir, 'temp.usda')
output_usd = os.path.join(output_dir, f'{chasis_name}.usda')

# modeling
print('Modeling...')
subprocess.Popen(["blenderproc", "run", "scripts/four_steering_wheels/modeling.py", "--", "-o", temp_usd, "-c", args.config]).wait()
print('Modeling done')

# fix usd problems
print('Fixing usd problems...')
subprocess.Popen(["python", "scripts/fix_usd.py", temp_usd, temp_usd]).wait()
print('Fixing usd problems done')

# setup physics
print('Setting up physics...')
subprocess.Popen([args.python_isaac, "scripts/four_steering_wheels/set_physics.py", temp_usd, output_usd, "-c", args.config]).wait()
print('Setting up physics done')

# clean up
os.remove(temp_usd)