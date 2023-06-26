import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('-c', '--config', help='Path to the YAML config file', default='configs/RangerMini2.yaml')
parser.add_argument('-o', '--output', help='Path to the output file', default='RangerMini2.usda')
args = parser.parse_args()

# modeling
subprocess.Popen(["blenderproc", "run", "four_steering_wheels_modeling.py", "--", "-o", "temp.usda", "-c", args.config]).wait()

# fix usd problems
subprocess.Popen(["python", "four_steering_wheels_fix.py", "--", "temp.usda", args.output]).wait()

# setup physics
