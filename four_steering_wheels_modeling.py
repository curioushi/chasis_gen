import blenderproc as bproc
import sys
import bpy
import math
import yaml
import cv2
import numpy as np
import argparse

def remove_all_cameras():
    """Remove all camera objects"""
    for cam in bpy.data.cameras:
        bpy.data.cameras.remove(cam)

def boolean_modify(obj1, obj2, operation='DIFFERENCE'):
    """Add a boolean modifier to obj1 to cut out obj2"""
    bpy.ops.object.select_all(action='DESELECT')
    obj1.select_set(True)
    bpy.context.view_layer.objects.active = obj1
    bpy.ops.object.modifier_add(type='BOOLEAN')
    bpy.context.object.modifiers["Boolean"].operation = operation
    bpy.context.object.modifiers["Boolean"].object = obj2
    bpy.ops.object.modifier_apply(modifier="Boolean")

def triangulate(obj):
    """Triangulate the mesh of an object"""
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.quads_convert_to_tris(quad_method='BEAUTY', ngon_method='BEAUTY')
    bpy.ops.object.editmode_toggle()

def bevel(obj, offset):
    """Bevel the edges of an object"""
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.ops.object.editmode_toggle()
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.bevel(offset=offset, offset_pct=0, affect='EDGES')
    bpy.ops.object.editmode_toggle()


def create_wheel(name, diameter, thickness, location, padding=0.01):
    """Create a wheel object"""
    wheel = bproc.object.create_primitive('CYLINDER', vertices=360, radius=diameter/2, depth=thickness, location=location)
    bpy.ops.transform.rotate(value=math.pi / 2, orient_axis='X')
    wheel.blender_obj.name = name
    wheel.blender_obj.data.name = name + '_mesh'
    wheel_bound = bproc.object.create_primitive('CYLINDER', vertices=360, radius=padding+math.hypot(diameter/2, thickness/2), depth=padding+diameter, location=location)
    return wheel, wheel_bound

def create_axis(name, location):
    bpy.ops.object.empty_add(type='PLAIN_AXES', align='WORLD', location=location, scale=(1, 1, 1))
    axis = bpy.context.selected_objects[0]
    axis.name = name
    return axis

def create_checker_texture(filepath, color1=(0, 0, 0), color2=(1, 1, 1), size=5, grid_size=64):
    tex = np.zeros((grid_size*size, grid_size*size, 3), dtype=float)
    for i in range(size):
        for j in range(size):
            tex[i*grid_size:(i+1)*grid_size, j*grid_size:(j+1)*grid_size] = color1 if (i+j) % 2 == 0 else color2
    tex = (tex * 255).astype(np.uint8)
    cv2.imwrite(filepath, tex)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output', help='Path to the output file', default='')
    parser.add_argument('-c', '--config', help='Path to the YAML config file', default='')
    args = parser.parse_args()

    if args.output == '':
        print('Please specify the output file: -o, --output')
        sys.exit(1)

    if args.config == '':
        print('No config file specified, using default values')
        chasis_name = 'chasis_name'
        wheel_diameter = 0.200 # m
        wheel_thickness = 0.060 # m
        wheel_base_x = 0.490 # m
        wheel_base_y = 0.370 # m
        wheel_bevel = 0.01 # m
        chasis_width = 0.5 # m
        chasis_length = 0.738 # m
        chasis_height = 0.140 # m
    else:
        with open(args.config) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            print('Using config file: ' + args.config)
            chasis_name = config['chasis_name']
            wheel_diameter = config['wheel_diameter']
            wheel_thickness = config['wheel_thickness']
            wheel_base_x = config['wheel_base_x']
            wheel_base_y = config['wheel_base_y']
            wheel_bevel = config['wheel_bevel']
            chasis_width = config['chasis_width']
            chasis_length = config['chasis_length']
            chasis_height = config['chasis_height']

    print('-' * 80)
    print('chasis_name: ' + chasis_name)
    print('wheel_diameter: ' + str(wheel_diameter))
    print('wheel_thickness: ' + str(wheel_thickness))
    print('wheel_base_x: ' + str(wheel_base_x))
    print('wheel_base_y: ' + str(wheel_base_y))
    print('wheel_bevel: ' + str(wheel_bevel))
    print('chasis_width: ' + str(chasis_width))
    print('chasis_length: ' + str(chasis_length))
    print('chasis_height: ' + str(chasis_height))
    print('-' * 80)

    bproc.init()
    bproc.clean_up(True)
    remove_all_cameras()

    chasis_axis = create_axis(chasis_name, (0, 0, 0))
    chasis = bproc.object.create_primitive('CUBE', scale = (chasis_length / 2, chasis_width/2, chasis_height/2), location = (0, 0, wheel_diameter + chasis_height / 2))
    chasis.blender_obj.name = 'chasis'
    chasis.blender_obj.data.name = 'chasis_mesh'
    steer1, steer1_bound = create_wheel('steer_left_front', wheel_diameter / 2, wheel_thickness * 1.2, (wheel_base_x / 2, wheel_base_y / 2, wheel_diameter / 2))
    steer2, steer2_bound = create_wheel('steer_left_back', wheel_diameter / 2, wheel_thickness * 1.2, (-wheel_base_x / 2, wheel_base_y / 2, wheel_diameter / 2))
    steer3, steer3_bound = create_wheel('steer_right_back', wheel_diameter / 2, wheel_thickness * 1.2, (-wheel_base_x / 2, -wheel_base_y / 2, wheel_diameter / 2))
    steer4, steer4_bound = create_wheel('steer_right_front', wheel_diameter / 2, wheel_thickness * 1.2, (wheel_base_x / 2, -wheel_base_y / 2, wheel_diameter / 2))
    wheel1, wheel1_bound = create_wheel('wheel_left_front', wheel_diameter, wheel_thickness, (wheel_base_x / 2, wheel_base_y / 2, wheel_diameter / 2))
    wheel2, wheel2_bound = create_wheel('wheel_left_back', wheel_diameter, wheel_thickness, (-wheel_base_x / 2, wheel_base_y / 2, wheel_diameter / 2))
    wheel3, wheel3_bound = create_wheel('wheel_right_back', wheel_diameter, wheel_thickness, (-wheel_base_x / 2, -wheel_base_y / 2, wheel_diameter / 2))
    wheel4, wheel4_bound = create_wheel('wheel_right_front', wheel_diameter, wheel_thickness, (wheel_base_x / 2, -wheel_base_y / 2, wheel_diameter / 2))

    create_checker_texture('/tmp/chasis_texture.png', (205/255, 117/255, 149/255), (0, 0, 0), 20, 64)
    chasis_mat = bproc.material.create_material_from_texture('/tmp/chasis_texture.png', 'chasis_mat')
    create_checker_texture('/tmp/wheel_texture.png', (182/255, 172/255, 77/255), (0, 0, 0), 10, 64)
    wheel_mat = bproc.material.create_material_from_texture('/tmp/wheel_texture.png', 'wheel_mat')
    create_checker_texture('/tmp/steer_texture.png', (101/255, 138/255, 255/255), (0, 0, 0), 4, 64)
    steer_mat = bproc.material.create_material_from_texture('/tmp/steer_texture.png', 'steer_mat')
    chasis.add_material(chasis_mat)
    wheel1.add_material(wheel_mat)
    wheel2.add_material(wheel_mat)
    wheel3.add_material(wheel_mat)
    wheel4.add_material(wheel_mat)
    steer1.add_material(steer_mat)
    steer2.add_material(steer_mat)
    steer3.add_material(steer_mat)
    steer4.add_material(steer_mat)

    bevel(wheel1.blender_obj, wheel_bevel)
    bevel(wheel2.blender_obj, wheel_bevel)
    bevel(wheel3.blender_obj, wheel_bevel)
    bevel(wheel4.blender_obj, wheel_bevel)

    # unknown bug: UNION <-> DIFFERENCE
    boolean_modify(wheel1.blender_obj, steer1.blender_obj, 'UNION')
    boolean_modify(wheel2.blender_obj, steer2.blender_obj, 'UNION')
    boolean_modify(wheel3.blender_obj, steer3.blender_obj, 'UNION')
    boolean_modify(wheel4.blender_obj, steer4.blender_obj, 'UNION')

    steer1_bound.delete(True)
    steer2_bound.delete(True)
    steer3_bound.delete(True)
    steer4_bound.delete(True)
    wheel1_bound.delete(True)
    wheel2_bound.delete(True)
    wheel3_bound.delete(True)
    wheel4_bound.delete(True)

    triangulate(chasis.blender_obj)
    triangulate(wheel1.blender_obj)
    triangulate(wheel2.blender_obj)
    triangulate(wheel3.blender_obj)
    triangulate(wheel4.blender_obj)

    chasis.blender_obj.parent = chasis_axis
    wheel1.blender_obj.parent = chasis_axis
    wheel2.blender_obj.parent = chasis_axis
    wheel3.blender_obj.parent = chasis_axis
    wheel4.blender_obj.parent = chasis_axis
    steer1.blender_obj.parent = chasis_axis
    steer2.blender_obj.parent = chasis_axis
    steer3.blender_obj.parent = chasis_axis
    steer4.blender_obj.parent = chasis_axis

    bpy.ops.wm.usd_export(filepath=args.output)

if __name__ == '__main__':
    main()