import blenderproc as bproc
import sys
import bpy
import math
import yaml
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


def create_wheel(name, diameter, thickness, location, padding=0.01):
    """Create a wheel object"""
    wheel = bproc.object.create_primitive('CYLINDER', vertices=360, radius=diameter/2, depth=thickness, location=location)
    bpy.ops.transform.rotate(value=math.pi / 2, orient_axis='X')
    wheel.blender_obj.name = name
    wheel.blender_obj.data.name = name + '_mesh'
    wheel_bound = bproc.object.create_primitive('CYLINDER', vertices=360, radius=padding+math.hypot(diameter/2, thickness/2), depth=diameter, location=location)
    return wheel, wheel_bound

def create_axis(name, location):
    bpy.ops.object.empty_add(type='PLAIN_AXES', align='WORLD', location=location, scale=(1, 1, 1))
    axis = bpy.context.selected_objects[0]
    axis.name = name
    return axis


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
        chasis_width = 0.5 # m
        chasis_length = 0.738 # m
        chasis_height = 0.233 # m
        chasis_z_offset = 0.107 # m
    else:
        with open(args.config) as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            print('Using config file: ' + args.config)
            chasis_name = config['chasis_name']
            wheel_diameter = config['wheel_diameter']
            wheel_thickness = config['wheel_thickness']
            wheel_base_x = config['wheel_base_x']
            wheel_base_y = config['wheel_base_y']
            chasis_width = config['chasis_width']
            chasis_length = config['chasis_length']
            chasis_height = config['chasis_height']
            chasis_z_offset = config['chasis_z_offset']

    print('-' * 80)
    print('chasis_name: ' + chasis_name)
    print('wheel_diameter: ' + str(wheel_diameter))
    print('wheel_thickness: ' + str(wheel_thickness))
    print('wheel_base_x: ' + str(wheel_base_x))
    print('wheel_base_y: ' + str(wheel_base_y))
    print('chasis_width: ' + str(chasis_width))
    print('chasis_length: ' + str(chasis_length))
    print('chasis_height: ' + str(chasis_height))
    print('chasis_z_offset: ' + str(chasis_z_offset))
    print('-' * 80)

    bproc.init()
    bproc.clean_up(True)
    remove_all_cameras()

    chasis_axis = create_axis(chasis_name, (0, 0, 0))
    chasis = bproc.object.create_primitive('CUBE', scale = (chasis_length / 2, chasis_width/2, chasis_height/2), location = (0, 0, chasis_z_offset + chasis_height / 2))
    chasis.blender_obj.name = 'chasis'
    chasis.blender_obj.data.name = 'chasis_mesh'
    steer1 = create_axis('steer_left_front', (wheel_base_x / 2, wheel_base_y / 2, wheel_diameter / 2))
    steer2 = create_axis('steer_left_back', (-wheel_base_x / 2, wheel_base_y / 2, wheel_diameter / 2))
    steer3 = create_axis('steer_right_back', (-wheel_base_x / 2, -wheel_base_y / 2, wheel_diameter / 2))
    steer4 = create_axis('steer_right_front', (wheel_base_x / 2, -wheel_base_y / 2, wheel_diameter / 2))
    wheel1, wheel1_bound = create_wheel('wheel_left_front', wheel_diameter, wheel_thickness, (wheel_base_x / 2, wheel_base_y / 2, wheel_diameter / 2))
    wheel2, wheel2_bound = create_wheel('wheel_left_back', wheel_diameter, wheel_thickness, (-wheel_base_x / 2, wheel_base_y / 2, wheel_diameter / 2))
    wheel3, wheel3_bound = create_wheel('wheel_right_back', wheel_diameter, wheel_thickness, (-wheel_base_x / 2, -wheel_base_y / 2, wheel_diameter / 2))
    wheel4, wheel4_bound = create_wheel('wheel_right_front', wheel_diameter, wheel_thickness, (wheel_base_x / 2, -wheel_base_y / 2, wheel_diameter / 2))

    boolean_modify(chasis.blender_obj, wheel1_bound.blender_obj, 'DIFFERENCE')
    boolean_modify(chasis.blender_obj, wheel2_bound.blender_obj, 'DIFFERENCE')
    boolean_modify(chasis.blender_obj, wheel3_bound.blender_obj, 'DIFFERENCE')
    boolean_modify(chasis.blender_obj, wheel4_bound.blender_obj, 'DIFFERENCE')

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
    steer1.parent = chasis_axis
    steer2.parent = chasis_axis
    steer3.parent = chasis_axis
    steer4.parent = chasis_axis

    bpy.ops.wm.usd_export(filepath=args.output)

if __name__ == '__main__':
    main()