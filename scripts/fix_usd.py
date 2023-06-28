from pxr import Usd, UsdGeom, Gf
from scipy.spatial.transform import Rotation as R
import numpy as np
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('input', help='Path to the input file')
parser.add_argument('output', help='Path to the output file')
args = parser.parse_args()

# open usd file
stage = Usd.Stage.Open(args.input)

# traverse the stage
first_prim = None
for prim in stage.Traverse():
    # check if the prim is a Xform
    if prim.IsA(UsdGeom.Xform):
        first_prim = prim
        break

# set first prim as the default prim
if first_prim is not None:
    stage.SetDefaultPrim(first_prim)
else:
    print('No Xform found in the stage, cannot set default prim')
    sys.exit(1)

# fix the transform
for prim in stage.Traverse():
    for attr in prim.GetAttributes():
        if attr.GetName() == 'xformOpOrder' and attr.Get() is not None and \
            len(attr.Get()) == 1 and attr.Get()[0] == 'xformOp:transform':
            xform = UsdGeom.Xform(prim)
            transform = xform.GetOrderedXformOps()[0]
            mat44 = np.array(transform.Get()).T
            mat33 = mat44[:3,:3]
            translate = mat44[:3, 3].tolist()
            scale = np.linalg.norm(mat33, axis=0)
            rot_mat = mat33 / np.array(scale)
            quat = R.from_matrix(rot_mat).as_quat()
            xform.ClearXformOpOrder()
            xform.AddTranslateOp().Set(Gf.Vec3d(*translate))
            xform.AddOrientOp().Set(Gf.Quatf(quat[3], quat[0], quat[1], quat[2]))
            xform.AddScaleOp().Set(Gf.Vec3d(*scale))
            print(f"{prim.GetPath()}.{attr.GetName()} = {attr.Get()}")
            prim.RemoveProperty('xformOp:transform')

stage.Export(args.output)
