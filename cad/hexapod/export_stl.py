"""STL files of the leg parts, one file per part, into stl/ (not in git). Each part is laid on its
biggest flat face, sitting on z 0 and centred on the origin, ready for the slicer.
The file name says how many to print (right legs and left legs are mirrored parts).
Run inside FreeCAD after build_all.py (the parts must be built)."""
import os

import FreeCAD as App
import MeshPart

HERE = os.path.dirname(os.path.abspath(__file__))
doc = App.getDocument("hexapod")
S = doc.getObject("Params")
density = float(getattr(S.get("pla_density"), "Value", S.get("pla_density")))

PARTS = {                           # object -> file name
    "Coxa_Main": "coxa_R_x3.stl",
    "Coxa_Main_L": "coxa_L_x3.stl",
    "Femur_Main": "femur_R_x3.stl",
    "Femur_Main_L": "femur_L_x3.stl",
    "Tibia_Horn": "tibia_horn_half_R_x3.stl",
    "Tibia_Horn_L": "tibia_horn_half_L_x3.stl",
    "Tibia_Bearing": "tibia_bearing_half_R_x3.stl",
    "Tibia_Bearing_L": "tibia_bearing_half_L_x3.stl",
    "Tibia_Presser": "tibia_presser_x6.stl",
}


def lay_flat(shape):
    """Turn the shape so its biggest flat face points down, then put it on z 0 around the origin."""
    flat = [f for f in shape.Faces if f.Surface.TypeId == "Part::GeomPlane"]
    face = max(flat, key=lambda f: f.Area)
    n = face.normalAt(*face.Surface.parameter(face.CenterOfMass))
    if face.Orientation == "Reversed":
        n = -n
    s = shape.copy()
    s.Placement = App.Placement()
    rot = App.Rotation(n, App.Vector(0, 0, -1))
    s.rotate(App.Vector(0, 0, 0), rot.Axis, rot.Angle * 180 / 3.141592653589793)
    bb = s.BoundBox
    s.translate(App.Vector(-bb.Center.x, -bb.Center.y, -bb.ZMin))
    return s


out_dir = os.path.join(HERE, "stl")
os.makedirs(out_dir, exist_ok=True)
for name, filename in PARTS.items():
    obj = doc.getObject(name)
    if obj is None:
        print("%s: not built, skipped" % name)
        continue
    s = lay_flat(obj.Shape)
    mesh = MeshPart.meshFromShape(Shape=s, LinearDeflection=0.02, AngularDeflection=0.1, Relative=False)
    mesh.write(os.path.join(out_dir, filename))
    bb = s.BoundBox
    print("%-30s %5.1f x %5.1f x %5.1f mm  %4.1f g  valid %s" % (filename, bb.XLength, bb.YLength, bb.ZLength,
                                                              s.Volume / 1000 * density, s.isValid()))
print(out_dir)
