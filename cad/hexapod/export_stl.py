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
    "Coxa_Body": "coxa_R_x3.stl",
    "Coxa_Body_L": "coxa_L_x3.stl",
    "Coxa_Arm": "coxa_bearing_arm_R_x3.stl",
    "Coxa_Arm_L": "coxa_bearing_arm_L_x3.stl",
    "Femur_Main": "femur_R_x3.stl",
    "Femur_Main_L": "femur_L_x3.stl",
    "Tibia_Horn": "tibia_horn_half_R_x3.stl",
    "Tibia_Horn_L": "tibia_horn_half_L_x3.stl",
    "Tibia_Bearing": "tibia_bearing_half_R_x3.stl",
    "Tibia_Bearing_L": "tibia_bearing_half_L_x3.stl",
    "Tibia_Presser": "tibia_presser_x6.stl",
}


# parts split on a plane print on that face: the outward normal of the face that goes down
DOWN = {
    "Tibia_Horn": (0, -1, 0), "Tibia_Horn_L": (0, 1, 0), "Tibia_Bearing": (0, 1, 0), "Tibia_Bearing_L": (0, -1, 0),
    "Coxa_Body": (0, -1, 0), "Coxa_Body_L": (0, 1, 0), "Coxa_Arm": (0, -1, 0), "Coxa_Arm_L": (0, 1, 0),
}


def lay_flat(shape, down=None):
    """Turn the shape so the given face direction (else its biggest flat face) points down, then put it
    on z 0 around the origin."""
    if down:
        n = App.Vector(*down)
    else:
        flat = [f for f in shape.Faces if f.Surface.TypeId == "Part::GeomPlane"]
        face = max(flat, key=lambda f: f.Area)
        n = face.normalAt(*face.Surface.parameter(face.CenterOfMass))
        if face.Orientation == "Reversed":
            n = -n
    s = shape.copy()
    s.Placement = App.Placement()
    rot = App.Rotation(n, App.Vector(0, 0, -1))
    s.rotate(App.Vector(0, 0, 0), rot.Axis, rot.Angle * 180 / 3.141592653589793)
    # placed by its mesh: a shape's BoundBox can stand off curved faces (2.7 mm off the coxa's rounded fork end,
    # so its STL floated over the bed and the slicer put supports under the joint face)
    bb = mesh_of(s).BoundBox
    s.translate(App.Vector(-bb.Center.x, -bb.Center.y, -bb.ZMin))
    return s


def mesh_of(s):
    return MeshPart.meshFromShape(Shape=s, LinearDeflection=0.02, AngularDeflection=0.1, Relative=False)


out_dir = os.path.join(HERE, "stl")
os.makedirs(out_dir, exist_ok=True)
for name, filename in PARTS.items():
    obj = doc.getObject(name)
    if obj is None:
        print("%s: not built, skipped" % name)
        continue
    s = lay_flat(obj.Shape, DOWN.get(name))
    mesh = mesh_of(s)
    mesh.write(os.path.join(out_dir, filename))
    bb = mesh.BoundBox
    print("%-30s %5.1f x %5.1f x %5.1f mm  %4.1f g  valid %s" % (filename, bb.XLength, bb.YLength, bb.ZLength,
                                                              s.Volume / 1000 * density, s.isValid()))

# ------------------------------------------------------------ tmp_: test prints, a short piece of a part
# The tibia's tip only, from the foot up past the upper screw (so the halves screw together), and its
# presser: to try the switch, its lead slot, the pin and the presser before printing whole tibias.
# One file each, to be placed on the bed by hand.
import Part
TIP_TEST_TOP = float(getattr(S.get("tib_screw_b_z"), "Value", S.get("tib_screw_b_z"))) + 3.0
cut = Part.makeBox(60, 60, 60, App.Vector(-30, -30, TIP_TEST_TOP - 60))
for name, filename, down, keep in (("Tibia_Horn", "tmp_tibia_tip_horn_half.stl", DOWN["Tibia_Horn"], cut),
                                   ("Tibia_Bearing", "tmp_tibia_tip_bearing_half.stl", DOWN["Tibia_Bearing"], cut),
                                   ("Tibia_Presser", "tmp_tibia_tip_presser.stl", (0, 0, -1), None)):
    sh = doc.getObject(name).Shape
    sh = lay_flat(sh.common(keep) if keep else sh, down)
    mesh = mesh_of(sh)
    mesh.write(os.path.join(out_dir, filename))
    bb = mesh.BoundBox
    print("%-30s %5.1f x %5.1f x %5.1f mm  %4.1f g" % (filename, bb.XLength, bb.YLength, bb.ZLength, sh.Volume / 1000 * density))
old = os.path.join(out_dir, "tmp_tibia_tip_test.stl")
if os.path.exists(old):
    os.remove(old)
print(out_dir)
