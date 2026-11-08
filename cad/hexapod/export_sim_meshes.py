"""Meshes of the robot for the MuJoCo sim (sim/legsim/hexapod), into sim/models/meshes/hexapod/.
Each file is in the frame of the sim link that carries it, in mm (the MJCF scales them):

  torso_bottom   bottom plate, columns, battery cartridge and latches   body frame (X right, Y forward,
  torso_top      top plate, ToF mount, inner wall                        Z up, origin at the femur axis
  torso_parts    cells, boards, ToF sensors, switch, XT30, coxa horns    height), as Body_Check places them
                 and bearings
  coxa_R/_L, coxa_servo_R/_L      leg frame: coxa axis, +X out along the leg, Z up
  femur_R/_L, femur_servos_R/_L   femur frame: hip axis, +X to the knee, femur level
  tibia_R/_L, presser             tibia frame: knee axis, -Z down the tibia

The _L files are the right ones mirrored across the leg plane (the left legs are printed mirrored).
Only the look comes from here: the sim's masses stay in simple_parts.py. Run inside FreeCAD after
build_all.py, then regenerate sim/models/hexapod.xml (sim/examples/build_model.py)."""
import os

import FreeCAD as App
import MeshPart
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "..", "sim", "models", "meshes", "hexapod"))
doc = App.getDocument("hexapod")
S = doc.getObject("Params")


def v(alias):
    x = S.get(alias)
    return float(getattr(x, "Value", x))


Pl = lambda x, y, z, rot=None: App.Placement(App.Vector(x, y, z), rot or App.Rotation())
Rz = lambda deg: App.Rotation(App.Vector(0, 0, 1), deg)
KNEE_ROT = App.Rotation(App.Matrix(0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1))
HIP_ROT = KNEE_ROT.multiply(Rz(180))
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
LEG_LINKS = ("coxa", "servo")          # Body_Check links that turn with the coxa: they go to the coxa link


def placed(obj, at):
    """Solids of a part (or of an App::Part's children) moved by `at`."""
    kids = obj.Group if obj.TypeId == "App::Part" else [obj]
    out = []
    for k in kids:
        if not hasattr(k, "Shape") or not k.Shape.Solids or k.Name in ("Bearing_ring", "Bearing_bore"):
            continue
        s = k.Shape.copy()
        s.Placement = at.multiply(s.Placement)
        out += s.Solids
    return out


def write(name, solids, deflection=0.25, max_faces=6000):
    mesh = MeshPart.meshFromShape(Shape=Part.makeCompound(solids), LinearDeflection=deflection, AngularDeflection=0.5, Relative=False)
    if mesh.CountFacets > max_faces:        # the plates' rib grids and lead-ins: tens of thousands of small facets
        mesh.decimate(0.5, 1 - max_faces / mesh.CountFacets)
    path = os.path.join(OUT, name + ".stl")
    mesh.write(path)
    print("%-18s %6d faces %5d KB" % (name, mesh.CountFacets, os.path.getsize(path) // 1024))


def mirrored(solids):
    return [s.mirror(App.Vector(0, 0, 0), App.Vector(0, 1, 0)) for s in solids]


os.makedirs(OUT, exist_ok=True)
groups = {"torso_bottom": [], "torso_top": [], "torso_parts": []}
for link in doc.getObject("Body_Check").Group:
    tag = link.Name.replace("BodyCheck_", "")
    if tag.split("_")[0] in LEG_LINKS:
        continue
    target = link.LinkedObject
    if target.Name in ("Body_Bottom", "Body_Column", "Battery_Cartridge", "Battery_Latch"):
        key = "torso_bottom"
    elif target.Name in ("Body_Top", "Body_ToF_Mount", "Body_Wall"):
        key = "torso_top"
    else:
        key = "torso_parts"
    groups[key] += placed(target, link.Placement)
for key, solids in groups.items():
    write(key, solids, 0.5, 10000)

# a right leg's links, from its Body_Check placement: the coxa's servo in the coxa's frame
coxa_at = doc.getObject("BodyCheck_coxa_RM").Placement
servo_at = coxa_at.inverse().multiply(doc.getObject("BodyCheck_servo_RM").Placement)
s_y, fl = v("joint_seat_y"), v("femur_len")
legs = {
    "coxa": placed(doc.getObject("Coxa_Main"), Pl(0, 0, 0)),
    "coxa_servo": placed(doc.getObject("Dummy_MG90S"), servo_at),
    "femur": placed(doc.getObject("Femur_Main"), Pl(0, 0, 0)),
    "femur_servos": placed(doc.getObject("Dummy_MG90S"), Pl(0, s_y, 0, HIP_ROT)) + placed(doc.getObject("Dummy_MG90S"), Pl(fl, s_y, 0, KNEE_ROT)),
    "tibia": placed(doc.getObject("Tibia_Horn"), Pl(0, 0, 0)) + placed(doc.getObject("Tibia_Bearing"), Pl(0, 0, 0)),
}
for key, solids in legs.items():
    write(key + "_R", solids)
    write(key + "_L", mirrored(solids))
write("presser", placed(doc.getObject("Tibia_Presser"), Pl(0, 0, 0)))
print(OUT)
