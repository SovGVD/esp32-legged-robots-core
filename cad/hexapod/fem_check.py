"""FEA of a printed part, in its own unsaved document, with CalculiX. Run inside
FreeCAD with init_globals={"CASE": "femur"|"tibia"|"coxa", "LOAD": name}, and for the
femur or coxa "BONDED": True to glue their servos in solid, the opposite limit to
the print alone: the real part is between the two.

  walk   foot force 8 N up (900 g on 3 feet, x3 for walking), next servo at stall
  twist  walk + 4 N sideways at the foot
  fall   30 N up + 15 N sideways at the foot, landing hard on one leg (an estimate)

CASE "frame" is the whole assembled body: both plates, the 8 columns, and the 6 coxa
links with their servos, horns and bearings, all joined as if glued. It stands on a
tripod (RF, LM, RR held at their femur joints) and the robot's weight, times the load's
factor, pushes down on the bottom plate under the cells: walk 3 x 11 N, twist adds a
third of that sideways, fall is the robot landing on the RF leg alone (30 N + 15 N
sideways).

CASE "body" stands the joined plates on a tripod and takes each leg's lever arm as a
twist through its horn and bearing. CASE "beam" is a check of the method: a cantilever
with a known textbook answer.

Each run draws a picture in fea/: stress in colour, held faces in blue, loads as red
arrows.

Solves and prints the result itself: FreeCAD's writer leaves the *CLOAD sections
empty on these faces (0 MPa, which looks like a pass), so this script adds the node
loads to the solver input, checks their sum, and reads the .frd file.

PLA, solid and isotropic: E from Params (Bambu PLA Basic across the layers), nu 0.36. Real
prints with infill are softer than this. Judge against pla_limit. Peaks at sharp points overstate; the 99.9% figure is the fairer
one. Foot movement counts the part only, with no servo, bearing or horn play.
"""
import os
import re
import subprocess
import tempfile

import FreeCAD as App
import ObjectsFem
import runpy
common = runpy.run_path(os.path.join(os.path.dirname(os.path.abspath(__file__)), "common.py"))

STALL_NM = 0.19
LOADS = {"walk": (8.0, 0.0, True), "twist": (8.0, 4.0, True), "fall": (30.0, 15.0, False)}
Z, Y, X = 3, 2, 1
ROT_IN_LEG = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))


def build(case, load, bonded):
    up, side, stall = LOADS[load]
    src = App.getDocument("hexapod")
    sheet = src.getObject("Params")

    def val(alias):
        v = sheet.get(alias)
        return float(getattr(v, "Value", v))

    name = "hexapod_fem"
    if name in App.listDocuments():
        App.closeDocument(name)
    doc = App.newDocument(name)
    obj = doc.addObject("Part::Feature", "Print")
    SERVO = ("MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft")

    def with_servos(shape, places):
        pieces = []
        for place in places:
            for n in SERVO:
                piece = src.getObject(n).Shape.copy()
                piece.Placement = place.multiply(piece.Placement)
                pieces.append(piece)
        out = shape.multiFuse(pieces).removeSplitter()
        if len(out.Solids) != 1:
            raise ValueError("did not fuse into one solid: %d" % len(out.Solids))
        return out

    seat_y = val("joint_seat_y")
    if case == "femur":
        shape = src.getObject("Femur_Main").Shape.copy()
        if bonded:
            shape = with_servos(shape, (App.Placement(App.Vector(0, seat_y, 0), ROT_IN_LEG.multiply(App.Rotation(App.Vector(0, 0, 1), 180))),
                                        App.Placement(App.Vector(val("femur_len"), seat_y, 0), ROT_IN_LEG)))
    elif case == "tibia":
        # glued at the split, the halves' rounded edges meet in tiny slivers at the outer corners of the
        # fork's crotch (x +-5.8, z apex) that gmsh fills with inverted elements: a small filler there,
        # in the FEA only (each printed half is fine on its own)
        import Part
        az = val("tib_a_apex_z")
        filler = Part.makeBox(14, 1.6, 1.5, App.Vector(-7, -0.8, az - 0.3))
        shape = src.getObject("Tibia_Horn").Shape.fuse([src.getObject("Tibia_Bearing").Shape, filler]).removeSplitter()
    elif case == "beam":
        import Part
        shape = Part.makeBox(100, 10, 5)
    elif case == "frame":
        import Part
        import math
        top_bot, top0 = val("body_plate_top"), val("top_plate_z0")
        pieces = [src.getObject("Body_Top").Shape.copy()]
        for o in src.getObject("Body_Check").Group:
            p0 = o.Placement.Base
            if o.Name.startswith("BodyCheck_column"):
                # the column fills its ring's clearance and reaches 0.2 into both plates: slivers don't mesh
                pieces.append(Part.makeCylinder((val("col_d") + val("col_fit")) / 2 + 0.05, top0 - top_bot + 0.4, App.Vector(p0.x, p0.y, top_bot - 0.2)))
        frame_legs = {}
        for o in src.getObject("Body_Check").Group:
            if not o.Name.startswith("BodyCheck_coxa_"):
                continue
            tag = o.Name.split("_")[-1]
            coxa = o.LinkedObject.Shape.copy()
            coxa.Placement = o.Placement.multiply(coxa.Placement)
            pieces.append(coxa)
            for prefix, names in (("servo_", SERVO), ("bearing_", ("Bearing_body",))):
                link = src.getObject("BodyCheck_" + prefix + tag)
                for n in names:
                    piece = src.getObject(n).Shape.copy()
                    piece.Placement = link.Placement.multiply(piece.Placement)
                    pieces.append(piece)
            # the glued horn fills its pocket: the pocket's own shape (the dummy's straight arm left
            # thin wedges in the tapered pocket, which meshed into inverted elements)
            hl = src.getObject("BodyCheck_horn_" + tag).Placement
            arm_dir = hl.Rotation.multVec(App.Vector(1, 0, 0))
            # (down through the cleared channel under it too: a hair stiffer at the rib notches, but the
            # notch walls next to the pocket's mouth meshed into inverted elements)
            r_low_z = val("top_plate_z0") - val("rib_h")
            # grown 0.5 all round: it also fills the 0.4 lead-ins at the channel's edges, which left thin voids
            pieces.append(common["horn_pocket"](val("horn_pocket_d") / 2 + 0.5, val("horn_pocket_w") / 2 + 0.5, val("horn_reach") - val("horn_tip_w") / 2,
                                                val("top_boss_z0") + val("horn_hub_h") + val("horn_fit") - r_low_z + 0.5, (hl.Base.x, hl.Base.y, r_low_z - 0.5),
                                                (arm_dir.x, arm_dir.y, 0), (0, 0, 1)))
            frame_legs[tag] = o.Placement
        shape = src.getObject("Body_Bottom").Shape.multiFuse(pieces).removeSplitter()
        if len(shape.Solids) != 1:
            raise ValueError("the frame did not join: %d solids" % len(shape.Solids))
    elif case == "body":
        # the plates and columns joined, as if the screws were glue
        columns = []
        for o in src.getObject("Body_Check").Group:
            if o.Name.startswith("BodyCheck_column"):
                # a solid rod 0.2 mm into each plate: faces that only touch don't mesh
                import Part
                p0 = o.Placement.Base
                columns.append(Part.makeCylinder(val("col_d") / 2, val("top_plate_z0") - val("body_plate_top") + 0.4,
                                                 App.Vector(p0.x, p0.y, val("body_plate_top") - 0.2)))
        shape = src.getObject("Body_Bottom").Shape.multiFuse(columns + [src.getObject("Body_Top").Shape]).removeSplitter()
        if len(shape.Solids) != 1:
            raise ValueError("the plates did not join: %d solids" % len(shape.Solids))
    else:
        shape = src.getObject("Coxa_Main").Shape.copy()
        if bonded:
            shape = with_servos(shape, (App.Placement(App.Vector(0, 0, val("coxa_seat_z")), App.Rotation(App.Vector(0, 0, 1), 90)),))
    # through STEP and back: gmsh then meshes shapes it otherwise leaves empty (the tibia's apex
    # where the halves meet); the faces found below are looked up by geometry, so nothing else changes
    import Part
    if globals().get("STEP_TRIP", case == "tibia"):
        step = os.path.join(tempfile.mkdtemp(prefix="hexapod_step_"), "part.step")
        shape.exportStep(step)
        shape = Part.read(step)
    obj.Shape = shape
    doc.recompute()
    an = ObjectsFem.makeAnalysis(doc, "Analysis")
    mat = ObjectsFem.makeMaterialSolid(doc, "PLA")
    mat.Material = {"Name": "PLA", "YoungsModulus": "%g MPa" % val("pla_E"), "PoissonRatio": "0.36", "Density": "1240 kg/m^3"}
    mat.References = [(obj, ("Solid1",))]
    an.addObject(mat)

    def cyl_faces(radius, x, axis_y=None, z=None):
        # axis_y: cylinders along Z only, this Y
        out = []
        for i, f in enumerate(obj.Shape.Faces):
            s = f.Surface
            if s.TypeId != "Part::GeomCylinder" or abs(s.Radius - radius) > 0.01 or abs(s.Center.x - x) > 0.01:
                continue
            if axis_y is not None and abs(s.Center.y - axis_y) > 0.01:
                continue
            if z is not None and abs(s.Center.z - z) > 0.01 and abs(f.BoundBox.ZMin + radius - z) > 0.01:
                continue
            out.append("Face%d" % (i + 1))
        if not out:
            raise ValueError("no cylinder r %g at x %g" % (radius, x))
        return out

    def plane_faces(axis, value):
        out = []
        for i, f in enumerate(obj.Shape.Faces):
            if f.Surface.TypeId != "Part::GeomPlane":
                continue
            bb = f.BoundBox
            lo, hi = {"x": (bb.XMin, bb.XMax), "y": (bb.YMin, bb.YMax), "z": (bb.ZMin, bb.ZMax)}[axis]
            if abs(lo - value) < 0.01 and abs(hi - value) < 0.01:
                out.append("Face%d" % (i + 1))
        if not out:
            raise ValueError("no plane at %s = %g" % (axis, value))
        return out

    pilot = val("mg90_screw_pilot_d") / 2
    servo_shaft = val("mg90_shaft_d") / 2
    near, far = val("mg90_hole_near"), val("mg90_hole_far")
    if case == "beam":
        # a check of the whole method against the textbook: cantilever, 1 N at its end
        held = plane_faces("x", 0)
        loads = [("tip", plane_faces("x", 100), 1.0, Z)]
        probe = None
    elif case == "body":
        # standing on a tripod. Each leg pushes up on its bearing, and its foot's lever arm
        # twists the body through the couple between the horn pocket and the bearing
        import math
        foot_lever = val("coxa_len") + 9.0
        couple_arm = val("coxa_horn_z") - (val("body_plate_top") - val("bearing_t") / 2)
        held = cyl_faces(pilot, 0)
        pairs = (("RM", val("coxa_mid_x"), 0.0, 0.0), ("LF", -val("coxa_front_x"), val("coxa_front_y"), 180 - sheet.get("front_leg_yaw").getValueAs("deg").Value),
                 ("LR", -val("coxa_front_x"), -val("coxa_front_y"), 180 - sheet.get("rear_leg_yaw").getValueAs("deg").Value))
        loads = []
        each = up + side
        for tag, x, y, yaw in pairs:
            push = each * foot_lever / couple_arm
            c, s_ = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            top = val("body_plate_top")
            seat = ("ring", x, y, val("bearing_seat_d") / 2, top - val("bearing_t"), top)
            horn = ("ring", x, y, val("horn_pocket_d") / 2, val("top_boss_z0"), val("coxa_horn_z"))
            loads.append(("%s up" % tag, seat, each, Z))
            loads.append(("%s low x" % tag, seat, -push * c, X))
            loads.append(("%s low y" % tag, seat, -push * s_, Y))
            loads.append(("%s top x" % tag, horn, push * c, X))
            loads.append(("%s top y" % tag, horn, push * s_, Y))
        probe = None
    elif case == "frame":
        def joint_faces(tag):
            """The coxa's seat for the femur's bearing and horn: cylinders on the femur axis."""
            pl = frame_legs[tag]
            base = pl.multVec(App.Vector(val("coxa_len"), 0, 0))
            axis = pl.Rotation.multVec(App.Vector(0, 1, 0))
            out = []
            for i, f in enumerate(obj.Shape.Faces):
                su = f.Surface
                if su.TypeId != "Part::GeomCylinder" or min(abs(su.Radius - val("bearing_seat_d") / 2), abs(su.Radius - val("horn_pocket_d") / 2)) > 0.01:
                    continue
                if abs(abs(su.Axis.dot(axis)) - 1) > 1e-6 or (su.Center - base).cross(axis).Length > 0.01:
                    continue
                out.append("Face%d" % (i + 1))
            if not out:
                raise ValueError("no femur joint faces on " + tag)
            return out
        # the robot's weight hangs on the battery cartridge's 2 turn latches, under the bottom plate
        weight = 1.132 * 9.81
        zb = val("body_bottom_z")
        pivots = [o.Placement.Base for o in src.getObject("Body_Check").Group if o.Name.startswith("BodyCheck_latch_")]
        pads = [("box", p.x - 3.4, p.x + 3.4, p.y - 3.4, p.y + 3.4, zb) for p in pivots]
        down, sideways = (30.0, 15.0) if load == "fall" else (3 * weight, weight if load == "twist" else 0.0)
        held = joint_faces("RF") if load == "fall" else joint_faces("RF") + joint_faces("LM") + joint_faces("RR")
        loads = []
        for k, pad in enumerate(pads):
            loads.append(("latch %d down" % (k + 1), pad, -down / len(pads), Z))
            if sideways:
                loads.append(("latch %d sideways" % (k + 1), pad, -sideways / len(pads), X))
        probe = None
    elif case == "femur":
        fl = val("femur_len")
        held = cyl_faces(val("fem_shaft_d") / 2, 0) + (cyl_faces(servo_shaft, 0) if bonded else cyl_faces(pilot, -near) + cyl_faces(pilot, far))
        shaft = cyl_faces(val("fem_shaft_d") / 2, fl)
        a = cyl_faces(servo_shaft, fl) if bonded else cyl_faces(pilot, fl - far)
        b = a if bonded else cyl_faces(pilot, fl + near)
        lever = 0.060
        arm = (val("joint_horn_y") - 2.0 if bonded else seat_y - val("fem_plate_floor") / 2) - (val("fem_floor_y0") - 1.5)
        couple = STALL_NM / (val("mg90_screw_spacing") / 1000) if stall and not bonded else 0.0
        twist = side * lever / (arm / 1000)
        loads = [("shaft", shaft, up / 2 - twist, Z), ("servo a", a, up / 4 + couple + twist / 2, Z), ("servo b", b, up / 4 - couple + twist / 2, Z)]
        if side:
            loads.append(("side", shaft, side, Y))
        probe = (fl, 0.0, lever)
    elif case == "tibia":
        held = cyl_faces(val("bearing_seat_d") / 2, 0) + cyl_faces(val("horn_pocket_d") / 2, 0)
        # the presser (old TibiaTip) stops under the top of its pocket: small faces, so by position
        top = ("near", val("tip_stop_z"), 3.1, 3.1)
        loads = [("foot up", top, up, Z)]
        if side:
            loads += [("side", top, side, Y), ("forward", top, side, X)]
        probe = None
    else:
        hub = val("coxa_len")
        held = cyl_faces(val("coxa_shaft_d") / 2, 0) + (cyl_faces(servo_shaft, 0) if bonded else cyl_faces(pilot, -near) + cyl_faces(pilot, far))
        seat = cyl_faces(val("bearing_seat_d") / 2, hub)
        horn = cyl_faces(val("horn_pocket_d") / 2, hub)
        lever = 0.100
        arm = val("coxa_arm_a_y1") - val("coxa_arm_b_y0") - 4.0
        twist = side * lever / (arm / 1000)
        stall_force = STALL_NM / (hub / 1000) if stall else 0.0
        loads = [("bearing", seat, up / 2 - twist, Z), ("horn", horn, up / 2 + twist, Z)]
        if side or stall_force:
            loads.append(("side", seat, side + stall_force, Y))
        probe = (hub, 0.0, lever)

    fixed = ObjectsFem.makeConstraintFixed(doc, "Held")
    fixed.References = [(obj, tuple(held))]
    an.addObject(fixed)
    mesh = ObjectsFem.makeMeshGmsh(doc, "Mesh")
    mesh.Shape = obj
    mesh.CharacteristicLengthMax = float(globals().get("MESH_MAX", 3.0 if case in ("body", "frame") else 1.6))
    mesh.CharacteristicLengthMin = float(globals().get("MESH_MIN", 0.3))
    mesh.ElementOrder = "2nd"
    mesh.SecondOrderLinear = True   # straight element edges: curved ones invert on small features
    mesh.OptimizeNetgen = bool(globals().get("NETGEN", case == "tibia"))   # untangles the few flat elements gmsh leaves at the tibia's apex
    an.addObject(mesh)
    doc.recompute()
    from femmesh.gmshtools import GmshTools
    # gmsh sometimes returns an empty mesh for the same input (seen on the tibia): try again,
    # nudging the size a little each time
    for attempt in range(4):
        err = GmshTools(mesh).create_mesh()
        if err:
            print("gmsh:", err[:300])
        if mesh.FemMesh.NodeCount:
            break
        mesh.CharacteristicLengthMax = mesh.CharacteristicLengthMax.Value * 0.97
        doc.recompute()
    print("mesh nodes", mesh.FemMesh.NodeCount, "solids", len(obj.Shape.Solids), "valid", obj.Shape.isValid())
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc, "CalculiX")
    an.addObject(solver)
    doc.recompute()
    from femtools import ccxtools
    work = tempfile.mkdtemp(prefix="hexapod_fem_")
    fea = ccxtools.FemToolsCcx(an, solver)
    fea.update_objects()
    fea.setup_working_dir(work)
    fea.setup_ccx()
    fea.write_inp_file()
    lines, totals, arrows = ["*CLOAD"], {}, []
    for label, faces, newtons, dof in loads:
        nodes = set()
        if faces[0] == "near":
            _, zv, xlim, ylim = faces
            nodes = {n for n, v in mesh.FemMesh.Nodes.items() if abs(v.z - zv) < 0.6 and abs(v.x) <= xlim and abs(v.y) <= ylim}
        elif faces[0] == "box":
            # nodes on a horizontal face at z, inside an x-y box
            _, x0, x1, y0, y1, zv = faces
            nodes = {n for n, v in mesh.FemMesh.Nodes.items() if abs(v.z - zv) < 0.05 and x0 <= v.x <= x1 and y0 <= v.y <= y1}
        elif faces[0] == "ring":
            # small faces, so by position: nodes on a cylinder around Z
            _, cx, cy, rad, z0, z1 = faces
            nodes = {n for n, v in mesh.FemMesh.Nodes.items() if abs(((v.x - cx) ** 2 + (v.y - cy) ** 2) ** 0.5 - rad) < 0.4 and z0 - 0.1 <= v.z <= z1 + 0.1}
        else:
            for face in faces:
                nodes.update(mesh.FemMesh.getNodesByFace(obj.Shape.getElement(face)))
        if not nodes:
            raise ValueError("no mesh nodes on %s (mesh has %d nodes, %s)" % (label, mesh.FemMesh.NodeCount, faces))
        lines += ["%d,%d,%.8E" % (n, dof, newtons / len(nodes)) for n in sorted(nodes)]
        totals[dof] = totals.get(dof, 0.0) + newtons
        arrows.append((label, nodes, newtons, dof))
    text = open(fea.inp_file_name).read()
    match = re.search(r"^\*END STEP", text, flags=re.M)
    open(fea.inp_file_name, "w").write(text[:match.start()] + "\n".join(lines) + "\n" + text[match.start():])
    run = subprocess.run([fea.ccx_binary, "-i", os.path.splitext(os.path.basename(fea.inp_file_name))[0]], cwd=work,
                         capture_output=True, text=True, timeout=500, env=dict(os.environ, OMP_NUM_THREADS="4"))
    if "Job finished" not in run.stdout:
        raise RuntimeError("ccx failed: " + run.stdout[-600:])
    held_nodes = set()
    for face in held:
        held_nodes.update(mesh.FemMesh.getNodesByFace(obj.Shape.getElement(face)))
    report(fea.inp_file_name[:-4] + ".frd", case, load, bonded, totals, held_nodes, arrows, probe, obj.Shape.Volume)
    App.closeDocument(name)
    App.setActiveDocument("hexapod")


def picture(path, title, coords, vm, held, arrows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    ids = sorted(vm)
    xyz = np.array([coords[n] for n in ids])
    val = np.array([vm[n] for n in ids])
    fig = plt.figure(figsize=(15, 7), facecolor="white")
    art = None
    for k, (elev, azim, name) in enumerate(((28, 120, "view 1"), (28, -60, "view 2"))):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d", computed_zorder=False)
        order = np.argsort(val)
        art = ax.scatter(xyz[order, 0], xyz[order, 1], xyz[order, 2], c=val[order], cmap="turbo", vmin=0, vmax=17.5, s=2.0, depthshade=False, zorder=1)
        h = np.array([coords[n] for n in held])
        ax.scatter(h[:, 0], h[:, 1], h[:, 2], c="#1f4fff", s=14, depthshade=False, zorder=5)
        ax.text2D(0.02, 0.95, "blue = held", color="#1f4fff", transform=ax.transAxes, fontsize=10)
        for label, nodes, newtons, dof in arrows:
            pts = np.array([coords[n] for n in nodes])
            c = pts.mean(axis=0)
            d = np.zeros(3)
            d[dof - 1] = 1.0 if newtons > 0 else -1.0
            size = 12 + 18 * min(abs(newtons) / 30.0, 1.0)
            ax.quiver(c[0] - d[0] * size, c[1] - d[1] * size, c[2] - d[2] * size, d[0], d[1], d[2], length=size, color="red", linewidth=2, zorder=6)
            ax.text(c[0] - d[0] * size, c[1] - d[1] * size, c[2] - d[2] * size, "%s %.1f N" % (label, abs(newtons)), color="red", fontsize=8, zorder=7)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(name)
        ax.set_xlabel("X, mm")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_box_aspect((np.ptp(xyz[:, 0]) + 30, np.ptp(xyz[:, 1]) + 30, np.ptp(xyz[:, 2]) + 30))
    bar = fig.colorbar(art, ax=fig.axes, shrink=0.7, pad=0.02)
    bar.set_label("von Mises, MPa (scale stops at the 17.5 MPa limit)")
    fig.suptitle(title, fontsize=12)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def report(frd, case, load, bonded, totals, held, arrows, probe, volume):
    coords, disp, stress, block = {}, {}, {}, None
    for line in open(frd):
        if line.startswith("    2C"):
            block = "coords"
        elif line.startswith(" -4"):
            block = line.split()[1]
        elif line.startswith(" -3"):
            block = None
        elif line.startswith(" -1") and block:
            node = int(line[3:13])
            vals = [float(line[13 + 12 * k:25 + 12 * k]) for k in range((len(line.rstrip()) - 13) // 12)]
            {"coords": coords, "DISP": disp, "STRESS": stress}.get(block, {})[node] = vals
    vm = {n: (0.5 * ((a - b) ** 2 + (b - c) ** 2 + (c - a) ** 2) + 3 * (d ** 2 + e ** 2 + f ** 2)) ** 0.5
          for n, (a, b, c, d, e, f) in stress.items()}
    ranked = sorted(vm, key=vm.get, reverse=True)
    if probe:
        # how far the joint moves and tips: the leg below turns the tip into foot movement
        x, z, lever = probe
        ring = [n for n in disp if abs(coords[n][0] - x) < 8 and abs(coords[n][2] - z) < 8]
        ys = sorted(coords[n][1] for n in ring)
        lo = [n for n in ring if coords[n][1] < ys[len(ys) // 4]]
        hi = [n for n in ring if coords[n][1] > ys[3 * len(ys) // 4]]
        mean = lambda nodes, k: sum(disp[n][k] for n in nodes) / len(nodes)
        dy = sum(coords[n][1] for n in hi) / len(hi) - sum(coords[n][1] for n in lo) / len(lo)
        tip = (mean(hi, 2) - mean(lo, 2)) / dy
        foot = abs(mean(ring, 1) + tip * lever * 1000)
    else:
        foot = max(sum(u * u for u in disp[n]) ** 0.5 for n in disp)
    label = "%s, %s, %s" % (case, load, "assembled, joints glued" if case == "frame" else  "servos bonded in" if bonded else ("print alone" if case != "tibia" else "halves joined"))
    moved = "foot moves sideways" if probe else "moves at most"
    summary = "%s: peak %.1f MPa, 99.9%% under %.1f MPa, %s %.2f mm" % (label, vm[ranked[0]], vm[ranked[len(ranked) // 1000]], moved, foot)
    print(summary + " (peak at %s; forces %s)" % ([round(c, 1) for c in coords[ranked[0]]], {k: round(v, 1) for k, v in totals.items()}))
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fea")
    os.makedirs(out, exist_ok=True)
    picture(os.path.join(out, "%s_%s_%s.png" % (case, load, "servos_bonded" if bonded else "alone")), summary, coords, vm, held, arrows)


try:
    build(globals().get("CASE", "femur"), globals().get("LOAD", "twist"), globals().get("BONDED", False))
finally:
    if "hexapod_fem" in App.listDocuments():
        App.closeDocument("hexapod_fem")
    App.setActiveDocument("hexapod")
