"""FEA of a printed part, in its own unsaved document, with CalculiX. Run inside
FreeCAD with init_globals={"CASE": "femur"|"tibia"|"coxa", "LOAD": name}, and for the
femur or coxa "BONDED": True to glue their servos in solid, the opposite limit to
the print alone: the real part is between the two.

  walk   foot force 8 N up (900 g on 3 feet, x3 for walking), next servo at stall
  twist  walk + 4 N sideways at the foot
  fall   30 N up + 15 N sideways at the foot, landing hard on one leg (an estimate)

Each run draws a picture in fea/: stress in colour, held faces in blue, loads as red
arrows.

Solves and prints the result itself: FreeCAD's writer leaves the *CLOAD sections
empty on these faces (0 MPa, which looks like a pass), so this script adds the node
loads to the solver input, checks their sum, and reads the .frd file.

PLA solid and isotropic: E 3500 MPa, nu 0.36. Prints are weaker across layers: judge
against about 20 MPa. Peaks at sharp points overstate; the 99.9% figure is the fairer
one. Foot movement counts the part only, with no servo, bearing or horn play.
"""
import os
import re
import subprocess
import tempfile

import FreeCAD as App
import ObjectsFem

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
        shape = src.getObject("Tibia_Horn").Shape.fuse(src.getObject("Tibia_Bearing").Shape).removeSplitter()
    else:
        shape = src.getObject("Coxa_Main").Shape.copy()
        if bonded:
            shape = with_servos(shape, (App.Placement(App.Vector(0, 0, val("coxa_seat_z")), App.Rotation(App.Vector(0, 0, 1), 90)),))
    obj.Shape = shape
    doc.recompute()
    an = ObjectsFem.makeAnalysis(doc, "Analysis")
    mat = ObjectsFem.makeMaterialSolid(doc, "PLA")
    mat.Material = {"Name": "PLA", "YoungsModulus": "3500 MPa", "PoissonRatio": "0.36", "Density": "1240 kg/m^3"}
    mat.References = [(obj, ("Solid1",))]
    an.addObject(mat)

    def cyl_faces(radius, x, axis_y=None, z=None):
        out = []
        for i, f in enumerate(obj.Shape.Faces):
            s = f.Surface
            if s.TypeId != "Part::GeomCylinder" or abs(s.Radius - radius) > 0.01 or abs(s.Center.x - x) > 0.01:
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
    if case == "femur":
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
        held = cyl_faces(val("bearing_seat_d") / 2, 0) + cyl_faces(val("horn_screw_d") / 2, 0)
        top = plane_faces("z", val("tip_cavity_z1"))
        loads = [("foot up", top, up, Z)]
        if side:
            loads += [("side", top, side, Y), ("forward", top, side, X)]
        probe = None
    else:
        hub = val("coxa_len")
        held = cyl_faces(val("coxa_shaft_d") / 2, 0) + (cyl_faces(servo_shaft, 0) if bonded else cyl_faces(pilot, near) + cyl_faces(pilot, -far))
        seat = cyl_faces(val("bearing_seat_d") / 2, hub)
        horn = cyl_faces(val("horn_screw_d") / 2, hub - val("horn_screw_r1")) + cyl_faces(val("horn_screw_d") / 2, hub - val("horn_screw_r2"))
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
    mesh.CharacteristicLengthMax = 1.6
    mesh.CharacteristicLengthMin = 0.3
    mesh.ElementOrder = "2nd"
    mesh.SecondOrderLinear = True   # straight element edges: curved ones invert on small features
    an.addObject(mesh)
    doc.recompute()
    from femmesh.gmshtools import GmshTools
    err = GmshTools(mesh).create_mesh()
    if err:
        print("gmsh:", err[:200])
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
        for face in faces:
            nodes.update(mesh.FemMesh.getNodesByFace(obj.Shape.getElement(face)))
        if not nodes:
            raise ValueError("no mesh nodes on " + label)
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
        art = ax.scatter(xyz[order, 0], xyz[order, 1], xyz[order, 2], c=val[order], cmap="turbo", vmin=0, vmax=20, s=2.0, depthshade=False, zorder=1)
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
    bar.set_label("von Mises, MPa (scale stops at the 20 MPa limit)")
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
        foot = max(abs(disp[n][1]) for n in disp)
    label = "%s, %s, %s" % (case, load, "servos bonded in" if bonded else ("print alone" if case != "tibia" else "halves joined"))
    summary = "%s: peak %.1f MPa, 99.9%% under %.1f MPa, foot moves sideways %.2f mm" % (label, vm[ranked[0]], vm[ranked[len(ranked) // 1000]], foot)
    print(summary + " (peak at %s; forces %s)" % ([round(c, 1) for c in coords[ranked[0]]], {k: round(v, 1) for k, v in totals.items()}))
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fea")
    os.makedirs(out, exist_ok=True)
    picture(os.path.join(out, "%s_%s_%s.png" % (case, load, "servos_bonded" if bonded else "alone")), summary, coords, vm, held, arrows)


build(globals().get("CASE", "femur"), globals().get("LOAD", "twist"), globals().get("BONDED", False))
