"""FEA of the femur print alone, in its own unsaved document, with CalculiX.

The servos stiffen the real part and are left out here, so this is well on the safe
side: between its straps this print has only the plate, and the servos are what
close the section. It holds the femur where the coxa does: at the hip shaft and the femur servo's
tab screws. The loads come in at the knee, half through its shaft, half through the
tibia servo's tab screws. Run inside FreeCAD with init_globals={"LOAD": name}; add
"BONDED": True for both servos glued in solid, the opposite limit.

  walk   foot force 8 N up (900 g on 3 feet, x3 for walking), tibia servo at stall
  twist  walk + 4 N sideways at the foot, 60 mm below the knee
  fall   30 N up + 15 N sideways at the foot: landing hard on one leg (an estimate)

Solves and prints the result itself: FreeCAD's writer leaves the *CLOAD sections
empty on these faces (0 MPa, which looks like a pass), so this script adds the node
loads to the solver input, checks their sum, and reads the .frd file.

PLA solid and isotropic: E 3500 MPa, nu 0.36. Prints are weaker across layers: judge
against about 20 MPa. Foot movement counts the print only, with no servo, bearing or
horn play.
"""
import os
import re
import subprocess
import tempfile

import FreeCAD as App
import ObjectsFem

STALL_NM = 0.19
LOADS = {"walk": (8.0, 0.0, True), "twist": (8.0, 4.0, True), "fall": (30.0, 15.0, False)}
TIBIA_M = 0.060


def build(load, bonded=False):
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
    obj.Shape = src.getObject("Femur_Main").Shape.copy()
    if bonded:
        # both servos glued in, as solid plastic: the opposite limit to the print alone.
        # The femur servo's shaft is then held by the coxa's horn, and the tibia's horn
        # loads the tibia servo's shaft
        rot = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
        seat_y = val("joint_seat_y")
        pieces = []
        for x, r in ((0, rot.multiply(App.Rotation(App.Vector(0, 0, 1), 180))), (val("femur_len"), rot)):
            for n in ("MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"):
                piece = src.getObject(n).Shape.copy()
                piece.Placement = App.Placement(App.Vector(x, seat_y, 0), r).multiply(piece.Placement)
                pieces.append(piece)
        obj.Shape = obj.Shape.multiFuse(pieces).removeSplitter()
        if len(obj.Shape.Solids) != 1:
            raise ValueError("did not fuse into one solid: %d" % len(obj.Shape.Solids))
    doc.recompute()
    an = ObjectsFem.makeAnalysis(doc, "Analysis")
    mat = ObjectsFem.makeMaterialSolid(doc, "PLA")
    mat.Material = {"Name": "PLA", "YoungsModulus": "3500 MPa", "PoissonRatio": "0.36", "Density": "1240 kg/m^3"}
    mat.References = [(obj, ("Solid1",))]
    an.addObject(mat)

    def cyl_faces(radius, x):
        out = []
        for i, f in enumerate(obj.Shape.Faces):
            s = f.Surface
            if s.TypeId == "Part::GeomCylinder" and abs(s.Radius - radius) < 0.01 and abs(s.Center.x - x) < 0.01:
                out.append("Face%d" % (i + 1))
        if not out:
            raise ValueError("no cylinder r %g at x %g" % (radius, x))
        return out

    fl = val("femur_len")
    near, far = val("mg90_hole_near"), val("mg90_hole_far")
    pilot, shaft = val("mg90_screw_pilot_d") / 2, val("fem_shaft_d") / 2
    fixed = ObjectsFem.makeConstraintFixed(doc, "HeldByCoxa")
    servo_shaft = val("mg90_shaft_d") / 2
    held = cyl_faces(servo_shaft, 0) if bonded else cyl_faces(pilot, -near) + cyl_faces(pilot, far)
    fixed.References = [(obj, tuple(cyl_faces(shaft, 0) + held))]
    an.addObject(fixed)

    knee_shaft = cyl_faces(shaft, fl)
    # bonded: the second load path is the tibia servo's shaft, both ways
    tab_a = cyl_faces(servo_shaft, fl) if bonded else cyl_faces(pilot, fl - far)
    tab_b = tab_a if bonded else cyl_faces(pilot, fl + near)
    arm = (val("joint_horn_y") - 2.0 if bonded else val("joint_seat_y") - val("fem_plate_floor") / 2) - (val("fem_floor_y0") - 1.5)
    couple = STALL_NM / (val("mg90_screw_spacing") / 1000) if stall and not bonded else 0.0
    twist = side * TIBIA_M / (arm / 1000)
    Z, Y = 3, 2
    loads = [
        ("shaft up", knee_shaft, up / 2 - twist, Z), ("tab a", tab_a, up / 4 + couple + twist / 2, Z),
        ("tab b", tab_b, up / 4 - couple + twist / 2, Z),
    ]
    if side:
        loads.append(("side", knee_shaft, side, Y))

    mesh = ObjectsFem.makeMeshGmsh(doc, "Mesh")
    mesh.Shape = obj
    mesh.CharacteristicLengthMax = 1.5
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
    lines, totals = ["*CLOAD"], {}
    for label, faces, newtons, dof in loads:
        nodes = set()
        for face in faces:
            nodes.update(mesh.FemMesh.getNodesByFace(obj.Shape.getElement(face)))
        if not nodes:
            raise ValueError("no mesh nodes on " + label)
        lines += ["%d,%d,%.8E" % (n, dof, newtons / len(nodes)) for n in sorted(nodes)]
        totals[dof] = totals.get(dof, 0.0) + newtons
    text = open(fea.inp_file_name).read()
    match = re.search(r"^\*END STEP", text, flags=re.M)
    open(fea.inp_file_name, "w").write(text[:match.start()] + "\n".join(lines) + "\n" + text[match.start():])
    run = subprocess.run([fea.ccx_binary, "-i", os.path.splitext(os.path.basename(fea.inp_file_name))[0]], cwd=work,
                         capture_output=True, text=True, timeout=500, env=dict(os.environ, OMP_NUM_THREADS="4"))
    if "Job finished" not in run.stdout:
        raise RuntimeError("ccx failed: " + run.stdout[-600:])
    held_nodes = set()
    for face in fixed.References[0][1]:
        held_nodes.update(mesh.FemMesh.getNodesByFace(obj.Shape.getElement(face)))
    arrows = []
    for label, faces, newtons, dof in loads:
        nodes = set()
        for face in faces:
            nodes.update(mesh.FemMesh.getNodesByFace(obj.Shape.getElement(face)))
        arrows.append((label, nodes, newtons, dof))
    report(fea.inp_file_name[:-4] + ".frd", load, totals, fl, held_nodes, arrows, bonded)
    App.closeDocument(name)
    App.setActiveDocument("hexapod")


def picture(path, title, coords, vm, held, arrows):
    """Stress on the mesh nodes, from 2 sides. Blue = held, red arrows = loads going in."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    ids = sorted(vm)
    xyz = np.array([coords[n] for n in ids])
    val = np.array([vm[n] for n in ids])
    fig = plt.figure(figsize=(15, 7), facecolor="white")
    for k, (elev, azim, name) in enumerate(((28, 120, "rim side"), (28, -60, "strap side"))):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        order = np.argsort(val)
        art = ax.scatter(xyz[order, 0], xyz[order, 1], xyz[order, 2], c=val[order], cmap="turbo", vmin=0, vmax=20, s=2.0, depthshade=False)
        h = np.array([coords[n] for n in held])
        ax.scatter(h[:, 0], h[:, 1], h[:, 2], c="#1f4fff", s=10, depthshade=False, label="held")
        for label, nodes, newtons, dof in arrows:
            pts = np.array([coords[n] for n in nodes])
            c = pts.mean(axis=0)
            d = np.zeros(3)
            d[dof - 1] = 1.0 if newtons > 0 else -1.0
            size = 12 + 18 * min(abs(newtons) / 30.0, 1.0)
            ax.quiver(c[0] - d[0] * size, c[1] - d[1] * size, c[2] - d[2] * size, d[0], d[1], d[2], length=size, color="red", linewidth=2)
            ax.text(c[0] - d[0] * size, c[1] - d[1] * size, c[2] - d[2] * size, "%s %.1f N" % (label, abs(newtons)), color="red", fontsize=8)
        ax.view_init(elev=elev, azim=azim)
        ax.set_title(name)
        ax.set_xlabel("X to the knee, mm")
        ax.set_ylabel("Y, horn side +")
        ax.set_zlabel("Z")
        ax.set_box_aspect((np.ptp(xyz[:, 0]) + 30, np.ptp(xyz[:, 1]) + 30, np.ptp(xyz[:, 2]) + 30))
    bar = fig.colorbar(art, ax=fig.axes, shrink=0.7, pad=0.02)
    bar.set_label("von Mises, MPa (scale stops at the 20 MPa limit)")
    fig.suptitle(title, fontsize=12)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print("       picture: " + path)


def report(frd, load, totals, fl, held, arrows, bonded):
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
    # the knee: mean movement, and how far it tips, which the tibia turns into foot movement
    ring = [n for n in disp if abs(coords[n][0] - fl) < 8 and abs(coords[n][2]) < 8]
    shaft = [n for n in ring if coords[n][1] < -17]
    plate = [n for n in ring if coords[n][1] > -4.5]
    mean = lambda nodes, k: sum(disp[n][k] for n in nodes) / len(nodes)
    dy = sum(coords[n][1] for n in plate) / len(plate) - sum(coords[n][1] for n in shaft) / len(shaft)
    tip = (mean(plate, 2) - mean(shaft, 2)) / dy
    foot_side = mean(ring, 1) + tip * TIBIA_M * 1000
    print("%-6s von Mises max %.2f MPa at %s; 99.9%% under %.2f, 99%% under %.2f" % (
        load, vm[ranked[0]], [round(c, 1) for c in coords[ranked[0]]], vm[ranked[len(ranked) // 1000]], vm[ranked[len(ranked) // 100]]))
    print("       knee moves up %.3f, side %.3f mm; tips %.3f deg; foot moves sideways %.3f mm; forces %s" % (
        mean(ring, 2), mean(ring, 1), abs(tip) * 57.3, abs(foot_side), {k: round(v, 2) for k, v in totals.items()}))
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fea")
    os.makedirs(out, exist_ok=True)
    tag = "%s_%s" % (load, "servos_bonded" if bonded else "print_alone")
    picture(os.path.join(out, "femur_%s.png" % tag),
            "Femur, %s, %s: peak %.1f MPa, 99.9%% under %.1f MPa, foot moves sideways %.2f mm" % (
                load, "servos bonded in" if bonded else "print alone, no servos", vm[ranked[0]], vm[ranked[len(ranked) // 1000]], abs(foot_side)),
            coords, vm, held, arrows)
    for n in ranked[:300:60]:
        print("         %.2f MPa at %s" % (vm[n], [round(c, 1) for c in coords[n]]))


build(globals().get("LOAD", "twist"), globals().get("BONDED", False))
