"""FEA of one finished print, in its own unsaved document, with CalculiX.

Run inside FreeCAD with init_globals={"CASE": "coxa"}, "femur" or "tibia", and optionally
"TWIST": True and, for the coxa, "BOTH": True (lower + top joined at the bosses,
held at the hip bearing and the horn: the realistic one) or "NO_TOP": True (the top
plate helps nothing, the worst case). Solves and
prints the result itself: FreeCAD's writer leaves the *CLOAD sections empty on
these faces (0 MPa, which looks like a pass), so this script adds the node loads
to the solver input, checks their sum, and reads the stresses from the .frd file.

Loads, on a standing robot of 815 g (params.py):
  foot force   815 g on 3 feet = 2.67 N, x3 for walking shocks = 8 N, up, at the
               next joint: half into its bearing cap, half into its servo tabs
  servo stall  the next servo at stall (0.19 N*m at 5 V) as a couple on its 2
               tab screws, 27.86 mm apart: +-6.8 N
  coxa stall   coxa link only: 0.19 N*m over the 60 mm link = 3.2 N sideways

  twist        a sideways foot force, friction 0.5 x 8 N = 4 N, at the next joint and
               acting through the leg below it: about the link's long axis
               0.4 N*m on the coxa (foot 100 mm below), 0.24 N*m on the femur (tibia
               60 mm), taken as a couple between the bearing cap and the servo tabs

PLA solid and isotropic: E 3500 MPa, nu 0.36. Real prints are weaker across the
layers, so judge the result against about 20 MPa, not the 50-60 MPa of the datasheet.
"""
import os
import re
import subprocess
import tempfile

import FreeCAD as App
import ObjectsFem
import Part

STALL_NM = 0.19
FOOT_N = 8.0
SIDE_N = 4.0
TWIST_LEVER = {"coxa": 0.100, "femur": 0.060}   # m, foot below the joint


def build(case, twist=False, no_top=False, both=False):
    src = App.getDocument("hexapod")
    sheet = src.getObject("Params")

    def val(alias):
        v = sheet.get(alias)
        return float(getattr(v, "Value", v))

    name = "hexapod_fem_" + case
    if name in App.listDocuments():
        App.closeDocument(name)
    doc = App.newDocument(name)
    obj = doc.addObject("Part::Feature", "Print")
    if case == "femur":
        obj.Shape = src.getObject("Femur_Main").Shape.copy()
    elif case == "tibia":
        obj.Shape = src.getObject("Tibia_Main").Shape.copy()
    elif both:
        # lower + top joined at the bosses, as if the 2 screws were glue
        obj.Shape = src.getObject("CoxaLink_Lower").Shape.fuse(src.getObject("CoxaLink_Top").Shape).removeSplitter()
        if len(obj.Shape.Solids) != 1:
            raise ValueError("the 2 prints did not join: %d solids" % len(obj.Shape.Solids))
    else:
        obj.Shape = src.getObject("CoxaLink_Lower").Shape.copy()
    zdir, ydir = 3, 2     # solver degrees of freedom
    doc.recompute()

    an = ObjectsFem.makeAnalysis(doc, "Analysis")
    mat = ObjectsFem.makeMaterialSolid(doc, "PLA")
    mat.Material = {"Name": "PLA", "YoungsModulus": "3500 MPa", "PoissonRatio": "0.36", "Density": "1240 kg/m^3"}
    mat.References = [(obj, ("Solid1",))]
    an.addObject(mat)

    def cyl_faces(radius, x, z=None):
        """Faces of cylinders of a radius with their axis through x (and z)."""
        out = []
        for i, f in enumerate(obj.Shape.Faces):
            s = f.Surface
            if s.TypeId != "Part::GeomCylinder" or abs(s.Radius - radius) > 0.01:
                continue
            c = s.Center
            if abs(c.x - x) < 0.01 and (z is None or abs(c.z - z) < 0.01):
                out.append("Face%d" % (i + 1))
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
        return out

    def fix(label, faces):
        if not faces:
            raise ValueError("no faces for " + label)
        c = ObjectsFem.makeConstraintFixed(doc, label)
        c.References = [(obj, tuple(faces))]
        an.addObject(c)

    loads = []

    def force(label, faces, newtons, axis):
        if not faces:
            raise ValueError("no faces for " + label)
        loads.append((label, faces, newtons, axis))

    seat_r = val("bearing_seat_d") / 2
    pilot_r = val("mg90_screw_pilot_d") / 2
    spacing = val("mg90_screw_spacing")
    couple = STALL_NM / (spacing / 1000)
    if case == "tibia":
        fix("Pin", cyl_faces(val("coxa_pin_d") / 2, 0))
        fix("HornScrews", cyl_faces(pilot_r, val("tibia_screw_x")) + cyl_faces(pilot_r, -val("tibia_screw_x")))
        # the foot force comes in through the screw across the shank
        screw = cyl_faces(pilot_r, 0, val("tip_screw_z"))
        force("FootUp", screw, FOOT_N, zdir)
        if twist:
            force("FootSide", screw, SIDE_N, ydir)
            force("FootForward", screw, SIDE_N, 1)
        return finish(doc, an, obj, loads, case)
    if case == "coxa":
        joint_x = val("coxa_len")
        fix("HipSeat", cyl_faces(seat_r, 0))
        pad_z = val("coxa_wall_top_z")
        if both:
            # the horn holds the top plate: its screws and the hub pocket
            faces = cyl_faces(val("horn_screw_d") / 2, val("horn_screw_r")) + cyl_faces(val("horn_screw_d") / 2, -val("horn_screw_r"))
            fix("Horn", faces + cyl_faces(val("horn_hub_d") / 2, 0))
        elif not no_top:
            fix("TopPlateBosses", plane_faces("z", pad_z))
    else:
        joint_x = val("femur_len")
        fix("Pin", cyl_faces(val("coxa_pin_d") / 2, 0))
        fix("HornScrews", cyl_faces(pilot_r, val("femur_screw_x")))
    hole_mid = joint_x + val("mg90_shaft_end") - val("mg90_body_len") / 2
    hole_a = cyl_faces(pilot_r, hole_mid - spacing / 2, 0)
    hole_b = cyl_faces(pilot_r, hole_mid + spacing / 2, 0)
    cap = cyl_faces(seat_r, joint_x, 0)
    force("FootOnCap", cap, FOOT_N / 2, zdir)
    force("TabInner", hole_a, FOOT_N / 4 + couple, zdir)
    force("TabOuter", hole_b, FOOT_N / 4 - couple, zdir)
    if case == "coxa":
        force("CoxaStall", cap, STALL_NM / (joint_x / 1000), ydir)
    if twist:
        tabs_y = val("coxa_seat_y") - val("coxa_wall_t") / 2
        cap_y = val("joint_cap_y0") + val("bearing_t") / 2
        pair = SIDE_N * TWIST_LEVER[case] / ((tabs_y - cap_y) / 1000)
        print("  twist %.2f N*m: +-%.1f N between cap and tabs, %.1f mm apart" % (SIDE_N * TWIST_LEVER[case], pair, tabs_y - cap_y))
        force("SideFoot", cap, SIDE_N, ydir)
        force("TwistCap", cap, -pair, zdir)
        force("TwistTabIn", hole_a, pair / 2, zdir)
        force("TwistTabOut", hole_b, pair / 2, zdir)

    return finish(doc, an, obj, loads, case)


def finish(doc, an, obj, loads, case):
    mesh = ObjectsFem.makeMeshGmsh(doc, "Mesh")
    mesh.Shape = obj
    mesh.CharacteristicLengthMax = 1.5
    mesh.CharacteristicLengthMin = 0.3
    mesh.ElementOrder = "2nd"
    mesh.SecondOrderLinear = True   # straight element edges: curved ones invert on small fillets and short edges
    mesh.OptimizeNetgen = True
    an.addObject(mesh)
    doc.recompute()
    from femmesh.gmshtools import GmshTools
    err = GmshTools(mesh).create_mesh()
    print("mesh", case, "nodes", mesh.FemMesh.NodeCount, "volumes", mesh.FemMesh.VolumeCount, "err", (err or "")[:200])
    solver = ObjectsFem.makeSolverCalculiXCcxTools(doc, "CalculiX")
    an.addObject(solver)
    doc.recompute()
    return solve(doc, an, solver, mesh, obj, loads, case)


def solve(doc, an, solver, mesh, obj, loads, case):
    from femtools import ccxtools
    work = tempfile.mkdtemp(prefix="hexapod_fem_%s_" % case)
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
        per_node = newtons / len(nodes)
        lines += ["%d,%d,%.8E" % (n, dof, per_node) for n in sorted(nodes)]
        totals[dof] = totals.get(dof, 0.0) + newtons
        print("  load %-12s %6.2f N along DOF %d on %d nodes" % (label, newtons, dof, len(nodes)))
    text = open(fea.inp_file_name).read()
    match = re.search(r"^\*END STEP", text, flags=re.M)
    text = text[:match.start()] + "\n".join(lines) + "\n" + text[match.start():]
    open(fea.inp_file_name, "w").write(text)
    name = os.path.splitext(os.path.basename(fea.inp_file_name))[0]
    run = subprocess.run([fea.ccx_binary, "-i", name], cwd=work, capture_output=True, text=True, timeout=500,
                         env=dict(os.environ, OMP_NUM_THREADS="4"))
    if "Job finished" not in run.stdout:
        raise RuntimeError("ccx failed: " + run.stdout[-600:])
    report(os.path.join(work, name + ".frd"), obj, case, totals)
    return doc.Name


def report(frd, obj, case, totals):
    """Stress and displacement from the .frd: fixed-width lines, -1 + node + values."""
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
            if block == "coords":
                coords[node] = vals
            elif block == "DISP":
                disp[node] = vals
            elif block == "STRESS":
                stress[node] = vals
    vm = {}
    for n, (xx, yy, zz, xy, yz, zx) in stress.items():
        vm[n] = (0.5 * ((xx - yy) ** 2 + (yy - zz) ** 2 + (zz - xx) ** 2) + 3 * (xy ** 2 + yz ** 2 + zx ** 2)) ** 0.5
    ranked = sorted(vm, key=vm.get, reverse=True)
    dmax = max(disp, key=lambda n: sum(v * v for v in disp[n]))
    print("  %s: %.1f g solid, %d nodes" % (case, obj.Shape.Volume / 1000 * 1.24, len(vm)))
    print("  von Mises max %.2f MPa at %s; 99.9%% of nodes below %.2f MPa; 99%% below %.2f MPa" % (
        vm[ranked[0]], [round(c, 1) for c in coords[ranked[0]]], vm[ranked[len(ranked) // 1000]], vm[ranked[len(ranked) // 100]]))
    print("  max displacement %.3f mm at %s, sum of applied forces %s" % (
        sum(v * v for v in disp[dmax]) ** 0.5, [round(c, 1) for c in coords[dmax]], totals))
    for n in ranked[:400:80]:
        print("    hot node %.2f MPa at %s" % (vm[n], [round(c, 1) for c in coords[n]]))
    return vm


print("doc", build(globals().get("CASE", "coxa"), globals().get("TWIST", False), globals().get("NO_TOP", False), globals().get("BOTH", False)))
