"""Shared helpers for the hexapod CAD scripts. Run inside FreeCAD (execute_code)."""
import os
import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
DOC_NAME = "hexapod"
DOC_PATH = os.path.join(HERE, "hexapod.FCStd")


def get_doc():
    """The model document: the open one, else the saved file, else a new one."""
    if DOC_NAME in App.listDocuments():
        doc = App.getDocument(DOC_NAME)
    elif os.path.exists(DOC_PATH):
        doc = App.openDocument(DOC_PATH)
    else:
        doc = App.newDocument(DOC_NAME)
        doc.saveAs(DOC_PATH)
    App.setActiveDocument(doc.Name)
    return doc


def broken(doc):
    doc.recompute()
    return [o.Name for o in doc.Objects if o.State and ("Invalid" in o.State or "Error" in o.State)]


def save(doc):
    bad = broken(doc)
    if bad:
        raise RuntimeError("recompute failed: %s" % bad)
    doc.save()


class Prims:
    """Primitives driven by Params expressions, inside one App::Part; hidden
    until fused and cut into a finished print."""

    def __init__(self, doc, part, prefix):
        self.doc, self.part, self.prefix = doc, part, prefix

    def _prim(self, kind, name, x, y, z):
        obj = self.doc.addObject(kind, self.prefix + "_" + name)
        self.part.addObject(obj)
        for axis, expr in zip("xyz", (x, y, z)):
            obj.setExpression("Placement.Base." + axis, expr)
        obj.ViewObject.Visibility = False
        return obj

    def box(self, name, lx, ly, lz, x, y, z):
        """x, y, z = low corner."""
        obj = self._prim("Part::Box", name, x, y, z)
        obj.setExpression("Length", lx)
        obj.setExpression("Width", ly)
        obj.setExpression("Height", lz)
        return obj

    def cyl(self, name, d, h, x, y, z, along_y=False):
        """From x, y, z along +Z, or along +Y."""
        obj = self._prim("Part::Cylinder", name, x, y, z)
        obj.setExpression("Radius", "(%s) / 2" % d)
        obj.setExpression("Height", h)
        if along_y:
            obj.Placement = App.Placement(obj.Placement.Base, App.Rotation(App.Vector(1, 0, 0), -90))
        return obj

    def sphere(self, name, r, x, y, z):
        obj = self._prim("Part::Sphere", name, x, y, z)
        obj.setExpression("Radius", r)
        return obj

    def gusset(self, name, corner, u, v, lu, lv, along, length):
        """45-degree style internal chamfer, a triangular prism driven by Params.
        corner = right-angle edge at the low end along `along`; u, v = axis and
        sign of each leg, e.g. ("x", 1), ("z", -1); lu, lv = leg lengths."""
        axes = {"x": App.Vector(1, 0, 0), "y": App.Vector(0, 1, 0), "z": App.Vector(0, 0, 1)}
        lu_vec, lv_vec = axes[u[0]] * u[1], axes[v[0]] * v[1]
        extrude = lu_vec.cross(lv_vec)
        rot = App.Rotation(App.Matrix(lu_vec.x, lv_vec.x, extrude.x, 0, lu_vec.y, lv_vec.y, extrude.y, 0,
                                      lu_vec.z, lv_vec.z, extrude.z, 0, 0, 0, 0, 1))
        # the prism grows along the local Z: start it from the far end if that points backwards
        flip = extrude.dot(axes[along]) < 0
        base = {a: e for a, e in zip("xyz", corner)}
        if flip:
            base[along] = "%s + %s" % (base[along], length)
        obj = self.doc.addObject("Part::Wedge", self.prefix + "_" + name)
        self.part.addObject(obj)
        for prop, expr in (("Xmin", "0 mm"), ("Ymin", "0 mm"), ("Zmin", "0 mm"), ("X2min", "0 mm"), ("X2max", "0 mm"),
                           ("Z2min", "0 mm"), ("Xmax", lu), ("Ymax", lv), ("Zmax", length), ("Z2max", length)):
            obj.setExpression(prop, expr)
        for a in "xyz":
            obj.setExpression("Placement.Base." + a, base[a])
        obj.Placement = App.Placement(obj.Placement.Base, rot)
        obj.ViewObject.Visibility = False
        return obj

    def lattice(self, name, run, run0, pitch, n, width_axis, w_low, w_size, normal, n0, t):
        """Triangular truss holes through a plate: n cells along `run`, each cut
        into 2 triangles leaving a diagonal rib. Returns the cutting prisms."""
        s, rib = "Params.lattice_s", "Params.lattice_rib"
        lu = "(%s) - %s - %s" % (pitch, s, rib)
        lv = "(%s) - %s" % (w_size, s)
        high = "(%s) + (%s)" % (w_low, w_size)
        low_n, len_n = "(%s) - 1 mm" % n0, "(%s) + 2 mm" % t
        out = []
        for i in range(n):
            start = "(%s) + %d * (%s) + %s / 2" % (run0, i, pitch, rib)
            stop = "(%s) + %d * (%s) - %s / 2" % (run0, i + 1, pitch, rib)
            for tag, r, w, sr, sw in (("a", start, w_low, 1, 1), ("b", stop, high, -1, -1)):
                corner = {run: r, width_axis: w, normal: low_n}
                out.append(self.gusset("%s_%d%s" % (name, i, tag), (corner["x"], corner["y"], corner["z"]),
                                       (run, sr), (width_axis, sw), lu, lv, normal, len_n))
        return out

    def fuse(self, name, items):
        obj = self.doc.addObject("Part::MultiFuse", self.prefix + "_" + name)
        self.part.addObject(obj)
        obj.Shapes = items
        obj.Refine = True
        for i in items:
            i.ViewObject.Visibility = False
        obj.ViewObject.Visibility = False
        return obj

    def cut(self, name, solid, holes, color, mirror=True):
        """A finished print: solid minus holes, and its mirror for the other side."""
        base = self.fuse(name + "_solid", solid)
        tool = holes[0] if len(holes) == 1 else self.fuse(name + "_holes", holes)
        obj = self.doc.addObject("Part::Cut", name)
        self.part.addObject(obj)
        obj.Base, obj.Tool = base, tool
        obj.Refine = True
        obj.ViewObject.ShapeColor = color
        if mirror:
            copy = self.doc.addObject("Part::Mirroring", name + "_L")
            self.part.addObject(copy)
            copy.Source = obj
            copy.Normal = App.Vector(0, 1, 0)
            copy.ViewObject.Visibility = False
            copy.ViewObject.ShapeColor = color
        return obj


def remove_built(doc, prefixes):
    for o in [o for o in doc.Objects if o.Name.startswith(prefixes)]:
        doc.removeObject(o.Name)


def solids_of(doc, names, placement=None):
    """The separate solids of some built objects, moved as one: testing them one
    by one, because a compound of overlapping pieces gives wrong common volumes."""
    out = []
    for name in names:
        shape = doc.getObject(name).Shape.copy()
        for solid in shape.Solids or [shape]:
            s = solid.copy()
            if placement is not None:
                s.Placement = placement.multiply(s.Placement)
            out.append(s)
    return out


def clash(group_a, group_b):
    return sum(a.common(b).Volume for a in group_a for b in group_b)
