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


def chamfer_edges(shape, size, skip, min_length=1.5):
    """Straight edges to chamfer, for a Part::Chamfer: all of them, inside corners too,
    except those `skip(edge)` says must stay sharp (servo fits). Longest first; edges
    the kernel can't do together with the others are left out, and counted."""
    wanted = [i for i, e in enumerate(shape.Edges)
              if e.Curve.TypeId == "Part::GeomLine" and e.Length > min_length * size and not skip(e)]
    wanted.sort(key=lambda i: -shape.Edges[i].Length)

    def works(items):
        try:
            result = shape.makeChamfer(size, [shape.Edges[i] for i in items])
            return result.isValid() and len(result.Solids) == 1
        except Exception:
            return False

    kept = wanted if works(wanted) else []
    if not kept:
        for i in wanted:
            if works(kept + [i]):
                kept.append(i)
    print("  chamfers: %d of %d straight edges" % (len(kept), len(wanted)))
    return [(i + 1, size, size) for i in kept]


def rounded_face(points, plane):
    """A flat outline with rounded corners. points = (u, v, radius) in order; plane is
    "xy", "xz" or "yz". Radii that don't fit are reduced."""
    import math
    import Part
    n = len(points)
    to3d = {"xy": lambda u, v: App.Vector(u, v, 0), "xz": lambda u, v: App.Vector(u, 0, v), "yz": lambda u, v: App.Vector(0, u, v)}[plane]
    corners = []
    for i in range(n):
        px, py, r = points[i]
        ax, ay = points[i - 1][:2]
        bx, by = points[(i + 1) % n][:2]
        ux, uy = ax - px, ay - py
        vx, vy = bx - px, by - py
        lu, lv = math.hypot(ux, uy), math.hypot(vx, vy)
        ux, uy, vx, vy = ux / lu, uy / lu, vx / lv, vy / lv
        half = math.acos(max(-1.0, min(1.0, ux * vx + uy * vy))) / 2
        cut = r / math.tan(half) if r > 0 else 0.0
        limit = 0.48 * min(lu, lv)
        if cut > limit:
            cut = limit
            r = cut * math.tan(half)
        corners.append((px, py, ux, uy, vx, vy, cut, r, half))
    edges = []
    for i in range(n):
        px, py, ux, uy, vx, vy, cut, r, half = corners[i]
        start = (px + ux * cut, py + uy * cut)
        end = (px + vx * cut, py + vy * cut)
        if r > 1e-6:
            bx, by = ux + vx, uy + vy
            lb = math.hypot(bx, by)
            bx, by = bx / lb, by / lb
            dist = r / math.sin(half)
            mid = (px + bx * (dist - r), py + by * (dist - r))
            edges.append(Part.Arc(to3d(*start), to3d(*mid), to3d(*end)).toShape())
        nx, ny, _, __, nvx, nvy, ncut, ___, ____ = corners[(i + 1) % n]
        n_start = (nx + _ * ncut, ny + __ * ncut)
        edges.append(Part.LineSegment(to3d(*end), to3d(*n_start)).toShape())
    return Part.Face(Part.Wire(edges))


def prism(points, plane, lo, hi):
    """rounded_face pushed along the plane's normal, from lo to hi."""
    normal = {"xy": App.Vector(0, 0, 1), "xz": App.Vector(0, 1, 0), "yz": App.Vector(1, 0, 0)}[plane]
    face = rounded_face(points, plane)
    face.translate(normal * lo)
    return face.extrude(normal * (hi - lo))


def block(x0, x1, y0, y1, z0, z1):
    import Part
    return Part.makeBox(x1 - x0, y1 - y0, z1 - z0, App.Vector(x0, y0, z0))


def rod(d, length, base, direction):
    """Cylinder from base along direction: "x", "y" or "z"."""
    import Part
    axis = {"x": App.Vector(1, 0, 0), "y": App.Vector(0, 1, 0), "z": App.Vector(0, 0, 1)}[direction]
    return Part.makeCylinder(d / 2.0, length, App.Vector(*base), axis)


def smooth(shape, size, skip):
    """Round the edges, except those skip(edge) says must stay sharp. Whatever the kernel
    refuses to round gets a chamfer if it can; the rest stay sharp, and are counted."""
    wanted = [i for i, e in enumerate(shape.Edges) if e.Length > size * 1.5 and not skip(e)]
    wanted.sort(key=lambda i: -shape.Edges[i].Length)

    def attempt(items, make):
        try:
            result = make(size, [shape.Edges[i] for i in items])
            return result if result.isValid() and len(result.Solids) == 1 else None
        except Exception:
            return None

    result = attempt(wanted, shape.makeFillet)
    kept = wanted
    if result is None:
        kept = []
        for i in wanted:
            trial = attempt(kept + [i], shape.makeFillet)
            if trial is not None:
                kept.append(i)
                result = trial
    print("  rounded %d of %d edges" % (len(kept), len(wanted)))
    return result if result is not None else shape


def as_part(doc, part, name, shape, color, visible=True):
    obj = doc.addObject("Part::Feature", name)
    part.addObject(obj)
    obj.Shape = shape
    obj.ViewObject.ShapeColor = color
    obj.ViewObject.Visibility = visible
    return obj


def add_mirror(doc, part, obj, color):
    mirror = doc.addObject("Part::Mirroring", obj.Name + "_L")
    part.addObject(mirror)
    mirror.Source = obj
    mirror.Normal = App.Vector(0, 1, 0)
    mirror.ViewObject.Visibility = False
    mirror.ViewObject.ShapeColor = color
    return mirror
