"""Pictures of a part alone, from all around, into views/ (not in git). Run inside FreeCAD
with init_globals={"TARGETS": {"name": ["Object", ...]}}. Puts what was showing back."""
import os

import FreeCAD as App
import FreeCADGui as Gui

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "views")
os.makedirs(OUT, exist_ok=True)
doc = App.getDocument("hexapod")
App.setActiveDocument("hexapod")
Gui.ActiveDocument = Gui.getDocument("hexapod")
view = Gui.ActiveDocument.ActiveView
if not hasattr(view, "saveImage"):
    raise RuntimeError("the active window isn't a 3D view")
was = {o.Name: o.ViewObject.Visibility for o in doc.Objects if hasattr(o, "ViewObject") and o.ViewObject}
camera = view.getCamera()
SIDES = (("iso", view.viewIsometric), ("front", view.viewFront), ("rear", view.viewRear), ("right", view.viewRight),
         ("left", view.viewLeft), ("top", view.viewTop), ("bottom", view.viewBottom))
try:
    for name, objects in TARGETS.items():
        for o in doc.Objects:
            if hasattr(o, "ViewObject") and o.ViewObject and o.TypeId != "App::Part":
                o.ViewObject.Visibility = False
        for n in objects:
            o = doc.getObject(n)
            o.ViewObject.Visibility = True
            parent = o.getParentGeoFeatureGroup()
            if parent:
                parent.ViewObject.Visibility = True
        for side, turn in SIDES:
            turn()
            Gui.Selection.clearSelection()
            for n in objects:
                Gui.Selection.addSelection(doc.getObject(n))
            Gui.SendMsgToActiveView("ViewSelection")
            Gui.Selection.clearSelection()
            Gui.updateGui()
            view.saveImage(os.path.join(OUT, "%s_%s.png" % (name, side)), 760, 560, "White")
finally:
    for n, shown in was.items():
        o = doc.getObject(n)
        if o:
            o.ViewObject.Visibility = shown
    view.setCamera(camera)
print(sorted(os.listdir(OUT))[:6], len(os.listdir(OUT)))
