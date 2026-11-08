"""The foot switch and presser exactly as in the old (gear) hexapod, which works after printing:
the empty space around the switch inside its tibia (tibiaCutForTip, tibiaCutForButton and the
pockets around them, taken from the finished halves so nothing is redrawn) and its TibiaTip
presser. Writes old_tip/voids.brep and old_tip/presser.brep, still in the old file's frame:
tibia 110 long along -Z, its axis at x 0, y 12 (the split plane), the presser's axis at x 0, y 0.
build_tibia.py moves them onto the new tibia's tip. Run once with freecad.cmd (or inside FreeCAD);
the BREPs are kept in git so the model builds without the old file.
"""
import os

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
OLD_FILE = "/home/sovgvd/Dropbox/robots/hexapod/hexapod.FCStd"
AXIS_Y = 12.0                   # the old tibia's axis and split plane
BOX = (3.6, 3.6, -106.0, -88.4)  # half x, half y, z from (below the halves' end) to (just under the old screw at -87.2)

old = App.openDocument(OLD_FILE)
by_label = {o.Label: o for o in old.Objects}
halves = by_label["TibiaWithButton_side1"].Shape.fuse(by_label["TibiaWithButton_side2"].Shape)
hx, hy, z0, z1 = BOX
box = Part.makeBox(2 * hx, 2 * hy, z1 - z0, App.Vector(-hx, AXIS_Y - hy, z0))
voids = max(box.cut(halves).Solids, key=lambda s: s.Volume)     # the box's edge clips off a sliver of a screw recess
presser = by_label["TibiaTip"].Shape.copy()
App.closeDocument(old.Name)

out = os.path.join(HERE, "old_tip")
os.makedirs(out, exist_ok=True)
voids.exportBrep(os.path.join(out, "voids.brep"))
presser.exportBrep(os.path.join(out, "presser.brep"))
print("voids %.1f mm3 %s, presser %.1f mm3 %s" % (voids.Volume, voids.BoundBox, presser.Volume, presser.BoundBox))
