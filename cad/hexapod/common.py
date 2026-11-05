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


def save(doc):
    doc.recompute()
    bad = [o.Name for o in doc.Objects if o.State and ("Invalid" in o.State or "Error" in o.State)]
    if bad:
        raise RuntimeError("recompute failed: %s" % bad)
    doc.save()
