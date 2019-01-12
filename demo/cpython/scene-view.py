#!/usr/bin/env python

from amino import *


sg = SceneGraph()
sg.add_frame_fixed("", "box", QuatTrans.identity())
sg.attach_geom("box",  Geom.Box(None, [1,1,1]))
sg.init()

win = SceneWin()
win.set_scenegraph(sg)

win.start(async=False)
