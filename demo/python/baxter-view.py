#!/usr/bin/env aarxpy


import aminopy as aa
import CL

scene = aa.load_scene( 'package://baxter_description/urdf/baxter.urdf' )
aa.win_set_scene(scene)
aa.win_view_collision()
aa.win_run_sync()
