#!/usr/bin/env aarxpy


import aminopy as aa
import CL

s = aa.scene(
    aa.load_scene( 'package://baxter_description/urdf/baxter.urdf' ),
    aa.frame_fixed( "", "block", aa.tf2(1, [1,0,0]) )
)

s = s.add_geom("block",
               aa.geom_box({'color': [1,0,0]},
                           [.25, .25, .25]))

aa.win_set_scene(s)
aa.win_view_collision()
aa.win_run_sync()
