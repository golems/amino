
#version 3.7;

#include "colors.inc"    // The include files contain
#include "out.inc"
//#include "povray/opt/ros/indigo/share/baxter_description/meshes/electric_gripper/electric_gripper_w_fingers.DAE.inc"
#include "/tmp/robray/povray/opt/ros/indigo/share/pr2_description/meshes/base_v0/base.inc"

camera {
    //location <1.5, 1, 1.5>
  location <2, 2, 2>
  look_at  <0,    .0,  0>
  right x*16/9 /* Widescreen */
}

background { color rgb <.5, .5, .5> }


global_settings { ambient_light rgb<.2, .2, .2> }
light_source{<10,20,-15>
  // Sun
  color White
  parallel
  jitter
  adaptive 1
}

mesh2 {
   base2_M1K_base2_M1KShape
}
