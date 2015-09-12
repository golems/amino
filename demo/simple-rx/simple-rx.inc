
#include "colors.inc"    // The include files contain

camera {
    //location <1.5, 1, 1.5>
  location <1., 1.0, 0.5>
  look_at  <0,    .0,  0>
  right x*16/9 /* Widescreen */
}


global_settings { ambient_light rgb<1, 1, 1> }
light_source{<10,20,15>
  // Sun
  color White
  parallel
  jitter
  adaptive 1
}

sky_sphere{ //<0,0,0>, 1
  //S_Cloud3
  pigment {
    //Blood_Sky
    //Blue_Sky
 //   Apocalypse
    White
  }


  //scale 0.3
  //rotate <0,3,3>
}
