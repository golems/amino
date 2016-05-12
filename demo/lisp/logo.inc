#include "colors.inc"    // The include files contain


#include "textures.inc"    // pre-defined scene elements
#include "shapes.inc"
#include "glass.inc"
#include "metals.inc"
#include "woods.inc"
#include "skies.inc"


global_settings { ambient_light rgb<.2, .2, .2> }
light_source{<10,20,-15>
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
    color White
 //   Apocalypse
  }


  scale 0.3
  rotate <0,3,3>
}
