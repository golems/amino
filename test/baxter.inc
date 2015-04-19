
#include "colors.inc"    // The include files contain
#include "stones.inc"    // pre-defined scene elements

#include "textures.inc"    // pre-defined scene elements
#include "shapes.inc"
#include "glass.inc"
#include "metals.inc"
#include "woods.inc"
#include "skies.inc"


#declare Black_Marble_Map =
color_map {
    [0.0 rgb <0.0, 0.0, 0.0>]
    [0.8 rgb <0.1, 0.1, 0.1>]
    [1.0 rgb <0.3, 0.3, 0.3>]
}

// Black marble with black veins.
#declare Black_Marble =
pigment {
    marble
    turbulence 2
    color_map { Black_Marble_Map }
}


camera {
    //location <1.5, 1, 1.5>
  location <3.0, 1.5, -1.0>
  look_at  <0,    .25,  0>
  right x*16/9 /* Widescreen */
}

#include "stars.inc"
sky_sphere{ //<0,0,0>, 1
  //S_Cloud3
  pigment {
    //Blood_Sky
    Blue_Sky
 //   Apocalypse
  }


  scale 0.3
  rotate <0,3,3>
}

box {

    <-1.5,0,-1.5>,  // Near lower left corner
    < 1.5, 0, 1.5>   // Far upper right corner

  //texture {

    //pigment{ color rgb<0.22,0.45,0>}
    //normal { bumps 0.75 scale 0.015 }
    //finish { phong 0.1 }
    //} // end of texture

  texture {
    scale 12*0.0254
    pigment
    {
      checker pigment { Black_Marble }, pigment { White_Marble }
      //checker color Black, color White
    }
    //finish{ phong 0.5 }
  }
  translate <0,-.9,0>

}

// The Sun
global_settings { ambient_light rgb<.5, .5, .5> }
light_source{<1500,3000,-2500> color White}
