#include "colors.inc"    // The include files contain
#include "stones.inc"    // pre-defined scene elements

#include "textures.inc"    // pre-defined scene elements
#include "shapes.inc"
#include "glass.inc"
#include "metals.inc"
#include "woods.inc"


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
    location <5, 3, 5>
    look_at  <0,    0,  0>
}

//light_source{<1500,3000,-2500> color White}

//global_settings { ambient_light rgb<.1, .1, .1> }

light_source
{ <10, 10, 00>, 1
  fade_distance 6 fade_power 2
  area_light x*3, y*3, 12, 12 circular orient adaptive 0
}


// sky -------------------------------------
plane{ <0,1,0>,1 hollow
       texture{
         pigment{ bozo turbulence 0.92
           color_map{
                 [0.00 rgb<0.05,0.15,0.45>]
                 [0.50 rgb<0.05,0.15,0.45>]
                 [0.70 rgb<1,1,1>        ]
                 [0.85 rgb<0.2,0.2,0.2>  ]
                 [1.00 rgb<0.5,0.5,0.5>  ]
                       } //
           scale<1,1,1.5>*2.5
           translate<0,0,0>
           } // end of pigment
         finish {ambient 1 diffuse 0}
        } // end of texture
       scale 10000}

// fog on the ground -----------------------

fog { fog_type   2
      distance   50
      color      rgb<1,1,1>*0.8
      fog_offset .01
      fog_alt    .2
      turbulence 10
    } //

box {
    <-1,0,-1>,  // Near lower left corner
    < 1, 2, 1>   // Far upper right corner
    texture {
      //T_Stone25     // Pre-defined from stones.inc
      //scale 4       // Scale by the same amount in all
                    // directions
      Jade
      finish { phong 1 }
    }
}

plane { <0, 1, 0>, 0

    //texture {

          //pigment{ color rgb<0.22,0.45,0>}
          //normal { bumps 0.75 scale 0.015 }
          //finish { phong 0.1 }
       //} // end of texture

    texture {

   pigment
   {
      checker pigment { Black_Marble }, pigment { White_Marble }
      //checker pigment { Jade }, pigment { White_Marble }
    }
    //finish{ phong 0.5 }
}
}
