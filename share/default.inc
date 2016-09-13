sky_sphere{
  pigment {
    color rgb<1,1,1>
  }
  scale 0.3
  rotate <0,3,3>
}


global_settings { ambient_light rgb<.2, .2, .2> }
light_source{<10,20,-15>
  // Sun
  color rgb<1,1,1>
  parallel
  jitter
  adaptive 1
}
