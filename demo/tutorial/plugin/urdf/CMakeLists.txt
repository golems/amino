cmake_minimum_required (VERSION 2.6)
project (AminoTutorials)

# pkg-config setup
find_package(PkgConfig)

# Search for the pkg-config packages
pkg_search_module(SDL2 REQUIRED sdl2)
pkg_search_module(GL REQUIRED gl)
pkg_search_module(AMINO REQUIRED amino)
pkg_search_module(AMINO_GL REQUIRED amino-gl)

# Scene graph compilation
add_custom_command(
  OUTPUT baxter-model.c baxter-model.c.h
  COMMAND aarxc "package://baxter_description/urdf/baxter.urdf" -o baxter-model.c -n "baxter"
  )

add_library( baxter-model MODULE baxter-model.c )
target_include_directories( baxter-model PUBLIC
  ${AMINO_INCLUDE_DIRS}
  )
target_link_libraries( baxter-model
  ${AMINO_GL_LIBRARIES}
  ${AMINO_LIBRARIES}
  )
