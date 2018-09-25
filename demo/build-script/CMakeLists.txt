cmake_minimum_required (VERSION 2.6)
project (AminoExample)

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

add_executable(baxter-simple baxter-simple.c baxter-model.c)

# Set the include directories
target_include_directories( baxter-simple PUBLIC
  ${SDL2_INCLUDE_DIRS}
  ${AMINO_INCLUDE_DIRS}
  )

# Link the necessary libraries
target_link_libraries( baxter-simple
  ${AMINO_GL_LIBRARIES}
  ${AMINO_LIBRARIES}
  ${SDL2_LIBRARIES}
  ${GL_LIBRARIES}
  )
