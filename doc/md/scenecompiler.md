Scene Graph Compiler {#scenecompiler}
====================

[TOC]

The scene graph compiler `aarxc` parses scene files and outputs C
code, which you can compile and link either statically into your
application or into a shared library.


Why compile scenes? {#scenecompiler_why}
===================

* **Fast:** Compiled scene are fast to load (<1ms) because the
  operating system directly maps into memory (via
  [mmap](https://en.wikipedia.org/wiki/Mmap)) the included mesh data,
  eliminating runtime parsing and processing.

* **Memory-Efficient:** Compiled scenes reduce memory use compared to
  runtime parsing when multiple processes operate on the same scene,
  because the memory mapped scene graphs in different processes share
  physical memory.

* **Convenient:** Compiled scenes are easy to distribute to other
  machines -- e.g., a cluster -- which may lack scene sources,
  utilities, or support libraries. Only the executable or shared
  library is required to load the compiled scene, reducing potential
  runtime dependencies.

* **Real-Time:** Compiled scenes avoid the need to include large
  parsing libraries -- e.g., an XML parser -- in real-time processes
  and reduce the dynamic allocations necessary to load the scene.

* **Composable:** Multiple compiled scenes can be efficiently composed
  at runtime.  Thus, the static scene graph for the robot could be
  compiled ahead of time for fast loading and then extended at runtime
  with the environment data either procedurally or by separately
  compiling the environment scene.

Compiled scenes do, however, present an initial, one-time compilation
cost, around 10-20 seconds for common robot models.  The following
table summarizes average compilation time (10 runs) -- including mesh
processing, code generation, and C compilation -- using Blender 2.77
and GCC 4.9.2 on an Intel(R) Core(TM) i7-4790:

| Robot          | Compilation Time | Run-Time Loading |
|----------------|------------------|------------------|
| Rethink Baxter | 14.0s            | 0.33ms           |
| Universal UR10 | 11.7s            | 0.17ms           |
| Kinova Jaco    | 15.6s            | 0.20ms           |

Compiling Scene Files {#scenecompiler_compiling}
=====================

The following command will convert the @ref scenefile_example into the
C file `table.c`:

    aarxc table.robray -n table -o table.c

To simply view a scene file, use the `--gui` option:

    aarxc --gui table.robray

**Note:** Aarxc converts any meshes referenced in the source scene
  file to [Wavefront OBJ]
  (https://en.wikipedia.org/wiki/Wavefront_.obj_file) using [Blender]
  (https://www.blender.org/) and then directly parse the OBJ file.  To
  load non-OBJ meshes (e.g., [COLLADA/DAE]
  (https://en.wikipedia.org/wiki/COLLADA)), you must have Blender
  installed.

@sa @ref scenefile

Loading Scene Graphs {#scenecompiler_loading}
====================

If your application needs to deal with only a particular scene graph,
then you can link directly against the compiled C file.

If your application may need to deal with a variety of scene graphs,
you can compile the C files for the scene graphs as shared objects and
[dynamically load](https://en.wikipedia.org/wiki/Dynamic_loading) them
using [aa_rx_dl_sg()](@ref aa_rx_dl_sg).


## Static Linking

If you have a single scene graph and a single application, then it is
sufficient to statically link the scene graph into your application,
just as you would with any other C file.  For example, to compile and
link using GCC:

    PKG_CONFIG_MODULES="amino amino-gl sdl2 glew"
    CFLAGS="$CFLAGS `pkg-config --cflags $PKG_CONFIG_MODULES`"
    LIBS="$LDLAGS `pkg-config --libs $PKG_CONFIG_MODULES`"
    gcc $CFLAGS $LIBS table.c MY_OTHER_SOURCE_FILES -o MY_PROGRAM

(Of course, you would typically use a build automation tool such as
Make, the Autotools, or CMake.)

Then, load the scene graph with the following C code:

~~~~~~~~~~~~~~~~~~~{.c}
struct aa_rx_sg *scenegraph = aa_rx_dl_sg__table(NULL);
~~~~~~~~~~~~~~~~~~~


@sa [Autools pkg-config support via PKG_CHECK_MODULES](https://autotools.io/pkgconfig/pkg_check_modules.html)
@sa CMake's `FindPkgConfig`


## Dynamic Linking

If multiple applications need to use the same scene graph, you will
reduce disk and memory use by compiling the scene graph into a shared
object.  The details of building shared libraries vary by platform and
are best managed with build automation tools such as the Autotools or
CMake.  For simple cases using gcc on GNU/Linux, you can build a
shared library as follows:

    PKG_CONFIG_MODULES="amino"
    CFLAGS="$CFLAGS `pkg-config --cflags $PKG_CONFIG_MODULES`"
    gcc -shared -fPIC $CFLAGS table.c -c -o libscene-table.so

The code to load the scene graph is identical to the static linking
case.


@sa [Shared Libraries with Automake](https://www.gnu.org/software/automake/manual/automake.html#A-Shared-Library)
@sa CMake's `add_library`

## Dynamic Loading

For dynamic loading, the scene graph C file must be compiled into a
shared object.  Amino provides the convenience function
[aa_rx_dl_sg](@ref aa_rx_dl_sg) which will load the shared object (via
[dlopen](https://en.wikipedia.org/wiki/Dynamic_loading)) and call the
contained function to load the scene graph.

To load the previous table example after compiling the C code to
shared object "libscene-table.so":

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
struct aa_rx_sg *scenegraph = aa_rx_dl_sg( "scene-table",
                                           "table",
                                           NULL );
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

@sa [Building plugins with Autotools](https://autotools.io/libtool/plugins.html)

Compiling URDF {#scenecompiler_urdf}
==============

Set the `ROS_PACKAGE_PATH` environment variable to point at your ROS
installation, and call `aarxc`.  For example, to compile the URDF for
the Baxter robot:

    export ROS_PACKAGE_PATH=/opt/ros/indigo/share/
    aarxc package://baxter_description/urdf/baxter.urdf -o baxter-model.c -n baxter

The scene compiler can also convert URDF to its human-readable
scene file syntax:

    aarxc package://baxter_description/urdf/baxter.urdf -s baxter.robray

Visualization {#scenecompiler_visualization}
=============

When given the `--gui option`, the scene compiler will launch the @ref viewer.

    aarxc --gui package://baxter_description/urdf/baxter.urdf

You can set joint positions using the `--var` and `--val` options.
The `--list-vars` option will display the names of all configuration
variables.

    aarxc --list-vars package://baxter_description/urdf/baxter.urdf
    aarxc --gui  package://baxter_description/urdf/baxter.urdf \
        --var=right_s1 --val=-.5 \
        --var=right_e1 --val=1

In addition to the @ref viewer, you can also create raytraced images
using POV-Ray:

    aarxc package://baxter_description/urdf/baxter.urdf \
        -p scene.pov --render \
        --cam-eye="2 2 1.75" \
        --cam-look="0 0 .5"

Options Summary {#scenecompiler_opts}
===============

@sa [Man Page] (man_aarxc.html)

@htmlonly
<iframe src="man_aarxc.html"
        height="800"
        width="100%"
        >

</iframe>
@endhtmlonly
