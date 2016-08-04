Scene Files {#scenefile}
=======================

[TOC]

Amino provides a convenient, human-readable syntax for specifying
scene graphs.  The scenefile syntax is curly-brace inspired and
supports constant definitions, object classes, and include files.
Amino's scenegraph compiler `aarxc` translates scenefiles to C code.


Statements {#scenefile_statements}
==========

Comments {#scenefile_comments}
-------

C-style block comments (`/* COMMENT */`), C++-style line comments (`//
COMMENT`), and shell-style line comments (`# COMMENT`) are supported.

### Comment Examples


    /* This is a block comment.
     * It can continue over multiple
     * lines.
    /*

    // This is a line comment.

    # This is also a line comment.


Include Statement {#scenefile_include_stmt}
-----------------

The include statement inserts the contents of the named file into the
current file.  Include files may be useful to reuse a shared set of
constants and class definitions over multiple scenes.

### Include Statement Grammar

    <INCLUDE_STMT> => "include" <STRING>

### Include Statement Example

    include "file1.robray"

Definition Statement {#scenefile_definition_stmt}
--------------------

The definition statement creates a named, numeric constant.  The value
may be computed as an arithmetic expression.

### Definition Statement Grammar

    <DEF_STMT> => "def" <ID> <EXP> ";"
    <EXP>      => <ID> | <FLOAT>
                | <EXP> <BINOP> <EXP>
                | "-" <EXP>
                | "(" <EXP> ")"

### Definition Statement Examples

#### Named Constant Definition

The following defines a single constant value:

    def my_length 0.1;

#### Expression Definition

The following defines constants for various length units:

    def cm 1e-2;
    def inch 2.54*cm;
    def foot 12*inch;


Frame Statement {#scenefile_frame_stmt}
---------------

The frame statement defines a kinematic frame, i.e., a pose in SE(3).

### Frame Statement Grammar

    <FRAME_STMT>              => "frame" <ID> "{" <FRAME_ATTRIBUTE_LIST> "}"
    <FRAME_ATTRIBUTE_LIST>    => <FRAME_ATTRIBUTE> |  <FRAME_ATTRIBUTE> <FRAME_ATTRIBUTE_LIST>
    <FRAME_ATTRIBUTE>         => <ID> <ATTRIBUTE_VALUE> ";"
                               | "geometry" "{" <GEOMETRY_ATTRIBUTE_LIST> "}"
                               | <FRAME_STMT>
    <GEOMETRY_ATTRIBUTE_LIST> => <GEOMETRY_ATTRIBUTE> |  <GEOMETRY_ATTRIBUTE> <GEOMETRY_ATTRIBUTE_LIST>
    <GEOMETRY_ATTRIBUTE>      => <ID> <ATTRIBUTE_VALUE> ";"
    <ATTRIBUTE_VALUE>         => <EXP> | <ARRAY>

### Frame Statement Example

    // Define some sizes for objects
    def table_size 1e-2;  //  1 cm
    def block_size .1;    // 10cm

    // Define a stacking height
    def table_stack (block_size + table_size)/2 + 1e-3;

    /* A table at the world origin */
    frame front_table {
        geometry {
            dimension [.75, .75, table_size];
            shape box;
            color [.6, .3, .6];
        }
    }

    // This block is placed on the table
    frame block_a {
        parent front_table;
        translation [.25, 0, table_stack];
        quaternion [0, 0, 1, 0]; // rotate pi about Z
        geometry {
            shape box;
            color [0, 1, 0];
            alpha 0.5;
            dimension [block_size, block_size, block_size];
        }
    }

    // This block is placed on the table
    frame block_b {
        parent front_table;
        translation [-.25, 0, table_stack];
        rpy [0, 0, pi];   /* also rotate pi about Z,
                           * pi is a float constant in the language */
        geometry {
            shape box;
            color [0, 1, 0];
            alpha 0.5;
            dimension [block_size, block_size, block_size];
        }
    }

### Frame Attributes

| Name        | Value Type | Description |
|-------------|------------|-------------|
| parent      | Identifier | The parent of this frame in the scene graph |
| translation | Array [3]  | Pose Translation |
| quaternion  | Array [4]  | Pose Rotation Quaternion |
| rpy         | Array [3]  | Pose Rotation Euler-Angle (Roll-Pitch-Yaw) |
| type        | Identifier | Joint type (fixed, prismatic, or revolute |
| axis        | Array [3]  | Axis of rotation (revolute) or translation (prismatic) |
| offset      | Float      | Value added to configuration variable  prismatic/revolute frames |


### Geometry Attributes

| Name        | Value Type | Description |
|-------------|------------|-------------|
| isa         | Identifier | Class for geometry (multiple isa attributes are allow) |
| shape       | Identifier | Geometry's shape (box, grid, cylinder, mesh) |
| color       | Array[3]   | RGB color |
| alpha       | Float      | Alpha layer (transparency), 0: clear, 1: opaque |
| dimension   | Array      | Geometry dimensions (for box, grid) |
| height      | Float      | Geometry height (for cylinder, cone) |
| radius      | Float      | Geometry radius (for cylinder, sphere) |


Class Statement {#scenefile_class_stmt}
---------------

The class statement defines a set of attributes (shape, color,
dimensions) which may be applied to multiple geometric objects.

### Class Statement Grammar

    <CLASS_STMT>              => "class" <ID> "{" <GEOMETRY_ATTRIBUTE_LIST> "}"
    <GEOMETRY_ATTRIBUTE_LIST> => <GEOMETRY_ATTRIBUTE> |  <GEOMETRY_ATTRIBUTE> <GEOMETRY_ATTRIBUTE_LIST>
    <GEOMETRY_ATTRIBUTE>      => <ID> <ATTRIBUTE_VALUE> ";"
    <ATTRIBUTE_VALUE>         => <EXP> | <ARRAY>

### Class Statement Example

    // Define some sizes for objects
    def table_size 1e-2;  //  1 cm
    def block_size .1;    // 10cm

    // Define a stacking height
    def table_stack (block_size + table_size)/2 + 1e-3;

    class block {
        shape box;
        color [0, 1, 0];
        alpha 0.5;
        dimension [block_size, block_size, block_size];
    }

    /* A table at the world origin */
    frame front_table {
        geometry {
            dimension [.75, .75, table_size];
            shape box;
            color [.6, .3, .6];
        }
    }

    // This block is placed on the table
    frame block_a {
        parent front_table;
        translation [.25, 0, table_stack];
        quaternion [0, 0, 1, 0]; // rotate pi about Z
        geometry {
            isa block;
        }
    }

    // This block is placed on the table
    frame block_b {
        parent front_table;
        translation [-.25, 0, table_stack];
        rpy [0, 0, pi];   /* also rotate pi about Z,
                           * pi is a float constant in the language */
        geometry {
            isa block;
        }
    }

Allowed Collision Statement {#scenefile_allow_collision_stmt}
---------------------------

The allowed collision statement instructs the collision checker to
ignore any potential collisions between two frames.

### Allowed Collision Grammar

    <ALLOWED_COLLISION_STMT>  => "allow_collision" <STRING> <STRING> ";"

### Allowed Collision Example

The following statement will ignore any collisions between geometry
attached to frames "right_arm_forearm" and "right_arm_wrist".

    allow_collision "right_arm_forearm" "right_arm_wrist";

Scene File Syntax {#scenefile_syntax}
=================

Grammar {#scenefile_grammar}
-------

    <START>                   => <INCLUDE_STMT> | <DEF_STMT>
                               | <CLASS_STMT> | <FRAME_STMT>
                               | <ALLOWED_COLLISION_STMT>
                               | <COMMENT>

    <COMMENT>                 => <LINE_COMMENT> | <BLOCK_COMMENT>

    <INCLUDE_STMT>            => "include" <STRING>

    <DEF_STMT>                => "def" <ID> <EXP> ";"
    <EXP>                     => <ID> | <FLOAT>
                               | <EXP> <BINOP> <EXP>
                               | "-" <EXP>
                               | "(" <EXP> ")"

    <CLASS_STMT>              => "class" <ID> "{" <GEOMETRY_ATTRIBUTE_LIST> "}"

    <FRAME_STMT>              => "frame" <ID> "{" <FRAME_ATTRIBUTE_LIST> "}"
    <FRAME_ATTRIBUTE_LIST>    => <FRAME_ATTRIBUTE> |  <FRAME_ATTRIBUTE> <FRAME_ATTRIBUTE_LIST>
    <FRAME_ATTRIBUTE>         => <ID> <ATTRIBUTE_VALUE> ";"
                               | "geometry" "{" <GEOMETRY_ATTRIBUTE_LIST> "}"
                               | <FRAME_STMT>
    <GEOMETRY_ATTRIBUTE_LIST> => <GEOMETRY_ATTRIBUTE> |  <GEOMETRY_ATTRIBUTE> <GEOMETRY_ATTRIBUTE_LIST>
    <GEOMETRY_ATTRIBUTE>      => <ID> <ATTRIBUTE_VALUE> ";"
    <ATTRIBUTE_VALUE>         => <EXP> | <ARRAY>

    <ARRAY>                   => "[" <ARRAY_ELEMENTS> "]"
    <ARRAY_ELEMENTS>          => <EXP> | <EXP> "," <ARRAY_ELEMENTS>

    <ALLOWED_COLLISION_STMT>  => "allow_collision" <STRING> <STRING> ";"

Terminals {#scenefile_terminals}
---------

| Description           | Terminal Symbol | Examples            | Regular Expression |
|-----------------------|-----------------|---------------------|--------------------|
| Identifier            |            <ID> | foo, foo_bar, bif42 | [a-zA-Z][a-zA-Z0-9_] |
| Integer               |           <INT> | 1, 42               |-?[0-9]+ |
| Floating Point Number |         <FLOAT> | 3.14159, 1e-2       | -?[0-9]+((\\\.[0-9]*)?([eds]-?[0-9]+)? |
| Binary Operator       |         <BINOP> | +, -, \*, /         | [\\+\\-\\*/] |
| String                |        <STRING> | "Hello World!"      | \\"[^\"]*\\" |
| Line Comment          |  <LINE_COMMENT> |  // foo, # foo      | (#\|//).*$ |
| Block Comment         | <BLOCK_COMMENT> | /* foo */           | /\\*.*\\*/ |

Editor Support {#scenefile_editor}
--------------

### Emacs

Amino includes a simple emacs mode for editing scene files.  To enable
it, add the following to your `.emacs` file:

    (require 'robray-mode)

### Vim

Amino includes syntax highlighting for Vim.  To enable it, run the
following commands (assuming amino was installed under /usr/local):

    mkdir -p ~/.vim/syntax
    ln -s /usr/local/share/amino/vim/syntax/robray.vim ~/.vim/syntax/
    mkdir -p ~/.vim/ftdetect
    ln -s /usr/local/share/amino/vim/ftdetect/robray.vim ~/.vim/ftdetect/

Complete Example {#scenefile_example}
================

File `class.robray`
------------------

    // Define some sizes for objects
    def table_size 1e-2;  //  1 cm
    def block_size .1;    // 10cm

    // Define a stacking height
    def table_stack (block_size + table_size)/2 + 1e-3;

    class block {
        shape box;
        color [0, 1, 0];
        alpha 0.5;
        dimension [block_size, block_size, block_size];
    }


File `table.robray`
-----------------

    include "class.robray"

    /* A table at the world origin */
    frame front_table {
        geometry {
            dimension [.75, .75, table_size];
            shape box;
            color [.6, .3, .6];
        }
    }

    // This block is placed on the table
    frame block_a {
        parent front_table;
        translation [.25, 0, table_stack];
        quaternion [0, 0, 1, 0]; // rotate pi about Z
        geometry {
            isa block;
        }
    }

    // This block is placed on the table
    frame block_b {
        parent front_table;
        translation [-.25, 0, table_stack];
        rpy [0, 0, pi];   /* also rotate pi about Z,
                           * pi is a float constant in the language */
        geometry {
            isa block;
        }
    }


Scene Graph Compiler {#scenegraph_compiler}
=============

The scene graph compiler `aarxc` parses scene files and outputs C
code, which you can compile and link either statically into your
application or into a shared library.  Compiled scene graphs are fast
to load because the operating system directly maps into memory (via
[mmap](https://en.wikipedia.org/wiki/Mmap)) the included mesh data,
eliminating runtime parsing and processing.  Furthermore, when
multiple processes operate on the same scene graph, using compiled
scene graphs reduces overall memory use compared to runtime parsing
because the memory mapped scene graphs in different processes share
physical memory.


Compiling Scene Files
---------------------

The following command will convert the previous example scene file into
the C file `table.c`:

    aarxc table.robray -n table -o table.c

Loading Scene Graphs
--------------------

If your application needs to deal with only a particular scene graph,
then you can link directly against the compiled C file.

If your application may need to deal with a variety of scene graphs,
you can compile the C files for the scene graphs as shared objects and
[dynamically load](https://en.wikipedia.org/wiki/Dynamic_loading) them
using [aa_rx_dl_sg()](@ref aa_rx_dl_sg).


### Static Linking

If you have a single scene graph and a single application, then it is
sufficient to statically link the scene graph into your application,
just as you would with any other C file.  For example, to compile and
link using GCC:

    PKG_CONFIG_MODULES="amino amino-gl sdl2 glew"
    CFLAGS="$CFLAGS `pkg-config --cflags $PKG_CONFIG_MODULES`"
    LDFLAGS="$LDLAGS `pkg-config --libs $PKG_CONFIG_MODULES`"
    gcc $CFLAGS $LDFLAGS table.c MY_OTHER_SOURCE_FILES -o MY_PROGRAM

(Of course, you would typically use a build automation tool such as
Make, the Autotools, or CMake.)

Then, load the scene graph with the following C code:

~~~~~~~~~~~~~~~~~~~{.c}
struct aa_rx_sg *scenegraph = aa_rx_dl_sg__table(NULL);
~~~~~~~~~~~~~~~~~~~


See also:
* [Autools pkg-config support via PKG_CHECK_MODULES](https://autotools.io/pkgconfig/pkg_check_modules.html)
* CMake's `FindPkgConfig`


### Dynamic Linking

If multiple applications need to use the same scene graph, you will
reduce disk and memory use by compiling the scene graph into a shared
object.  The details of building shared libraries vary by platform and
are best managed with build automation tools such as the Autotools or
CMake.  For simple cases using gcc on GNU/Linux, you can build a
shared library as follows:

    PKG_CONFIG_MODULES="amino amino-gl sdl2 glew"
    CFLAGS="$CFLAGS `pkg-config --cflags $PKG_CONFIG_MODULES`"
    gcc -shared -fPIC $CFLAGS table.c -o libscene-table.so

The code to load the scene graph is identical to the static linking
case.

See also:
* [Shared Libraries with Automake](https://www.gnu.org/software/automake/manual/automake.html#A-Shared-Library)
* CMake's `add_library`

### Dynamic Loading

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

See also:
[Building plugins with Autotools](https://autotools.io/libtool/plugins.html)

Compiling URDF
--------------

Set the `ROS_PACKAGE_PATH` environment variable to point at your ROS
installation, and call `aarxc`.  For example, to compile the URDF for
the Baxter robot:

    export ROS_PACKAGE_PATH=/opt/ros/indigo/share/
    aarxc 'package://baxter_description/urdf/baxter.urdf' -o baxter-model.c -n baxter

Other Formats {#scenefile_other_formats}
=============

* Scenegraphs can be defined procedurally via API calls to create
  frames and attach geometry.
* The scenegraph compiler `aarxc` can load
  [URDF](http://wiki.ros.org/urdf) files.
