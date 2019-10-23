Scene Graphs {#tutorial_scene}
============

[TOC]

\f[
\newcommand{\normtwo}[1]{\left| #1 \right|}
\newcommand{\MAT}[1]{\boldsymbol{#1}}
\newcommand{\unitvec}[1]{\boldsymbol{\hat{#1}}}
\newcommand{\ielt}[0]{\unitvec{\imath}}
\newcommand{\jelt}[0]{\unitvec{\jmath}}
\newcommand{\kelt}[0]{\unitvec{k}}
\newcommand{\quat}[1]{\mathcal{#1}}
\newcommand{\sequat}[0]{\quat{h}}
\newcommand{\qmul}[0]{\otimes}
\newcommand{\dotprod}{\boldsymbol{\cdot}}
\newcommand{\tf}[3]{{^{#2}{#1}_{#3}}}
\newcommand{\tfmat}[3]{{^{#2}\MAT{#1}_{#3}}}
\newcommand{\tfquat}[3]{{^{#2}\quat{#1}_{#3}}}
\newcommand{\qmul}[0]{\otimes}
\newcommand{\dualelt}[0]{\varepsilon}
\newcommand{\setranssym}[0]{v}
\newcommand{\setrans}[0]{\vec{\setranssym}}
\newcommand{\setransvel}[0]{\dot{\setrans}}
\newcommand{\confsym}[0]{\theta}
\newcommand{\conf}[0]{\confsym}
\newcommand{\confvec}[0]{\boldsymbol{\confsym}}
\f]

This tutorial introduces the scene graph data structure and Scene
Window viewer.

Overview {#tutorial_scene_overview}
========

The scene graph is a tree representing the local coordinate frames and
objects in the scene.  Each node in the scene graph represents a local
coordinate frames.  Edges in the scene graph indicate the relative
transforms between local coordinate frames.  Objects are represented
in the scene graph as geometry, i.e., meshes or primitive shapes like
boxes and spheres, attached to local coordinate frames.  Both the
environment and robot are represented in this same data structure.
The frames for the environment will typically be fixed, while frames
for robot joints will vary based on the robot's configuration.


Initial Example Scene {#tutorial_scene_example}
=====================

First, we will construct a simple scene containing a few boxes and a
grid.  The scene looks like the following:

![Simple Scene in Scene Viewer](simple-scene-screenshot.png)


Structurally, we will arrange the local coordinate frames as shown in
the following:

![Frame Structure of the Simple Scene ](simplescene.svg)

There is a single root frame, labeled the code as empty string `""`.
Child of the root is a frame for the grid.  Red box `A` and blue box
`C` are placed on the grid, so their frames are children of the grid.
Green box `B` is placed on box `A`, so `B` is a child of `A`.

The following code constructs the scene and displays in the viewer:

@include python/t3.0-scene.py



Click and drag the mouse and to rotate the view, and zoom with
the scroll wheel.  Close the window the exit.


Robot Scene {#tutorial_scene_robot}
===========

Next, we will procedurally construct a serial robot.  We model the
robot joints as frames whose transform is parameterized by a
configuration variable (i.e., the joint angle).  In other words, the
joint angle tells us the relative transform between a joint's local
coordinate frame and the joint's parent frame.

Robot Joints
------------

The most common types of joints are *revolute* (rotating) and
*prismatic* (linear).



\htmlonly
<div align="center" style="float:right; margin-left: 1em;">
<video width="400" height="300" controls autoplay loop>
  <source src="prismatic.mp4" type="video/mp4">
  <source src="prismatic.ogg" type="video/ogg">
  Your browser does not support the video tag.
</video>
</div>
\endhtmlonly

### Prismatic Joints

Prismatic joints create a linear motion.  We compute the relative
translation along the axis of motion:
\f[
    \setrans = \ell \unitvec{u}
\; .
\f]

<div style="clear: both"></div>


\htmlonly
<div align="center" style="float:right; margin-left: 1em;">
<video width="400" height="300" controls autoplay loop>
  <source src="revolute.mp4" type="video/mp4">
  <source src="revolute.ogg" type="video/ogg">
  Your browser does not support the video tag.
</video>
</div>
\endhtmlonly

### Revolute Joints

Revolute joints create a rotating motion.  We compute the relative
rotation about the axis of motion from the quaternion exponential
\f[
    \sequat = \exp\left(\frac{\conf}{2} \unitvec{u}\right)
\; .
\f]

<div style="clear: both"></div>

### Summary

The following table summarizes the relative rotation and translation
for these joints.  The joint frame is labeled *c* and its parent *p*.
For the revolute frame, we compute the rotation quaternion via the
exponential; the translation \f$ \tf{\vec{v}}{p}{c} \f$ is constant.
For the prismatic frame, rotation \f$\tf{\quat{h}}{p}{c}\f$ is
constant and we compute translation along the joint axis
\f$\vec{u}\f$.


| Type      | Rotation                                          | Translation                | Diagram                                         |
|-----------|---------------------------------------------------|----------------------------|-------------------------------------------------|
| Revolute  | \f$\exp \left( \frac{\theta}{2}\vec{u} \right)\f$ | \f$ \tf{\vec{v}}{p}{c} \f$ | ![Revolute Frame Diagram](joint-revolute.svg)   |
| Prismatic | \f$\tf{\quat{h}}{p}{c}\f$                         | \f$ \ell \vec{u} \f$       | ![Prismatic Frame Diagram](joint-prismatic.svg) |


Example Robot Construction
--------------------------

We will construct the following robot:

![Procedurally constructed 4-DoF Robot](procedural-robot.png)

This robot has four joints: three at the shoulder and one at the
elbow.  For each joint, we create a "revolute" (rotating) frame and
specify the joint's axis of rotation.  To represent the robot's links,
we attach geometry, i.e., a sphere, cylinders, and a cone, to various
frames.

The following code constructs the robot, displays it in the viewer,
and waves the elbow:

@include python/t3.1-robot.py


Scene Compiler {#tutorial_scene_compiler}
==============

Finally, we will use the scene compiler to generate a plugin (shared
library) for a scene.  The scene compilers supports definitions in a
block-oriented (curly-braced) syntax and in URDF.

Scene Files {#tutorial_scene_scenefile}
-----------

We will redefine our serial robot in a scene file, compile the scene,
and load it in the viewer.

<ol>

<li> Use the following scene file:

@include plugin/scenefile/example.robray
</li>


<li> Use the scene compiler to build the plugin.  While you may call
the scene compiler directly from the shell, it will be preferable to
use a build script.  The following CMake file will build a plugin for
the serial robot:

@include plugin/scenefile/CMakeLists.txt

From the directory containing the CMakLists.txt, build the plugin by
running:

~~~{.sh}
cmake .
make
~~~
</li>

<li>Use the following code to load the scene file and display in the
viewer:

@include tutorial/python/t3.2-scenefile.py
</li>

</li>

<li>The resulting display should look identical to the
procedurally-defined robot [above](@ref tutorial_scene_robot).

</ol>


URDF {#tutorial_scene_urdf}
----

We will compile the URDF file for the Baxter robot, load the plugin,
and display the scene in the viewer.

![Baxter in Viewer](baxter-win.png)

<ol>

<li> Obtain the baxter URDF, which describes the kinematics and
     geometry (meshes) of the robot.

<ul> <li> If you already have an existing ROS installation, you can
         install the `baxter_description` ROS package. For example on
         ROS Indigo:

~~~{.sh}
sudo apt-get install ros-indigo-baxter-description
export ROS_PACKAGE_PATH=/opt/ros/indigo/share
~~~
</li>


<li> An existing ROS installation is not necessary, however, and you
     can install only the baxter URDF and meshes:

~~~{.sh}
mkdir -p ~/ros_ws/src
git clone https://github.com/RethinkRobotics/baxter_common
export ROS_PACKAGE_PATH=~/ros_ws/src/
~~~
</li> </ul> </li>

<li> Now, check that you can load the Baxter model.  The following
     command will visualize the Baxter in the viewer.  Click and drag
     the mouse and to rotate the view, and zoom with the scroll wheel.
     Close the window the exit.

~~~{.sh}
aarxc --gui package://baxter_description/urdf/baxter.urdf
~~~
</li>


<li> Use the scene compiler to build the plugin.
  The following CMake file will build a plugin for the baxter:

@include plugin/urdf/CMakeLists.txt

From the directory containing the CMakLists.txt, build the plugin by
running:

~~~{.sh}
cmake .
make
~~~
</li>

The command will generate a C file `baxter-model.c` from the URDF.
Then, it will compile the baxter model into the plugin
`libbaxter-model.so` which describes the kinematic layout and all meshes
of the robot.

<li>Load the plugin using the following code:

@include python/t3.2-urdf.py

</li>


</ol>



See Also {#tutorial_scene_sa}
========

## Python

* @ref amino.scenegraph.SceneGraph
* @ref amino.scenegraph.SubSceneGraph
* @ref amino.scenegraph.Geom
* @ref amino.scenegraph.GeomOpt

## C
* @ref scenegraph.h
* @ref scene_sub.h
* @ref scene_win.h

## Other Documentation

* @ref viewer
* @ref scenecompiler
* @ref scenefile
