Amino {#mainpage}
=================

Amino is package of basic utilites for robotics software.  It
includes mathematical and linear algebra routines, memory
management, and time-handling.  Design goals are easy integration,
efficiency, and simplicity.

- <a href="installation.html"><b>Installation</b></a>
- [License](@ref copying): 3-clause BSD (permissive)
- <a href="https://github.com/golems/amino"><b>Source Code</b></a>: `git clone https://github.com/golems/amino.git`

\htmlonly
<iframe src="carousel.html" style="float:right" width="640" height="480"></iframe>
\endhtmlonly


Features
========

Visualization
-------------
* Import robot models from:
  - [Scene Files] (@ref scenefile)
  - any [Blender](https://www.blender.org/)-supported mesh
  - [URDF] (http://wiki.ros.org/urdf)
* Online 3D visualization using OpenGL and SDL
* High-quality ray-traced output using [POV-ray] (http://www.povray.org/)
  - Network distributed rendering
  - Movie generation using [libav] (https://libav.org/)

\sa @ref viewer
\sa scenegraph.h
\sa scene_win.h

Motion Planning Interface
-------------------------
* Interface to the [Flexible Collision Library]
  (https://github.com/flexible-collision-library/fcl) (FCL)
* Interface to the [Open Motion Planning Library]
  (http://ompl.kavrakilab.org/) (OMPL)

\sa scene_collision.h
\sa scene_planning.h

Extensive SE(3) Support
-----------------------
* Menagerie of Representations:
  - Rotation and Transformation Matrices
  - Ordinary Quaternions
  - Dual Quaternions
  - Quaternion-Vector
  - Axis-Angle / Rotation-Vector / Log-map
  - Euler Angles
* Derivatives and Exact Integration
* Numerically stable logarithm and exponential

\sa tf.h



Region-based Memory Allocation
------------------------------
* Container for LIFO-ordered memory allocation
* O(1) allocation and deallocation

\sa mem.h


Linear Algebra
--------------
Amino provides some light-weight wrappers over BLAS and LAPACK plus
a few other convenience routines.  Care is taken to avoid
heap-allocation in all calls, making performance suitable for
real-time operation.

* Lightweight LAPACK wrapper:
  Handles work-array creation for LAPACK using memory regions
* Multiple Runge-Kutta integrators, including adaptive integrators

\sa math.h
