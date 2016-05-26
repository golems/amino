Amino {#mainpage}
=================

Amino is package of basic utilites for robotics software.  It
includes mathematical and linear algebra routines, memory
management, and time-handling.  Design goals are easy integration,
efficiency, and simplicity.

- **Source Code:** https://github.com/golems/amino
- **License:** 3-clause BSD (permissive)

\htmlonly

<div id="myCarousel" class="carousel slide" data-ride="carousel"
  style="float:right">
  <!-- Indicators -->
  <ol class="carousel-indicators">
    <li data-target="#myCarousel" data-slide-to="0" class="active"></li>
    <li data-target="#myCarousel" data-slide-to="1"></li>
    <li data-target="#myCarousel" data-slide-to="2"></li>
    <li data-target="#myCarousel" data-slide-to="3"></li>
  </ol>

  <!-- Wrapper for slides -->
  <div class="carousel-inner" role="listbox">
    <div class="item active">
      <img src="baxter.png" alt="Baxter">
    </div>

    <div class="item">
      <img src="ur.png" alt="Universal Robot">
    </div>

    <div class="item">
      <img src="aminogl.png" alt="GL Window">
    </div>

    <div class="item">
      <img src="biglogo.png" alt="Amino Logo">
    </div>
  </div>

  <!-- Left and right controls -->
  <a class="left carousel-control" href="#myCarousel" role="button" data-slide="prev">
    <span class="glyphicon glyphicon-chevron-left" aria-hidden="true"></span>
    <span class="sr-only">Previous</span>
  </a>
  <a class="right carousel-control" href="#myCarousel" role="button" data-slide="next">
    <span class="glyphicon glyphicon-chevron-right" aria-hidden="true"></span>
    <span class="sr-only">Next</span>
  </a>
</div>

\endhtmlonly

Features
========

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
* Numerically stable log and exponential

\sa amino/tf.h

Visualization
-------------
* Import robot geometry from URDF and mesh files
* Real-time visualization via OpenGL and SDL
* Offline raytracing via POV-ray
* Distribute POV-ray rendering over multiple machines

Region-based Memory Allocation
------------------------------

* Container for LIFO-ordered memory allocation
* O(1) allocation and deallocation

 \sa amino/mem.h

Linear Algebra
---------------

Amino provides some light-weight wrappers over BLAS and LAPACK plus
a few other convenience routines.  Care is taken to avoid
heap-allocation in all calls, making performance suitable for
real-time operation.

* Lightweight LAPACK wrapper:
  Handles work-array creation for LAPACK using memory regions
* Multiple Runge-Kutta integrators, including adaptive integrators

\sa amino/math.h
