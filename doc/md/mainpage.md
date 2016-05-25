Amino {#mainpage}
=================

Amino is package of basic utilites for robotics software.  It
includes mathematical and linear algebra routines, memory
management, and time-handling.  Design goals are easy integration,
efficiency, and simplicity.

- **Source Code:** https://github.com/golems/amino

Features
========

Extensive SE(3) Support
-----------------------

Amino provides operations for transformation matrices, quaternions,
axis-angle representations, and some of the euler-angle
conventions. \sa amino/tf.h

* Menagerie of Representations:
  - Rotation and Transformation Matrices
  - Ordinary Quaternions
  - Dual Quaternions
  - Quaternion-Vector
  - Axis-Angle / Rotation-Vector / Log-map
  - Euler Angles
* Derivatives and Exact Integration
* Numerically stable log and exponential

Region-based Memory Allocation
------------------------------

Amino provides two O(1), non-fragmenting allocators.  The region
allocator performs variable-sized allocation and frees all objects
with a single call.  The pooled allocator allocates and frees
fixed-sized blocks. \sa amino/mem.h

* Container for LIFO-ordered memory allocation
* O(1) allocation and deallocation

Linear Algebra
---------------

Amino provides some light-weight wrappers over BLAS and LAPACK plus
a few other convenience routines.  Care is taken to avoid
heap-allocation in all calls, making performance suitable for
real-time operation.  \sa amino/math.h

* Lightweight LAPACK wrapper:
  Handles work-array creation for LAPACK using memory regions
* Multiple Runge-Kutta integrators, including adaptive integrators

Ray Tracing
-----------
* Import robot geometry from URDF and Collada
* Generate POV-ray scene files for animation frames
* Distribute POV-ray rendering over multiple machines
