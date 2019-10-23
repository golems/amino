Collision {#tutorial_col}
=========

[TOC]



This tutorial introduces the collision checking interface.

Amino provides an interface to the [Flexible Collision Library](https://github.com/flexible-collision-library/fcl)

Collision Detection {#tutorial_col_check}
===================

The collision detection interface checks whether there are any
collisions between geometry (meshes, primitive shapes) at a given
configuration.  Optionally, it can output a set of all colliding frame
pairs.

Robot models often contain geometry that is *always* in collision,
either due to physical robot parts that are in contact by design or
due to approximations of the true geometry.  We ignore such contacts
during collision detection by specifying allowable collisions as a set
of frame pairs.  The collision detection interface will all collisions
between frames in the allowed collisions set.

Example Code
------------

@include python/t5.1-collision-check.py

Signed Distance {#tutorial_col_dist}
===============
TODO


See Also {#tutorial_col_sa}
========

## Python
* @ref amino.collision.SceneCollisionSet
* @ref amino.collision.SceneCollision

## C

* @ref scene_collision.h

References
==========


* J. Pan, S. Chitta, and D. Manocha.
  [FCL: A general purpose library for collision and proximity
  queries](https://doi.org/10.1109/ICRA.2012.6225337).
  ICRA 2012.
