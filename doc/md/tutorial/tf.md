Transformation {#tutorial_tf}
==============

[TOC]

\f[
\newcommand{\normtwo}[1]{\left| #1 \right|}
\newcommand{\MAT}[1]{\boldsymbol{#1}}
\newcommand{\unitvec}[1]{\boldsymbol{\hat{#1}}}
\newcommand{\ielt}[0]{\unitvec{\imath}}
\newcommand{\jelt}[0]{\unitvec{\jmath}}
\newcommand{\kelt}[0]{\unitvec{k}}
\newcommand{\quat}[1]{\mathcal{#1}}
\newcommand{\qmul}[0]{\otimes}
\newcommand{\dotprod}{\boldsymbol{\cdot}}
\newcommand{\tf}[3]{{^{#2}{#1}_{#3}}}
\newcommand{\qmul}[0]{\otimes}
\f]

This tutorial covers the combination of rotation and translation in
three dimensions.

Euclidean Transformation {#tutorial_tf_euclidean}
========================

Motion of a rigid body in 3D space consists of both rotation (angular
motion) and translation (linear motion).  We call the combination of a
rotation and translation a *Euclidean transformation* because
Euclidean distances between points on the rigid body are preserved
under motion.

![Robot Local Frames](euclideantf.svg)

Euclidean transformations are also called *rigid transformations*
(since bodies are rigid) or *Euclidean isometry* ("equal measure").

Local Coordinate Frames
-----------------------

It is often convenient to define the points on a rigid body in terms
of a *local coordinate frame* affixed to the body.  Then, these local
coordinates of points on the body remain constant regardless as to how
the body rotates and translates.

![Robot Local Frames](bodyframe.svg)

The global coordinates of all points on the body following a Euclidean
transformation are fully defined by the rotation and translation of
the local frame:

![Local Coordinate Frames](localframe.svg)


Since we often must deal with many local coordinate frames, it is
helpful to adopt a notation to keep track of relevant frames.
Specifically, we will write the parent frame as a leading prefix and
the child frame as a trailing suffix.  For example, in the figure
above, \f$\tf{x}{0}{1}\f$ is the x-translation from parent frame 0 to
child frame 1.


<!-- ![Local Coordinates for a Point](localpoint.svg) -->

Transforming a Point
--------------------

The Euclidean transformation lets us map points between coordinate
frames.  For example, we may commonly want to find the global
coordinates of some point, which is possible using the local
coordinates of the point and the Euclidean transformation of the local
coordinate frame.

In the figure below, we start with the local coordinates of point
\f$p\f$ in frame b.  To obtain the coordinates of the point in frame
a, we first rotate and then we translate the point:

![Transforming a Point](tfpoint.svg)

Chaining Transforms
-------------------

Sometimes we need to chain a sequence of transformations.  This need
occurs when dealing with robot manipulators, where we have a series of
links, each with its own local coordinate frame:

![Robot Local Frames](robotframe.svg)

Generally, we chain transformations with a multiplication, either
matrix or quaternion as discussed [below] (#tutorial_tf_rep).  Our
notation convention for parent and child frames helps us keep track of
the appropriate products.  To chain the transorm from a-to-b
\f$\tf{T}{a}{b}\f$ and from b-to-c \f$\tf{T}{b}{c}\f$, we multiply:

\f[ \tf{T}{a}{b}\,\tf{T}{b}{c} = \tf{T}{a}{c} \f]


![Transforming a Point](tfchain.svg)

Representations {#tutorial_tf_rep}
===============

Transformation Matrices {#tutorial_tf_rep_mat}
-----------------------

Dual Number Quaternions {#tutorial_tf_rep_duqu}
-----------------------

Implicit Dual Quaternions {#tutorial_tf_rep_imp}
-------------------------


Example Code {#tutorial_tf_code}
============

References {#tutorial_tf_references}
==========
* Lynch and Park. [Modern Robotics]
  (http://hades.mech.northwestern.edu/index.php/Modern_Robotics).
  Ch. 3, 4.
* Murray, Li, and Sastry. [A Mathematical Introduction to Robot
  Manipulation] (http://www.cds.caltech.edu/~murray/mlswiki/). Ch. 2,
  3.
* Dantam, N., 2018. [Practical Exponential Coordinates using Implicit
  Dual Quaternions]
  (http://dyalab.mines.edu/papers/dantam2018practical.pdf). Workshop
  on the Algorithmic Foundations of Robotics (WAFR). 2018.
