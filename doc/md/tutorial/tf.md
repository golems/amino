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
\newcommand{\mytf}[3]{{^{#2}\!}{#1}_{#3}}
\f]

Euclidean Transformation {#tutorial_tf_euclidean}
========================

Local Coordinate Frames
-----------------------

We model robots as chains or trees of local coordinate frames.  For
each rigid body or link in the robot, we represent the points of that
link in the link's local coordinate frame.  Each joint of the robot
creates a three-dimensional displacement or **transformation** that
varies with the position of the joint.

![Robot Local Frames](robotframe.svg)

The transformation between any pair of frames has a rotation part and
a translation (linear displacement) part.

![Local Coordinate Frames](localframe.svg)

Since we have to keep track of many different frames, it is helpful to
adopt a notation convention that indicates which frames a particular
variable relates to.  A useful convention we will use is to indicate
parent frame with a leading superscript and a child frame with a
trailing subscript.

\f[
    \mytf{X}{\rm parent}{\rm child}
\f]


We can represent a point using coordinates in different frames.  For
example, we can represent point p below in either frame a or frame b:

![Local Coordinates for a Point](localpoint.svg)

Transforming a Point
--------------------

![Transforming a Point](tfpoint.svg)

Chaining Transforms
-------------------

![Transforming a Point](tfchain.svg)

Dual Number Quaternions {#tutorial_tf_duqu}
=======================

Transformation Matrices {#tutorial_tf_mat}
=======================

Example Code {#tutorial_tf_code}
============
