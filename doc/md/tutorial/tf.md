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
\newcommand{\tfmat}[3]{{^{#2}\MAT{#1}_{#3}}}
\newcommand{\tfquat}[3]{{^{#2}\quat{#1}_{#3}}}
\newcommand{\qmul}[0]{\otimes}
\newcommand{\dualelt}[0]{\varepsilon}
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

We may combine a rotation matrix and a translation vector to produce a
transformation matrix.  To represent point transformation and
transform chaining as matrix-vector and matrix-matrix products, we
must augment the points and matrices.  We augment points with an
additional one,

\f[
    \begin{bmatrix}
    p_x \\ p_y \\ p_z
    \end{bmatrix}
       \rightarrow
    \begin{bmatrix}
    p_x \\ p_y \\ p_z \\ 1
    \end{bmatrix}
    \; .
\f]

Then, we construct the transformation matrix as follows,

\f[
    \tfmat{T}{a}{b} =
    \begin{bmatrix}
    \tfmat{R}{a}{b}
    &
    \tfmat{v}{a}{b}
    \\
    0_{1 \times 3} & 1
    \end{bmatrix}
    =
    \begin{bmatrix}
    r_{11} & r_{12} & r_{13} & v_x \\
    r_{21} & r_{22} & r_{23} & v_y \\
    r_{31} & r_{32} & r_{33} & v_z \\
    0 & 0 & 0 & 1
    \end{bmatrix}
    \;
    .
\f]

Transforming a point corresponds to a matrix-vector product:

\f[
    \tfmat{p}{a}{}
    =
    \tfmat{T}{a}{b}
    \,
    \tfmat{p}{b}{}
    \quad\rightarrow\quad
    \begin{bmatrix}
    \tf{p}{a}{x} \\
    \tf{p}{a}{y} \\
    \tf{p}{a}{z} \\
    1
    \end{bmatrix}
    \begin{bmatrix}
    \tfmat{R}{a}{b}
    &
    \tfmat{v}{a}{b}
    \\
    0_{1 \times 3} & 1
    \end{bmatrix}
    \begin{bmatrix}
    \tf{p}{b}{x} \\
    \tf{p}{b}{y} \\
    \tf{p}{b}{z} \\
    1
    \end{bmatrix}
\f]

Chaining transformations corresponds to a matrix-matrix product:
\f[
    \tfmat{T}{a}{c}
    =
    \tfmat{T}{a}{b}
    \,
    \tfmat{T}{b}{c}
\f]




Since the bottom row of the transformation matrix is constant, we can
save memory and computation time by omitting this row from our
storage and computation.  Thus, we store the transformation matrix in
memory using only the rotation matrix and translation vector,
requiring a total of 12 numbers:
\f[
    \begin{bmatrix}
    r_{11} & r_{12} & r_{13} & v_x \\
    r_{21} & r_{22} & r_{23} & v_y \\
    r_{31} & r_{32} & r_{33} & v_z \\
    0 & 0 & 0 & 1
    \end{bmatrix}
    \rightarrow
    \begin{array}{|c|c|}
    \hline
    r_{11} & r_{21} & r_{31} &
    r_{12} & r_{22} & r_{32} &
    r_{13} & r_{23} & r_{33} &
    v_x & v_y & v_z \\
    \hline
    \end{array}
    \;
    .
\f]

Dual Number Quaternions {#tutorial_tf_rep_duqu}
-----------------------

Dual quaternions a compact, computationally efficient, and
analytically convenient representation for Euclidean transformations.
A dual quaternion combines the ordinary quaternions and the dual
numbers.  A dual number contains the dual element \f$\dualelt\f$, such
that, \f[ \dualelt^2 = 0 \quad\text{and}\quad \dualelt \neq 0 \; .
\f]

Combing the quaternion units and the dual element yields eight factors
in the dual quaternion,

\f[
\quat{S}
=
\underbrace{
\left(
r_x \ielt
+ r_y \jelt
+ r_z \kelt
+ r_w
\right)
}_{\text{real part}}
+
\underbrace{
\left(
  d_x \ielt
+ d_y \jelt
+ d_z \kelt
+ d_w
\right)
}_{\text{dual part}}
\dualelt
\; .
\f]

We construct the dual quaternion for a rotation \f$\quat{h}\f$ and
translation \f$\vec{v}\f$ as follows:

\f[
    \quat{S}
    = \quat{h} + \quat{d}\dualelt
    = \quat{h} + \frac{1}{2} \vec{v} \qmul \quat{h} \dualelt
    \; .
\f]

Chaining dual quaternions corresponds to multiplication.  Note that
the product of the two dual parts cancels as \f$\dualelt^2 = 0\f$ and
that the product of the real parts is a chaining of the rotations.

\f[ \tfquat{S}{a}{c} = \tfquat{S}{a}{b} \qmul \tfquat{S}{b}{c} =
\left( \tfquat{r}{a}{b} + \tfquat{d}{a}{b}\dualelt \right) \qmul
\left( \tfquat{r}{b}{c} + \tfquat{d}{b}{c}\dualelt \right) =
\tfquat{r}{a}{b} \qmul \tfquat{r}{b}{c} + \left( \tfquat{r}{a}{b}
\qmul \tfquat{d}{b}{c} + \tfquat{d}{a}{b} \qmul \tfquat{r}{b}{c}
\right) \dualelt \f]


To represent the dual quaternion in-memory, we need eight numbers,
which we store as separate oridinary quaternions for the  real-part
and dual-part,

\f[
\left(
r_x \ielt
+ r_y \jelt
+ r_z \kelt
+ r_w
\right)
+
\left(
  d_x \ielt
+ d_y \jelt
+ d_z \kelt
+ d_w
\right)
\dualelt
\quad\rightarrow\quad
\begin{array}{|c|c|}
\hline
r_x
& r_y
& r_z
& r_w
& d_x
& d_y
& d_z
& d_w \\
\hline
\end{array}
\; .
\f]


    double dual_quaternion_as_array[8];

    struct duqu {
       struct quat real;
       struct quat dual;
    };


Implicit Dual Quaternions {#tutorial_tf_rep_imp}
-------------------------

Since we can construct the dual quaternion for a Euclidean
transformation for an ordinary quaternion and translation vector, we
take this quaternion and vector as the *implicit* representation of a
dual quaternion.  Such a quaternion-vector representation is more
compact, requiring only 7 elements, compared to the eight elements of
the explicit dual quaternion.  Moreover, common operations such as
chain transformations, is more efficient using the implicit
representation. We retain the view of this quaternion-vector as a dual
quaternion for analysis, e.g., when considering derivatives and
velocities, where chaining as multiplication and defined logarithm and
exponential maps simplify some operations.

To represent the implicit dual quaternion in-memory, we need a total
of 7 numbers for the ordinary (rotation) quaternion and translation
vector:


\f[
\left\lgroup
\left(
h_x \ielt
+ h_y \jelt
+ h_z \kelt
+ h_w
\right),\
\left(
  v_x, v_y, v_z
\right)
\right\rgroup
\quad\rightarrow\quad
\begin{array}{|c|c|}
\hline
h_x
& h_y
& h_z
& h_w
& v_x
& v_y
& v_z \\
\hline
\end{array}
\; .
\f]


    double implicit_dual_quaternion_as_array[7];

    struct qutr {
       struct quat rotation;
       double translation[3];
    };

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
