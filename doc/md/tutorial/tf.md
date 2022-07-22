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
\newcommand{\sequat}[0]{\quat{h}}
\newcommand{\setranssym}[0]{v}
\newcommand{\setrans}[0]{\vec{\setranssym}}
\newcommand{\qmul}[0]{\otimes}
\newcommand{\dotprod}{\boldsymbol{\cdot}}
\newcommand{\tf}[3]{{^{#2}{#1}_{#3}}}
\newcommand{\mytf}[3]{{^{#2}\!}{#1}_{#3}}
\newcommand{\tfmat}[3]{{^{#2}\MAT{#1}_{#3}}}
\newcommand{\tfquat}[3]{{^{#2}\quat{#1}_{#3}}}
\newcommand{\qmul}[0]{\otimes}
\newcommand{\dualelt}[0]{\boldsymbol{\varepsilon}}
\newcommand{\pttf}[2]{{^{#2}\!}{#1}}
\newcommand{\qconj}[1]{{#1}^*}
\newcommand\overcmt[2]{\overbrace{{#1}}^{#2}}
\newcommand\undercmt[2]{\underbrace{{#1}}_{#2}}
\newcommand{\dualmark}[1]{\tilde{#1}}
\require{cancel}
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
transformation are fully defined by the rotation and translation
(linear displacement) of the local frame:

![Local Coordinate Frames](localframe.svg)


<!-- We can represent a point using coordinates in different frames.  For -->
<!-- example, we can represent point p below in either frame a or frame b: -->


<!-- ![Local Coordinates for a Point](localpoint.svg) -->





Since we have to keep track of many different frames, it is helpful to
adopt a notation convention that indicates which frames a particular
variable relates to.  A useful convention we will use is to indicate
parent frame with a leading superscript and a child frame with a
trailing subscript:

\f[
    \mytf{X}{\rm parent}{\rm child}
    \; .
\f]

For example, in the figure above, \f$\tf{x}{0}{1}\f$ is the
x-translation from parent frame 0 to child frame 1.




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

To transform a point from frame b to frame a, we first rotate the
point and then add the translation.

\f[
  \mytf{p}{a}{} =
  \overcmt{
    (\mytf{\sequat}{a}{b}) \qmul (\mytf{p}{b}{}) \qmul
    {(\mytf{\sequat}{a}{b})}^*
    }{\text{rotate}}
    \
    +
    \overcmt{\mytf{\setrans}{a}{b}}{\text{translate}}
\f]


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
       \leadsto
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
    \quad\leadsto\quad
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
    \leadsto
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


Dual Number Quaternions {#tutorial_tf_duqu}
=======================



<!-- \f[ -->
<!-- f(a + b \dualelt) -->
<!-- \quad=\quad -->
<!-- f(a) -->
<!-- + \frac{f'(a)}{1!}(b\dualelt) -->
<!-- + \cancelto{0}{\frac{f''(a)}{2!}(b\dualelt)^2} -->
<!-- + \cancelto{0}{\frac{f'''(a)}{3!}b\dualelt)^3} -->
<!-- + \cancelto{0}{\ldots} -->
<!-- \quad=\quad -->
<!-- f(a) + f'(a)b\dualelt -->
<!-- \f] -->



Dual quaternions a compact, computationally efficient, and
analytically convenient representation for Euclidean transformations.
Dual quaternions are an extension of the ordinary quaternions that is
capable of representing both rotation and translation.  A dual
quaternion is a quaternion of **dual numbers**.  Dual numbers are
constructed using the dual element \f$\dualelt\f$, where:

\f[
    \dualelt^2 = 0
    \quad{\rm and}\quad
    \dualelt \neq 0
\f]


Multiplication of dual numbers cancels the product of the dual parts:

\f[
    (a_r + a_d \dualelt) * (b_r + b_d \dualelt)
    \quad = \quad
    a_r b_r + a_r b_d \dualelt + b_r a_d \dualelt + \cancelto{0}{a_d b_d \dualelt^2}
    \quad = \quad
    a_r b_r + (a_r b_d + b_r a_d) \dualelt
\f]

Dual numbers yield interesting properties.  In particular, the Taylor
series for any dual number, evaluated at the real part, consists of
only two terms; all higher order terms contain and \f$\dualelt^2\f$
and cancel to zero.  Consequently, we can evaluate any dual function,
such as sine, cosine, exponential, and logarithm, in terms of the real
function and its derivative.


Combining the quaternion units and the dual element yields eight factors
in the dual quaternion,


![Dual Quaternion Coefficients](duqu.svg)


<!-- \f[ -->
<!-- \quat{S} -->
<!-- = -->
<!-- \underbrace{ -->
<!-- \left( -->
<!-- r_x \ielt -->
<!-- + r_y \jelt -->
<!-- + r_z \kelt -->
<!-- + r_w -->
<!-- \right) -->
<!-- }_{\text{real part}} -->
<!-- + -->
<!-- \underbrace{ -->
<!-- \left( -->
<!--   d_x \ielt -->
<!-- + d_y \jelt -->
<!-- + d_z \kelt -->
<!-- + d_w -->
<!-- \right) -->
<!-- }_{\text{dual part}} -->
<!-- \dualelt -->
<!-- \; . -->
<!-- \f] -->

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
\quad\leadsto\quad
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


~~~{.c}
double dual_quaternion_as_array[8];

struct duqu {
    struct quat real;
    struct quat dual;
};
~~~


Implicit Dual Quaternions {#tutorial_tf_rep_imp}
-------------------------

Since we can construct the dual quaternion for a Euclidean
transformation from an ordinary quaternion and translation vector, we
take this quaternion and vector as the *implicit* representation of a
dual quaternion.  Such a quaternion-vector representation is more
compact, requiring only 7 elements, compared to the eight elements of
the explicit dual quaternion.  Moreover, common operations such as
chaining transformations, are more efficient using the implicit
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
\quad\leadsto\quad
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

~~~{.c}
double implicit_dual_quaternion_as_array[7];

struct qutr {
   struct quat rotation;
   double translation[3];
};
~~~

=======
We can chain the transforms from a to b and from b to c, giving a
single transform from a to consider.  Consider transforming a point in
c first to b and then to a.

![Transforming a Point](tfchain.svg)

\f[
\mytf{p}{a}{}
      =
      \left( \mytf{\quat{h}}{a}{b}\right)
      \qmul
      \overcmt{
      \left(
        \left( \mytf{\quat{h}}{b}{c}) \qmul (\mytf{p}{c}{} \right)
        \qmul
        \qconj{\left(\mytf{\quat{h}}{b}{c}\right)}
        +
        \mytf{v}{b}{c}
      \right)
      }{\mytf{p}{b}{}}
      \qmul
      \qconj{\left(\mytf{\quat{h}}{a}{b}\right)}
      +
      \mytf{v}{a}{b}
\f]

\f[
      \pttf{p}{a}
      =
      \undercmt{
        \left(\mytf{\quat{h}}{a}{b}\right)
        \qmul
        \left( \mytf{\quat{h}}{b}{c}\right)
      }{\mytf{\quat{h}}{a}{c}}
      \qmul
      \left(\mytf{p}{c}{} \right)
      \qmul
      \qconj{
        \undercmt{
          \left(\mytf{\quat{h}}{a}{b}
            \qmul
            \mytf{\quat{h}}{b}{c}\right)
        }{\mytf{\quat{h}}{a}{c}}
      }
      +
      \undercmt{
        \left(\mytf{\quat{h}}{a}{b}\right)
        \qmul
        \mytf{v}{b}{c}
        \qmul
        \qconj{\left(\mytf{\quat{h}}{a}{b}\right)}
        +
        \mytf{v}{a}{b}
      }{
        \mytf{v}{a}{c}
      }
\f]

Example Code {#tutorial_tf_code}
============

<!-- <ol> -->

<!-- <li> Import the package: -->
<!-- ~~~{.py} -->
<!-- from amino import Vec3, XAngle, YAngle, ZAngle, AxAng, EulerRPY, Quat, RotMat, TfMat, DualQuat, QuatTrans -->
<!-- from math import pi -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Specify a rotation and translation.  Label the parent as frame -->
<!--      `0` and child as frame `1`: -->

<!-- ~~~{.py} -->
<!-- rot_0_1   = ZAngle(pi/4) -->
<!-- trans_0_1 = Vec3([1,2,3]) -->
<!-- tf_0_1 = (rot_0_1,trans_0_1) -->
<!-- ~~~ -->
<!-- </li> -->


<!-- <li> Construct the transforms: -->

<!-- ~~~{.py} -->
<!-- S_0_1 = DualQuat( tf_0_1 ) -->
<!-- print S_0_1 -->

<!-- T_0_1 = TfMat( tf_0_1 ) -->
<!-- print T_0_1 -->

<!-- E_0_1 = QuatTrans( tf_0_1 ) -->
<!-- print E_0_1 -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Convert between Representations: -->

<!-- ~~~{.py} -->
<!-- T_S = TfMat(S_0_1) -->
<!-- T_E = TfMat(E_0_1) -->
<!-- print T_S -->
<!-- print T_E -->

<!-- S_T = DualQuat(T_0_1) -->
<!-- S_E = DualQuat(E_0_1) -->
<!-- print S_T -->
<!-- print S_E -->

<!-- E_T = QuatTrans(T_0_1) -->
<!-- E_S = QuatTrans(S_0_1) -->
<!-- print E_T -->
<!-- print E_S -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Specify point `a` with coordinates in frame `1`: -->

<!-- ~~~{.py} -->
<!-- p_1_a = Vec3([3,5,7]) -->
<!-- print p_1_a -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Transform the point to frame `0`: -->

<!-- ~~~{.py} -->
<!-- p_0_a_T = T_0_1.transform(p_1_a) -->
<!-- p_0_a_E = E_0_1.transform(p_1_a) -->
<!-- p_0_a_S = S_0_1.transform(p_1_a) -->

<!-- print p_0_a_T -->
<!-- print p_0_a_E -->
<!-- print p_0_a_S -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Specify a transform between parent frame `1` to child frame `2`: -->

<!-- ~~~{.py} -->
<!-- rot_1_2   = YAngle(pi/2) -->
<!-- trans_1_2 = Vec3([2,4,8]) -->
<!-- tf_1_2 = (rot_1_2,trans_1_2) -->

<!-- S_1_2 = DualQuat( tf_1_2 ) -->
<!-- T_1_2 = TfMat( tf_1_2 ) -->
<!-- E_1_2 = QuatTrans( tf_1_2 ) -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Specify point `b` with coordinates in frame `2`: -->

<!-- ~~~{.py} -->
<!-- p_2_b = Vec3([2,1,0]) -->
<!-- print p_2_b -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Transform `b` to frame `0` with two successive transformations: -->

<!-- ~~~{.py} -->
<!-- p_1_b_T = T_1_2.transform(p_2_b) -->
<!-- p_0_b_T = T_0_1.transform(p_1_b_T) -->

<!-- p_1_b_E = E_1_2.transform(p_2_b) -->
<!-- p_0_b_E = E_0_1.transform(p_1_b_E) -->

<!-- p_1_b_S = S_1_2.transform(p_2_b) -->
<!-- p_0_b_S = S_0_1.transform(p_1_b_S) -->

<!-- print p_0_b_E -->
<!-- print p_0_b_T -->
<!-- print p_0_b_S -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Chain the transforms between frame `0` and `2`: -->

<!-- ~~~{.py} -->
<!-- S_0_2 = S_0_1 * S_1_2 -->
<!-- T_0_2 = T_0_1 * T_1_2 -->
<!-- E_0_2 = E_0_1 * E_1_2 -->
<!-- ~~~ -->
<!-- </li> -->

<!-- <li> Use chained transforms to transform `b`.  The result is -->
<!--      equivalent to the successive transformation of `b` from `2` to -->
<!--      `1` and then from `1` to `0`. -->

<!-- ~~~{.py} -->
<!-- p_0_b_T_chain = T_0_2.transform(p_2_b) -->
<!-- p_0_b_E_chain = E_0_2.transform(p_2_b) -->
<!-- p_0_b_S_chain = S_0_2.transform(p_2_b) -->

<!-- print p_0_b_E_chain -->
<!-- print p_0_b_T_chain -->
<!-- print p_0_b_S_chain -->
<!-- ~~~ -->
<!-- </li> -->
<!-- </ol> -->

@include python/t2-tf.py

See Also {#tutorial_tf_sa}
========

## Python

* @ref amino.tf.TfMat
* @ref amino.tf.DualQuat
* @ref amino.tf.QuatTrans

## C

* @ref tf.h

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
