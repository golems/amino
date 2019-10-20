Kinematics {#tutorial_kin}
==========

[TOC]

\f[
\newcommand{\normtwo}[1]{\left| #1 \right|}
\newcommand{\MAT}[1]{\boldsymbol{#1}}
\newcommand{\VEC}[1]{\boldsymbol{#1}}
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
\newcommand{\confsym}[0]{\theta}
\newcommand{\conf}[0]{\confsym}
\newcommand{\confvec}[0]{\boldsymbol{\confsym}}
\newcommand{\sequat}[0]{\quat{h}}
\newcommand{\sequatdual}[0]{\quat{d}}
\newcommand{\seduqu}[0]{\quat{S}}
\newcommand{\serotvel}[0]{\omega}
\newcommand{\setranssym}[0]{v}
\newcommand{\setrans}[0]{\vec{\setranssym}}
\newcommand{\setransvel}[0]{\dot{\setrans}}
\newcommand{\setwist}[0]{{\Omega}}
\newcommand{\jacobian}[0]{\boldsymbol{J}}
\newcommand{\realset}[0]{\mathbb{R}}
\newcommand{\overcmt}[2]{\overbrace{#1}^{#2}}
\newcommand{\undercmt}[2]{\underbrace{#1}_{#2}}
\newcommand{\setbuilder}[2]{\left\{\left.{#1}\ \right|\  {#2} \right\}}
\f]

This tutorial introduces forward, differential, and inverse robot
kinematics.

Robot Model {#tutorial_kin_robot}
===========

First, we will extend our 4 degree-of-freedom (DoF) robot to a 7 DoF
model so the robot can reach any orientation and translation.

<ol>

<li> We use the following scene file:

@include plugin/7dof/7dof.robray </li>

<li> Compile the scene using the following build script:

@include plugin/7dof/CMakeLists.txt </li>

</ol>


Forward Kinematics {#tutorial_kin_fk}
==================

Forward kinematics finds the absolute (root-coordinates) pose
\f$\tf{\quat{S}}{0}{c}\f$ of a target frame in the robot (scene) for a
given configuration (joint positions) \f$\confvec\f$.  Each edge in
the scene graph represents the *relative* transform between parent and
child, which could be constant for fixed frames or varying based on
configuration for joints.  Generally, the absolute pose of frame *c*
is defined recursively from the relative pose as:

\f[
   \tf{\quat{S}}{0}{c}(\confvec) =
   \begin{cases}
     \tf{\quat{S}}{p}{c}(\confvec), & \text{if } p = 0 \\
    \tf{\quat{S}}{0}{p}(\confvec) \qmul \tf{\quat{S}}{p}{c}(\confvec), & \text{if } p \neq 0
   \end{cases}
   \; ,
\f]

where *p* is the parent of *c* and 0 is the root frame.



For example, consider the simple scene with a grid and three blocks.
![Frame Structure of the Simple Scene ](simplescene.svg)

We compute the absolute poses of the blocks as follows:
\f{align*}{
    \tf{\seduqu}{0}{A} & =
    \tf{\seduqu}{0}{\rm grid}
    \qmul
    \tf{\seduqu}{\rm grid}{A}
    \\
    \tf{\seduqu}{0}{B} & =
    \tf{\seduqu}{0}{\rm grid}
    \qmul
    \tf{\seduqu}{\rm grid}{A}
    \qmul
    \tf{\seduqu}{A}{B}
    \\
    \tf{\seduqu}{0}{C} & =
    \tf{\seduqu}{0}{\rm grid}
    \qmul
    \tf{\seduqu}{\rm grid}{C}
\f}


Often, we want the absolute transform of every frame in the then
scene, e.g., for visualization or [collision](@ref tutorial_collision)
checking.  We efficiently compute every absolute transform by ordering
the frames such that a parent frame always precedes its children.
Then, a single scan through the ordered frames lets us compute every
absolute pose since the absolute transform of the parent will always
be available when we reach the child.

If we need the relative transforms between two frames, we compute
this from the absolute transforms of each frame:
\f[
    \tf{\quat{S}}{a}{b} = \left(\tf{\quat{S}}{0}{a}\right)^* \qmul
    \tf{\quat{S}}{0}{b}
\f]


Example Code {#tutorial_kin_fk_code}
------------

The following example code computes the forward kinematics for the
7-DoF robot:

@include python/t4.1-fk.py

Differential Kinematics {#tutorial_kin_diff}
======================

Differential kinematics relates change (e.g., velocities) in
configuration (joint positions) and pose.  We will construct and solve
a differential equation for this relationship.

Representing Pose Change {#tutorial_kin_diff_rep}
------------------------

Due to the special nature of rotations, there are multiple ways we may
represent *change* of the target pose.  We will describe pose changes
represented as velocities, transform derivatives, and twists.

### Velocity

Though three-parameter representations of rotational *position* suffer
from singularities, this is not a problem for rotational *velocity*.
Thus, we may represent change in pose with the 6-vector consisting of
rotational velocity \f$\serotvel\f$ about the principal axes and
translational velocity \f$\dot{\vec{v}}\f$ along these axes:

\f[
    \begin{bmatrix}
    \serotvel \\
    \dot{\vec{v}}
    \end{bmatrix}
    \in \realset^6
    \; .
\f]


### Transform Derivative

Another natural representation for pose change is the derivative of a
transformation.  For rotation quaternions, the derivative and velocity
are related as follows :

\f[
\dot{\sequat} = \frac{1}{2}\serotvel \qmul \sequat
\quad\text{ and }\quad
\serotvel = 2 \dot{\sequat} \qmul \sequat^*
\; .  \f]

For dual quaternions representing transformation (rotation and
translation) we differentiate as follows (using the sum and product
derivative rules):

\f{align*}{
  & \seduqu
   =
  \sequat + \frac{1}{2} \setrans
      \qmul \sequat\dualelt \\

  \text{derivative:}\qquad
  & \dot{\seduqu}
   =
  \frac{d}{dt} \left( \sequat + \frac{1}{2} \setrans
      \qmul \sequat\dualelt \right)
  \\
  \text{sum rule:}\qquad
  & \phantom{\dot{\seduqu}} = \dot\sequat + \frac{1}{2}
    \left(
    \setransvel \qmul \sequat
    +
    \setrans \qmul \dot\sequat
  \right)\dualelt
  \\
  \text{product rule:}\qquad &
\phantom{\dot{\seduqu}}
  =
  \frac{1}{2}\left(\serotvel \qmul \sequat
    +
    \frac{1}{2}
    \left(
    \setransvel \qmul \sequat
    +
    \frac{1}{2}\setrans \qmul \serotvel \qmul \sequat
    \right)\dualelt
    \right)
    \; .
\f}

With a bit of algebra, we rearrange the dual quaternion derivative
into a form that is structurally similar to the ordinary quaternion
derivative:

\f[
  \dot{\seduqu} =
  \frac{1}{2}\setwist \qmul \seduqu\; ,
  \quad \text{where}
  \quad
  \setwist =
  \serotvel +
    \left(\setransvel + \setrans
      \times \serotvel \right)\dualelt
  \; .
\f]

The value \f$\setwist\f$ is the *twist* represented as a pure dual
quaternion, i.e., zero scalar (w) in the real and dual parts.

### Twist

The twist \f$\setwist\f$ describes the coupling of rotation and
translation: when a rigid body rotates about some point, all other
points on that body will experience translation.

We write the twist as the following pure dual quaternion:
\f[
  \setwist
  = 2 \dot{\seduqu} \seduqu^*
  =
  \serotvel +
    \left(\setransvel + \setrans
      \times \serotvel \right)\dualelt
  \; .
\f]

Since the twist has only six non-zero elements, we may also write it
as the following vector:
\f[
  \setwist
  =
  \begin{bmatrix}
  \serotvel \\
  \setransvel + \setrans \times \serotvel
  \end{bmatrix}
  \in \realset^6
  \; .
\f]

Computationally, twists offer useful properties for differential
kinematics.  Matrix operations are cheaper for both velocities and
twists than for the dual quaternion derivative because the velocity
and twist have six non-zero elements, while the dual quaternion
derivative has eight.  The specific advantages of the twist arise when
computing and applying the manipulator Jacobian, where the twists
inherent rotation-translation coupling simplifies Jacobian
computation.

Manipulator Jacobian {#tutorial_kin_diff_jacobian}
--------------------

The manipulator Jacobian defines the differential equation relating
change in pose and configuration.

Generally, the Jacobian of a function is a matrix of first-order
partial derivatives.  For function \f$\VEC{f} : \realset^n \mapsto \realset^m\f$,
the Jacobian is:
\f[
    \VEC{y} = \VEC{f}({\VEC{x}})
    \\
    \jacobian
    = \frac{\partial \VEC{f}}{\partial \VEC{x}}
    =
    \begin{bmatrix}
        \frac{\partial \VEC{f}}{\partial x_1}
        &
        \ldots
        &
        \frac{\partial \VEC{f}}{\partial x_n}
    \end{bmatrix}
    =
    \begin{bmatrix}
        \frac{\partial {f_1}}{\partial x_1}
        &
        \ldots
        &
        \frac{\partial {f_1}}{\partial x_n}
        \\
        \vdots & \ddots  & \vdots
        \\
        \frac{\partial {f_m}}{\partial x_1}
        &
        \ldots
        &
        \frac{\partial {f_m}}{\partial x_n}
    \end{bmatrix}
\f]

We use the Jacobian to relate change in the domain and range of a
function via the chain rule:
\f{align*}{
    & \VEC{y} = \VEC{f}(\VEC{x})
    \\
    \text{derivative: } \quad
    & \frac{d \VEC{y}}{dt} = \frac{d}{dt}\left(\VEC{f}(\VEC{x})\right)
    \\
    \text{chain rule: } \quad
    & \frac{d \VEC{y}}{dt} = \frac{\partial \VEC{f}}{\partial \VEC{x}}
    \frac{d \VEC{x}}{dt}
    \\
    \text{Jacobian: } \quad
    & \dot{\VEC{y}} = \jacobian \dot{\VEC{x}}
\f}

Thus, the Jacobian matrix provides a mapping between
\f$\dot{\VEC{x}}\f$ and \f$\dot{\VEC{y}}\f$ as a system of linear
equations.

There are multiple formulations of the manipulator Jacobian due to the
multiple ways we may represent pose change.  In each formulation, the
columns of the manipulator Jacobian correspond to the robot's joints
and the rows correspond to the specific representation of pose change.



\f[
  \text{pose change}
  =
  \overcmt{
  \begin{bmatrix}
  \begin{pmatrix}
    \VEC{j}_1
    \end{pmatrix}
    &
    \ldots
    &
    \begin{pmatrix}
    \VEC{j}_n
    \end{pmatrix}
    \end{bmatrix}
    }{\MAT{J}}
        \overcmt{
          \begin{bmatrix}
            \dot{\conf}_1 \\
             \vdots \\
            \strut\dot{\conf}_n
          \end{bmatrix}
        }{\dot\confvec}
\f]


We will discuss the Jacobians for the velocity, transform derivative,
and twist.

<!-- \f[ -->
<!--   \begin{bmatrix} -->
<!--     \serotvel \\ -->
<!--     \setransvel -->
<!--   \end{bmatrix} -->
<!--   = \jacobian_x \dot{\confvec} \; , -->
<!--   \quad -->
<!--   \setwist -->
<!--   = \jacobian_\setwist \dot{\confvec}\; , -->
<!--   \quad\text{and}\quad -->
<!--   \dot\seduqu -->
<!--   = \frac{\partial \seduqu}{\partial \confvec} \dot{\confvec} -->
<!--   \; . -->
<!-- \f] -->


## Velocity Jacobian

The velocity Jacobian relates configuration and Cartesian velocities:
\f[
    \begin{bmatrix}
    \serotvel \\
    \setransvel
    \end{bmatrix}
    = \jacobian_x \dot{\confvec}
\f]

Each column of the velocity Jacobian thus contains a rotational
velocity part \f$\VEC{j}_r\f$ and a translational velocity part
\f$\VEC{j}_p\f$.
\f[
        \begin{bmatrix}
          \omega \\
          \dot v
        \end{bmatrix}
        =
        \overcmt{
          \begin{bmatrix}
            \begin{pmatrix}
              \VEC{j}_{r_1}\\
              \VEC{j}_{v_1}
            \end{pmatrix}
            &
            \begin{matrix}
              \ldots \\ \ldots
            \end{matrix}
            &
            \begin{pmatrix}
              \VEC{j}_{r_n}\\
              \VEC{j}_{v_n}
            \end{pmatrix}
          \end{bmatrix}
        }{\MAT{J}}
        \overcmt{
          \begin{bmatrix}
            \dot{\conf}_1 \\
             \vdots \\
            \strut\dot{\conf}_n
          \end{bmatrix}
        }{\dot\confvec}
\f]

The velocity Jacobian offers a direct geometric interpretation: we may
consider the contribution of each joint to the end-effector velocity
to construct the Jacobian column for that joint.


\htmlonly
<div align="center" style="float:right">
<video width="400" height="300" controls autoplay loop>
  <source src="prismatic-jacobian.mp4" type="video/mp4">
  <source src="prismatic-jacobian.ogg" type="video/ogg">
  Your browser does not support the video tag.
</video>
</div>
\endhtmlonly

**Prismatic Joint Columns:**
A prismatic joints creates linear motion of the end-effector along the
prismatic joint axis and no rotational motion.  The Jacobian
column for a prismatic joint *c* is:

\f[
    \begin{pmatrix}
    \VEC{j}_{r_c}\\
    \VEC{j}_{v_c}
    \end{pmatrix}
    =
    \begin{pmatrix}
    0 \\ \tf{\unitvec{u}}{G}{c}
    \end{pmatrix}
\f]
where \f$\tf{\unitvec{u}}{G}{c}\f$ is the joint axis in global
coordinates.

<div style="clear: both"></div>


\htmlonly
<div align="center" style="float:right">
<video width="400" height="300" controls autoplay loop>
  <source src="revolute-jacobian.mp4" type="video/mp4">
  <source src="revolute-jacobian.ogg" type="video/ogg">
  Your browser does not support the video tag.
</video>
</div>
\endhtmlonly

**Revolute Joint Columns:** A revolute joint creates both rotational
and translational motion. The rotational motion is about the joint
axis.  The translational motion is along the tangent of the circle
centered at the joint axis and with a radius drawn to the
end-effector; we may find this tangent motion using the cross product
of the joint axis and the circle radius (from joint origin to
end-effector). The Jacobian column for a revolute joint *c* is:

\f[
    \begin{pmatrix}
      \VEC{j}_{r_c}\\
      \VEC{j}_{p_c}
    \end{pmatrix}
    =
    \begin{pmatrix}
      \tf{\unitvec{u}}{G}{c}\\
      \tf{\unitvec{u}}{G}{c}
      \times
      \left(
        \tf{\vec{v}}{G}{e}
        -
        \tf{\vec{v}}{G}{c}
      \right)
    \end{pmatrix}
\f]
where \f$\tf{\unitvec{u}}{G}{c}\f$ is the joint axis,
\f$\tf{\setrans}{G}{c}\f$ is the joint origin,
and \f$\tf{\setrans}{G}{e}\f$ is the end-effector translation,
all in global coordinates.

<div style="clear: both"></div>


## Twist Jacobian

With a bit of matrix algebra, we derive the twist Jacobian in terms of
the velocity Jacobian.  First, we will rewrite the twist as a
matrix-vector product with the velocity vector.  We rewrite the cross
product in matrix form and factor:
\f{align*}{
    & \setwist
     =
    \begin{bmatrix}
    \serotvel \\
    \setransvel
    +
    \setrans
    \times
    \serotvel
    \end{bmatrix}
    \\
    \text{cross product matrix:}\quad
    & \phantom{\setwist} =
    \begin{bmatrix}
    \serotvel \\
    \left[\setranssym\right]
    \serotvel
    +
    \setransvel
    \end{bmatrix}
    \;,\quad\text{where }
    [\setranssym]
    = \begin{bmatrix}
    0 & -\setranssym_z & \setranssym_y \\
    \setranssym_z & 0 & -\setranssym_x \\
    -\setranssym_y &  \setranssym_x & 0
    \end{bmatrix}
    \\
    \text{factor:}\quad
    & \phantom{\setwist} =
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\setranssym]& \MAT{I}
    \end{bmatrix}
    \begin{bmatrix}
    \serotvel \\
    \setransvel
    \end{bmatrix}
\f}

Next, we substitute the velocity Jacobian and configuration
velocity for the Cartesian velocity vector:

\f{align*}{
    \setwist
   & =
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\setranssym] & \MAT{I}
    \end{bmatrix}
    \begin{bmatrix}\serotvel \\ \setransvel\end{bmatrix}
    \\
    & =
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\setranssym] & \MAT{I}
    \end{bmatrix}
    \left(
    \jacobian_x\
    \dot{\confvec}
    \right)
    \\
    & =
    \undercmt{
    \left(
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\setranssym] & \MAT{I}
    \end{bmatrix}
    \jacobian_x\right)
    }{\text{twist Jacobian }\jacobian_\setwist}
    \dot{\confvec}
\f}

Thus, we may construct each column of the twist Jacobian as a
matrix-vector product from the velocity Jacobian:

\f{align*}{
    \jacobian_\setwist &=
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\setranssym] & \MAT{I}
    \end{bmatrix}
    \jacobian_x
    \\
    \begin{bmatrix}
    \begin{pmatrix}
    \VEC{j}_{r_1}\\
    \VEC{j}_{p_1}
    \end{pmatrix}
    &
    \begin{matrix}
    \ldots \\ \ldots
    \end{matrix}
    &
    \begin{pmatrix}
    \VEC{j}_{r_n}\\
    \VEC{j}_{p_n}
    \end{pmatrix}
    \end{bmatrix}
    & =
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\setranssym] & \MAT{I}
    \end{bmatrix}
    \begin{bmatrix}
    \begin{pmatrix}
    \VEC{j}_{r_1}\\
    \VEC{j}_{v_1}
    \end{pmatrix}
    &
    \begin{matrix}
    \ldots \\ \ldots
    \end{matrix}
    &
    \begin{pmatrix}
    \VEC{j}_{r_n}\\
    \VEC{j}_{v_n}
    \end{pmatrix}
    \end{bmatrix}
   \\
    \begin{pmatrix}
    \VEC{j}_{r_i}\\
    \VEC{j}_{p_i}
    \end{pmatrix}
    & =
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\setranssym] & \MAT{I}
    \end{bmatrix}
    \begin{pmatrix}
    \VEC{j}_{r_i}\\
    \VEC{j}_{v_i}
    \end{pmatrix}

\f}

We see that the rotational parts \f$\VEC{j}_{r_i}\f$ of the twist and
velocity Jacobians are identical.  However, there may be differences
in the translational parts \f$\VEC{j}_{p_i}\f$ and
\f$\VEC{j}_{v_i}\f$.

**Prismatic Joint Columns:** Since prismatic joints do not create any
rotational velocity, the additional \f$\setrans \times \serotvel\f$
term of the twist is zero.  Thus, the twist Jacobian column for
prismatic joints is the same as the velocity Jacobian column:

\f{align*}{
    \begin{pmatrix}
    \VEC{j}_{r_c}\\
    \VEC{j}_{p_c}
    \end{pmatrix}
    & =
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\setranssym] & \MAT{I}
    \end{bmatrix}
    \begin{pmatrix}
    0 \\ \tf{\unitvec{u}}{G}{c}
    \end{pmatrix}
    \\
    & =
    \begin{pmatrix}
    0 \\ \tf{\unitvec{u}}{G}{c}
    \end{pmatrix}

\f}


**Revolute Joint Columns:** In the case of revolute joints, the
inclusion of end-effector translation in the twist cancels the
end-effector translation in the velocity Jacobian column:

\f{align*}{
  \begin{pmatrix}
      \VEC{j}_{r_c}\\
      \VEC{j}_{p_c}
    \end{pmatrix}
    & =
    \begin{bmatrix}
    \MAT{I} & 0 \\
    [\tf{\setranssym}{G}{e}] & \MAT{I}
    \end{bmatrix}
    \begin{pmatrix}
      \tf{\unitvec{u}}{G}{c}\\
      \tf{\unitvec{u}}{G}{c}
      \times
      \left(
        \tf{\vec{v}}{G}{e}
        -
        \tf{\vec{v}}{G}{c}
      \right)
    \end{pmatrix}
    \\
    & =
    \begin{pmatrix}
    \tf{\unitvec{u}}{G}{c} \\
    \tf{\vec{v}}{G}{e} \times
    \tf{\unitvec{u}}{G}{c} +
   \tf{\unitvec{u}}{G}{c}
      \times
      \left(
        \tf{\vec{v}}{G}{e}
        -
        \tf{\vec{v}}{G}{c}
      \right)
    \end{pmatrix}
    \\
    &
    =
    \begin{pmatrix}
    \tf{\unitvec{u}}{G}{c} \\
    \tf{\setrans}{G}{c} \times \tf{\unitvec{u}}{G}{c}
    \end{pmatrix}
\f}


Practically speaking, the twist Jacobian eliminates the need to
include the end-effector position in the Jacobian.  This can be
computationally useful if we need to consider multiple
"end-effectors," e.g., if we are considering velocities of multiple
links in a chain instead of just the final link.

## Derivative Jacobian

We will obtain the derivative Jacobian from the twist Jacobian.  First
we rewrite the derivative-twist relationship as a matrix
multiplication.

Generally, we may rewrite ordinary quaternion multiplication as a
matrix-vector product:
\f{align*}{
  \quat{q} \qmul \quat{p}
  & =
  \overcmt{
    \begin{bmatrix}
      \quat{q}_w & - \quat{q}_z &   \quat{q}_y & \quat{q}_x \\
      \quat{q}_z &   \quat{q}_w & - \quat{q}_x & \quat{q}_y \\
      -\quat{q}_y &   \quat{q}_x &   \quat{q}_w & \quat{q}_z \\
      -\quat{q}_x & - \quat{q}_y & - \quat{q}_z & \quat{q}_w \\
    \end{bmatrix}
  }{[\quat{q}]_L}
  \begin{bmatrix}
    \quat{p}_x \\
    \quat{p}_y \\
    \quat{p}_z \\
    \quat{p}_w
  \end{bmatrix}
  \nonumber
  = \left[\quat{q}\right]_L \VEC{p}
  \\
  & =
  \undercmt{
    \begin{bmatrix}
      {\quat{p}}_w &  \quat{p}_z & -\quat{p}_y & \quat{p}_x \\
      -{\quat{p}}_z &  \quat{p}_w &  \quat{p}_x & \quat{p}_y \\
      {\quat{p}}_y & -\quat{p}_x &  \quat{p}_w & \quat{p}_z  \\
      -{\quat{p}}_x & -\quat{p}_y & -\quat{p}_z & \quat{p}_w
    \end{bmatrix}
  }{[\quat{p}]_R}
  \begin{bmatrix}
    \quat{q}_x \\
    \quat{q}_y \\
    \quat{q}_z \\
    \quat{q}_w
  \end{bmatrix}
  = \left[\quat{p}\right]_R \VEC{q}
  \; .
\f}

Similarly, we may view a dual quaternion as the 8-element vector,
\f[
  \seduqu =
  \sequat + \sequatdual\dualelt
  \quad\stackrel{\text{as vector}}{\leadsto}\quad
  \VEC{S} =
  \begin{bmatrix}
    \sequat_x &
    \sequat_y &
    \sequat_z &
    \sequat_w &
    \sequatdual_x &
    \sequatdual_y &
    \sequatdual_z &
    \sequatdual_w
  \end{bmatrix}^T
  \; ,
\f]

and rewrite dual quaternion multiplication as a matrix-vector product:

\f{align*}{
  \quat{C} \qmul \quat{S}
   =
  \begin{bmatrix}
    \quat{C}_r \qmul \quat{S}_r \\
    \quat{C}_d \qmul \quat{S}_r +
    \quat{C}_r \qmul \quat{S}_d
  \end{bmatrix}
  & =
  \overcmt{
    \begin{bmatrix}
      \left([\quat{C}_r]_L\right) & \MAT{0}_{4\times 4} \\
      \left([\quat{C}_d]_L\right) & \left([\quat{C}_r]_L\right)
    \end{bmatrix}
  }{[\quat{C}]_L}
  \VEC{S}
  = \left[\quat{C}\right]_L \VEC{S}
                                    \nonumber\\
  & =
  \undercmt{
    \begin{bmatrix}
      \left([\quat{S}_r]_R\right) & \MAT{0}_{4\times 4} \\
      \left([\quat{S}_d]_R\right) & \left([\quat{S}_r]_R\right)
    \end{bmatrix}
  }{[\quat{S}]_R}
  \VEC{C}
  = \left[\quat{S}\right]_R \VEC{C}
  \; ,
\f}

where blocks of the form \f$[\quat{S_r}]_R\f$ are the ordinary
quaternion multiplication matrices constructed for the real (\f$r\f$)
or dual (\f$d\f$) parts of \f$\quat{S}\f$ and \f$\quat{C}\f$.

Now, we rewrite the derivative-twist equation as a matrix-vector
product:

\f{align*}{
  \dot{\seduqu} =
  \frac{1}{2}\setwist \qmul \seduqu
  =
  \frac{1}{2}\left[\seduqu\right]_R \setwist
\;.
\f}

Then, we substitute the twist Jacobian and configuration velocity for
the twist:

\f{align*}{
  \dot{\seduqu}
  & =
  \frac{1}{2}\left[\seduqu\right]_R
  \overcmt{
    \left(\jacobian_\setwist
    \dot{\confvec}\right)
    }{\setwist}
  \\
  & =
  \undercmt{\left(
  \frac{1}{2}\left[\seduqu\right]_R\jacobian_\setwist
  \right)}{\text{derivative Jacobian }\frac{\partial \seduqu}{\partial \confvec}}
  \dot{\confvec}
\f}

Jacobian Pseudoinverse Methods {#tutorial_kin_diff_jpinv}
------------------------------

Once we have obtained the manipulator Jacobian, we use it to construct
a system of linear equations relating configuration and Cartesian
velocities.  For example, in the case of twist, the equation is:

\f[
    \setwist = \jacobian_\setwist \dot{\confvec}
\f]

If we want to find the configuration velocity to achieve a desired
end-effector velocity, we then solve this system of equations.  One
approach to find this solution is to use the pseudoinverse of the
Jacobian:

\f[
    \setwist_{\rm ref} = \jacobian_\setwist \dot{\confvec}_{\rm cmd}
    \quad\leadsto\quad
    \left({\jacobian_\setwist}^+\right) \setwist_{\rm ref} =  \dot{\confvec}_{\rm cmd}
\f]

Though there are more efficient methods for solving linear systems
than explicitly computing the pseudoinverse \f$\jacobian^+\f$, we are
able to use the explicit \f$\jacobian^+\f$ to further condition the
robot's motion.

### Proportional Position Control

If we are attempting to track a Cartesian trajectory on a physical
robot, we must correct any tracking error.  One simple approach is to
apply proportional control to the position error.  First, we compute
the position error based on the relative transform:
\f[
    \setwist_{\rm perr} = \ln \left( \seduqu_{\rm act} \qmul
    {\seduqu_{\rm ref}}^*\right)
    \; .
\f]

Then, we modify the above linear equation solution to apply a
proportional gain on the position error:
\f[
\dot{\confvec}_{\rm cmd}
=
\left({\jacobian_\setwist}^+\right) \setwist_{\rm ref}
+
\MAT{K}_p \setwist_{\rm perr}
\; ,
\f]
where \f$\MAT{K}_p\f$ is the proportional gain matrix (typically, a
diagonal matrix).  This equation computes feed-forward control on a
velocity reference and applies proportional control to position error.

### Nullspace Projection

For *redundant* manipulators (more than 7 DoF), there may be
infinitely many solutions to the system of linear equations relating
configuration and Cartesian velocities.  Using only the pseudoinverse
above, we will find the minimum-norm solution (smallest joint velocity
magnitude).  If we want a different solution, we can project a
reference joint velocity into the nullspace of the manipulator
Jacobian.


Generally, the nullspace of a matrix is the set of vectors whose
product with the matrix is zero:
\f[
\textrm{null}(\MAT{A})
=
\setbuilder{\VEC{x}}{\MAT{A}\VEC{x} = \VEC{0}}
\; .
\f]

In the case of the manipulator Jacobian, a vector in the nullspace
will have no effect on Cartesian motion:
\f[
    \VEC{0}
    = \jacobian \dot{\confvec}_{\rm null}
    \; .
\f]

We project any configuration velocity into the nullspace using the
following projection matrix:
\f[
\dot{\confvec}_{\rm null}
= \left(\MAT{I} - \jacobian^+\jacobian\right)
\dot\confvec
\f]

A typical use-case of the nullspace projection is to move joints
towards their center position, which helps to avoid reaching joint
limits during robot motion,

\f[
      \dot{\confvec}_{\rm cmd}
      =
      \left(\jacobian^+\right) \setwist_{\rm ref}
      +
      \MAT{K}_\phi
      (\MAT{I} - \MAT{J}^+\MAT{J})
      \left(
        {\confvec}_{\rm center}
        -
        {\confvec}_{\rm act}
      \right)
  \; ,
\f]
where \f$\MAT{K}_\phi\f$ is a proportional gain, typically a scalar or
      a diagonal matrix.


### Singularity-robust methods

*Singularities* occur when the system of linear equations has no
solution.  For example, try stretching your arm straight out in front
of you.  Now, at what velocity must your elbow move to pull your hand
straight back?  There is no solution, since any elbow velocity will
instantaneously move your hand to the side.  When we try to solve the
system of linear equations near the singularity, we end up with very
large velocities than may produce "jerky" motion of the robot.
However, we can apply singularity-robust methods to achieve acceptable
performance near singularities.


**Damped Least Squares:**  The damped least squares adds a small
damping term to compute a *damped psuedoinverse*:
\f[
    \jacobian^\dagger
    =
    \left(\jacobian^T\jacobian + k \MAT{I}\right)^{-1}\jacobian^T
    =
    \jacobian^T\left(\jacobian\jacobian^{T} + k \MAT{I}\right)^{-1}
\f]

The damping term *k* introduces a small amount of error but ensures
that we can always invert the matrix.

**Singular Value Decomposition:**  We can also compute a
singularity-robust pseudoinverse using the singular value
decomposition.

Generally, the singular value decomposition (SVD) of an \f$m \times
 n\f$ matrix \f$\MAT{J}\f$ is the factorization:

\f[
\MAT{J} = \MAT{U}\MAT{\Sigma}\MAT{V}^T
      =
      \begin{bmatrix}
        \VEC{u}_1 & \ldots \VEC{u}_m
      \end{bmatrix}
      \begin{bmatrix}
        \sigma_1 & \ldots & 0 \\
        \vdots        & \ddots & \vdots \\
        0        & \ldots & \sigma_m \\
      \end{bmatrix}
      \begin{bmatrix}
        \VEC{v}_1 & \ldots \VEC{v}_n
      \end{bmatrix}^T
\f]

where:
- \f$\MAT{U}\f$ is an \f$m \times m\f$ orthogonal matrix whose columns
  are the *left-singular vectors* of \f$\MAT{J}\f$

- \f$\MAT{\Sigma}\f$ is an \f$m\times n\f$ diagonal matrix containing
  the *singular values* of \f$\MAT{J}\f$

- \f$\MAT{V}\f$ is an \f$n \times n\f$ orthogonal matrix whose columns
  are the *right-singular vectors* of \f$\MAT{J}\f$


We may compute the pseudoinverse of the Jacobian by inverting the SVD:
\f{align*}{
        \jacobian^+ &= \MAT{V}\MAT{\Sigma}^+\MAT{U}^T
                    \\
                  & =
                    \begin{bmatrix}
                      \VEC{v}_1 & \ldots \VEC{v}_n
                    \end{bmatrix}
                                  \begin{bmatrix}
                                    \frac{1}{\sigma_1} & \ldots & 0 \\
                                    \vdots        & \ddots & \vdots \\
                                    0        & \ldots & \frac{1}{\sigma_m} \\
                                  \end{bmatrix}
        \begin{bmatrix}
          \VEC{u}_1 & \ldots \VEC{u}_m
        \end{bmatrix}^T
                      \nonumber \\
                  & =
                    \sum_{i=1}^m \frac{1}{\sigma_i} \VEC{v}_i
                    \VEC{u}_i^T
    \; .
\f}

At singularities, some of the singular values \f$\sigma_i\f$ go to
zero, resulting in an undefined pseudoinverse.

We may compute the damped psuedoinverse by adding in the damping
factor to the inverted SVD:
\f[
        \MAT{J}^\dagger = \sum_{i=1}^m \frac{\sigma_i^2}{\sigma_i^2 + k^2} \VEC{v}_i \VEC{u}_i^T
\f]

Finally, we may compute a singularity robust pseudo-inverse by simply
omitting singular values below some threshold:
\f[
\MAT{J}^\dagger =
 \sum_{i=1}^m
 \begin{cases}
   \frac{1}{\sigma_i} \VEC{v}_i \VEC{u}_i^T &
     \textrm{when } \sigma_i \geq \epsilon
     \\
     \MAT{0} & \textrm{when } \sigma_i < \epsilon
    \end{cases}
\f]

Example Code {#tutorial_kin_diff_code}
------------

The following example code computes the differential kinematics for
the 7-DoF robot:

@include python/t4.2-dk.py


Inverse Position Kinematics {#tutorial_kin_ik}
===========================

Inverse position kinematics (IK) finds the configuration (joint
positions) for a given target frame pose.  Generally, IK has a direct
interpretation as constrained optimization to minimize error *f*
between actual (\f$\tf{\quat{S}}{}{\rm act}\f$) and reference
(\f$\tf{\quat{S}}{}{\rm ref}\f$) poses, subject to joint limits:

\f[
  \underset{\confvec}{\text{minimize}}
  \quad
    f\left(
    \tf{S}{}{\rm act}(\confvec),\
    \tf{S}{}{\rm ref}
    \right)
    \nonumber \\
  \text{subject to}
  \quad
    \confvec_{\rm min} \leq \confvec \leq
    \confvec_{\rm max}
\; .
\f]

Classical approaches for inverse kinematics employed the
Newton-Raphson method or Levenberg-Marquardt algorithm.  However, such
methods offer poor robustness regarding joint limits.  Recent work has
used sequential quadratic programming (SQP) to solve inverse
kinematics problems.  The SQP formulation directly handles joint
limits, improving robustness. Moreover, careful implementations of the
SQP formulation offer excellent performance: Amino's SQP-based IK
solver handles 6-7 joint problems in an average of 0.1 milliseconds on
a 2017 Intel CPU.

We may define the pose error *f* based on the distance between the
reference and actual pose.  The error as a transformation is:

\f[
    {\tf{\seduqu}{}{\rm act}}
    \qmul
    \tf{\seduqu}{}{\rm err}
    =
    \tf{\seduqu}{}{\rm ref}
    \qquad\leadsto\qquad
    \tf{\seduqu}{}{\rm err}
    =
    {\tf{\seduqu}{}{\rm act}}^*
    \qmul
    \tf{\seduqu}{}{\rm ref}
    \; .
\f]

The norm of the logarithm is a measure for both rotational and
translational distances.  Alternatively, we may separately find angle
error as the norm of the rotation log and translation error as
Euclidean distance of the relative.  Thus, we may use the following as
objective functions:

\f{align*}
   f_{\rm dq}
   & =
   \normtwo{
   \ln \left(
    {\tf{\seduqu}{}{\rm act}}^*
    \qmul
    \tf{\seduqu}{}{\rm ref}
    \right)
   }^2
   \; ,
   \\
   f_{\rm qv}
   & =
   \normtwo{
   \ln \left(
    {\tf{\sequat}{}{\rm act}}^*
    \qmul
    \tf{\sequat}{}{\rm ref}
    \right)
   }^2
    +
   \normtwo{
   \setrans_{\rm act}
   -
   \setrans_{\rm ref}
   }^2
   \; .
\f}

To solve the optimization problem, we need a gradient and, depending
on the specific method, the Hessian of the objective function.
Solvers employing *quasi-Newton methods* approximate the Hessian,
which both avoids the need to analytically derive the Hessian and is
often faster than computing an exact Hessian.  However, we must still
find the gradient.  It is possible to approximate the gradient via
finite difference, but the repeated evaluation of the forward
kinematics to find the finite difference gradient is slow. Instead,
Amino includes analytic gradients for the above objective functions.

Example Code
------------

@include python/t4.3-ik.py

References {#tutorial_kin_refs}
==========

* Lynch and Park. [Modern Robotics]
  (http://hades.mech.northwestern.edu/index.php/Modern_Robotics).
  Ch. 4, 6, and 6.
* Murray, Li, and Sastry. [A Mathematical Introduction to Robot
  Manipulation] (http://www.cds.caltech.edu/~murray/mlswiki/). Ch. 2,
  3.
* Samuel  Buss.
  [Introduction to inverse kinematics with Jacobian transpose, pseudoinverse and damped least squares methods](http://math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf)
  Technical Report. 2004.
* Beeson and
  Ames. [TRAC-IK](https://doi.org/10.1109/HUMANOIDS.2015.7363472).
  Humanoids 2015.
