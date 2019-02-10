Rotations {#tutorial_rot}
=========

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
\f]

Complex Numbers {#tutorial_rot_c}
===============

Complex numbers offer a way to represent planar (2D) rotations.  The
imaginary unit \f$\ielt\f$ is defined as a number whose square is -1:


\f[
    \ielt^2 = -1
\f]

[Euler's Formula](https://en.wikipedia.org/wiki/Euler%27s_formula)
provides the relationship between complex numbers and planar angles.
If we expand the Taylor series for the complex exponential, we can
rearrange the terms into the Taylor series for \f$\sin\f$ and
\f$\cos\f$:

\f[
    \begin{array}{ccl}
    {e}^{\theta \ielt} & = &  \overbrace{
    \frac{1}{0!}
    + \frac{(\theta \ielt)^1}{1!}
    + \frac{(\theta \ielt)^2}{2!}
    + \frac{(\theta \ielt)^3}{3!}
    + \frac{(\theta \ielt)^4}{4!}
    + \frac{(\theta \ielt)^5}{5!}
    + \ldots
    }^{\textrm{Taylor Series Expansion of } e^{\theta \ielt}} \\
    & = &  \overbrace{
    \left(\frac{1}{0!}
    - \frac{\theta^2}{2!}
    + \frac{\theta^4}{4!}
    + \ldots\right)
    }^{\cos \theta}
    +
    \ielt \overbrace{
      \left(\frac{\theta}{1!}
    - \frac{\theta^3}{3!}
    + \frac{\theta^5}{5!}
    + \ldots\right)
    }^{\sin \theta} \\
    & = & \cos \theta + \ielt \sin \theta
    \end{array}
\f]

We can see Euler's formula illustrated in the complex plane:

![Complex Plane](cplane.svg)


We can apply Euler's formula to rotate points in the plane by viewing
the point in polar coordinates.

\f[
    x_1 + y_1 \ielt = r \left(\cos \theta_1 + \ielt \sin \theta_1
    \right) =
    r e^{\theta_1\ielt}
\f]

where \f$r = \sqrt{{x_1}^2 + {y_1}^2}\f$ and \f$\theta_1 = \tan^{-1}
\frac{y_1}{x_1}\f$.

To rotate the polar coordinate point, we need only add the rotation
angle to the polar coordinate angle:

![Complex Plane](eulerrot.svg)

Addition of the polar coordinate angle and rotation angle corresponds
to multiplication of the exponential coordinates:

\f[
    x_2 + \ielt y_2
    =
    r\left(\cos \left(\theta_1 + \theta_r\right)
           +
           \ielt \sin \left(\theta_1 + \theta_r\right)
      \right)
    =
    re^{\left(\theta_1+ \theta_r\right)\ielt}
    =
    re^{\theta_1\ielt} e^{\theta_r\ielt}
\f]


Though Euler's formula is (at the least) a "neat trick," the
advantages of representing angles in the plane using imaginary numbers
and exponentials may be less certain.  In the plane, all angles are
about the z-axis, so we can apply successive rotations by summing the
angles.  However, when we move the 3D rotations---where we can rotate
about arbitrary axes---then we can no longer merely sum angles and the
benefits of the exponential formulation are more definitive.

Axis-Angle {#tutorial_rot_aa}
==========

In the plane, all rotations are about the axis orthogonal to the
plane.  For example, in the XY plane, all rotations are about the Z
axis: ![Planar Rotation](planerot.svg)

When we rotate in 3D, there are now three axes we can rotate about
(all at the same time!): ![3D Rotation](spacerot.svg)

A visually-meaningful representation of 3D rotation is *Axis-Angle*
form, given by the unit axis \f$\unitvec{u}\f$ about which we rotate
and the angle \f$\theta\f$ by which we rotate:
![Axis-Angle](axang.svg)

While the axis-angle form is easy enough to visualize, it is not
especially efficient for computation, for example, if we want to chain
to successive rotations.  For efficient computation, we move on to the
Quaternions and Rotation Matrices.


Quaternions {#tutorial_rot_quat}
===========

Representation
--------------

Quaternions offer a particularly compact and efficient representation
for rotations.  The quaternions generalize complex numbers to three
dimensions.  Whereas in the plane we needed one imaginary unit to
describe rotation about one axis, quaternions describe rotations about
three possible axes using three imaginary units, plus a real part,
giving four (quaternary) elements total.  The quaternion axiom is:

\f[
    \ielt^2 = \jelt^2 = \kelt^2 = \ielt\jelt\kelt = -1
\f]

The four elements of the quaternion are the imaginary unit factors
(the vector part) and the real term (the scalar part):

\f[
    h
    =
    \overbrace{x \ielt + y \jelt + z \kelt}^{\textrm{vector}}
    +
    \overbrace{w}^{\textrm{scalar}}
    = \vec{v} + w
    = |v| \unitvec{u} + w
\f]

Computationally, we may represent the quaternion as an array of four
elements or as a struct with the four parts.  Either form (array or
struct) will have the same memory layout.

    double quaternion_as_array[4];

    struct quat {
        double x;
        double y;
        double z;
        double w;
    };

Multiplication
--------------

From the quaternion axiom, we can derive the products of pairs of
quaternion elements.  For example, the derivation of
\f$\ielt\jelt\f$:

\f[
    \left\lgroup \ielt\jelt\kelt=-1 \right\rgroup
    \rightarrow \left\lgroup \ielt\jelt\kelt^2=-\kelt \right\rgroup
    \rightarrow \left\lgroup -\ielt\jelt=-\kelt \right\rgroup
    \rightarrow \left\lgroup \ielt\jelt=\kelt \right\rgroup
\f]

Following a similar derivation for the other products, we obtain the
following relations:

\f[
    \begin{array}
    \ielt\jelt &=& -\jelt\ielt &=& \kelt \\
    \jelt\kelt &=& -\kelt\jelt &=& \ielt \\
    \kelt\ielt &=& -\ielt\kelt &=& \jelt
    \end{array}
\f]

(An informed reader may notice that multiplication of quaternion units
corresponds to cross products of the standard basis vectors.  In fact,
the vector cross product is itself based on quaternion
multiplication!)

From the quaternion unit products, we can expand and simplify the
multiplication of two quaternions:

\f[
  \begin{array}
  \quat{a} \qmul \quat{b}
  & = &
  (a_w + a_x\ielt + a_y \jelt + a_z \kelt) \qmul
    (b_w + b_x\ielt + b_y \jelt + b_z \kelt) \\
  & = &
    \phantom{+\ }(a_wb_w
    - a_xb_x
    - a_yb_y
    - a_zb_z)\\
    & & +\ (
     a_w b_x
     + a_xb_w
     + a_yb_z
     - a_zb_y
    ) \ielt\\
    & & +\ (
     a_wb_y
     - a_xb_z
     + a_yb_w
     + a_zb_x
    )\jelt\\
    & & +\ (
    a_wb_z
    + a_xb_y
    - a_y b_x
    + a_zb_w
    )\kelt
  \end{array}
\f]

A single quaternion multiply actually performs both a cross product
and dot product.  We can restate the quaternion multiply as:

\f[
\begin{array}
  \quat{a} \qmul \quat{b}
   & = &
  (a_w + \vec{a}_v) \qmul
    (b_w + \vec{b}_v) \\
   & = &
    \underbrace{\left(
        \vec{a}_v \times \vec{b}_v + a_w\vec{b}_v +
        b_w\vec{a}_v
    \right)}_{\textrm{vector}}
    +
    \underbrace{
    \left(
        a_w b_w - \vec{a}_v \dotprod \vec{b}_v
    \right)
    }_{\textrm{scalar}}
\end{array}
\f]

Rotations
---------

We construct a quaternion representing a rotation analogously to the
planar, complex number case. For quaternions, the imaginary part is
now constructed from the axis of rotation. We must also scale the
angle by one half.

\f[
    e^{\theta \unitvec{u}} = \unitvec{u} \sin \frac{\theta}{2} + \cos \theta
\f]

While it is more challenging to directly visualize a four element
quaternion than a two element complex number, we can still find some
insight by projecting the quaternion onto a plane.  We take the scalar
(real) quaternion part as one plane axis and the magnitude of the
vector (imaginary) part as the other plane axis:

![Quaternion Complex Plane](qplane.svg)


Rotation Matrices {#tutorial_rot_rotmat}
=================

The matrix representation of rotations is especially efficient for
rotating points, thought not as compact or efficient for chaining as
quaternions.


2D (Planar) Rotations
---------------------

The planar (2D) rotation matrix is constructed as:


\f[
    R(\theta) =
    \begin{bmatrix}
    \cos \theta & -\sin \theta \\
    \sin \theta & \cos \theta
    \end{bmatrix}
\f]

Rotation matrices offer a direct, visual interpretation.  The columns
of the rotation matrix are the principal axes of the child (rotated)
coordinate frame in the parent (non-rotated) frame:

![Quaternion Complex Plane](rotmat2d.svg)

Planar rotation using a rotation matrix corresponds to rotation using
Euler's formula.  While Euler's formula essentially operates on polar
coordinates, rotation matrices operate on rectangular coordinates.

\f[
    \begin{bmatrix}
    x_1 \\ y_1
    \end{bmatrix}
    =
    r
    \begin{bmatrix}
    \cos \theta_1\\
    \sin \theta_1\\
    \end{bmatrix}
\f]
![Complex Plane](eulerrot.svg)

Then we ran can derive the rotation operation through some
trigonometric identities and factoring:

\f[
    \begin{bmatrix}
    x_2 \\ y_2
    \end{bmatrix}
    =
    r
    \begin{bmatrix}
    \cos \left( \theta_1 + \theta_r \right) \\
    \sin  \left(\theta_1 + \theta_r \right) \\
    \end{bmatrix}
    =
    r
    \begin{bmatrix}
    \cos \theta_1 \cos \theta_r - \sin \theta_1 \sin \theta_r \\
    \cos \theta_1 \sin \theta_r + \sin \theta_1 \cos \theta_r
    \end{bmatrix}
    =
    \begin{bmatrix}
     \cos \theta_r & -  \sin \theta_r \\
     \sin \theta_r &   \cos \theta_r
    \end{bmatrix}
    \begin{bmatrix}
    x_1 \\ y_1
    \end{bmatrix}

\f]


3D Rotations
------------

In 3 dimensions, we construct rotation matrix as:

\f[
    \MAT{R}
    =
    e^{[\theta \unitvec{u}]} = \MAT{I} +
    \left({\sin{\theta}}\right) [\unitvec{u}]
    +
    \left({1 - \cos{\theta}}\right) [\unitvec{u}]^2,
    \quad{\rm where}\quad
    [\unitvec{u}]
    = \begin{bmatrix}
        0 & -u_z & u_y \\
        u_z & 0 & -u_x \\
        -u_y & u_x & 0 \\
    \end{bmatrix}
    \; .
\f]

3D rotation matrices again offer the visual interpretation: the columns
of the rotation matrix are the principal axes of the child (rotated)
coordinate frame in the parent (non-rotated) frame.

Rotation is analgous to the 2D case:

\f[
    \begin{bmatrix}
    x_2 \\ y_2 \\ z_2
    \end{bmatrix}
    =
    \MAT{R}
    \begin{bmatrix}
    x_1 \\ y_1 \\ z_1
    \end{bmatrix}
\f]

Additional Representations {#tutorial_rot_hm}
==========================

Euler Angles  {#tutorial_rot_hm_euler}
------------

Denavit-Hartenberg Parameters {#tutorial_rot_hm_dh}
-----------------------------
