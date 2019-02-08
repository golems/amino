Rotations {#tutorial_rot}
=========

[TOC]

\f[
\newcommand{\unitvec}[1]{\boldsymbol{\hat{#1}}}
\newcommand{\ielt}[0]{\unitvec{\imath}}
\newcommand{\jelt}[0]{\unitvec{\jmath}}
\newcommand{\kelt}[0]{\unitvec{k}}
\newcommand{\quat}[1]{\mathcal{#1}}
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
    = v \unitvec{u} + w
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


We construct a quaternion representing a rotation analogously to the
planar, complex number case. For quaternions, the imaginary is now
constructed from the axis of rotation. We must also scale the angle by
one half.

\f[
    h = \unitvec{u} \sin \frac{\theta}{2} + \cos \theta
\f]

While it is more challenging to directly visualize a four element
quaternion than a two element complex number, we can still find some
insight by projecting the quaternion onto a plane.  We take the scalar
(real) quaternion part as one plane axis and the magnitude of the
vector (imaginary) part as the other plane axis:

![Quaternion Complex Plane](qplane.svg)

Rotation Matrices {#tutorial_rot_rotmat}
=================


Additional Representations {#tutorial_rot_hm}
==========================

Euler Angles  {#tutorial_rot_hm_euler}
------------

Denavit-Hartenberg Parameters {#tutorial_rot_hm_dh}
-----------------------------
