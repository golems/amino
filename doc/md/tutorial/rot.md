Rotations {#tutorial_rot}
=========

[TOC]

\f[
\newcommand{\unitvec}[1]{\boldsymbol{\hat{#1}}}
\newcommand{\ielt}[0]{\unitvec{\imath}}
\newcommand{\jelt}[0]{\unitvec{\jmath}}
\newcommand{\kelt}[0]{\unitvec{k}}
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


Quaternions {#tutorial_rot_quat}
===========

Rotation Matrices {#tutorial_rot_rotmat}
=================


Additional Representations {#tutorial_rot_hm}
==========================

Euler Angles  {#tutorial_rot_hm_euler}
------------

Denavit-Hartenberg Parameters {#tutorial_rot_hm_dh}
-----------------------------
