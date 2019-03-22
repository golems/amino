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
\newcommand{\pttf}[2]{{^{#2}\!}{#1}}
\newcommand{\qconj}[1]{{#1}^*}
\newcommand\overcmt[2]{\overbrace{{#1}}^{#2}}
\newcommand\undercmt[2]{\underbrace{{#1}}_{#2}}
\newcommand{\dualmark}[1]{\tilde{#1}}
\newcommand{\dualelt}[0]{\epsilon}
\require{cancel}
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

To transform a point from frame b to frame a, we first rotate the
point and then add the translation.

\f[
      \mytf{p}{a}{} =
        (\mytf{\quat{h}}{a}{b}) \qmul (\mytf{p}{b}{}) \qmul
        {(\mytf{\quat{h}}{a}{b})}^*
      \
      +
        \mytf{v}{a}{b}
\f]

![Transforming a Point](tfpoint.svg)


Chaining Transforms
-------------------

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

Dual Number Quaternions {#tutorial_tf_duqu}
=======================

Dual quaternions are an extension of the ordinary quaternions that is
capable of representing both rotation and translation.  A dual
quaternion is a quaternion of **dual numbers**.  Dual numbers are
constructed using the dual element ε, where:

\f[
    \dualelt^2 = 0
    \quad{\rm and}\quad
    \dualelt \neq 0
\f]

A dual numbers thus has two coefficients, a real part and a dual part:

\f[
    \dualmark{n} = r + d \dualelt
\f]

Multiplication of dual numbers cancels the product of the dual parts:

\f[
    (a_r + a_d \dualelt) * (b_r + b_d \dualelt)
    \quad = \quad
    a_r b_r + a_r b_d \dualelt + b_r a_d \dualelt + \cancelto{0}{a_d b_d \dualelt^2}
    \quad = \quad
    a_r b_r + (a_r b_d + b_r a_d) \dualelt
\f]

A dual number quaternion combines the quaternion units and the dual
element:


![Dual Quaternion Coefficients](duqu.svg)

<!-- Dual numbers yield interesting properties.  In particular, the Taylor -->
<!-- series for any dual number, evaluated at the real part, consists of -->
<!-- only two terms; all higher order terms contain and \f$\dualelt^2\f$ -->
<!-- and cancel to zero.  Consequently, we can evaluate any dual function, -->
<!-- such as sine, cosine, exponential, and logarithm, in terms of the real -->
<!-- function and its derivative. -->

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



Transformation Matrices {#tutorial_tf_mat}
=======================

Example Code {#tutorial_tf_code}
============
