!! -*- mode: F90; -*-
!!
!! Copyright (c) 2012, Georgia Tech Research Corporation
!! All rights reserved.
!!
!! Author(s): Neil T. Dantam <ntd@gatech.edu>
!! Georgia Tech Humanoid Robotics Lab
!! Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
!!
!!
!! This file is provided under the following "BSD-style" License:
!!
!!
!!   Redistribution and use in source and binary forms, with or
!!   without modification, are permitted provided that the following
!!   conditions are met:
!!
!!   * Redistributions of source code must retain the above copyright
!!     notice, this list of conditions and the following disclaimer.
!!
!!   * Redistributions in binary form must reproduce the above
!!     copyright notice, this list of conditions and the following
!!     disclaimer in the documentation and/or other materials provided
!!     with the distribution.
!!
!!   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
!!   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
!!   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
!!   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
!!   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
!!   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
!!   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
!!   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
!!   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
!!   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
!!   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
!!   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
!!   POSSIBILITY OF SUCH DAMAGE.

#define W_INDEX   4
#define XYZ_INDEX  1:3
#define X_INDEX  1
#define Y_INDEX  2
#define Z_INDEX  3

#define R_INDEX  :,1:3
#define T_INDEX  :,4

#define DQ_REAL 1:4
#define DQ_DUAL 5:8

#define DQ_REAL_XYZ 1:3
#define DQ_REAL_W 4
#define DQ_DUAL_XYZ 5:7
#define DQ_DUAL_W 8

module amino_tf
  use ISO_C_BINDING
  use amino_la
  implicit none

  interface aa_tf_quat2rotmat
     subroutine aa_tf_quat2rotmat( q, r ) &
          bind(C,name="aa_tf_quat2rotmat")
       use ISO_C_BINDING
       real(C_DOUBLE), intent(in), dimension(4) :: q
       real(C_DOUBLE), intent(out), dimension(3,3) :: r
     end subroutine aa_tf_quat2rotmat
  end interface aa_tf_quat2rotmat

contains
  pure subroutine aa_tf_9( R1, p1, p ) &
       bind( C, name="aa_tf_9" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: p1(3)
    real(C_DOUBLE), intent(out) :: p(3)
    integer :: i

    !p = matmul(R1,p1)
    forall(i=1:3)
       p(i) = dot_product(R1(i,:),p1)
    end forall

  end subroutine aa_tf_9



#include "mac_tf.f90"


  ! pure subroutine aa_tf_9mul( R1, R2, R3 ) &
  !      bind( C, name="aa_tf_9mul" )
  !   real(C_DOUBLE), intent(in)  :: R1(3,3)
  !   real(C_DOUBLE), intent(in)  :: R2(3,3)
  !   real(C_DOUBLE), intent(out) :: R3(3,3)

  !   R3 = matmul(R1,R2)
  ! end subroutine aa_tf_9mul


  pure subroutine aa_tf_93( R1, v1, p1, p) &
       bind( C, name="aa_tf_93" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: v1(3)
    real(C_DOUBLE), intent(in)  :: p1(3)
    real(C_DOUBLE), intent(out) :: p(3)
    !p = matmul(R1,p1) + v1
    call aa_tf_9( R1, p1, p )
    p = p + v1
  end subroutine aa_tf_93


  pure subroutine aa_tf_qv( q, v, p1, p2) &
       bind( C, name="aa_tf_qv" )
    real(C_DOUBLE), intent(in)  :: q(4), v(3), p1(3)
    real(C_DOUBLE), intent(out) :: p2(3)
    call aa_tf_qrot( q, p1, p2 )
    p2 = p2 + v
  end subroutine aa_tf_qv


  pure subroutine aa_tf_93chain( R1, v1, R2, v2, R3, v3 ) &
       bind( C, name="aa_tf_93chain" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: v1(3)
    real(C_DOUBLE), intent(in)  :: R2(3,3)
    real(C_DOUBLE), intent(in)  :: v2(3)
    real(C_DOUBLE), intent(out) :: R3(3,3)
    real(C_DOUBLE), intent(out) :: v3(3)

    !R3 = matmul(R1,R2)
    !v3 = matmul(R1,v2) + v1

    call aa_tf_9mul( R1, R2, R3 )
    call aa_tf_93(R1, v1, v2, v3)
  end subroutine aa_tf_93chain

  pure subroutine aa_tf_12chain( T1, T2, T3 ) &
       bind( C, name="aa_tf_12chain" )
    real(C_DOUBLE), intent(in)  :: T1(3,4)
    real(C_DOUBLE), intent(in)  :: T2(3,4)
    real(C_DOUBLE), intent(out) :: T3(3,4)
    ! call aa_tf_93chain( &
    !      T1(R_INDEX), T1(T_INDEX), &
    !      T2(R_INDEX), T2(T_INDEX), &
    !      T3(R_INDEX), T3(T_INDEX) )

    !! Factorized
    T3(1,1) =  T1(1,1)*T2(1,1) + T1(1,2)*T2(2,1) + T1(1,3)*T2(3,1);
    T3(2,1) =  T1(2,1)*T2(1,1) + T1(2,2)*T2(2,1) + T1(2,3)*T2(3,1);
    T3(3,1) =  T1(3,1)*T2(1,1) + T1(3,2)*T2(2,1) + T1(3,3)*T2(3,1);

    T3(1,2) =  T1(1,1)*T2(1,2) + T1(1,2)*T2(2,2) + T1(1,3)*T2(3,2);
    T3(2,2) =  T1(2,1)*T2(1,2) + T1(2,2)*T2(2,2) + T1(2,3)*T2(3,2);
    T3(3,2) =  T1(3,1)*T2(1,2) + T1(3,2)*T2(2,2) + T1(3,3)*T2(3,2);

    T3(1,3) =  T1(1,1)*T2(1,3) + T1(1,2)*T2(2,3) + T1(1,3)*T2(3,3);
    T3(2,3) =  T1(2,1)*T2(1,3) + T1(2,2)*T2(2,3) + T1(2,3)*T2(3,3);
    T3(3,3) =  T1(3,1)*T2(1,3) + T1(3,2)*T2(2,3) + T1(3,3)*T2(3,3);

    T3(1,4) = T1(1,1)*T2(1,4) + T1(1,2)*T2(2,4) + T1(1,3)*T2(3,4) + T1(1,4);
    T3(2,4) = T1(2,1)*T2(1,4) + T1(2,2)*T2(2,4) + T1(2,3)*T2(3,4) + T1(2,4);
    T3(3,4) = T1(3,1)*T2(1,4) + T1(3,2)*T2(2,4) + T1(3,3)*T2(3,4) + T1(3,4);

  end subroutine aa_tf_12chain

  pure subroutine aa_tf_12( T, p1, p) &
       bind( C, name="aa_tf_12" )
    real(C_DOUBLE), intent(in)  :: T(3,4)
    real(C_DOUBLE), intent(in)  :: p1(3)
    real(C_DOUBLE), intent(out) :: p(3)

    call aa_tf_93( T(:,1:3), T(:,4), p1, p )
  end subroutine aa_tf_12


  !!! Matrices
  pure subroutine aa_tf_xangle2rotmat( theta, R ) &
       bind( C, name="aa_tf_xangle2rotmat" )
    real(C_DOUBLE), intent(in), value :: theta
    real(C_DOUBLE), intent(out) :: R(3,3)

    R(1,:) = [1d0, 0d0, 0d0]
    R(2:3,1) = 0d0

    R(2,2) = cos(theta)
    R(3,2) = sin(theta)

    R(2,3) = -sin(theta)
    R(3,3) = cos(theta)
  end subroutine aa_tf_xangle2rotmat

  pure subroutine aa_tf_yangle2rotmat( theta, R ) &
       bind( C, name="aa_tf_yangle2rotmat" )
    real(C_DOUBLE), intent(in), value :: theta
    real(C_DOUBLE), intent(out) :: R(3,3)

    R(2,:) = [0d0, 1d0, 0d0]
    R(1,2) = 0d0
    R(3,2) = 0d0

    R(1,1) = cos(theta)
    R(3,1) = -sin(theta)

    R(1,3) = sin(theta)
    R(3,3) = cos(theta)
  end subroutine aa_tf_yangle2rotmat

  pure subroutine aa_tf_zangle2rotmat( theta, R ) &
       bind( C, name="aa_tf_zangle2rotmat" )
    real(C_DOUBLE), intent(in), value :: theta
    real(C_DOUBLE), intent(out) :: R(3,3)

    R(3,:) = [0d0, 0d0, 1d0]
    R(1:2,3) = 0d0

    R(1,1) = cos(theta)
    R(2,1) = sin(theta)

    R(1,2) = -sin(theta)
    R(2,2) = cos(theta)
  end subroutine aa_tf_zangle2rotmat

  !!! Quaternions

  pure subroutine aa_tf_qnormalize( q ) &
       bind( C, name="aa_tf_qnormalize" )
    real(C_DOUBLE), dimension(4), intent(inout) :: q
    q = q / sqrt(dot_product(q,q))
  end subroutine aa_tf_qnormalize


  pure subroutine aa_tf_qnormalize2( q, qnorm ) &
       bind( C, name="aa_tf_qnormalize2" )
    real(C_DOUBLE), dimension(4), intent(in) :: q
    real(C_DOUBLE), dimension(4), intent(out) :: qnorm
    qnorm = q / sqrt(dot_product(q,q))
  end subroutine aa_tf_qnormalize2

  pure subroutine aa_tf_qconj( q, qc ) &
       bind( C, name="aa_tf_qconj" )
    real(C_DOUBLE), dimension(4), intent(in) :: q
    real(C_DOUBLE), dimension(4), intent(out) :: qc
    qc(XYZ_INDEX) = -q(XYZ_INDEX)
    qc(W_INDEX) = q(W_INDEX)
  end subroutine aa_tf_qconj

  pure subroutine aa_tf_qmul( a, b, q) &
       bind( C, name="aa_tf_qmul" )
    real(C_DOUBLE), dimension(4), intent(out) :: q
    real(C_DOUBLE), dimension(4), intent(in) :: a,b

    !q(1) =   a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)
    !q(2) = - a(1)*b(3) + a(2)*b(4) + a(3)*b(1) + a(4)*b(2)
    !q(3) =   a(1)*b(2) - a(2)*b(1) + a(3)*b(4) + a(4)*b(3)
    !q(4) = - a(1)*b(1) - a(2)*b(2) - a(3)*b(3) + a(4)*b(4)

    !q(1) =   a(4)*b(1) - a(3)*b(2) + a(2)*b(3) + a(1)*b(4)
    !q(2) =   a(3)*b(1) + a(4)*b(2) - a(1)*b(3) + a(2)*b(4)
    !q(3) = - a(2)*b(1) + a(1)*b(2) + a(4)*b(3) + a(3)*b(4)
    !q(4) = - a(1)*b(1) - a(2)*b(2) - a(3)*b(3) + a(4)*b(4)

    q(1) =    a(1)*b(4) + a(2)*b(3) + a(4)*b(1) - a(3)*b(2)
    q(2) =    a(3)*b(1) + a(4)*b(2) + a(2)*b(4) - a(1)*b(3)
    q(3) =    a(4)*b(3) + a(3)*b(4) + a(1)*b(2) - a(2)*b(1)
    q(4) = - (a(2)*b(2) + a(1)*b(1) + a(3)*b(3) - a(4)*b(4))


    !q(W_INDEX) = a(W_INDEX)*b(W_INDEX) - dot_product(a(XYZ_INDEX),b(XYZ_INDEX))
    !call aa_tf_cross(a(XYZ_INDEX), b(XYZ_INDEX), q(XYZ_INDEX))
    !q(XYZ_INDEX) = q(XYZ_INDEX) + a(W_INDEX)*b(XYZ_INDEX) +  a(XYZ_INDEX)*b(W_INDEX)

  end subroutine aa_tf_qmul

  !! Multiply conj(a) and b
  pure subroutine aa_tf_qcmul( a, b, q) &
       bind( C, name="aa_tf_qcmul" )
    real(C_DOUBLE), dimension(4), intent(out) :: q
    real(C_DOUBLE), dimension(4), intent(in) :: a,b
    q(1) = + a(3)*b(2) + a(4)*b(1) -   (a(2)*b(3) + a(1)*b(4))
    q(2) = + a(1)*b(3) + a(4)*b(2) -   (a(3)*b(1) + a(2)*b(4))
    q(3) = + a(2)*b(1) + a(4)*b(3) -   (a(1)*b(2) + a(3)*b(4))
    q(4) = + a(4)*b(4) + a(3)*b(3) - (-(a(1)*b(1) + a(2)*b(2)))
  end subroutine aa_tf_qcmul

  !! Multiply a and conj(b)
  pure subroutine aa_tf_qmulc( a, b, q) &
       bind( C, name="aa_tf_qmulc" )
    real(C_DOUBLE), dimension(4), intent(out) :: q
    real(C_DOUBLE), dimension(4), intent(in) :: a,b
    q(1) = + a(3)*b(2) + a(1)*b(4) -   (a(2)*b(3) + a(4)*b(1))
    q(2) = + a(1)*b(3) + a(2)*b(4) -   (a(3)*b(1) + a(4)*b(2))
    q(3) = + a(2)*b(1) + a(3)*b(4) -   (a(1)*b(2) + a(4)*b(3))
    q(4) = + a(1)*b(1) + a(2)*b(2) - (-(a(3)*b(3) + a(4)*b(4)))
  end subroutine aa_tf_qmulc


  !! Multiply vector and quaternion
  pure subroutine aa_tf_vqmul( v, q, y) &
       bind( C, name="aa_tf_vqmul" )
    real(C_DOUBLE), intent(in) :: v(3), q(4)
    real(C_DOUBLE), intent(out) :: y(4)

    ! y(W_INDEX) =  - dot_product(v,q(XYZ_INDEX))
    ! call aa_tf_cross(v, q(XYZ_INDEX), y(XYZ_INDEX))
    ! y(XYZ_INDEX) = (y(XYZ_INDEX)  + q(W_INDEX) * v)

    y(1) = +  v(2)*q(3) + v(1)*q(4) - v(3)*q(2)
    y(2) = +  v(3)*q(1) + v(2)*q(4) - v(1)*q(3)
    y(3) = +  v(1)*q(2) + v(3)*q(4) - v(2)*q(1)
    y(4) = - (v(1)*q(1) + v(2)*q(2) - (-v(3)*q(3)))


    !q(W_INDEX) = a(W_INDEX)*b(W_INDEX) - dot_product(a(XYZ_INDEX),b(XYZ_INDEX))
    !call aa_tf_cross(a(XYZ_INDEX), b(XYZ_INDEX), q(XYZ_INDEX))
    !q(XYZ_INDEX) = q(XYZ_INDEX) + a(W_INDEX)*b(XYZ_INDEX) +  a(XYZ_INDEX)*b(W_INDEX)

  end subroutine aa_tf_vqmul

  pure Subroutine aa_tf_qinv( q, r ) &
    bind( C, name="aa_tf_qinv" )
    Real(C_DOUBLE), Dimension(4), intent(out) :: r
    Real(C_DOUBLE), Dimension(4), intent(in) :: q
    Call aa_tf_qconj( q, r )
    r = r / dot_product(q,q)
  End Subroutine aa_tf_qinv

  pure Subroutine aa_tf_cross(a,b,c) &
       bind( C, name="aa_tf_cross" )
    real(C_DOUBLE), intent(in), dimension(3) :: a,b
    real(C_DOUBLE), intent(out), dimension(3) :: c

    c(1) =  a(2)*b(3) - a(3)*b(2)
    c(2) =  a(3)*b(1) - a(1)*b(3)
    c(3) =  a(1)*b(2) - a(2)*b(1)

  End Subroutine aa_tf_cross

  pure Subroutine aa_tf_qrot( q, v, r ) &
       bind( C, name="aa_tf_qrot" )
    real(C_DOUBLE), Dimension(3), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE), Dimension(3), intent(in) :: v
    !real(C_DOUBLE), Dimension(4) :: qv, qr1, qr2, qi

    !! slow implementation
    ! qv(W_INDEX) = 0d0
    ! qv(XYZ_INDEX) = v
    ! Call aa_tf_qconj( q, qi )
    ! call aa_tf_qmul( q, qv, qr1 )
    ! call aa_tf_qmul( qr1, qi, qr2 )
    ! r = qr2(1:3)

    !! Optimized implementation
    !real(C_DOUBLE) :: tmp(3)
    !call aa_tf_cross(q(1:3), v, tmp)
    !tmp = tmp + q(4)*v
    !call aa_tf_cross(q(1:3), tmp, r)
    !r = v + 2*r

    !! factored implementation
    r(1) =  q(2) * ( v(3)*q(4) + v(2)*q(1) - v(1)*q(2) ) - q(3) * ( v(2)*q(4) + v(1)*q(3) - q(1)*v(3) )
    r(2) =  q(3) * ( v(1)*q(4) + v(3)*q(2) - v(2)*q(3) ) - q(1) * ( v(3)*q(4) + v(2)*q(1) - v(1)*q(2) )
    r(3) =  q(1) * ( v(2)*q(4) + v(1)*q(3) - v(3)*q(1) ) - q(2) * ( v(1)*q(4) + v(3)*q(2) - v(2)*q(3) )
    r = 2*r + v

  End Subroutine aa_tf_qrot


  pure Subroutine aa_tf_qexp( q, r ) &
       bind( C, name="aa_tf_qexp" )
    real(C_DOUBLE), Dimension(4), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE) :: vnorm,ew
    vnorm = sqrt( dot_product(q,q) )
    ew = exp(q(W_INDEX))
    r(W_INDEX) = ew * cos(vnorm)
    r(XYZ_INDEX) = (ew * sin(vnorm) / vnorm) * q(XYZ_INDEX)
  end Subroutine aa_tf_qexp

  pure subroutine aa_tf_qln( q, r ) &
       bind( C, name="aa_tf_qln" )
    real(C_DOUBLE), Dimension(4), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE) :: vnorm, qnorm, theta
    vnorm = aa_la_norm2( q(XYZ_INDEX) )
    qnorm = aa_la_norm2( q )
    ! Maybe should use atan2 here
    theta =  acos( q(W_INDEX) / qnorm )
    r(W_INDEX) = log(qnorm)
    r(XYZ_INDEX) = theta / vnorm *  q(XYZ_INDEX)
  end subroutine aa_tf_qln

  !> Compute q**a
  pure subroutine aa_tf_qpow( q, a, r ) &
       bind( C, name="aa_tf_qpow" )
    real(C_DOUBLE), Dimension(4), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE), value, intent(in) :: a
    real(C_DOUBLE) :: qnorm, vnorm, theta, pa
    ! intermediates
    qnorm = aa_la_norm2(q)
    vnorm = aa_la_norm2( q(XYZ_INDEX) )
    ! Maybe should use atan2 here
    theta = acos( q(W_INDEX) / qnorm )
    pa = qnorm ** a
    ! output
    r(W_INDEX) = pa * cos(a*theta)
    r(XYZ_INDEX) = (pa * sin(a*theta) / vnorm) * q(XYZ_INDEX)
  end subroutine aa_tf_qpow

  pure subroutine aa_tf_qslerp( tau, q1, q2, r ) &
       bind( C, name="aa_tf_qslerp" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE) :: theta, d, s1, s2
    if( 0 == tau ) then
       r = q1
    elseif ( 1 == tau ) then
       r = q2
    else
       theta = aa_la_angle( q1, q2 )
       if( 0 == theta ) then
          r = q1
       else
          d = 1d0 / sin(theta)
          s1 = sin( (1 - tau) * theta )
          s2 = sin( tau * theta )
          if( dot_product(q1,q2) < 0.0 ) then
             r = (s1*q1 - s2*q2) * d
          else
             r = (s1*q1 + s2*q2) * d
          end if
       end if
    end if
  end subroutine aa_tf_qslerp


  !> Algebraic computation of SLERP
  pure subroutine aa_tf_qslerpalg( tau, q1, q2, r ) &
       bind( C, name="aa_tf_qslerpalg" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE), dimension(4) :: q1i, qm, qp
    if( 0 >= tau ) then
       r = q1
    elseif ( 1 <= tau ) then
       r = q2
    else
       ! ( q2*q1^-1 ) ^ t  *
       call aa_tf_qinv( q1, q1i )
       call aa_tf_qmul( q2, q1i, qm )
       call aa_tf_qpow( qm, tau, qp )
       call aa_tf_qmul( qp, q1, r )
    end if
  end subroutine aa_tf_qslerpalg

  !> Spherical Cubic Interpolation
  pure subroutine aa_tf_qsquad( h, q1, q2, s1, s2, r ) &
       bind( C, name="aa_tf_qsquad" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2, s1, s2
    real(C_DOUBLE), value, intent(in) :: h
    real(C_DOUBLE), dimension(4) :: rq, rs
    call aa_tf_qslerp( h, q1, q2, rq )
    call aa_tf_qslerp( h, s1, s2, rs )
    call aa_tf_qslerp( 2*h*(1-h), rq, rs, r )
  end subroutine aa_tf_qsquad

  !> Compute s_i parameters for SQUAD
  pure subroutine aa_tf_qsquad_param( q1, q2, q3, s2 ) &
       bind( C, name="aa_tf_qsquad_param" )
    real(C_DOUBLE), dimension(4), intent(out) :: s2
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2, q3
    real(C_DOUBLE), dimension(4) :: q2_inv, q21, l21, q23, l23,  qd, qe
    call aa_tf_qinv( q2, q2_inv )
    call aa_tf_qmul( q2_inv, q1, q21 )
    call aa_tf_qln( q21, l21 )
    call aa_tf_qmul( q2_inv, q3, q23 )
    call aa_tf_qln( q23, l23 )
    qd =  - (l21 + l23) / 4.0
    call aa_tf_qexp( qd, qe )
    call aa_tf_qmul( q2, qe, s2 )
  end subroutine aa_tf_qsquad_param

  !! Derivative of a SLERPed quaternion
  !! Note, this is not a time deriviative, but derivative by the slerp parameter tau
  !! Use the chain rule if you need the time derivative (ie, to find a velocity)
  pure subroutine aa_tf_qslerpdiff( tau, q1, q2, r ) &
       bind( C, name="aa_tf_qslerpdiff" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE) :: theta, s1, s2, d
    if( 0.0 > tau .or. 1.0 < tau ) then
       r = 0d0
       return
    end if
    theta = aa_la_angle( q1, q2 )
    if( 0 == theta ) then
       r = 0d0
       return
    end if
    d = 1.0 / sin(theta)
    s1 = -theta * cos( (1 - tau) * theta )
    s2 = theta * cos( theta * tau )
    if( dot_product(q1,q2) < 0.0 ) then
       r = (s1*q1 - s2*q2) * d
    else
       r = (s1*q1 + s2*q2) * d
    end if
  end subroutine aa_tf_qslerpdiff


  !! Derivative of a SLERPed quaternion, computed algebraicly
  pure subroutine aa_tf_qslerpdiffalg( tau, q1, q2, dq ) &
       bind( C, name="aa_tf_qslerpdiffalg" )
    real(C_DOUBLE), dimension(4), intent(out) :: dq
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE), dimension(4) :: q1i, qm, ql, q
    if( 0.0 > tau .or. 1.0 < tau ) then
       dq = 0d0
       return
    end if
    ! dq = ln(q2*q1^-1) * slerp(q1,q2,u)
    call aa_tf_qslerp( tau, q1, q2, q )
    call aa_tf_qinv( q1, q1i )
    call aa_tf_qmul( q2, q1i, qm );
    call aa_tf_qln( qm, ql )
    call aa_tf_qmul( ql, q, dq )
  end subroutine aa_tf_qslerpdiffalg


  !! Chain Rule Derivative of a SLERPed quaternion
  !! Assumes that q1 and q2 are unit quaternions
  pure subroutine aa_tf_qslerpchaindiff( u, du, q1, dq1, q2, dq2, q, dq ) &
       bind( C, name="aa_tf_qslerpchaindiff" )
    real(C_DOUBLE), dimension(4), intent(out) :: q, dq
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2, dq1, dq2
    real(C_DOUBLE), value, intent(in) :: u, du
    ! locals
    real(C_DOUBLE) :: q1q2, theta, dtheta, a, b, da, db, s, c, sa, sb, ca, cb
    ! check interpolation bounds
    if( 0.0 > u ) then
       dq = 0d0
       q = q1
       return
    elseif ( 1.0 < u ) then
       dq = 0d0
       q = q2
       return
    end if
    ! get angle
    q1q2 = dot_product(q1,q2)
    theta = acos( q1q2 )
    if( 0 == theta ) then
       dq = 0d0
       q = q1
       return
    end if
    ! find parameters
    dtheta = ( dot_product(q1, dq2) + dot_product(dq1, q2) ) / sqrt(1 - q1q2**2)

    s = sin(theta)
    c = cos(theta)
    sa = sin((1-u)*theta)
    ca = cos((1-u)*theta)
    sb = sin(u*theta)
    cb = cos(u*theta)

    a = sa / s
    b = sb / s

    da = ( ca * (dtheta*(1-u) - du*theta) ) / s - &
         ( dtheta * c * sa ) / s**2

    db = ( (dtheta*u + theta*du) *cb ) / s - &
         ( dtheta * c * sb ) / s**2

    ! get the result
    if( q1q2 < 0.0 ) then
       q = q1*a - q2*b
       dq = (dq1*a + q1*da) - (dq2*b + q2*db)
    else
       q = q1*a + q2*b
       dq = (dq1*a + q1*da) + (dq2*b + q2*db)
    end if
  end subroutine aa_tf_qslerpchaindiff

  !! Perform a triad of slerps and computer the derivative as well
  pure subroutine aa_tf_qslerp3diff( u12, du12, q1, q2, u34, du34, q3, q4, u, du, q, dq ) &
    bind( C, name="aa_tf_qslerp3diff" )
    real(C_DOUBLE), dimension(4), intent(out) :: q, dq
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2, q3, q4
    real(C_DOUBLE), value, intent(in) :: u12, du12, u34, du34, u, du
    ! locals
    real(C_DOUBLE), dimension(4) :: q12, dq12, q34, dq34
    ! Intermediates
    call aa_tf_qslerp( u12, q1, q2, q12 )
    call aa_tf_qslerpdiff( u12, q1, q2, dq12 )
    dq12 = dq12 * du12 ! chain rule
    call aa_tf_qslerp( u34, q3, q4, q34 )
    call aa_tf_qslerpdiff( u34, q3, q4, dq34 )
    dq34 = dq34 * du34 ! chain rule
    ! Final
    call aa_tf_qslerpchaindiff( u, du, q12, dq12, q34, dq34, q, dq )
  end subroutine aa_tf_qslerp3diff


  !! Convert differential quaternion to velocity
  !! Note, pass the time derivative of the quaternion as dq_dt
  subroutine aa_tf_qdiff2vel( q, dq_dt, v ) &
       bind( C, name="aa_tf_qdiff2vel" )
    real(C_DOUBLE), dimension(3), intent(out) :: v
    real(C_DOUBLE), dimension(4), intent(in) :: q, dq_dt
    real(C_DOUBLE), dimension(4) :: qc, qtmp, qv
    call aa_tf_qconj(q, qc)
    qtmp = 2*dq_dt
    call aa_tf_qmul(qtmp, qc, qv)
    v = qv(XYZ_INDEX)
  end subroutine aa_tf_qdiff2vel

  subroutine aa_tf_qvel2diff( q, v, dq_dt ) &
       bind( C, name="aa_tf_qvel2diff" )
    real(C_DOUBLE), intent(in) :: v(3), q(4)
    real(C_DOUBLE), intent(out) :: dq_dt(4)
    ! dq/dt = 1/2 * v * q
    call aa_tf_vqmul( v, q, dq_dt )
    dq_dt = dq_dt / 2
  end subroutine aa_tf_qvel2diff


  subroutine aa_tf_qrk1( q0, dq, dt, q1 ) &
       bind( C, name="aa_tf_qrk1" )
    real(C_DOUBLE), intent(in) :: dq(4), q0(4)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: q1(4)
    q1 = q0 + dt * dq         ! euler integration
    call aa_tf_qnormalize(q1)
  end subroutine aa_tf_qrk1


  !! Integrate rotational velocity, Runge-Kutta 2 (euler integration)
  subroutine aa_tf_qvelrk1( q0, v, dt, q1 ) &
       bind( C, name="aa_tf_qvelrk1" )
    real(C_DOUBLE), intent(in) :: v(3), q0(4)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: q1(4)
    real(C_DOUBLE), dimension(4) :: dq
    call aa_tf_qvel2diff(q0, v, dq)
    call aa_tf_qrk1( q0, dq, dt, q1 )
  end subroutine aa_tf_qvelrk1


  !! Integrate rotational velocity, Runge-Kutta 4
  subroutine aa_tf_qvelrk4( q0, v, dt, q1 ) &
       bind( C, name="aa_tf_qvelrk4" )
    real(C_DOUBLE), intent(in) :: v(3), q0(4)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: q1(4)
    real(C_DOUBLE) :: dq(4,4), qtmp(4)

    ! k1
    call aa_tf_qvel2diff(q0, v, dq(:,1))
    ! k2
    call term(dq(:,1), dt/2, dq(:,2))
    ! k3
    call term(dq(:,2), dt/2, dq(:,3))
    ! k4
    call term(dq(:,3), dt, dq(:,4))

    ! combine
    qtmp = dq(:,1) + dq(:,4) + 2 * ( dq(:,2) + dq(:,3) )
    call aa_tf_qrk1( q0, qtmp, dt/6, q1 )

  contains
    subroutine term( dq0, dt, dq1 )
      real(C_DOUBLE), intent(in) ::  dq0(4)
      real(C_DOUBLE), intent(in), value :: dt
      real(C_DOUBLE), intent(out) :: dq1(4)
      real(C_DOUBLE) :: qtmp(4)
      call aa_tf_qrk1( q0, dq0, dt, qtmp )
      call aa_tf_qvel2diff(qtmp, v, dq1)
    end subroutine term
  end subroutine aa_tf_qvelrk4


  function aa_tf_sinc( theta ) result(s)
    real(C_DOUBLE), value :: theta
    real(C_DOUBLE) :: s
    real(C_DOUBLE) :: x
    if( 0d0 == theta ) then
       s = 1d0
    elseif ( abs(theta) < sqrt(sqrt(epsilon(theta))) ) then
       x = theta**2
       ! Rewrite partial Taylor series using Horners rule.  First
       ! three terms should be accurate within machine precision,
       ! because the third term is less than theta**4 and thus less
       ! than epsilon.
       s = 1d0 + x * (-1d0/6 + x/120d0)
    else
       s = sin(theta)/theta
    end if

  end function aa_tf_sinc

  !! Integrate rotational velocity
  subroutine aa_tf_qsvel( q0, w, dt, q1 ) &
       bind( C, name="aa_tf_qsvel" )
    real(C_DOUBLE), intent(in) :: w(3), q0(4)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: q1(4)
    real(C_DOUBLE) :: e(4), wnorm, theta, dt2
    !! Exponential map method
    wnorm = sqrt(dot_product(w,w))
    dt2 = dt/2
    theta = wnorm * dt2
    e(XYZ_INDEX) = aa_tf_sinc(theta) * dt2 * w
    e(W_INDEX) = cos(theta)
    call aa_tf_qmul(e,q0, q1)
  end subroutine aa_tf_qsvel


  !!! Dual Quaternions
  !!!
  !!! Stored as 8 doubles
  !!! - First four are "real" (rotation)
  !!! - Second four are "dual" (translation)


  !> Dual quaternion scalar multplication
  subroutine aa_tf_duqu_smul( s, d, e ) &
       bind( C, name="aa_tf_duqu_smul" )
    real(C_DOUBLE), intent(in), value :: s
    real(C_DOUBLE), intent(in) :: d(8)
    real(C_DOUBLE), intent(out) ::e(8)
    e = s*d
  end subroutine aa_tf_duqu_smul

  !> Dual quaternion addition
  subroutine aa_tf_duqu_add( d1, d2, e ) &
       bind( C, name="aa_tf_duqu_add" )
    real(C_DOUBLE), intent(in), dimension(8) :: d1, d2
    real(C_DOUBLE), intent(out), dimension(8) :: e
    e = d1+d2
  end subroutine aa_tf_duqu_add

  !> Dual quaternion multiplication
  subroutine aa_tf_duqu_mul( d1, d2, e ) &
       bind( C, name="aa_tf_duqu_mul" )
    real(C_DOUBLE), intent(in), dimension(8) :: d1, d2
    real(C_DOUBLE), intent(out), dimension(8) :: e
    real(C_DOUBLE), dimension(4) :: t1, t2

    call aa_tf_qmul( d1(DQ_REAL), d2(DQ_REAL), e(DQ_REAL) )
    call aa_tf_qmul( d1(DQ_REAL), d2(DQ_DUAL), t1 )
    call aa_tf_qmul( d1(DQ_DUAL), d2(DQ_REAL), t2 )
    e(DQ_DUAL) = t1 + t2
  end subroutine aa_tf_duqu_mul

  !> Dual quaternion conjugate
  subroutine aa_tf_duqu_conj( d, e ) &
       bind( C, name="aa_tf_duqu_conj" )
    real(C_DOUBLE), intent(in), dimension(8) :: d
    real(C_DOUBLE), intent(out), dimension(8) :: e
    call aa_tf_qconj( d(DQ_REAL), e(DQ_REAL) )
    call aa_tf_qconj( d(DQ_DUAL), e(DQ_DUAL) )
  end subroutine aa_tf_duqu_conj

  subroutine aa_tf_duqu_cmul( d1, d2, e ) &
       bind( C, name="aa_tf_duqu_cmul" )
    real(C_DOUBLE), intent(in), dimension(8) :: d1, d2
    real(C_DOUBLE), intent(out), dimension(8) :: e
    real(C_DOUBLE), dimension(8) :: d1c
    call aa_tf_duqu_conj( d1, d1c )
    call aa_tf_duqu_mul( d1c, d2, e );
  end subroutine aa_tf_duqu_cmul

  subroutine aa_tf_duqu_mulc( d1, d2, e ) &
       bind( C, name="aa_tf_duqu_mulc" )
    real(C_DOUBLE), intent(in), dimension(8) :: d1, d2
    real(C_DOUBLE), intent(out), dimension(8) :: e
    real(C_DOUBLE), dimension(8) :: d2c
    call aa_tf_duqu_conj( d2, d2c )
    call aa_tf_duqu_mul( d1, d2c, e );
  end subroutine aa_tf_duqu_mulc

  !> Normalize a dual quaternion
  subroutine aa_tf_duqu_normalize( d ) &
       bind( C, name="aa_tf_duqu_normalize" )
    real(C_DOUBLE), intent(inout), dimension(8) :: d
    d = d / aa_la_norm2( d(DQ_REAL) )
  end subroutine aa_tf_duqu_normalize

  !> Dual quaternion norm
  subroutine aa_tf_duqu_norm( d, nreal, ndual ) &
       bind( C, name="aa_tf_duqu_norm" )
    real(C_DOUBLE), intent(in), dimension(8) :: d
    real(C_DOUBLE), intent(out) :: nreal, ndual
    nreal = sqrt( dot_product(d(DQ_REAL), d(DQ_REAL)) )
    ndual = dot_product( d(DQ_REAL), d(DQ_DUAL) ) / nreal
  end subroutine aa_tf_duqu_norm

  !> Dual quaternion construction from unit quaternion and translation
  !> vector.
  subroutine aa_tf_qv2duqu( q, v, d ) &
       bind( C, name="aa_tf_qv2duqu" )
    real(C_DOUBLE), intent(in), dimension(3) :: q(4), v(3)
    real(C_DOUBLE), intent(out), dimension(8) :: d
    d(DQ_REAL) = q

    ! d%dual = 1/2 * v * q, Note, this is the same formula as
    ! converting an angular velocity to a quaternion derivative
    call aa_tf_qvel2diff( q, v, d(DQ_DUAL))

  end subroutine aa_tf_qv2duqu

  !> Dual quaternion translation vector
 subroutine aa_tf_duqu_trans( d, t ) &
       bind( C, name="aa_tf_duqu_trans" )
    real(C_DOUBLE), intent(in), dimension(8) :: d
    real(C_DOUBLE), intent(out), dimension(3) :: t

    ! t = 2 * d(dual) * d(real)^*, and ignore t(w)
    real(C_DOUBLE) ::  mul(4)
    call aa_tf_qmulc( d(DQ_DUAL), d(DQ_REAL), mul )
    t = 2*mul(XYZ_INDEX)

    !call aa_tf_cross(d(XYZ_INDEX), d(DQ_DUAL_XYZ), t)
    !t = 2 * ( t + d(DQ_REAL_W)*d(DQ_DUAL_XYZ) - d(DQ_DUAL_W)*d(DQ_REAL_XYZ) )

  end subroutine aa_tf_duqu_trans

  !> Dual quaternion to transformation matrix
  subroutine aa_tf_duqu2tfmat( d, t ) &
       bind( C, name="aa_tf_duqu2tfmat" )
    real(C_DOUBLE), intent(in), dimension(8) :: d
    real(C_DOUBLE), intent(out), dimension(3,4) :: t
    call aa_tf_quat2rotmat( d(DQ_REAL), t(R_INDEX) )
    call aa_tf_duqu_trans( d, t(T_INDEX) )
  end subroutine aa_tf_duqu2tfmat

  !> Transform a point using the dual quaternion
  subroutine aa_tf_duqu( d, p0, p1 ) &
       bind( C, name="aa_tf_duqu" )
    real(C_DOUBLE), intent(in) :: d(8), p0(3)
    real(C_DOUBLE), intent(out) :: p1(3)
    ! p1 = d * p * d^{-1}
    ! p1 = (2*dual + real*p0) * conj(real)

    real(C_DOUBLE) :: ax(3), aw

    ! ax = real_w p + real_x x p + 2 dual_x
    call aa_tf_cross( d(DQ_REAL_XYZ), p0, ax )
    ax = ax + d(DQ_REAL_W)*p0 + 2*d(DQ_DUAL_XYZ)

    ! aw = dot( real_x, p ) - 2 dual_w
    aw = dot_product( d(DQ_REAL_XYZ), p0 ) - 2*d(DQ_DUAL_W)

    ! p1 = real_x x ax + real_w ax + aw real_x
    call aa_tf_cross( d(DQ_REAL_XYZ), ax, p1 )
    p1 = p1 + d(DQ_REAL_W)*ax + aw*d(DQ_REAL_XYZ)

  end subroutine aa_tf_duqu


  !! Convert spatial velocity to quaternion derivative
  subroutine aa_tf_duqu_vel2diff( d, dx, dd ) &
       bind( C, name="aa_tf_duqu_vel2diff" )
    real(C_DOUBLE), intent(in) :: d(8), dx(6)
    real(C_DOUBLE), intent(out) :: dd(8)
    real(C_DOUBLE) :: a(4), b(4), c(4)
    ! orientation
    call aa_tf_qvel2diff( d(DQ_REAL), dx(4:6), dd(DQ_REAL) )
    ! translation
    ! dd_dual = (dx*d_real + x*dd_real) / 2
    ! dd_dual = dx*r/2 + d*r_conj*dr)
    call aa_tf_vqmul( dx(1:3), d(DQ_REAL), a )
    call aa_tf_qmulc( d(DQ_DUAL), d(DQ_REAL), b)
    call aa_tf_qmul( b, dd(DQ_REAL), c )
    dd(DQ_DUAL) = a/2d0 + c
  end subroutine aa_tf_duqu_vel2diff

  !! Convert spatial velocity to quaternion derivative
  subroutine aa_tf_duqu_diff2vel( d, dd, dx ) &
       bind( C, name="aa_tf_duqu_diff2vel" )
    real(C_DOUBLE), intent(in) :: d(8), dd(8)
    real(C_DOUBLE), intent(out) :: dx(6)
    real(C_DOUBLE) :: t1(4), t2(4)
    ! orientation
    call aa_tf_qdiff2vel( d(DQ_REAL), dd(DQ_REAL), dx(4:6) )
    ! translation
    ! dx/dt = 2 * ( d_dual/dt conj(r) + d_dual conj(d_real/dt) )
    call aa_tf_qmulc( dd(DQ_DUAL), d(DQ_REAL), t1 )
    call aa_tf_qmulc( d(DQ_DUAL), dd(DQ_REAL), t2 )
    dx(1:3) = 2 * (t1(XYZ_INDEX) + t2(XYZ_INDEX))
  end subroutine aa_tf_duqu_diff2vel

  !! TODO: Some questions on numerical accuracy here.  Is it be better
  !! to extract translational velocity from the dual quaternion
  !! derivative, and integrate orientation and translation
  !! independently, or to just RK1 the dual quaternion derivative?

  ! ! Integrate dual quaternion, runge-kutta 1
  ! subroutine aa_tf_duqu_rk1( d0, dd, dt, d1 ) &
  !      bind( C, name="aa_tf_duqu_rk1" )
  !   real(C_DOUBLE), intent(in) :: dd(8), d0(8)
  !   real(C_DOUBLE), intent(in), value :: dt
  !   real(C_DOUBLE), intent(out) :: d1(8)
  !   d1 = d0 + dt * dd         ! euler integration
  !   call aa_tf_duqu_normalize(d1)
  ! end subroutine aa_tf_duqu_rk1


  ! subroutine aa_tf_duqu_rk4( d0, dd, dt, d1 ) &
  !      bind( C, name="aa_tf_duqu_rk4" )
  !   real(C_DOUBLE), intent(in) :: dd(8), d0(8)
  !   real(C_DOUBLE), intent(in), value :: dt
  !   real(C_DOUBLE), intent(out) :: d1(8)
  !   d1 = d0 + dt * dd         ! euler integration
  !   call aa_tf_duqu_normalize(d1)

  ! contains
  !   subroutine term( dd0, dt, dd1 )
  !     real(C_DOUBLE), intent(in) ::  dq0(8)
  !     real(C_DOUBLE), intent(in), value :: dt
  !     real(C_DOUBLE), intent(out) :: dq1(8)
  !     real(C_DOUBLE) :: qtmp(8)
  !     call aa_tf_duqu_rk1( q0, dq0, dt, qtmp )
  !     call aa_tf_qvel2diff(qtmp, v, dq1)
  !   end subroutine term
  ! end subroutine aa_tf_duqu_rk4


  !! Integrate spatial velocity to to get dual quaternion
  subroutine aa_tf_duqu_svel( d0, dx, dt, d1 ) &
       bind( C, name="aa_tf_duqu_svel" )
    real(C_DOUBLE), intent(in) :: dx(6), d0(8)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: d1(8)
    real(C_DOUBLE) :: x0(3), x1(3), q1(4)
    ! translation
    call aa_tf_duqu_trans( d0, x0 )
    x1 = x0 + dt*dx(1:3) ! constant dx => rk4 == euler
    ! orientation
    call aa_tf_qsvel( d0(1:4), dx(4:6), dt, q1 )
    call aa_tf_qv2duqu( q1, x1, d1 )
  end subroutine aa_tf_duqu_svel

  !! Integrate dual quaternion derivative
  subroutine aa_tf_duqu_sdiff( d0, dd, dt, d1 ) &
       bind( C, name="aa_tf_duqu_sdiff" )
    real(C_DOUBLE), intent(in) :: dd(8), d0(8)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: d1(8)
    real(C_DOUBLE) :: dx(6)
    call aa_tf_duqu_diff2vel( d0, dd, dx )
    call aa_tf_duqu_svel( d0, dx, dt, d1 );
  end subroutine aa_tf_duqu_sdiff

#include "aa_tf_euler.f90"

end module amino_tf
