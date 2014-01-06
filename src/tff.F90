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

#define PI (4d0*atan(1d0))
#define PI_2 (2d0*atan(1d0))

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
  implicit none

  interface aa_tf_rotmat2quat
     subroutine aa_tf_rotmat2quat( r, q ) &
          bind(C,name="aa_tf_rotmat2quat")
       use ISO_C_BINDING
       real(C_DOUBLE), intent(in), dimension(3,3) :: r
       real(C_DOUBLE), intent(out), dimension(4) :: q
     end subroutine aa_tf_rotmat2quat
  end interface aa_tf_rotmat2quat


  interface aa_tf_rotmat2axang
     subroutine aa_tf_rotmat2axang( r, a ) &
          bind(C,name="aa_tf_rotmat2axang")
       use ISO_C_BINDING
       real(C_DOUBLE), intent(in), dimension(3,3) :: r
       real(C_DOUBLE), intent(out), dimension(4) :: a
     end subroutine aa_tf_rotmat2axang
  end interface aa_tf_rotmat2axang

  type aa_tf_dual_t
     real(C_DOUBLE) :: r
     real(C_DOUBLE) :: d
  end type aa_tf_dual_t

  Interface Operator(+)
     MODULE PROCEDURE aa_tf_dual_add
  End Interface
  Interface Operator(-)
     MODULE PROCEDURE aa_tf_dual_sub
  End Interface
  Interface Operator(*)
     MODULE PROCEDURE aa_tf_dual_mul
  End Interface
  Interface Operator(/)
     MODULE PROCEDURE aa_tf_dual_div
  End Interface

  Interface sin
     MODULE PROCEDURE aa_tf_dual_sin
  End Interface
  Interface cos
     MODULE PROCEDURE aa_tf_dual_cos
  End Interface
  Interface acos
     MODULE PROCEDURE aa_tf_dual_acos
  End Interface
  Interface atan2
     MODULE PROCEDURE aa_tf_dual_atan2
  End Interface
  Interface exp
     MODULE PROCEDURE aa_tf_dual_exp
  End Interface
  Interface log
     MODULE PROCEDURE aa_tf_dual_log
  End Interface
  Interface sqrt
     MODULE PROCEDURE aa_tf_dual_sqrt
  End Interface

contains
  pure subroutine aa_tf_9( R1, p1, p ) &
       bind( C, name="aa_tf_9" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: p1(3)
    real(C_DOUBLE), intent(out) :: p(3)
    p(1) = R1(1,1)*p1(1) + R1(1,2)*p1(2) + R1(1,3)*p1(3)
    p(2) = R1(2,1)*p1(1) + R1(2,2)*p1(2) + R1(2,3)*p1(3)
    p(3) = R1(3,1)*p1(1) + R1(3,2)*p1(2) + R1(3,3)*p1(3)
  end subroutine aa_tf_9

  pure Subroutine aa_tf_9mul(R1, R2, R3) &
       bind( C, name="aa_tf_9mul" )
    real(C_DOUBLE), intent(in), dimension(3,3) :: R1, R2
    real(C_DOUBLE), intent(out), dimension(3,3) :: R3
    R3(1,1) =  R1(1,1)*R2(1,1) + R1(1,2)*R2(2,1) + R1(1,3)*R2(3,1);
    R3(2,1) =  R1(2,1)*R2(1,1) + R1(2,2)*R2(2,1) + R1(2,3)*R2(3,1);
    R3(3,1) =  R1(3,1)*R2(1,1) + R1(3,2)*R2(2,1) + R1(3,3)*R2(3,1);

    R3(1,2) =  R1(1,1)*R2(1,2) + R1(1,2)*R2(2,2) + R1(1,3)*R2(3,2);
    R3(2,2) =  R1(2,1)*R2(1,2) + R1(2,2)*R2(2,2) + R1(2,3)*R2(3,2);
    R3(3,2) =  R1(3,1)*R2(1,2) + R1(3,2)*R2(2,2) + R1(3,3)*R2(3,2);

    R3(1,3) =  R1(1,1)*R2(1,3) + R1(1,2)*R2(2,3) + R1(1,3)*R2(3,3);
    R3(2,3) =  R1(2,1)*R2(1,3) + R1(2,2)*R2(2,3) + R1(2,3)*R2(3,3);
    R3(3,3) =  R1(3,1)*R2(1,3) + R1(3,2)*R2(2,3) + R1(3,3)*R2(3,3);
  End subroutine aa_tf_9mul

  pure subroutine aa_tf_93( R1, v1, p1, p) &
       bind( C, name="aa_tf_93" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: v1(3)
    real(C_DOUBLE), intent(in)  :: p1(3)
    real(C_DOUBLE), intent(out) :: p(3)
    call aa_tf_9( R1, p1, p )
    p = p + v1
  end subroutine aa_tf_93

  pure subroutine aa_tf_tf_qv( q, v, p1, p2) &
       bind( C, name="aa_tf_tf_qv" )
    real(C_DOUBLE), intent(in)  :: q(4), v(3), p1(3)
    real(C_DOUBLE), intent(out) :: p2(3)
    call aa_tf_qrot( q, p1, p2 )
    p2 = p2 + v
  end subroutine aa_tf_tf_qv

  pure subroutine aa_tf_93chain( R1, v1, R2, v2, R3, v3 ) &
       bind( C, name="aa_tf_93chain" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: v1(3)
    real(C_DOUBLE), intent(in)  :: R2(3,3)
    real(C_DOUBLE), intent(in)  :: v2(3)
    real(C_DOUBLE), intent(out) :: R3(3,3)
    real(C_DOUBLE), intent(out) :: v3(3)
    call aa_tf_9mul( R1, R2, R3 )
    call aa_tf_93(R1, v1, v2, v3)
  end subroutine aa_tf_93chain

  pure subroutine aa_tf_qv_chain( q1, v1, q2, v2, q3, v3 ) &
       bind( C, name="aa_tf_qv_chain" )
    real(C_DOUBLE), intent(in)  :: q1(4), v1(3), q2(4), v2(3)
    real(C_DOUBLE), intent(out) :: q3(4), v3(3)
    call aa_tf_qmul( q1, q2, q3 )
    call aa_tf_tf_qv(q1, v1, v2, v3)
  end subroutine aa_tf_qv_chain

  pure subroutine aa_tf_qv_conj( q, v, qc, vc ) &
       bind( C, name="aa_tf_qv_conj" )
    real(C_DOUBLE), intent(in)  :: q(4), v(3)
    real(C_DOUBLE), intent(out) :: qc(4), vc(3)
    call aa_tf_qconj(q, qc)
    call aa_tf_qrot(qc, v, vc)
    vc = -vc
  end subroutine aa_tf_qv_conj


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

  ! screw symmetrix matrix for u
  subroutine aa_tf_skew_sym( u, R ) &
       bind( C, name="aa_tf_skew_sym" )
    real(C_DOUBLE), intent(in) ::  u(3)
    real(C_DOUBLE), intent(out) :: R(3,3)
    real(C_DOUBLE) :: x,y,z

    x = u(1)
    y = u(2)
    z = u(3)

    R(1,1) = 0d0
    R(2,1) = z
    R(3,1) = -y

    R(1,2) = -z
    R(2,2) = 0d0
    R(3,2) = x

    R(1,3) = y
    R(2,3) = -x
    R(3,3) = 0d0
  end subroutine aa_tf_skew_sym

  subroutine aa_tf_skewsym_scal1( a, u, R ) &
       bind( C, name="aa_tf_skewsym_scal1" )
    real(C_DOUBLE), intent(in) ::  u(3), a
    real(C_DOUBLE), intent(out) :: R(3,3)
    real(C_DOUBLE) :: x,y,z

    x = a*u(1)
    y = a*u(2)
    z = a*u(3)

    R(1,1) = 0d0
    R(2,1) = z
    R(3,1) = -y

    R(1,2) = -z
    R(2,2) = 0d0
    R(3,2) = x

    R(1,3) = y
    R(2,3) = -x
    R(3,3) = 0d0
  end subroutine aa_tf_skewsym_scal1


  subroutine aa_tf_skewsym_scal_c( u, a, b, R ) &
       bind( C, name="aa_tf_skewsym_scal_c" )
    real(C_DOUBLE), intent(in) ::  u(3), a(3), b(3)
    real(C_DOUBLE), intent(out) :: R(3,3)
    real(C_DOUBLE) :: bu(3), bc(3)
    real(C_DOUBLE) :: cx,cy,cz, dx,dy,dz, ex,ey,ez

    bu = b*u

    bc(1) = b(1)*u(2)
    bc(2) = b(2)*u(3)
    bc(3) = b(3)*u(1)

    cx = 1d0 - bu(3) - bu(2)
    cy = 1d0 - bu(3) - bu(1)
    cz = 1d0 - bu(2) - bu(1)

    dx = a(3) + bc(1)
    dy = a(1) + bc(2)
    dz = a(2) + bc(3)

    ex = bc(3) - a(2)
    ey = bc(1) - a(3)
    ez = bc(2) - a(1)

    R(1,1) = cx
    R(2,1) = dx
    R(3,1) = ex

    R(1,2) = ey
    R(2,2) = cy
    R(3,2) = dy

    R(1,3) = dz
    R(2,3) = ez
    R(3,3) = cz
  end subroutine aa_tf_skewsym_scal_c

  subroutine aa_tf_skewsym_scal2( a, b, u, R ) &
       bind( C, name="aa_tf_skewsym_scal2" )
    real(C_DOUBLE), intent(in) ::  u(3)
    real(C_DOUBLE), intent(in), value ::  a, b
    real(C_DOUBLE), intent(out) :: R(3,3)
    real(C_DOUBLE) :: au(3), bu(3)
    au = a*u
    bu = b*u
    call aa_tf_skewsym_scal_c(u,au,bu,R)
  end subroutine aa_tf_skewsym_scal2


  subroutine aa_tf_quat2rotmat( q, R ) &
       bind( C, name="aa_tf_quat2rotmat" )

    real(C_DOUBLE), intent(in) :: q(4)
    real(C_DOUBLE), intent(out) :: R(3,3)
    real(C_DOUBLE) :: a(3), b(3)
    b = 2*q(XYZ_INDEX)
    a = q(4)*b
    call aa_tf_skewsym_scal_c(q(XYZ_INDEX),a,b,R)

    ! real(C_DOUBLE) :: rx,ry,rz,rw
    ! rx = q(1)
    ! ry = q(2)
    ! rz = q(3)
    ! rw = q(4)

    ! R(1,1) = -rz**2-ry**2+rx**2+rw**2
    ! R(1,2) = 2*rx*ry-2*rw*rz
    ! R(1,3) = 2*rx*rz+2*rw*ry

    ! R(2,1) = 2*rw*rz+2*rx*ry
    ! R(2,2) = -rz**2+ry**2-rx**2+rw**2
    ! R(2,3) = 2*ry*rz-2*rw*rx

    ! R(3,1) = 2*rx*rz-2*rw*ry
    ! R(3,2) = 2*ry*rz+2*rw*rx
    ! R(3,3) = rz**2-ry**2-rx**2+rw**2

  end subroutine aa_tf_quat2rotmat


  subroutine aa_tf_unskewsym( R, u ) &
       bind( C, name="aa_tf_unskewsym" )
    real(C_DOUBLE), intent(out) ::  u(3)
    real(C_DOUBLE), intent(in) :: R(3,3)
    real(C_DOUBLE) :: tr, c

    tr = R(1,1) + R(2,2) + R(3,3)
    c = sqrt( tr + 1 ) / 2d0

    u(1) = c * (R(3,2) - R(2,3))
    u(2) = c * (R(1,3) - R(3,1))
    u(3) = c * (R(2,1) - R(1,2))

  end subroutine aa_tf_unskewsym


  subroutine aa_tf_unskewsym_scal( a, R, u ) &
       bind( C, name="aa_tf_unskewsym_scal" )
    real(C_DOUBLE), intent(out) ::  u(3)
    real(C_DOUBLE), intent(in) :: R(3,3), a
    u(1) = a * (R(3,2) - R(2,3))
    u(2) = a * (R(1,3) - R(3,1))
    u(3) = a * (R(2,1) - R(1,2))
  end subroutine aa_tf_unskewsym_scal

  subroutine aa_tf_rotmat_exp_aa( axang, E ) &
       bind( C, name="aa_tf_rotmat_exp_aa" )
    real(C_DOUBLE), intent(in) ::  axang(4)
    real(C_DOUBLE), intent(out) :: E(3,3)
    call aa_tf_skewsym_scal2( sin(axang(4)), 1d0-cos(axang(4)), axang(1:3), E )
  end subroutine aa_tf_rotmat_exp_aa

  subroutine aa_tf_rotmat_expv( rv, E ) &
       bind( C, name="aa_tf_rotmat_expv" )
    real(C_DOUBLE), intent(in) ::  rv(3)
    real(C_DOUBLE), intent(out) :: E(3,3)
    real(C_DOUBLE) :: sc, cc, theta, theta2

    theta2 = dot_product(rv,rv)
    theta = sqrt(theta2)

    if ( theta2 < sqrt(epsilon(theta2)) ) then
       ! taylor series
       sc = aa_tf_sinc_series2(theta2) ! approx. 1
       cc = theta * aa_tf_horner3( theta2, 1d0/2, -1d0/24, 1d0/720 )
    else
       sc = sin(theta)/theta
       cc = (1d0-cos(theta)) / theta2
    end if

    call aa_tf_skewsym_scal2( sc, cc, rv, E )
  end subroutine aa_tf_rotmat_expv


  subroutine aa_tf_rotmat_angle( R, c, s, theta )
    real(C_DOUBLE), intent(in) ::  R(3,3)
    real(C_DOUBLE), intent(out) ::  c,s,theta
    c = ( R(1,1) + R(2,2) + R(3,3) - 1 ) / 2
    s = sqrt( 1 - c*c ) ! efficiently compute sin, always positive
    theta = atan2(s,c)  ! always positive
  end subroutine aa_tf_rotmat_angle

  subroutine aa_tf_rotmat_lnv( R, v ) &
       bind( C, name="aa_tf_rotmat_lnv" )
    real(C_DOUBLE), intent(out) ::  v(3)
    real(C_DOUBLE), intent(in) :: R(3,3)
    real(C_DOUBLE) :: a, c, s, theta
    call aa_tf_rotmat_angle(R, c, s, theta )
    if ( theta < sqrt(sqrt(epsilon(theta))) ) then
       a = aa_tf_invsinc_series(theta)
    else
       a = theta / s
    end if
    call aa_tf_unskewsym_scal( a/2d0, R, v )
  end subroutine aa_tf_rotmat_lnv

  subroutine aa_tf_tfmat_expv( v, T ) &
       bind( C, name="aa_tf_tfmat_expv" )
    real(C_DOUBLE), intent(in) ::  v(6)
    real(C_DOUBLE), intent(out) :: T(3,4)
    real(C_DOUBLE) :: sc, cc, ssc, theta2, theta, K(3,3)

    theta2 = dot_product(v(4:6),v(4:6))
    theta = sqrt(theta2)

    if ( theta2 < sqrt(epsilon(theta2)) ) then
       ! taylor series
       sc = aa_tf_sinc_series2(theta2) ! approx. 1
       cc = theta * aa_tf_horner3( theta2, 1d0/2, -1d0/24, 1d0/720 )
       ssc = aa_tf_horner3( theta2, 1d0/6, -1d0/120, 1d0/5040 )
    else
       sc = sin(theta)/theta
       cc = (1d0 - cos(theta)) / theta2
       ssc = (theta - sin(theta)) / (theta2*theta)
    end if

    call aa_tf_skewsym_scal2( sc, cc,  v(4:6), T(:,1:3) )
    call aa_tf_skewsym_scal2( cc, ssc, v(4:6), K )

    call aa_tf_9(K, v(1:3), T(:,4))

  end subroutine aa_tf_tfmat_expv

  subroutine aa_tf_tfmat_lnv( T, v ) &
       bind( C, name="aa_tf_tfmat_lnv" )
    real(C_DOUBLE), intent(out) ::  v(6)
    real(C_DOUBLE), intent(in) :: T(3,4)
    real(C_DOUBLE) :: theta, c, s, a, b, K(3,3)

    call aa_tf_rotmat_angle(T(:,1:3), c, s, theta )

    if ( theta < sqrt(sqrt(epsilon(theta))) ) then
       a = aa_tf_invsinc_series2(theta**2)
       b = aa_tf_horner3( theta**2, 1d0/12, 1d0/720, 1d0/30240 )
    else
       a = theta / s
       b = ( 2*s - theta*(1+c) ) / ( 2*theta**2 * s )
    end if
    call aa_tf_unskewsym_scal( a/2d0, T(:,1:3), v(4:6) )
    call aa_tf_skewsym_scal2( -0.5d0, b, v(4:6), K )
    call aa_tf_9( K, T(:,4), v(1:3) )
  end subroutine aa_tf_tfmat_lnv

  subroutine aa_tf_rotmat_vel2diff( R, w, dR ) &
       bind( C, name="aa_tf_rotmat_vel2diff" )
    real(C_DOUBLE), intent(in) :: R(3,3), w(3)
    real(C_DOUBLE), intent(out) :: dR(3,3)
    real(C_DOUBLE) :: V(3,3)
    call aa_tf_skew_sym( w, V )
    call aa_tf_9mul(V,R,dR)
  end subroutine aa_tf_rotmat_vel2diff

  subroutine aa_tf_rotmat_diff2vel( R, dR, w ) &
       bind( C, name="aa_tf_rotmat_diff2vel" )
    real(C_DOUBLE), intent(in) :: R(3,3), dR(3,3)
    real(C_DOUBLE), intent(out) ::  w(3)
    real(C_DOUBLE) :: V(3,3)
    V = matmul( dR, transpose(R) )
    call aa_tf_unskewsym(V,w)
  end subroutine aa_tf_rotmat_diff2vel

  subroutine aa_tf_qv2tfmat( q, v, T ) &
       bind( C, name="aa_tf_qv2tfmat" )
    real(C_DOUBLE), intent(in)  :: q(4), v(3)
    real(C_DOUBLE), intent(out)  :: T(3,4)
    call aa_tf_quat2rotmat( q, T(:,1:3) )
    T(:,4) = v
  end subroutine aa_tf_qv2tfmat

  !! Integrate rotational velocity
  subroutine aa_tf_rotmat_svel( R0, w, dt, R1 ) &
       bind( C, name="aa_tf_rotmat_svel" )
    real(C_DOUBLE), intent(in) :: w(3), R0(3,3)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: R1(3,3)
    real(C_DOUBLE)  :: delta(3), e(3,3)
    !! Exponential map method
    !! q1 = exp(w*dt/2) * q0
    delta = w*dt
    call aa_tf_rotmat_expv( delta, e )
    call aa_tf_9mul(e,R0, R1)
  end subroutine aa_tf_rotmat_svel

  !! Integrate rotational velocity
  subroutine aa_tf_tfmat_svel( T0, dx, dt, T1 ) &
       bind( C, name="aa_tf_tfmat_svel" )
    real(C_DOUBLE), intent(in) :: dx(6), T0(3,4)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: T1(3,4)
    real(C_DOUBLE)  :: delta(6), e(3,4)
    !! Exponential map method
    !! q1 = exp(w*dt/2) * q0
    delta(4:6) = dx(4:6) * dt
    call aa_tf_cross( dx(4:6), T0(:,4), delta(1:3) )
    delta(1:3) = (dx(1:3) - delta(1:3)) * dt ! twist
    call aa_tf_tfmat_expv(delta, e)
    call aa_tf_12chain(e, T0, T1)
  end subroutine aa_tf_tfmat_svel

  subroutine aa_tf_tfmat_vel2diff( T, dx, dT ) &
       bind( C, name="aa_tf_tfmat_vel2diff" )
    real(C_DOUBLE), intent(in) :: T(3,4), dx(6)
    real(C_DOUBLE), intent(out) :: dT(3,4)
    call aa_tf_rotmat_vel2diff( T(:,1:3), dx(4:6), dT(:,1:3) )
    dT(:,4) = dx(1:3)
  end subroutine aa_tf_tfmat_vel2diff

  subroutine aa_tf_tfmat_diff2vel( T, dT, dx ) &
       bind( C, name="aa_tf_tfmat_diff2vel" )
    real(C_DOUBLE), intent(in) :: T(3,4), dT(3,4)
    real(C_DOUBLE), intent(out) ::  dx(6)
    call aa_tf_rotmat_diff2vel( T(:,1:3), dT(:,1:3), dx(4:6) )
    dx(1:3) =  dT(:,4)
  end subroutine aa_tf_tfmat_diff2vel

  !!! Quaternions

  pure function aa_tf_qnorm( q ) result(n) &
       bind( C, name="aa_tf_qnorm" )
    real(C_DOUBLE), dimension(4), intent(in) :: q
    real(C_DOUBLE) :: n
    n =  sqrt(dot_product(q,q))
  end function aa_tf_qnorm

  pure function aa_tf_qvnorm( q ) result(n) &
       bind( C, name="aa_tf_qvnorm" )
    real(C_DOUBLE), dimension(4), intent(in) :: q
    real(C_DOUBLE) :: n
    n =  sqrt(dot_product(q(XYZ_INDEX),q(XYZ_INDEX)))
  end function aa_tf_qvnorm

  pure subroutine aa_tf_qnormalize( q ) &
       bind( C, name="aa_tf_qnormalize" )
    real(C_DOUBLE), dimension(4), intent(inout) :: q
    q = q / aa_tf_qnorm(q)
  end subroutine aa_tf_qnormalize

  pure subroutine aa_tf_qminimize( q ) &
       bind( C, name="aa_tf_qminimize" )
    real(C_DOUBLE), dimension(4), intent(inout) :: q
    ! Make W positive. This puts the quaternion in the right hand side
    ! of the complex plane.  Thus, it will represent a rotation with a
    ! minimum angle.
    if( q(W_INDEX) < 0 ) then
       q = -q
    end if
  end subroutine aa_tf_qminimize

  pure subroutine aa_tf_qminimize2( q, qm ) &
       bind( C, name="aa_tf_qminimize2" )
    real(C_DOUBLE), dimension(4), intent(in) :: q
    real(C_DOUBLE), dimension(4), intent(out) :: qm
    if( q(W_INDEX) < 0 ) then
       qm = -q
    else
       qm = q
    end if
  end subroutine aa_tf_qminimize2

  pure subroutine aa_tf_qnormalize2( q, qnorm ) &
       bind( C, name="aa_tf_qnormalize2" )
    real(C_DOUBLE), dimension(4), intent(in) :: q
    real(C_DOUBLE), dimension(4), intent(out) :: qnorm
    qnorm = q
    call aa_tf_qnormalize(qnorm)
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

    ! q(1) = a(2)*b(3) + a(4)*b(1) + (a(1)*b(4) - a(3)*b(2))
    ! q(2) = a(2)*b(4) + a(4)*b(2) - (a(1)*b(3) - a(3)*b(1))
    ! q(3) =  + a(4)*b(3) - a(2)*b(1) +  a(1)*b(2) + a(3)*b(4)
    ! q(4) =  + a(4)*b(4) - a(2)*b(2) - (a(1)*b(1) + a(3)*b(3))

    ! q(1) = + a(1)*b(4) + (-a(3)*b(2)) + (a(4)*b(1) +   a(2)*b(3))
    ! q(2) = + a(2)*b(4) +   a(4)*b(2)  + (a(3)*b(1) + (-a(1)*b(3)))
    ! q(3) =  - (-a(1)*b(2)) + a(3)*b(4) + (   a(4)*b(3)  - a(2)*b(1) )
    ! q(4) =  -   a(2)*b(2)  + a(4)*b(4) + ( (-a(3)*b(3)) - a(1)*b(1) )

    q(1) =    a(1)*b(4) + a(2)*b(3) + a(4)*b(1) - a(3)*b(2)
    q(2) =    a(3)*b(1) + a(4)*b(2) + a(2)*b(4) - a(1)*b(3)
    q(3) =    a(4)*b(3) + a(3)*b(4) + a(1)*b(2) - a(2)*b(1)
    q(4) = - (a(2)*b(2) + a(1)*b(1) + a(3)*b(3) - a(4)*b(4))


    !q(W_INDEX) = a(W_INDEX)*b(W_INDEX) - dot_product(a(XYZ_INDEX),b(XYZ_INDEX))
    !call aa_tf_cross(a(XYZ_INDEX), b(XYZ_INDEX), q(XYZ_INDEX))
    !q(XYZ_INDEX) = q(XYZ_INDEX) + a(W_INDEX)*b(XYZ_INDEX) +  a(XYZ_INDEX)*b(W_INDEX)

  end subroutine aa_tf_qmul


  ! Matrix for left quaternion in multiply
  pure subroutine aa_tf_qmatrix_l( q, m) &
       bind( C, name="aa_tf_qmatrix_l" )
    real(C_DOUBLE), dimension(4), intent(in) :: q
    real(C_DOUBLE), dimension(4,4), intent(out) :: m

    m(1,1) = q(4)
    m(2,1) = q(3)
    m(3,1) = -q(2)
    m(4,1) = -q(1)

    m(1,2) = -q(3)
    m(2,2) = q(4)
    m(3,2) = q(1)
    m(4,2) = -q(2)

    m(1,3) = q(2)
    m(2,3) = -q(1)
    m(3,3) = q(4)
    m(4,3) = -q(3)

    m(1,4) = q(1)
    m(2,4) = q(2)
    m(3,4) = q(3)
    m(4,4) = q(4)
  end subroutine aa_tf_qmatrix_l

  ! Matrix for right quaternion in multiply
  pure subroutine aa_tf_qmatrix_r( q, m) &
       bind( C, name="aa_tf_qmatrix_r" )
    real(C_DOUBLE), dimension(4), intent(in) :: q
    real(C_DOUBLE), dimension(4,4), intent(out) :: m

    m(1,1) = q(4)
    m(2,1) = -q(3)
    m(3,1) = q(2)
    m(4,1) = -q(1)

    m(1,2) = q(3)
    m(2,2) = q(4)
    m(3,2) = -q(1)
    m(4,2) = -q(2)

    m(1,3) = -q(2)
    m(2,3) = q(1)
    m(3,3) = q(4)
    m(4,3) = -q(3)

    m(1,4) = q(1)
    m(2,4) = q(2)
    m(3,4) = q(3)
    m(4,4) = q(4)
  end subroutine aa_tf_qmatrix_r

  !! Multiply conj(a) and b
  pure subroutine aa_tf_qcmul( a, b, q) &
       bind( C, name="aa_tf_qcmul" )
    real(C_DOUBLE), dimension(4), intent(out) :: q
    real(C_DOUBLE), dimension(4), intent(in) :: a,b
    q(1) = ( a(3)*b(2) - a(2)*b(3) ) + ( a(4)*b(1) - a(1)*b(4) )
    q(2) = ( a(1)*b(3) - a(3)*b(1) ) + ( a(4)*b(2) - a(2)*b(4) )
    q(3) = ( a(2)*b(1) - a(1)*b(2) ) + ( a(4)*b(3) - a(3)*b(4) )
    q(4) = ( a(4)*b(4) + a(3)*b(3) ) + ( a(1)*b(1) + a(2)*b(2) )
  end subroutine aa_tf_qcmul

  !! Multiply a and conj(b)
  pure subroutine aa_tf_qmulc( a, b, q) &
       bind( C, name="aa_tf_qmulc" )
    real(C_DOUBLE), dimension(4), intent(out) :: q
    real(C_DOUBLE), dimension(4), intent(in) :: a,b
    q(1) = ( a(3)*b(2) - a(2)*b(3) ) + ( a(1)*b(4) - a(4)*b(1) )
    q(2) = ( a(1)*b(3) - a(3)*b(1) ) + ( a(2)*b(4) - a(4)*b(2) )
    q(3) = ( a(2)*b(1) - a(1)*b(2) ) + ( a(3)*b(4) - a(4)*b(3) )
    q(4) = ( a(1)*b(1) + a(2)*b(2) ) + ( a(3)*b(3) + a(4)*b(4) )
  end subroutine aa_tf_qmulc


  !! Multiply vector and quaternion
  pure subroutine aa_tf_qmul_vq( v, q, y) &
       bind( C, name="aa_tf_qmul_vq" )
    real(C_DOUBLE), intent(in) :: v(3), q(4)
    real(C_DOUBLE), intent(out) :: y(4)
    y(1) = +  v(2)*q(3) - v(3)*q(2) + v(1)*q(4)
    y(2) = +  v(3)*q(1) - v(1)*q(3) + v(2)*q(4)
    y(3) = +  v(1)*q(2) - v(2)*q(1) + v(3)*q(4)
    y(4) = -  v(1)*q(1) - v(2)*q(2) - v(3)*q(3)

    ! y(1) = + v(1)*q(4) + v(2)*q(3) - v(3)*q(2)
    ! y(2) = - v(1)*q(3) + v(2)*q(4) + v(3)*q(1)
    ! y(3) = + v(1)*q(2) - v(2)*q(1) + v(3)*q(4)
    ! y(4) = - v(1)*q(1) - v(2)*q(2) - v(3)*q(3)
  end subroutine aa_tf_qmul_vq

  pure subroutine aa_tf_qmul_qv( q, v, y) &
       bind( C, name="aa_tf_qmul_qv" )
    real(C_DOUBLE), intent(in) :: v(3), q(4)
    real(C_DOUBLE), intent(out) :: y(4)
    y(1) = + q(2)*v(3) - q(3)*v(2) + q(4)*v(1)
    y(2) = + q(3)*v(1) - q(1)*v(3) + q(4)*v(2)
    y(3) = + q(1)*v(2) - q(2)*v(1) + q(4)*v(3)
    y(4) = - q(1)*v(1) - q(2)*v(2) - q(3)*v(3)

    ! y(1) = + q(4)*v(1) - q(3)*v(2) + q(2)*v(3)
    ! y(2) = + q(3)*v(1) + q(4)*v(2) - q(1)*v(3)
    ! y(3) = - q(2)*v(1) + q(1)*v(2) + q(4)*v(3)
    ! y(4) = - q(1)*v(1) - q(2)*v(2) - q(3)*v(3)
  end subroutine aa_tf_qmul_qv


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

    !! slow implementation
    ! qv(W_INDEX) = 0d0
    ! qv(XYZ_INDEX) = v
    ! Call aa_tf_qconj( q, qi )
    ! call aa_tf_qmul( q, qv, qr1 )
    ! call aa_tf_qmul( qr1, qi, qr2 )
    ! r = qr2(1:3)

    !! Optimized implementation
    real(C_DOUBLE) :: a(3), b(3)
    call aa_tf_cross(q(1:3), v, a)
    a = a + q(4)*v
    call aa_tf_cross(q(1:3), a, b)
    r = b + b + v

  End Subroutine aa_tf_qrot

  pure Subroutine aa_tf_qexp( q, r ) &
       bind( C, name="aa_tf_qexp" )
    real(C_DOUBLE), Dimension(4), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE) :: vv, vnorm, ew, sc, c
    ew = exp(q(W_INDEX))
    vv = dot_product(q(XYZ_INDEX), q(XYZ_INDEX))
    ! Avoid division by very small vnorm
    if( vv < sqrt(epsilon(vv)) ) then
       sc = aa_tf_sinc_series2(vv) ! approx. 1
       c = aa_tf_cos_series2(vv)   ! approx. 1
    else
       vnorm = sqrt(vv)
       sc = sin(vnorm)/vnorm
       c = cos(vnorm)
    end if
    r(W_INDEX) = ew * c
    r(XYZ_INDEX) = ew * sc * q(XYZ_INDEX)
  end Subroutine aa_tf_qexp

  pure subroutine aa_tf_qln( q, r ) &
       bind( C, name="aa_tf_qln" )
    real(C_DOUBLE), Dimension(4), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE) :: vv, vnorm, qnorm, theta, a
    ! compute qnorm and vnorm efficiently
    vv = dot_product(q(XYZ_INDEX), q(XYZ_INDEX))
    qnorm = sqrt( vv + q(W_INDEX)**2 )
    vnorm = sqrt(vv) ! for unit quaternions, vnorm = sin(theta)
    theta = atan2( vnorm, q(W_INDEX) ) ! always positive
    ! Try to avoid division by very small vnorm
    if ( theta < sqrt(sqrt(epsilon(theta))) ) then
       ! a = theta*qnorm / (vnorm*qnorm) = theta/(sin(theta)*qnorm)
       a = aa_tf_invsinc_series(theta)/qnorm ! approx. 1/qnorm
    else
       a = theta/vnorm
    end if
    r(XYZ_INDEX) = a*q(XYZ_INDEX)
    r(W_INDEX) = log(qnorm) ! for unit quaternion, zero
  end subroutine aa_tf_qln

  !> Compute q**a
  pure subroutine aa_tf_qpow( q, a, r ) &
       bind( C, name="aa_tf_qpow" )
    real(C_DOUBLE), Dimension(4), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE), value, intent(in) :: a
    real(C_DOUBLE) :: qln(4)

    !! r = exp( ln(q) * a )
    call aa_tf_qln( q, qln )
    qln = a*qln
    call aa_tf_qexp(qln, r )

    !! inlined implementation
    ! ! intermediates
    ! vv = dot_product(q(XYZ_INDEX), q(XYZ_INDEX))
    ! vnorm = sqrt(vv)
    ! qnorm = sqrt( vv + q(W_INDEX)**2 )
    ! !theta = acos( q(W_INDEX) / qnorm )
    ! theta = atan2( vnorm, q(W_INDEX) )
    ! pa = qnorm ** a
    ! ! output
    ! r(W_INDEX) = pa * cos(a*theta)
    ! r(XYZ_INDEX) = (pa * sin(a*theta) / vnorm) * q(XYZ_INDEX)
  end subroutine aa_tf_qpow


  Subroutine aa_tf_rotvec2quat( a, q ) &
       bind( C, name="aa_tf_rotvec2quat" )
    real(C_DOUBLE), Dimension(4), intent(out) :: q
    real(C_DOUBLE), Dimension(3), intent(in) :: a
    real(C_DOUBLE) :: t(4)
    ! q = normalize( exp( theta/2 * u ) )
    t(XYZ_INDEX) = a/2d0
    t(W_INDEX) = 0d0
    call aa_tf_qexp( t, q )
    call aa_tf_qnormalize(q) ! not required, but helps stability
    !call aa_tf_qminimize(q)
  end Subroutine aa_tf_rotvec2quat

  Subroutine aa_tf_xangle2quat( theta, q ) &
       bind( C, name="aa_tf_xangle2quat" )
    real(C_DOUBLE), Dimension(4), intent(out) :: q
    real(C_DOUBLE), intent(in), value :: theta
    q(1) = sin(theta/2)
    q(2) = 0d0
    q(3) = 0d0
    q(W_INDEX) = cos(theta/2)
  end Subroutine aa_tf_xangle2quat

  Subroutine aa_tf_yangle2quat( theta, q ) &
       bind( C, name="aa_tf_yangle2quat" )
    real(C_DOUBLE), Dimension(4), intent(out) :: q
    real(C_DOUBLE), intent(in), value :: theta
    q(1) = 0d0
    q(2) = sin(theta/2)
    q(3) = 0d0
    q(W_INDEX) = cos(theta/2)
  end Subroutine aa_tf_yangle2quat

  Subroutine aa_tf_zangle2quat( theta, q ) &
       bind( C, name="aa_tf_zangle2quat" )
    real(C_DOUBLE), Dimension(4), intent(out) :: q
    real(C_DOUBLE), intent(in), value :: theta
    q(1) = 0d0
    q(2) = 0d0
    q(3) = sin(theta/2)
    q(W_INDEX) = cos(theta/2)
  end Subroutine aa_tf_zangle2quat

  Subroutine aa_tf_axang2quat( a, q ) &
       bind( C, name="aa_tf_axang2quat" )
    real(C_DOUBLE), Dimension(4), intent(out) :: q
    real(C_DOUBLE), Dimension(4), intent(in) :: a
    real(C_DOUBLE) :: rv(3)
    rv = a(1:3) * a(4)
    call aa_tf_rotvec2quat(rv, q)
  end Subroutine aa_tf_axang2quat

  Subroutine aa_tf_quat2rotvec( q, rv ) &
       bind( C, name="aa_tf_quat2rotvec" )
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE), Dimension(3), intent(out) :: rv
    real(C_DOUBLE) :: qmin(4),qrv(4)
    !! log method
    call aa_tf_qminimize2(q, qmin)
    call aa_tf_qln( qmin, qrv )
    rv = 2*qrv(1:3) ! qrv(4) will be 0 for unit quaternions
  end Subroutine aa_tf_quat2rotvec

  Subroutine aa_tf_quat2axang( q, a ) &
       bind( C, name="aa_tf_quat2axang" )
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE), Dimension(4), intent(out) :: a
    real(C_DOUBLE) :: rv(3)
    !real(C_DOUBLE) :: e(4)
    ! assume unit quaternion
    !a(4) = 2d0*acos(q(W_INDEX))
    call aa_tf_quat2rotvec( q, rv )
    a(4) = sqrt(dot_product(rv,rv))
    if( 0d0 == a(4) ) then
       a(1:3) = 0d0
    else
       a(1:3) = rv / a(4)
    end if
    ! vnorm = aa_tf_qvnorm(q)
    ! if( abs(vnorm) < epsilon(vnorm) ) then
    !    a = 0d0
    ! else
    !    !! log method
    !    !call aa_tf_qln( q, e )
    !    !a(4) = 2d0 * aa_tf_qvnorm(e)
    !    !a(1:3) = e(XYZ_INDEX) / aa_tf_qvnorm(e)

    !    !! optimized method
    !    a(4) = 2d0 * atan2( vnorm, q(W_INDEX) )
    !    s = sqrt(1-q(4)**2) ! sin(theta/2)
    !    a(1:3) = q(XYZ_INDEX) / s
    ! end if
  end Subroutine aa_tf_quat2axang


  pure subroutine aa_tf_qslerp_param( q1, q2, theta, d1, d2 ) &
       bind( C, name="aa_tf_qslerp_param" )
    real(C_DOUBLE), intent(out) :: theta, d1, d2
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    !theta = abs(aa_la_angle( q1, q2))
    theta = abs(aa_tf_quangle2(q1, q2))
    d1 = sin(theta)
    if( theta > PI_2 ) then
       ! Go the short way
       theta = PI - theta
       d2 = -d1
    else
       d2 = d1
    end if
  end subroutine aa_tf_qslerp_param

  pure subroutine aa_tf_qslerp( tau, q1, q2, r ) &
       bind( C, name="aa_tf_qslerp" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE) :: theta, d1,d2,s1, s2
    call aa_tf_qslerp_param( q1, q2, theta, d1, d2 )
    if( 0d0 == theta ) then
       r = q1
    else
       s1 = sin( theta - tau*theta )
       s2 = sin( tau*theta )
       r = s1/d1 * q1 + s2/d2 * q2
    end if
    call aa_tf_qnormalize(r)
  end subroutine aa_tf_qslerp


  !> Algebraic computation of SLERP
  pure subroutine aa_tf_qslerpalg( tau, q1, q2, r ) &
       bind( C, name="aa_tf_qslerpalg" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE), dimension(4) :: qm, qp
    ! q1 * (conj(q1) * q2) ^ t  *
    call aa_tf_qcmul( q1, q2, qm )
    call aa_tf_qminimize(qm)        ! go the short way
    call aa_tf_qpow( qm, tau, qp )
    call aa_tf_qmul( q1, qp, r )
    call aa_tf_qnormalize(r)
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
    real(C_DOUBLE), dimension(4) :: q21, l21, q23, l23,  qd, qe
    call aa_tf_qcmul( q2, q1, q21 )
    call aa_tf_qln( q21, l21 )
    call aa_tf_qcmul( q2, q3, q23 )
    call aa_tf_qln( q23, l23 )
    qd =  - (l21 + l23) / 4.0
    call aa_tf_qexp( qd, qe )
    call aa_tf_qmul( q2, qe, s2 )
  end subroutine aa_tf_qsquad_param


  pure subroutine aa_tf_qslerpdiff_param( q1, q2, theta, d1, d2 ) &
       bind( C, name="aa_tf_qslerpdiff_param" )
    real(C_DOUBLE), intent(out) :: theta, d1, d2
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    !theta = abs(aa_la_angle( q1, q2))
    theta = abs(aa_tf_quangle2(q1, q2))
    if( theta > PI_2 ) then
       ! Go the short way
       theta = PI - theta
       d1 = aa_tf_sinc(theta)
       d2 = -d1
    else
       d1 = aa_tf_sinc(theta)
       d2 = d1
    end if
  end subroutine aa_tf_qslerpdiff_param

  !! Derivative of a SLERPed quaternion
  !! Note, this is not a time deriviative, but derivative by the slerp parameter tau
  !! Use the chain rule if you need the time derivative (ie, to find a velocity)
  pure subroutine aa_tf_qslerpdiff( tau, q1, q2, r ) &
       bind( C, name="aa_tf_qslerpdiff" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE) :: theta, s1, s2, d1,d2
    call aa_tf_qslerpdiff_param( q1, q2, theta, d1, d2 )
    s1 = cos( theta - tau*theta )
    s2 = cos( tau*theta )
    r = -s1/d1*q1 + s2/d2*q2
  end subroutine aa_tf_qslerpdiff


  !! Derivative of a SLERPed quaternion, computed algebraicly
  pure subroutine aa_tf_qslerpdiffalg( tau, q1, q2, dq ) &
       bind( C, name="aa_tf_qslerpdiffalg" )
    real(C_DOUBLE), dimension(4), intent(out) :: dq
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE), dimension(4) :: qm, ql, q
    ! dq = ln(q2*q1^-1) * slerp(q1,q2,u)
    call aa_tf_qslerpalg( tau, q1, q2, q )
    call aa_tf_qmulc( q2, q1, qm );
    call aa_tf_qminimize(qm)
    call aa_tf_qln( qm, ql )
    call aa_tf_qmul( ql, q, dq )
  end subroutine aa_tf_qslerpdiffalg

  !! Angle between two unit quaternions
  pure function aa_tf_quangle2(x, y) result(theta)
    real(C_DOUBLE), dimension(4), intent(in) :: x, y
    real(C_DOUBLE) :: theta
    real(C_DOUBLE) :: s, c
    real(C_DOUBLE) :: a(4), b(4)
    a = x-y
    b = x+y
    s = aa_tf_qnorm(a)
    c = aa_tf_qnorm(b)
    theta = 2d0 * atan2(s, c)
  end function aa_tf_quangle2


  !! Chain Rule Derivative of a SLERPed quaternion
  !! Assumes that q1 and q2 are unit quaternions
  pure subroutine aa_tf_qslerpchaindiff( u, du, q1, dq1, q2, dq2, q, dq ) &
       bind( C, name="aa_tf_qslerpchaindiff" )
    real(C_DOUBLE), dimension(4), intent(out) :: q, dq
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2, dq1, dq2
    real(C_DOUBLE), value, intent(in) :: u, du
    ! locals
    real(C_DOUBLE) :: theta, dtheta, dtheta_c, a, b, da, db, s, c, sa, sb, ca, cb, d1, d2
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
    call aa_tf_qslerp_param( q1, q2, theta, d1, d2 )
    if( 0 == theta ) then
       dq = 0d0
       q = q1
       return
    end if

    s = sin(theta) ! nonzero
    c = cos(theta)
    sa = sin((1-u)*theta)
    ca = cos((1-u)*theta)
    sb = sin(u*theta)
    cb = cos(u*theta)

    ! find parameters
    ! TODO: what if cos(theta) == 0?
    ! Try using the derivative of the atan2 formula for vector angle
    ! instead.
    dtheta_c = dot_product(q1, dq2) + dot_product(dq1, q2)
    dtheta =  dtheta_c / c

    ! TODO: Taylor series evaluation when sin(theta) -> 0

    a = sa / d1
    b = sb / d2

    da = ( ca * (dtheta*(1-u) - du*theta) ) / d1
    da = da - ( dtheta_c * sa ) / d1**2

    db = ( (dtheta*u + theta*du) * cb ) / d2
    db = db - ( dtheta_c * b ) / d1

    ! get the result
    q = q1*a + q2*b
    dq = (dq1*a + q1*da) + (dq2*b + q2*db)
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
    real(C_DOUBLE), dimension(4) :: qv
    !! v = 2 * dq/dt * conj(q)
    call aa_tf_qmulc(dq_dt, q, qv)
    v = 2*qv(XYZ_INDEX)
  end subroutine aa_tf_qdiff2vel

  subroutine aa_tf_qvel2diff( q, v, dq_dt ) &
       bind( C, name="aa_tf_qvel2diff" )
    real(C_DOUBLE), intent(in) :: v(3), q(4)
    real(C_DOUBLE), intent(out) :: dq_dt(4)
    real(C_DOUBLE) :: v2(3)
    ! dq/dt = 1/2 * v * q
    v2 = v/2
    call aa_tf_qmul_vq( v2, q, dq_dt )
  end subroutine aa_tf_qvel2diff


  subroutine aa_tf_rotvec_qjac( v, G ) &
       bind( C, name="aa_tf_rotvec_qjac" )
    real(C_DOUBLE), dimension(3), intent(in) :: v
    real(C_DOUBLE), dimension(4,3), intent(out) :: G
    real(C_DOUBLE) :: n, s, c, a
    n = sqrt(dot_product(v,v))
    !! TODO: this is more or less the jacobian of the quaternion
    !! exponential.  We could be more general by just using that
    !! jacobian.
    if( 0 == n ) then
       ! limit as v->0
       G(1,1) = 0.5d0
       G(2,1) = 0d0
       G(3,1) = 0d0
       G(4,1) = -v(1)/4

       G(1,2) = 0d0
       G(2,2) = 0.5d0
       G(3,2) = 0d0
       G(4,2) = -v(2)/4

       G(1,3) = 0d0
       G(2,3) = 0d0
       G(3,3) = 0.5d0
       G(4,3) = -v(3)/4
    else
       s = sin(n/2)
       c = cos(n/2)
       a = c*n - 2*s

       G(1,1) = 2*n*n*s + v(1)*v(1)*a
       G(2,1) = v(1)*v(2)*a
       G(3,1) = v(1)*v(3)*a
       G(4,1) = -v(1)*n*n*s

       G(1,2) = v(1)*v(2)*a
       G(2,2) = 2*n*n*s + v(2)*v(2)*a
       G(3,2) = v(2)*v(3)*a
       G(4,2) = -v(2)*n*n*s

       G(1,3) = v(1)*v(3)*a
       G(2,3) = v(2)*v(3)*a
       G(3,3) = 2*n*n*s + v(3)*v(3)*a
       G(4,3) = -v(3)*n*n*s

       G = G / (2*n**3)
    end if

  end subroutine aa_tf_rotvec_qjac

  subroutine aa_tf_rotvec_diff2qdiff( v, dv, dq ) &
       bind( C, name="aa_tf_rotvec_diff2qdiff" )
    real(C_DOUBLE), dimension(4), intent(out) :: dq
    real(C_DOUBLE), dimension(3), intent(in) :: v, dv

    real(C_DOUBLE) :: y(3), dy(3), phi2, phi, sinc, k, ydy

    y = v / 2
    dy = dv / 2
    phi2 = dot_product(y,y)

    if( phi2 < sqrt(epsilon(phi)) ) then
       ! Taylor series approx
       sinc = aa_tf_sinc_series2(phi2)
       k = aa_tf_horner3( phi2, -1d0/3, 1d0/30, -1d0/840 )
    else
       phi = sqrt(phi2)
       sinc = sin(phi)/phi
       k = (cos(phi)-sinc) / phi2
    end if

    ydy = dot_product(y,dy)

    dq(W_INDEX) = -ydy*sinc
    dq(XYZ_INDEX) = sinc*dy + k*ydy*y

    ! real(C_DOUBLE) :: G(4,3)
    ! call aa_tf_rotvec_qjac( v, G )
    ! dq = matmul( G, dv )

  end subroutine aa_tf_rotvec_diff2qdiff

  subroutine aa_tf_rotvec_diff2vel( v, dv, w ) &
       bind( C, name="aa_tf_rotvec_diff2vel" )
    real(C_DOUBLE), dimension(3), intent(out) :: w
    real(C_DOUBLE), dimension(3), intent(in) :: v, dv
    real(C_DOUBLE) :: dq(4), q(4)
    call aa_tf_rotvec_diff2qdiff(v,dv,dq)
    call aa_tf_rotvec2quat(v, q)
    call aa_tf_qdiff2vel( q, dq, w )
  end subroutine aa_tf_rotvec_diff2vel


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

  !! Evaluate a polynomial via Horners rule
  pure function aa_tf_horner( x, a ) result(y)
    real(C_DOUBLE), value :: x
    real(C_DOUBLE), intent(in) :: a(:)
    real(C_DOUBLE) :: y
    integer :: i
    y = a(size(a))
    do i=(1-size(a)),1
       y = a(i) + x*y
    end do
  end function aa_tf_horner

  !! Evaluate a polynomial via Horners rule
  pure function aa_tf_horner3( x, a0, a1, a2 ) result(y)
    real(C_DOUBLE), value :: x, a0, a1, a2
    real(C_DOUBLE) :: y
    y = a2
    y = a1 + x*y
    y = a0 + x*y
  end function aa_tf_horner3

  !! Evaluate a polynomial via Horners rule
  pure function aa_tf_horner4( x, a0, a1, a2, a3) result(y)
    real(C_DOUBLE), value :: x, a0, a1, a2, a3
    real(C_DOUBLE) :: y
    y = a3
    y = a2 + x*y
    y = a1 + x*y
    y = a0 + x*y
  end function aa_tf_horner4

  !! Evaluate a polynomial via Horners rule
  pure function aa_tf_horner5( x, a0, a1, a2, a3, a4) result(y)
    real(C_DOUBLE), value :: x, a0, a1, a2, a3, a4
    real(C_DOUBLE) :: y
    y = a4
    y = a3 + x*y
    y = a2 + x*y
    y = a1 + x*y
    y = a0 + x*y
  end function aa_tf_horner5



  pure function aa_tf_sinc_series2( theta2 ) result(s)
    real(C_DOUBLE), value :: theta2
    real(C_DOUBLE) :: s
    s = aa_tf_horner3( theta2, 1d0, -1d0/6d0, 1d0/120d0)
  end function aa_tf_sinc_series2

  ! sin(theta)/theta
  pure function aa_tf_sinc_series( theta ) result(s)
    real(C_DOUBLE), value :: theta
    real(C_DOUBLE) :: s
    ! Rewrite partial Taylor series using Horners rule.  First
    ! three terms should be accurate within machine precision,
    ! because the third term is less than theta**4 and thus less
    ! than epsilon.
    s = aa_tf_sinc_series2( theta**2 )
  end function aa_tf_sinc_series

  pure function aa_tf_cos_series2( theta2 ) result(s)
    real(C_DOUBLE), value :: theta2
    real(C_DOUBLE) :: s
    s = aa_tf_horner3( theta2, 1d0, -1d0/2d0, 1d0/24d0)
  end function aa_tf_cos_series2

  ! cos(theta) for small theta
  pure function aa_tf_cos_series( theta ) result(s)
    real(C_DOUBLE), value :: theta
    real(C_DOUBLE) :: s
    s = aa_tf_cos_series2(theta)
  end function aa_tf_cos_series


  pure function aa_tf_invsinc_series2( theta2 ) result(s)
    real(C_DOUBLE), value :: theta2
    real(C_DOUBLE) :: s
    s = aa_tf_horner3( theta2, 1d0, 1d0/6d0, 7d0/360d0 )
  end function aa_tf_invsinc_series2

  ! theta/sin(theta)
  pure function aa_tf_invsinc_series( theta ) result(s)
    real(C_DOUBLE), value :: theta
    real(C_DOUBLE) :: s
    s = aa_tf_invsinc_series2( theta**2 )
  end function aa_tf_invsinc_series

  pure function aa_tf_sinc( theta ) result(s)
    real(C_DOUBLE), value :: theta
    real(C_DOUBLE) :: s
    if( 0d0 == theta ) then
       s = 1d0
    elseif ( abs(theta) < sqrt(sqrt(epsilon(theta))) ) then
       s = aa_tf_sinc_series(theta)
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
    real(C_DOUBLE)  :: wq(4), e(4)
    !! Exponential map method
    !! q1 = exp(w*dt/2) * q0
    wq(XYZ_INDEX) = w*(dt/2)
    wq(W_INDEX) = 0d0
    call aa_tf_qexp( wq, e )
    call aa_tf_qmul(e,q0, q1)
  end subroutine aa_tf_qsvel

  !! Integrate rotational velocity
  subroutine aa_tf_qsdiff( q0, dq, dt, q1 ) &
       bind( C, name="aa_tf_qsdiff" )
    real(C_DOUBLE), intent(in) :: dq(4), q0(4)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: q1(4)
    real(C_DOUBLE)  :: w(3)
    call aa_tf_qdiff2vel(q0, dq, w )
    call aa_tf_qsvel( q0, w, dt, q1 );
  end subroutine aa_tf_qsdiff

  subroutine aa_tf_quat_davenport_matrix( n, w, q,  M ) &
       bind( C, name="aa_tf_quat_davenport_matrix" )

    integer(C_SIZE_T), intent(in), value :: n
    real(C_DOUBLE), intent(in) :: w(n), q(4,n)
    real(C_DOUBLE), intent(out) :: M(4,4)
    real(C_DOUBLE) :: tmp(4,4),  p(4,1)

    integer(C_SIZE_T) :: i
    ! see: F. Landis Markley, et. al. "Averaging Quaternions"
    M = real(0.0,C_DOUBLE)
    do i=1,n
       p(:,1) = q(:,i)
       tmp = matmul(p, transpose(p))
       M = M + w(i) * tmp
    end do
  end subroutine aa_tf_quat_davenport_matrix

  !!! Dual numbers

  elemental subroutine aa_tf_dual_pack2( r, d, c )
    real(C_DOUBLE), intent(in)  :: r,d
    type(aa_tf_dual_t), intent(out) :: c
    c = aa_tf_dual_t(r,d)
  end subroutine aa_tf_dual_pack2

  pure subroutine aa_tf_dual_pack1( x, c )
    real(C_DOUBLE), intent(in) :: x(:)
    type(aa_tf_dual_t), intent(out) :: c(:)
    integer :: n
    n = size(x)/2
    call aa_tf_dual_pack2( x(1:n), x(n+1:n+n), c(1:n) )
  end subroutine aa_tf_dual_pack1

  elemental function aa_tf_dual_r( a ) result (r)
    type(aa_tf_dual_t), intent(in) :: a
    real(C_DOUBLE) :: r
    r = a%r
  end function aa_tf_dual_r

  elemental function aa_tf_dual_d( a ) result (d)
    type(aa_tf_dual_t), intent(in) :: a
    real(C_DOUBLE) :: d
    d = a%d
  end function aa_tf_dual_d

  pure subroutine aa_tf_dual_unpack( c, x )
    real(C_DOUBLE), intent(out) :: x(:)
    type(aa_tf_dual_t), intent(in) :: c(:)
    integer :: n
    n = size(c)
    x(1:n) = aa_tf_dual_r(c)
    x(n+1:n+n) = aa_tf_dual_d(c)
  end subroutine aa_tf_dual_unpack


  elemental subroutine aa_tf_dual_mul4( ar, ad, br, bd, cr, cd )
    real(C_DOUBLE), intent(in) :: ar, ad, br, bd
    real(C_DOUBLE), intent(out) :: cr, cd
    cr = ar*br
    cd = ar*bd + ad*br
  end subroutine aa_tf_dual_mul4

  elemental subroutine aa_tf_dual_scal4( ar, ad, br, bd )
    real(C_DOUBLE), intent(in) :: ar, ad
    real(C_DOUBLE), intent(inout) :: br, bd
    real(C_DOUBLE) :: cr, cd
    call aa_tf_dual_mul4( ar, ad, br, bd, cr, cd )
    br = cr
    bd = cd
  end subroutine aa_tf_dual_scal4

  elemental function aa_tf_dual_mul( a, b ) result (c)
    type(aa_tf_dual_t), intent(in) :: a,b
    type(aa_tf_dual_t) :: c
    real(C_DOUBLE) :: cr, cd
    call aa_tf_dual_mul4( a%r, a%d, b%r, b%d, cr, cd )
    c =  aa_tf_dual_t( cr, cd )
  end function aa_tf_dual_mul

  subroutine aa_tf_dual_scalv( ar, ad, x )
    real(C_DOUBLE), intent(in) :: ar, ad
    real(C_DOUBLE), intent(inout) :: x(:)
    integer :: n
    n = size(x)/2
    call aa_tf_dual_scal4( ar, ad, x(1:n), x(n+1:n+n))
  end subroutine aa_tf_dual_scalv

  elemental function aa_tf_dual_add( a, b ) result (c)
    type(aa_tf_dual_t), intent(in) :: a,b
    type(aa_tf_dual_t) :: c
    c =  aa_tf_dual_t( a%r+b%r, a%d+b%d )
  end function aa_tf_dual_add

  elemental function aa_tf_dual_sub( a, b ) result (c)
    type(aa_tf_dual_t), intent(in) :: a,b
    type(aa_tf_dual_t) :: c
    c =  aa_tf_dual_t( a%r-b%r, a%d-b%d )
  end function aa_tf_dual_sub

  elemental function aa_tf_dual_div( a, b ) result (c)
    type(aa_tf_dual_t), intent(in) :: a,b
    type(aa_tf_dual_t) :: c
    c =  aa_tf_dual_t( a%r / b%r, (a%d*b%r - a%r*b%d) / b%r**2 )
  end function aa_tf_dual_div

  elemental function aa_tf_dual_sin( x ) result (y)
    type(aa_tf_dual_t), intent(in) :: x
    type(aa_tf_dual_t) :: y
    y = aa_tf_dual_t( sin(x%r), cos(x%r)*x%d )
  end function aa_tf_dual_sin

  elemental function aa_tf_dual_cos( x ) result (y)
    type(aa_tf_dual_t), intent(in) :: x
    type(aa_tf_dual_t) :: y
    y = aa_tf_dual_t( cos(x%r), -sin(x%r)*x%d )
  end function aa_tf_dual_cos

  elemental function aa_tf_dual_acos( x ) result (y)
    type(aa_tf_dual_t), intent(in) :: x
    type(aa_tf_dual_t) :: y
    y = aa_tf_dual_t( acos(x%r), - x%d / sqrt(1 - x%r**2) )
  end function aa_tf_dual_acos

  elemental function aa_tf_dual_atan2( a, b ) result (y)
    type(aa_tf_dual_t), intent(in) :: a, b
    type(aa_tf_dual_t) :: y
    y%r = atan2(a%r, b%r)
    y%d = (b%r*a%d - a%r*b%d) / (a%r**2 + b%r**2)
  end function aa_tf_dual_atan2

  elemental function aa_tf_dual_exp( x ) result (y)
    type(aa_tf_dual_t), intent(in) :: x
    type(aa_tf_dual_t) :: y
    y = aa_tf_dual_t( exp(x%r), exp(x%r)*x%d )
  end function aa_tf_dual_exp

  elemental function aa_tf_dual_log( x ) result (y)
    type(aa_tf_dual_t), intent(in) :: x
    type(aa_tf_dual_t) :: y
    y = aa_tf_dual_t( log(x%r), x%d / x%r )
  end function aa_tf_dual_log


  elemental function aa_tf_dual_sqrt( x ) result (y)
    type(aa_tf_dual_t), intent(in) :: x
    type(aa_tf_dual_t) :: y
    y = aa_tf_dual_t( sqrt(x%r), x%d / (2d0*sqrt(x%r)) )
  end function aa_tf_dual_sqrt

  !!! Dual Quaternions
  !!!
  !!! Stored as 8 doubles
  !!! - First four are "real" (rotation)
  !!! - Second four are "dual" (translation)

  ! pure function aa_tf_duqu_dual_w( d ) result(y)
  !   real(C_DOUBLE), intent(in) :: d(8)
  !   complex(C_DOUBLE) :: y
  !   y = aa_tf_dual( d(DQ_REAL_W), d(DQ_DUAL_W) )
  ! end function aa_tf_duqu_dual_w

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
    d = d / aa_tf_qnorm( d(DQ_REAL) )
  end subroutine aa_tf_duqu_normalize

  !> Dual quaternion angle minimization
  subroutine aa_tf_duqu_minimize( d ) &
       bind( C, name="aa_tf_duqu_minimize" )
    real(C_DOUBLE), intent(inout), dimension(8) :: d
    if ( d(DQ_REAL_W) < 0 ) then
       d = -d
    end if
  end subroutine aa_tf_duqu_minimize

  !> Dual quaternion norm
  subroutine aa_tf_duqu_norm( d, nreal, ndual ) &
       bind( C, name="aa_tf_duqu_norm" )
    real(C_DOUBLE), intent(in), dimension(8) :: d
    real(C_DOUBLE), intent(out) :: nreal, ndual
    nreal = aa_tf_qnorm( d(DQ_REAL) )
    ndual = dot_product( d(DQ_REAL), d(DQ_DUAL) ) / nreal
  end subroutine aa_tf_duqu_norm

  !> Dual quaternion norm of vector component
  subroutine aa_tf_duqu_vnorm( d, nreal, ndual ) &
       bind( C, name="aa_tf_duqu_vnorm" )
    real(C_DOUBLE), intent(in), dimension(8) :: d
    real(C_DOUBLE), intent(out) :: nreal, ndual
    nreal = aa_tf_qvnorm( d(DQ_REAL) )
    ndual = dot_product( d(DQ_REAL_XYZ), d(DQ_DUAL_XYZ) ) / nreal
  end subroutine aa_tf_duqu_vnorm

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

  subroutine aa_tf_xyz2duqu( x, y, z, d ) &
       bind( C, name="aa_tf_xyz2duqu" )
    real(C_DOUBLE), intent(in), value :: x,y,z
    real(C_DOUBLE), intent(out), dimension(8) :: d
    d(DQ_REAL_XYZ) = real(0.0,C_DOUBLE)
    d(DQ_REAL_W) = real(1.0,C_DOUBLE)
    d(DQ_DUAL_W) = real(0.0,C_DOUBLE)
    d(4+1) = 0.5 * x
    d(4+2) = 0.5 * y
    d(4+3) = 0.5 * z
  end subroutine aa_tf_xyz2duqu

  ! x angle and translations to dual quaternion
  subroutine aa_tf_xxyz2duqu( theta, x, y, z, d ) &
       bind( C, name="aa_tf_xxyz2duqu" )
    real(C_DOUBLE), intent(in), value :: theta, x, y, z
    real(C_DOUBLE), intent(out), dimension(8) :: d
    real(C_DOUBLE) :: q(4), v(3)
    call aa_tf_xangle2quat(theta,q)
    v(1) = x
    v(2) = y
    v(3) = z
    call aa_tf_qv2duqu(q,v,d)
  end subroutine aa_tf_xxyz2duqu

  ! y angle and translations to dual quaternion
  subroutine aa_tf_yxyz2duqu( theta, x, y, z, d ) &
       bind( C, name="aa_tf_yxyz2duqu" )
    real(C_DOUBLE), intent(in), value :: theta, x, y, z
    real(C_DOUBLE), intent(out), dimension(8) :: d
    real(C_DOUBLE) :: q(4), v(3)
    call aa_tf_yangle2quat(theta,q)
    v(1) = x
    v(2) = y
    v(3) = z
    call aa_tf_qv2duqu(q,v,d)
  end subroutine aa_tf_yxyz2duqu

  ! z angle and translations to dual quaternion
  subroutine aa_tf_zxyz2duqu( theta, x, y, z, d ) &
       bind( C, name="aa_tf_zxyz2duqu" )
    real(C_DOUBLE), intent(in), value :: theta, x, y, z
    real(C_DOUBLE), intent(out), dimension(8) :: d
    real(C_DOUBLE) :: q(4), v(3)
    call aa_tf_zangle2quat(theta,q)
    v(1) = x
    v(2) = y
    v(3) = z
    call aa_tf_qv2duqu(q,v,d)
  end subroutine aa_tf_zxyz2duqu

  subroutine aa_tf_duqu2qv( d, q, v ) &
       bind( C, name="aa_tf_duqu2qv" )
    real(C_DOUBLE), intent(out), dimension(3) :: q(4), v(3)
    real(C_DOUBLE), intent(in), dimension(8) :: d
    q = d(DQ_REAL)
    call aa_tf_duqu_trans( d, v )
  end subroutine aa_tf_duqu2qv

  !> Dual quaternion construction from unit quaternion and translation
  !> vector, with minimized angle
  subroutine aa_tf_qv2duqu_min( q, v, d ) &
       bind( C, name="aa_tf_qv2duqu_min" )
    real(C_DOUBLE), intent(in), dimension(3) :: q(4), v(3)
    real(C_DOUBLE), intent(out), dimension(8) :: d
    real(C_DOUBLE) :: qmin(4)
    call aa_tf_qminimize2(q, qmin)
    call aa_tf_qv2duqu(qmin, v, d)
  end subroutine aa_tf_qv2duqu_min

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

  subroutine aa_tf_tfmat2duqu( t, d ) &
       bind( C, name="aa_tf_tfmat2duqu" )
    real(C_DOUBLE), intent(out), dimension(8) :: d
    real(C_DOUBLE), intent(in), dimension(3,4) :: t
    real(C_DOUBLE) :: q(4)
    call aa_tf_rotmat2quat( t(1:3,1:3), q )
    call aa_tf_qv2duqu(q, t(1:3,4), d)
  end subroutine aa_tf_tfmat2duqu

  !> Transform a point using the dual quaternion
  subroutine aa_tf_tf_duqu( d, p0, p1 ) &
       bind( C, name="aa_tf_tf_duqu" )
    real(C_DOUBLE), intent(in) :: d(8), p0(3)
    real(C_DOUBLE), intent(out) :: p1(3)
    real(C_DOUBLE) :: a(4), b(4)
    ! p1 = d * p * d^{-1}
    ! p1 = (2*dual + real*p0) * conj(real)
    call aa_tf_qmul_qv( d(DQ_REAL), p0, a )
    a =  a + d(DQ_DUAL) + d(DQ_DUAL)
    call aa_tf_qmulc( a, d(DQ_REAL), b )
    p1 = b(XYZ_INDEX)

    ! real(C_DOUBLE) :: rx,ry,rz,rw, dx,dy,dz,dw, vx,vy,vz, rxx,ryy,rzz,rww, rvx, rvy, rvz

    ! rx=d(1)
    ! ry=d(2)
    ! rz=d(3)
    ! rw=d(4)

    ! dx=d(5)
    ! dy=d(6)
    ! dz=d(7)
    ! dw=d(8)

    ! vx=p0(1)
    ! vy=p0(2)
    ! vz=p0(3)

    ! ! real(C_DOUBLE) :: ax(3), aw

    ! ! ! ax = real_w p + real_x x p + 2 dual_x
    ! ! call aa_tf_cross( d(DQ_REAL_XYZ), p0, ax )
    ! ! ax = ax + d(DQ_REAL_W)*p0 + 2*d(DQ_DUAL_XYZ)

    ! ! ! aw = dot( real_x, p ) - 2 dual_w
    ! ! aw = dot_product( d(DQ_REAL_XYZ), p0 ) - 2*d(DQ_DUAL_W)

    ! ! ! p1 = real_x x ax + real_w ax + aw real_x
    ! ! call aa_tf_cross( d(DQ_REAL_XYZ), ax, p1 )
    ! ! p1 = p1 + d(DQ_REAL_W)*ax + aw*d(DQ_REAL_XYZ)

    ! rxx = rx*rx
    ! ryy = ry*ry
    ! rzz = rz*rz
    ! rww = rw*rw

    ! rvx = rx*vx
    ! rvy = ry*vy
    ! rvz = rz*vz

    ! p1(1) = rx*(rvz + rvy) + rw*(ry*vz - rz*vy) + rw*dx - dw*rx + dz*ry - dy*rz
    ! p1(2) = ry*(rvx + rvz) + rw*(rz*vx - rx*vz) + rw*dy - dw*ry + dx*rz - dz*rx
    ! p1(3) = rz*(rvy + rvx) + rw*(rx*vy - ry*vx) + rw*dz - dw*rz + dy*rx - dx*ry

    ! p1(1) = p1(1) + p1(1) + vx*(rww - rzz + rxx - ryy)
    ! p1(2) = p1(2) + p1(2) + vy*(rww - rxx + ryy - rzz)
    ! p1(3) = p1(3) + p1(3) + vz*(rww - ryy + rzz - rxx)
  end subroutine aa_tf_tf_duqu


  !! Convert spatial velocity to quaternion derivative
  ! subroutine aa_tf_duqu_vel2diff( d, dx, dd ) &
  !      bind( C, name="aa_tf_duqu_vel2diff" )
  !   real(C_DOUBLE), intent(in) :: d(8), dx(6)
  !   real(C_DOUBLE), intent(out) :: dd(8)
  !   real(C_DOUBLE) :: a(4), b(4), c(4)
  !   ! orientation
  !   call aa_tf_qvel2diff( d(DQ_REAL), dx(4:6), dd(DQ_REAL) )
  !   ! translation
  !   ! dd_dual = (dx*d_real + x*dd_real) / 2
  !   ! dd_dual = dx*r/2 + d*r_conj*dr)
  !   call aa_tf_vqmul( dx(1:3), d(DQ_REAL), a )
  !   call aa_tf_qmulc( d(DQ_DUAL), d(DQ_REAL), b)
  !   call aa_tf_qmul( b, dd(DQ_REAL), c )
  !   dd(DQ_DUAL) = a/2d0 + c
  ! end subroutine aa_tf_duqu_vel2diff


  subroutine aa_tf_duqu_vel2twist( d, dx, t ) &
       bind( C, name="aa_tf_duqu_vel2twist" )
    real(C_DOUBLE), intent(in) :: d(8), dx(6)
    real(C_DOUBLE), intent(out) :: t(8)
    real(C_DOUBLE) :: p(3)
    ! t = omega + \eps * ( cross(x,omega) + dx )
    t(DQ_REAL_XYZ) = dx(4:6)
    t(DQ_REAL_W) = 0d0
    call aa_tf_duqu_trans( d, p )
    call aa_tf_cross( p, dx(4:6), t(DQ_DUAL_XYZ) )
    t(DQ_DUAL_XYZ) = t(DQ_DUAL_XYZ) + dx(1:3)
    t(DQ_DUAL_W) = 0d0
  end subroutine aa_tf_duqu_vel2twist


  subroutine aa_tf_duqu_twist2vel( d, t, dx ) &
       bind( C, name="aa_tf_duqu_twist2vel" )
    real(C_DOUBLE), intent(in) :: d(8), t(8)
    real(C_DOUBLE), intent(out) :: dx(6)
    real(C_DOUBLE) :: p(3), pxw(3)
    ! dx = Omega - [0, p x omega ]
    dx(4:6) = t(1:3)
    call aa_tf_duqu_trans( d, p )
    call aa_tf_cross( p, t(1:3), pxw )
    dx(1:3) = t(5:7) - pxw
  end subroutine aa_tf_duqu_twist2vel

  subroutine aa_tf_duqu_vel2diff( d, dx, dd ) &
       bind( C, name="aa_tf_duqu_vel2diff" )
    real(C_DOUBLE), intent(in) :: d(8), dx(6)
    real(C_DOUBLE), intent(out) :: dd(8)
    real(C_DOUBLE), dimension(8) :: t

    ! dd = twist * d / 2
    call aa_tf_duqu_vel2twist( d, dx, t )
    call aa_tf_duqu_mul( t, d, dd )
    dd = dd / 2d0

    ! ! orientation
    ! call aa_tf_qvel2diff( d(DQ_REAL), dx(4:6), dd(DQ_REAL) )
    ! ! translation
    ! ! dd_dual = (dx*d_real + x*dd_real) / 2
    ! ! dd_dual = dx*r/2 + d*r_conj*dr)
    ! call aa_tf_vqmul( dx(1:3), d(DQ_REAL), a )
    ! call aa_tf_qmulc( d(DQ_DUAL), d(DQ_REAL), b)
    ! call aa_tf_qmul( b, dd(DQ_REAL), c )
    ! dd(DQ_DUAL) = a/2d0 + c
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

  !! Exponential of a dual quaternion
  subroutine aa_tf_duqu_exp( d, e ) &
       bind( C, name="aa_tf_duqu_exp" )
    real(C_DOUBLE), intent(in) :: d(8)
    real(C_DOUBLE), intent(out) :: e(8)
    real(C_DOUBLE) :: nr, c, vv, vd, ar, ad
    type(aa_tf_dual_t) :: expw
    vv = dot_product(d(DQ_REAL_XYZ), d(DQ_REAL_XYZ))
    if( vv < sqrt(epsilon(vv)) ) then
       ar = aa_tf_sinc_series2(vv) ! approx. 1
       c = aa_tf_cos_series2(vv)   ! approx. 1
       ! Taylor series for cos(nr)/nr**2 - sin(nr)/nr**3
       ad = aa_tf_horner3( vv, -1d0/3d0, 1d0/30d0, -1d0/840 )
    else
       nr = sqrt(vv)
       ar = sin(nr)/nr
       c = cos(nr)
       ad = (c-ar)/vv
    end if
    vd = dot_product(d(DQ_REAL_XYZ), d(DQ_DUAL_XYZ))
    ad = vd*ad
    e(DQ_REAL_XYZ) = ar * d(DQ_REAL_XYZ)
    e(DQ_REAL_W) = c
    e(DQ_DUAL_XYZ) = (ar * d(DQ_DUAL_XYZ)) + (ad * (d(DQ_REAL_XYZ)))
    e(DQ_DUAL_W) = -ar * vd

    ! impure part
    if ( 0d0 /= d(DQ_REAL_W) .or. 0d0 /= d(DQ_DUAL_W) ) then
       expw = exp( aa_tf_dual_t( d(DQ_REAL_W), d(DQ_DUAL_W) ) )
       call aa_tf_dual_scalv( expw%r, expw%d, e )
    end if
  end subroutine aa_tf_duqu_exp

  subroutine aa_tf_duqu_ln( d, e ) &
       bind( C, name="aa_tf_duqu_ln" )
    real(C_DOUBLE), intent(in) :: d(8)
    real(C_DOUBLE), intent(out) :: e(8)
    real(C_DOUBLE) :: vv, vd, theta, nr, mr2, mr, ar, ad
    !! Norms
    vv = dot_product(d(DQ_REAL_XYZ), d(DQ_REAL_XYZ))
    vd = dot_product(d(DQ_REAL_XYZ), d(DQ_DUAL_XYZ))
    mr2 = vv+d(DQ_REAL_W)**2
    mr = sqrt( mr2 )

    !! Scalar part
    !e_w = log(m)
    e(DQ_REAL_W) = log(mr)
    e(DQ_DUAL_W) = (vd + d(DQ_REAL_W)*d(DQ_DUAL_W))/mr2

    !! Vector part
    ! Dual number computation
    ! call aa_tf_duqu_vnorm( d, nv%r, nv%d )
    ! a = atan2( nv, dh(W_INDEX) ) / nv

    ! expanded dual computation
    nr = sqrt(vv)                   ! nr is positive
    theta = atan2( nr, d(W_INDEX) ) ! theta is always positive

    ! Try to avoid small number division
    if( theta < sqrt(sqrt(epsilon(theta))) ) then
       ! ad = 1/mr * 1d0/sin(x)**2 * ( cos(x) - x/sin(x) )
       ad = aa_tf_horner3( theta**2, -2d0/3d0, -1d0/5d0, -17d0/420d0 ) / mr
       ar = aa_tf_invsinc_series2(theta**2)/mr
    else
       ar = theta/nr
       ad =  (d(W_INDEX) - ar*mr2) / vv
    end if
    ad = (vd*ad - d(DQ_DUAL_W)) / mr2
    e(DQ_REAL_XYZ) = ar * d(DQ_REAL_XYZ)
    e(DQ_DUAL_XYZ) = ad * d(DQ_REAL_XYZ) + ar * d(DQ_DUAL_XYZ)
  end subroutine aa_tf_duqu_ln

  !! Integrate twist to to get dual quaternion
  subroutine aa_tf_duqu_stwist( d0, twist, dt, d1 ) &
       bind( C, name="aa_tf_duqu_stwist" )
    real(C_DOUBLE), intent(in) :: twist(8), d0(8)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: d1(8)
    real(C_DOUBLE) :: twist1(8), etwist(8)
    twist1 = (dt/2d0)*twist
    call aa_tf_duqu_exp( twist1, etwist )
    call aa_tf_duqu_mul( etwist, d0, d1 )
  end subroutine aa_tf_duqu_stwist

  !! Integrate spatial velocity to to get dual quaternion
  subroutine aa_tf_duqu_svel( d0, dx, dt, d1 ) &
       bind( C, name="aa_tf_duqu_svel" )
    real(C_DOUBLE), intent(in) :: dx(6), d0(8)
    real(C_DOUBLE), intent(in), value :: dt
    real(C_DOUBLE), intent(out) :: d1(8)
    real(C_DOUBLE) :: twist(8)
    call aa_tf_duqu_vel2twist( d0, dx, twist )
    call aa_tf_duqu_stwist( d0, twist, dt, d1 );
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
#include "tf/qv.f90"

end module amino_tf
