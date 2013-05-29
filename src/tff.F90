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

#define R_INDEX  :,1:3
#define T_INDEX  :,4

module amino_tf
  use ISO_C_BINDING
  use amino_la
  implicit none


contains
  pure subroutine aa_tf_9( R1, p1, p ) &
       bind( C, name="aa_tf_9" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: p1(3)
    real(C_DOUBLE), intent(out) :: p(3)

    p = matmul(R1,p1)
  end subroutine aa_tf_9

  pure subroutine aa_tf_93( R1, v1, p1, p) &
       bind( C, name="aa_tf_93" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: v1(3)
    real(C_DOUBLE), intent(in)  :: p1(3)
    real(C_DOUBLE), intent(out) :: p(3)

    p = matmul(R1,p1) + v1
  end subroutine aa_tf_93

  pure subroutine aa_tf_12( T, p1, p) &
       bind( C, name="aa_tf_12" )
    real(C_DOUBLE), intent(in)  :: T(3,4)
    real(C_DOUBLE), intent(in)  :: p1(3)
    real(C_DOUBLE), intent(out) :: p(3)

    call aa_tf_93( T(:,1:3), T(:,4), p1, p )
  end subroutine aa_tf_12

  pure subroutine aa_tf_93chain( R1, v1, R2, v2, R3, v3 ) &
       bind( C, name="aa_tf_93chain" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: v1(3)
    real(C_DOUBLE), intent(in)  :: R2(3,3)
    real(C_DOUBLE), intent(in)  :: v2(3)
    real(C_DOUBLE), intent(out) :: R3(3,3)
    real(C_DOUBLE), intent(out) :: v3(3)

    R3 = matmul(R1,R2)
    v3 = matmul(R1,v2) + v1
  end subroutine aa_tf_93chain

  pure subroutine aa_tf_12chain( T1, T2, T3 ) &
       bind( C, name="aa_tf_12chain" )
    real(C_DOUBLE), intent(in)  :: T1(3,4)
    real(C_DOUBLE), intent(in)  :: T2(3,4)
    real(C_DOUBLE), intent(out) :: T3(3,4)
    call aa_tf_93chain( &
         T1(R_INDEX), T1(T_INDEX), &
         T2(R_INDEX), T2(T_INDEX), &
         T3(R_INDEX), T3(T_INDEX) )
  end subroutine aa_tf_12chain

  pure subroutine aa_tf_9mul( R1, R2, R3 ) &
       bind( C, name="aa_tf_9mul" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: R2(3,3)
    real(C_DOUBLE), intent(out) :: R3(3,3)

    R3 = matmul(R1,R2)
  end subroutine aa_tf_9mul

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
    real(C_DOUBLE) :: x(3)
    q(W_INDEX) = a(W_INDEX)*b(W_INDEX) - &
         dot_product(a(XYZ_INDEX),b(XYZ_INDEX))
    call aa_la_cross_sub(a(XYZ_INDEX), b(XYZ_INDEX), x)
    q(XYZ_INDEX) = a(W_INDEX)*b(XYZ_INDEX) + &
         a(XYZ_INDEX)*b(W_INDEX) + &
         x
  end subroutine aa_tf_qmul

  pure subroutine aa_tf_qslerp( tau, q1, q2, r ) &
       bind( C, name="aa_tf_qslerp" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE) :: theta, d, s1, s2
    if( 0 >= tau ) then
       r = q1
    elseif ( 1 <= tau ) then
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

  !! Derivative of a SLERP'ed quaternion
  !! Note, this is not a time deriviative, but derivative by the slerp parameter tau
  !! Use the chain rule if you need the time derivative (ie, to find a velocity)
  pure subroutine aa_tf_qslerpdiff( tau, q1, q2, r ) &
       bind( C, name="aa_tf_qslerpdiff" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE) :: theta, s1, s2, d
    if( 0.0 >= tau .or. 1.0 <= tau ) then
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
    real(C_DOUBLE), dimension(4) :: qv
    qv(W_INDEX) = 0d0
    qv(XYZ_INDEX) = 0.5 * v
    call aa_tf_qmul(qv, q, dq_dt)
  end subroutine aa_tf_qvel2diff

end module amino_tf
