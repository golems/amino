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

  pure Subroutine aa_tf_qinv( q, r ) &
    bind( C, name="aa_tf_qinv" )
    Real(C_DOUBLE), Dimension(4), intent(out) :: r
    Real(C_DOUBLE), Dimension(4), intent(in) :: q
    Call aa_tf_qconj( q, r )
    r = r / dot_product(q,q)
  End Subroutine aa_tf_qinv

  pure Subroutine aa_tf_qrot( q, v, r ) &
       bind( C, name="aa_tf_qrot" )
    real(C_DOUBLE), Dimension(3), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE), Dimension(3), intent(in) :: v
    real(C_DOUBLE), Dimension(4) :: qv, qr1, qr2, qi
    qv(W_INDEX) = 0d0
    qv(XYZ_INDEX) = v
    Call aa_tf_qinv( q, qi )
    call aa_tf_qmul( q, qv, qr1 )
    call aa_tf_qmul( qr1, qi, qr2 )
    r = qr2(1:3)
  End Subroutine aa_tf_qrot


  pure Subroutine aa_tf_qexp( q, r ) &
       bind( C, name="aa_tf_qexp" )
    real(C_DOUBLE), Dimension(4), intent(out) :: r
    real(C_DOUBLE), Dimension(4), intent(in) :: q
    real(C_DOUBLE) :: vnorm,ew
    vnorm = aa_la_norm2( q(XYZ_INDEX) )
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


  !! Derivative of a SLERP'ed quaternion, computed algebraicly
  pure subroutine aa_tf_qslerpdiffalg( tau, q1, q2, dq ) &
       bind( C, name="aa_tf_qslerpdiffalg" )
    real(C_DOUBLE), dimension(4), intent(out) :: dq
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: tau
    real(C_DOUBLE), dimension(4) :: q1i, qm, ql, q
    if( 0.0 >= tau .or. 1.0 <= tau ) then
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


  !! Chain Rule Derivative of a SLERP'ed quaternion
  !! Assumes that q1 and q2 are unit quaternions
  pure subroutine aa_tf_qslerpchaindiff( u, du, q1, dq1, q2, dq2, q, dq ) &
       bind( C, name="aa_tf_qslerpchaindiff" )
    real(C_DOUBLE), dimension(4), intent(out) :: q, dq
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2, dq1, dq2
    real(C_DOUBLE), value, intent(in) :: u, du
    ! locals
    real(C_DOUBLE) :: q1q2, theta, dtheta, a, b, d1, d2, da, db, s, c, sa, sb, ca, cb
    ! check interpolation bounds
    if( 0.0 >= u ) then
       dq = 0d0
       q = q1
       return
    elseif ( 1.0 <= u ) then
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
    sa = sin(1-u*theta)
    ca = cos(1-u*theta)
    sb = sin(u*theta)
    cb = cos(u*theta)
    a = sa / s
    b = sb / s
    d1 = (theta*du + u*dtheta) / s
    d2 = dtheta*c / s**2
    da = d2*sa - d1*ca
    db = - d2*sb + d1*cb
    ! get the result
    if( q1q2 >= 0.0 ) then
       q = q1*a + q2*b
       dq = (dq1*a + q1*da) + (dq2*b + q2*db)
    else
       q = q1*a - q2*b
       dq = (dq1*a + q1*da) - (dq2*b + q2*db)
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
    real(C_DOUBLE), dimension(4) :: qv
    qv(W_INDEX) = 0d0
    qv(XYZ_INDEX) = 0.5 * v
    call aa_tf_qmul(qv, q, dq_dt)
  end subroutine aa_tf_qvel2diff

end module amino_tf
