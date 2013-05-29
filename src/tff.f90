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
         T1(:,1:3), T1(:,4), &
         T2(:,1:3), T2(:,4), &
         T3(:,1:3), T3(:,4) )
  end subroutine aa_tf_12chain

  pure subroutine aa_tf_9mul( R1, R2, R3 ) &
       bind( C, name="aa_tf_9mul" )
    real(C_DOUBLE), intent(in)  :: R1(3,3)
    real(C_DOUBLE), intent(in)  :: R2(3,3)
    real(C_DOUBLE), intent(out) :: R3(3,3)

    R3 = matmul(R1,R2)
  end subroutine aa_tf_9mul


  subroutine aa_tf_qslerp( t, q1, q2, r ) &
       bind( C, name="aa_tf_qslerp" )
    real(C_DOUBLE), dimension(4), intent(out) :: r
    real(C_DOUBLE), dimension(4), intent(in) :: q1, q2
    real(C_DOUBLE), value, intent(in) :: t
    real(C_DOUBLE) :: theta, d, s1, s2
    if( 0 >= t ) then
       r = q1
    elseif ( 1 <= t ) then
       r = q2
    else
       theta = aa_la_angle( q1, q2 )
       if( 0 == theta ) then
          r = q1
       else
          d = 1d0 / sin(theta)
          s1 = sin( (1 - t) * theta )
          s2 = sin( t * theta )
          if( dot_product(q1,q2) < 0.0 ) then
             r = (s1*q1 - s2*q2) * d
          else
             r = (s1*q1 + s2*q2) * d
          end if
       end if
    end if
  end subroutine aa_tf_qslerp


end module amino_tf
