!! -*- mode: F90; -*-
!!
!! Copyright (c) 2013, Georgia Tech Research Corporation
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


#define QUTR_Q 1:4
#define QUTR_V 5:7

!!!!!!!!!
!! OPS !!
!!!!!!!!!


subroutine aa_tf_qutr_wavg( n, w, EE, ldee,  a ) &
     bind( C, name="aa_tf_qutr_wavg" )
  integer(C_SIZE_T), intent(in), value :: n, ldee
  real(C_DOUBLE), intent(in) :: EE(ldee,n), w(n)
  real(C_DOUBLE), intent(out) :: a(7)
  integer(C_SIZE_T) :: i
  !! average translation, davenport q method
  call aa_tf_quat_davenport( n, w, EE, ldee, a(1:4) )
  !! TODO: Should we consider coupling between orientation and translation?
  !! TODO: Kahan summation
  a(5:7)=real(0,C_DOUBLE)
  do i=1,n
     a(5:7) = a(5:7) + w(i) * EE(5:7,i)
  end do
end subroutine aa_tf_qutr_wavg

!!!!!!!!!!!!!!
!! CALCULUS !!
!!!!!!!!!!!!!!

!! Integrate velocity
subroutine aa_tf_qutr_svel( e0, dx, dt, e1 ) &
     bind( C, name="aa_tf_qutr_svel" )
  real(C_DOUBLE), intent(in) :: dx(6), e0(7)
  real(C_DOUBLE), intent(in), value :: dt
  real(C_DOUBLE), intent(out) :: e1(7)
  real(C_DOUBLE)  :: S0(8), S1(8)
  call aa_tf_qutr2duqu( e0, S0 )
  call aa_tf_duqu_svel( S0, dx, dt, S1 )
  call aa_tf_duqu2qutr( S1, e1 )
end subroutine aa_tf_qutr_svel


subroutine aa_tf_qutr_diff2vel( e, de, dx ) &
     bind( C, name="aa_tf_qutr_diff2vel" )
  real(C_DOUBLE), intent(in) :: de(7), e(7)
  real(C_DOUBLE), intent(out) :: dx(6)
  call aa_tf_qdiff2vel( e(QUTR_Q), de(QUTR_Q), dx(4:6) )
  dx(1:3) = de(QUTR_V)
end subroutine aa_tf_qutr_diff2vel


subroutine aa_tf_qutr_vel2diff( e, dx, de ) &
     bind( C, name="aa_tf_qutr_vel2diff" )
  real(C_DOUBLE), intent(in) :: e(7), dx(6)
  real(C_DOUBLE), intent(out) :: de(7)
  call aa_tf_qvel2diff( e(QUTR_Q), dx(4:6), de(QUTR_Q) )
  de(QUTR_V) =  dx(1:3)
end subroutine aa_tf_qutr_vel2diff

subroutine aa_tf_qutr_sdiff( e0, de, dt, e1 ) &
     bind( C, name="aa_tf_qutr_sdiff" )
  real(C_DOUBLE), intent(in) :: de(7), e0(7)
  real(C_DOUBLE), intent(in), value :: dt
  real(C_DOUBLE), intent(out) :: e1(7)
  real(C_DOUBLE)  :: dx(6)
  call aa_tf_qutr_diff2vel( e0, de, dx );
  call aa_tf_qutr_svel( e0, dx, dt, e1 )
end subroutine aa_tf_qutr_sdiff
