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

!! FILE: la_float.F90
!! BRIEF: Linear Algebra Functions
!! AUTHOR: Neil T. Dantam
!!
!! This file is included multiple times with macros defined for
!! different types


#include "amino/def.h"

!! Angle

function AA_LA_FMOD(angle)( p, q ) result(a)
  AA_LA_FTYPE, dimension(:), intent(in) :: p,q
  AA_LA_FTYPE :: a
  a = acos( dot_product(p, q) )
end function AA_LA_FMOD(angle)

function AA_LA_FMOD_C(angle)( n, p, q ) result(a)
  integer(C_SIZE_T), value :: n
  AA_LA_FTYPE, intent(in), dimension(n) :: p,q
  AA_LA_FTYPE :: a
  a = AA_LA_FMOD(angle)(p,q)
end function AA_LA_FMOD_C(angle)

!! Proj

pure subroutine AA_LA_FMOD(proj)( a, b, r )
  AA_LA_FTYPE, dimension(:), intent(in) :: a,b
  AA_LA_FTYPE, dimension(:), intent(out) :: r
  r = (dot_product(a,b) * b) / dot_product(b,b)
end subroutine AA_LA_FMOD(proj)

!! Orth

pure subroutine AA_LA_FMOD(orth)( a, b, r )
  AA_LA_FTYPE, dimension(:), intent(in) :: a,b
  AA_LA_FTYPE, dimension(:), intent(out) :: r
  r = a - (dot_product(a,b) * b) / dot_product(b,b)
end subroutine AA_LA_FMOD(orth)

!! Colmean

pure subroutine AA_LA_FMOD(colmean)( A, x )
  AA_LA_FTYPE, dimension(:,:), intent(in) :: A
  AA_LA_FTYPE, dimension(:), intent(out) :: x
  integer :: i
  x = 0
  do i = 1,size(A,2)
     x = x + A(:,i)
  end do
  x = x / size(A,2)
end subroutine AA_LA_FMOD(colmean)

subroutine AA_LA_FMOD_C(colmean)( m, n, A, lda, x )
  integer(C_SIZE_T), value :: m,n,lda
  AA_LA_FTYPE, dimension(lda,n), intent(in) :: A
  AA_LA_FTYPE, dimension(m), intent(out) :: x
  call AA_LA_FMOD(colmean)(A,x)
end subroutine AA_LA_FMOD_C(colmean)

!! Colcov

pure subroutine AA_LA_FMOD(colcov)( A, x, E )
  AA_LA_FTYPE, dimension(:,:), intent(in) :: A
  AA_LA_FTYPE, dimension(:), intent(in) :: x
  AA_LA_FTYPE, dimension(:,:), intent(out) :: E

  integer :: k,i,j
  E = 0
  do k = 1,size(A,2)
     forall (i=1:size(x))
        forall (j=1:size(x))
           E(i,j) = E(i,j) + (A(i,k)-x(i)) * (A(j,k)-x(j))
        end forall
     end forall
  end do
  E = E / (size(A,2)-1)
end subroutine AA_LA_FMOD(colcov)

subroutine AA_LA_FMOD_C(colcov)( m, n, A, lda, x, E, lde )
  integer(C_SIZE_T), value :: m,n,lda,lde
  AA_LA_FTYPE, dimension(lda,n), intent(in) :: A
  AA_LA_FTYPE, dimension(m), intent(in) :: x
  AA_LA_FTYPE, dimension(lde,m), intent(out) :: e
  call AA_LA_FMOD(colcov)(A,x,E)
end subroutine AA_LA_FMOD_C(colcov)

#include "amino/undef.h"
