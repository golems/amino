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


#include "amino/def.F90"

!!------------------!!
!! FLOATING and INT !!
!!------------------!!


pure subroutine AA_FMOD(la,cross_sub)(a, b, r)
  AA_FTYPE(AA_FSIZE), intent(out) :: r(:)
  AA_FTYPE(AA_FSIZE), intent(in) :: a(:),b(:)
  integer, dimension(3), parameter :: yzx = [2,3,1], zxy = [3,1,2]
  r = a(yzx)*b(zxy) - a(zxy)*b(yzx)
end subroutine AA_FMOD(la,cross_sub)

pure function AA_FMOD(la,cross_fun)(a, b) result(r)
  AA_FTYPE(AA_FSIZE), intent(in) :: a(:),b(:)
  AA_FTYPE(AA_FSIZE) :: r(3)
  call AA_FMOD(la,cross_sub)(a,b,r)
end function AA_FMOD(la,cross_fun)

pure subroutine AA_FMOD_C(la,cross)( a, b, r)
  AA_FTYPE(AA_FSIZE), intent(out) :: r(3)
  AA_FTYPE(AA_FSIZE), intent(in) :: a(3), b(3)
  call AA_FMOD(la,cross_sub)(a,b,r)
end subroutine AA_FMOD_C(la,cross)

!! SSD

pure function AA_FMOD(la,ssd)(x, y) result(a)
  AA_FTYPE(AA_FSIZE), intent(in) :: x(:), y(:)
  AA_FTYPE(AA_FSIZE) :: a
  a = dot_product(x-y,x-y)
end function AA_FMOD(la,ssd)

pure function AA_FMOD_C(la,ssd)(n, x, incx, y, incy) result(a)
  integer (c_size_t), intent(in), value :: n, incx, incy
  AA_FTYPE(AA_FSIZE), intent(in) :: x(n*incx), y(n*incy)
  AA_FTYPE(AA_FSIZE) :: a
  a = AA_FMOD(la,ssd)(x(1:n*incx:incx),y(1:n*incy:incy))
end function AA_FMOD_C(la,ssd)

pure subroutine AA_FMOD(la,colssd)(A, B, C)
  AA_FTYPE(AA_FSIZE), intent(in) :: A(:,:), B(:,:)
  AA_FTYPE(AA_FSIZE), intent(out) :: C(:,:)
  integer :: i,j
  forall (i=1:size(C,1))
     forall (j=1:size(C,2))
        C(i,j) = AA_FMOD(la,ssd)(A(:,i), B(:,j))
     end forall
  end forall
end subroutine AA_FMOD(la,colssd)

pure subroutine AA_FMOD(la,colssd_c)(m,n,A,lda,B,ldb,C,ldc)
  integer(c_size_t),intent(in),value :: m,n,lda,ldb,ldc
  AA_FTYPE(AA_FSIZE), intent(in) :: A(lda,m),B(ldb,n)
  AA_FTYPE(AA_FSIZE), intent(out) :: C(ldc,n)
  call AA_FMOD(la,colssd)(A,B,C)
end subroutine AA_FMOD(la,colssd_c)


!!---------------------!!
!! FLOATING POINT ONLY !!
!!---------------------!!

#if defined AA_TYPE_DOUBLE || defined AA_TYPE_FLOAT

!! Angle

pure function AA_FMOD(la,angle)( p, q ) result(r)
  real(AA_FSIZE), intent(in) :: p(:),q(:)
  real(AA_FSIZE) :: r
  r = acos( dot_product(p, q) / sqrt( dot_product(p,p)*dot_product(q,q) ) )
end function AA_FMOD(la,angle)

pure function AA_FMOD_C(la,angle)( n, p, q ) result(a)
  integer(C_SIZE_T), intent(in), value   :: n
  real(AA_FSIZE), intent(in) :: p(n),q(n)
  real(AA_FSIZE) :: a
  a = AA_FMOD(la,angle)(p,q)
end function AA_FMOD_C(la,angle)


!! Norm2

pure function AA_FMOD(la,norm2)( p ) result(a)
  real(AA_FSIZE), intent(in) :: p(:)
  real(AA_FSIZE) :: a
  a = sqrt(dot_product(p,p))
end function AA_FMOD(la,norm2)

pure subroutine AA_FMOD(la,unit_sub1)( p )
  real(AA_FSIZE), intent(inout) :: p(:)
  p = p / AA_FMOD(la,norm2)(p)
end subroutine AA_FMOD(la,unit_sub1)

pure subroutine AA_FMOD(la,unit_sub2)( p, u )
  real(AA_FSIZE), intent(in) :: p(:)
  real(AA_FSIZE), intent(out) :: u(:)
  u = p / AA_FMOD(la,norm2)(p)
end subroutine AA_FMOD(la,unit_sub2)

pure function AA_FMOD(la,unit_fun)( p ) result(u)
  real(AA_FSIZE), intent(in) :: p(:)
  real(AA_FSIZE)  :: u(size(p))
  call AA_FMOD(la,unit_sub2)(p,u)
end function AA_FMOD(la,unit_fun)

!! Proj

pure subroutine AA_FMOD(la,proj_sub)( a, b, r )
  real(AA_FSIZE), intent(in)  :: a(:),b(:)
  real(AA_FSIZE), intent(out) :: r(:)
  r = (dot_product(a,b) * b) / dot_product(b,b)
end subroutine AA_FMOD(la,proj_sub)

pure function AA_FMOD(la,proj_fun)( a, b ) result(r)
  real(AA_FSIZE), intent(in)  :: a(:),b(:)
  real(AA_FSIZE) :: r(size(a))
  call AA_FMOD(la,proj_sub)(a,b,r)
end function AA_FMOD(la,proj_fun)

!! Orth

pure subroutine AA_FMOD(la,orth_sub)( a, b, r )
  real(AA_FSIZE), intent(in)  :: a(:),b(:)
  real(AA_FSIZE), intent(out) :: r(:)
  r = a - (dot_product(a,b) * b) / dot_product(b,b)
end subroutine AA_FMOD(la,orth_sub)

pure function AA_FMOD(la,orth_fun)( a, b ) result(r)
  real(AA_FSIZE), intent(in)  :: a(:),b(:)
  real(AA_FSIZE) :: r(size(a))
  call AA_FMOD(la,orth_sub)(a,b,r)
end function AA_FMOD(la,orth_fun)


!! Colmean

pure subroutine AA_FMOD(la,colmean)( A, x )
  real(AA_FSIZE), intent(in) :: A(:,:)
  real(AA_FSIZE), intent(out) :: x(:)
  integer :: i
  !x = TOREAL(0)
  !do i = 1,size(A,2)
     !x = x + A(:,i)
  !end do
  !x = x / TOREAL(size(A,2))
  forall (i = 1:size(x))
     x(i) = sum(A(i,:)) / TOREAL(size(A,2))
  end forall
end subroutine AA_FMOD(la,colmean)

pure subroutine AA_FMOD_C(la,colmean)( m, n, A, lda, x )
  integer(C_SIZE_T), intent(in), value :: m,n,lda
  real(AA_FSIZE), intent(in) :: A(lda,n)
  real(AA_FSIZE), intent(out) :: x(m)
  call AA_FMOD(la,colmean)(A,x)
end subroutine AA_FMOD_C(la,colmean)


pure subroutine AA_FMOD(la,rowmean)( A, x )
  real(AA_FSIZE), intent(in) :: A(:,:)
  real(AA_FSIZE), intent(out) :: x(:)
  integer :: i
  forall (i = 1:size(x))
     x(i) = sum(A(:,i)) / TOREAL(size(A,1))
  end forall
end subroutine AA_FMOD(la,rowmean)

pure subroutine AA_FMOD_C(la,rowmean)( m, n, A, lda, x )
  integer(C_SIZE_T), intent(in), value :: m,n,lda
  real(AA_FSIZE), intent(in) :: A(lda,n)
  real(AA_FSIZE), intent(out) :: x(m)
  call AA_FMOD(la,rowmean)(A,x)
end subroutine AA_FMOD_C(la,rowmean)

!! Colcov

pure subroutine AA_FMOD(la,colcov)( A, x, E )
  real(AA_FSIZE), intent(in) :: A(:,:)
  real(AA_FSIZE), intent(in) :: x(:)
  real(AA_FSIZE), intent(out) :: E(:,:)

  integer :: k,i,j
  E = TOREAL(0)
  do k = 1,size(A,2)
     forall (i=1:size(x))
        forall (j=1:size(x))
           E(i,j) = E(i,j) + (A(i,k)-x(i)) * (A(j,k)-x(j))
        end forall
     end forall
  end do
  E = E / TOREAL(size(A,2)-1)
end subroutine AA_FMOD(la,colcov)


pure subroutine AA_FMOD_C(la,colcov)( m, n, A, lda, x, E, lde )
  integer(C_SIZE_T), intent(in), value :: m,n,lda,lde
  real(AA_FSIZE), intent(in) :: A(lda,n)
  real(AA_FSIZE), intent(in) :: x(m)
  real(AA_FSIZE), intent(out) :: E(lde,m)
  call AA_FMOD(la,colcov)(A,x,E)
end subroutine AA_FMOD_C(la,colcov)

!! Fits
pure subroutine AA_FMOD(la,colfit)( A, x )
  real(AA_FSIZE), intent(in) :: A(:,:)
  real(AA_FSIZE), intent(out) :: x(:)

  real(AA_FSIZE) :: At(size(A,2),size(A,1))
  real(AA_FSIZE) :: b(size(A,2)), xout(size(A,1))
  integer :: m,n,i
  real(AA_FSIZE) :: d
  m = size(A,1) ! space size
  n = size(A,2) ! data points

  ! construct normed A,b matrix
  forall (i=1:n)
     At(i,:) = A(:,i) / aa_la_norm2(A(:,i))
     b(i) = TOREAL(-1) / aa_la_norm2(A(:,i))
  end forall
  ! solve
  call AA_NAME(la,lls)( int(n,c_size_t), int(m,c_size_t), int(1,c_size_t), &
       At, int(n,c_size_t), b, int(n,c_size_t), xout, int(m,c_size_t) )
  ! normalize
  d = aa_la_norm2(xout)
  x(1:m) = xout/d
  x(m+1) = TOREAL(1)/d
end subroutine AA_FMOD(la,colfit)

pure subroutine AA_FMOD_C(la,colfit)( m, n, A, lda, x  )
  integer(c_size_t), intent(in), value :: m,n,lda
  real(AA_FSIZE), intent(out) :: A(lda,n), x(m)
  call AA_FMOD(la,colfit)( A, x )
end subroutine AA_FMOD_C(la,colfit)

#endif
#if 0
!! Float
#endif


#include "amino/undef.F90"
