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

!> Cross product
!!
!! \param[in] x first vector
!! \param[in] y second vector
!! \param[out] z result
pure subroutine AA_FMOD(la,cross_sub)(x, y, z)
  AA_FTYPE(AA_FSIZE), intent(out) :: z(:)
  AA_FTYPE(AA_FSIZE), intent(in) :: x(:),y(:)
  !integer, dimension(3), parameter :: yzx = [2,3,1], zxy = [3,1,2]
  !z = x(yzx)*y(zxy) - x(zxy)*y(yzx)
  z(1) = x(2)*y(3) - y(2)*x(3)
  z(2) = y(1)*x(3) - x(1)*y(3)
  z(3) = x(1)*y(2) - y(1)*x(2)
end subroutine AA_FMOD(la,cross_sub)

!> Cross product
!!
!! \param[in] x first vector
!! \param[in] y second vector
pure function AA_FMOD(la,cross_fun)(x, y) result(z)
  AA_FTYPE(AA_FSIZE), intent(in) :: x(:),y(:)
  AA_FTYPE(AA_FSIZE) :: z(3)
  call AA_FMOD(la,cross_sub)(x,y,z)
end function AA_FMOD(la,cross_fun)

!> Cross product, C interface
!!
!! \param[in] x first vector
!! \param[in] y second vector
!! \param[out] z result
pure subroutine AA_FMOD_C_BEGIN(la, cross, x, y, z)
  AA_FTYPE(AA_FSIZE), intent(out) :: z(3)
  AA_FTYPE(AA_FSIZE), intent(in) :: x(3), y(3)
  call AA_FMOD(la,cross_sub)(x,y,z)
end subroutine AA_FMOD_C_END(la,cross)

!! SSD

!> Sum of squared differences
!!
!! \param[in] x first vector
!! \param[in] y second vector
pure function AA_FMOD(la,ssd)(x, y) result(a)
  AA_FTYPE(AA_FSIZE), intent(in) :: x(:), y(:)
  AA_FTYPE(AA_FSIZE) :: a
  !a = dot_product(x-y,x-y)
  a = sum( (x-y)**2 )
end function AA_FMOD(la,ssd)

!> Sum of squared differences, C interface
!!
!! \param[in] n vector sizes
!! \param[in] x first vector
!! \param[in] incx stepsize of x
!! \param[in] y second vector
!! \param[in] incy stepsize of y
pure function AA_FMOD_C_BEGIN(la, ssd, n, x, incx, y, incy) result(a)
  integer (c_size_t), intent(in), value :: n, incx, incy
  AA_FTYPE(AA_FSIZE), intent(in) :: x(n*incx), y(n*incy)
  AA_FTYPE(AA_FSIZE) :: a
  a = AA_FMOD(la,ssd)(x(1:n*incx:incx),y(1:n*incy:incy))
end function AA_FMOD_C_END(la,ssd)

!> Sum of squared differences of columns
!!
!! \param[in] A first data matrix
!! \param[in] B second data matrix
!! \param[out] C SSD between colums of A and B,
!!      row of C is index to col of A and
!!      col of C is index to col of B
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

!> Sum of squared differences of columns, C interface
!!
!! \param[in] m rows of A and B
!! \param[in] n cols of A and rows of C
!! \param[in] p cols of B and cols of C
!! \param[in] A first data matrix
!! \param[in] lda leading dimension of A
!! \param[in] B second data matrix
!! \param[in] ldb leading dimension of B
!! \param[out] C SSD between colums of A and B,
!!      row of C is index to col of A and
!!      col of C is index to col of B
!! \param[in] ldc leading dimension of C
pure subroutine AA_FMOD(la,colssd_c)(m,n,p,A,lda,B,ldb,C,ldc)
  integer(c_size_t),intent(in),value :: m,n,p,lda,ldb,ldc
  AA_FTYPE(AA_FSIZE), intent(in) :: A(lda,n),B(ldb,n)
  AA_FTYPE(AA_FSIZE), intent(out) :: C(ldc,p)
  call AA_FMOD(la,colssd)( A(1:m,:), B(1:m,:), C(1:n,:) )
end subroutine AA_FMOD(la,colssd_c)


!!---------------------!!
!! FLOATING POINT ONLY !!
!!---------------------!!

#if defined AA_TYPE_DOUBLE || defined AA_TYPE_FLOAT

!! Angle

!> Angle between vectors
!!
!! Method from: Kahan, Willliam. How futile are mindless assessments
!! of roundoff in floating-point computation. 2006
!!
!! \param[in] x first vector
!! \param[in] y second vector
pure function AA_FMOD(la,angle)( x, y ) result(r)
  real(AA_FSIZE), intent(in) :: x(:),y(:)
  real(AA_FSIZE) :: r, nx, ny, s, c
  integer i
  nx = sqrt(dot_product(x,x))
  ny = sqrt(dot_product(y,y))
  s = real(0, AA_FSIZE)
  c = real(0, AA_FSIZE)
  do i=1,size(x)
     s = s + (ny*x(i) - nx*y(i))**2
     c = c + (ny*x(i) + nx*y(i))**2
  end do
  r = real(2,AA_FSIZE)*atan2(sqrt(s),sqrt(c))
end function AA_FMOD(la,angle)

!> Angle between vectors, C interface
!!
!! \param[in] n length of vectors
!! \param[in] x first vector
!! \param[in] incx stepsize of x
!! \param[in] y second vector
!! \param[in] incy stepsize of y
pure function AA_FMOD_C_BEGIN(la, angle, n, x, incx, y, incy ) result(a)
  integer(C_SIZE_T), intent(in), value   :: n, incx, incy
  real(AA_FSIZE), intent(in) :: x(n*incx),y(n*incy)
  real(AA_FSIZE) :: a
  a = AA_FMOD(la,angle)( x(1:n:incx), y(1:n:incy) )
end function AA_FMOD_C_END(la,angle)


!! Norm2

!> Norm-2 of vector
!!
!! \param[in] x input vector
pure function AA_FMOD(la,norm2)( x ) result(a)
  real(AA_FSIZE), intent(in) :: x(:)
  real(AA_FSIZE) :: a
  real(AA_FSIZE) :: scl, ssq
  integer :: i
  ssq = real(1,AA_FSIZE)
  scl = real(0,AA_FSIZE)
  do i=1,size(x)
     if ( real(0,AA_FSIZE) /= x(i) ) then
        if( scl < abs(x(i)) ) then
           ssq = real(1,AA_FSIZE) + ssq * (scl/abs(x(i)))**2
           scl = abs(x(i))
        else
           ssq = ssq + (abs(x(i))/scl)**2
        end if
     end if
  end do
  a = scl * sqrt(ssq)
end function AA_FMOD(la,norm2)

!> Make unit vector
!!
!! \param[inout] y vector to make unit
pure subroutine AA_FMOD(la,unit_sub1)( y )
  real(AA_FSIZE), intent(inout) :: y(:)
  y = y / AA_FMOD(la,norm2)(y)
end subroutine AA_FMOD(la,unit_sub1)

!> Make unit vector
!!
!! \param[in] x input vector
!! \param[out] y unit vector with same direction as x
pure subroutine AA_FMOD(la,unit_sub2)( x, y )
  real(AA_FSIZE), intent(in) :: x(:)
  real(AA_FSIZE), intent(out) :: y(:)
  y = x / AA_FMOD(la,norm2)(x)
end subroutine AA_FMOD(la,unit_sub2)

!> Make unit vector
!!
!! \param[in] x input vector
pure function AA_FMOD(la,unit_fun)( x ) result(y)
  real(AA_FSIZE), intent(in) :: x(:)
  real(AA_FSIZE)  :: y(size(x))
  call AA_FMOD(la,unit_sub2)(x,y)
end function AA_FMOD(la,unit_fun)

!! Proj

!> Vector projection
pure subroutine AA_FMOD(la,proj_sub)( a, b, r )
  real(AA_FSIZE), intent(in)  :: a(:),b(:)
  real(AA_FSIZE), intent(out) :: r(:)
  r = (dot_product(a,b) * b) / dot_product(b,b)
end subroutine AA_FMOD(la,proj_sub)

!> Vector projection
pure function AA_FMOD(la,proj_fun)( a, b ) result(r)
  real(AA_FSIZE), intent(in)  :: a(:),b(:)
  real(AA_FSIZE) :: r(size(a))
  call AA_FMOD(la,proj_sub)(a,b,r)
end function AA_FMOD(la,proj_fun)

!! Orth

!> Vector orthogonal projection
pure subroutine AA_FMOD(la,orth_sub)( a, b, r )
  real(AA_FSIZE), intent(in)  :: a(:),b(:)
  real(AA_FSIZE), intent(out) :: r(:)
  r = a - (dot_product(a,b) * b) / dot_product(b,b)
end subroutine AA_FMOD(la,orth_sub)

!> Vector orthogonal projection
pure function AA_FMOD(la,orth_fun)( a, b ) result(r)
  real(AA_FSIZE), intent(in)  :: a(:),b(:)
  real(AA_FSIZE) :: r(size(a))
  call AA_FMOD(la,orth_sub)(a,b,r)
end function AA_FMOD(la,orth_fun)


!! Mean/STD
pure function AA_FMOD(la,vecstd)(x,mu) result(sigma)
  AA_FTYPE(AA_FSIZE), intent(in) :: x(:), mu
  AA_FTYPE(AA_FSIZE) :: sigma
  sigma = sqrt( TOREAL(1)/(TOREAL(size(x)-1)) * sum( (x-mu)**2 ) )
end function AA_FMOD(la,vecstd)

pure function AA_FMOD_C_BEGIN(la, vecstd, n,x,incx,mu) result(sigma)
  integer(c_size_t), intent(in), value :: n,incx
  AA_FTYPE(AA_FSIZE), intent(in), value :: mu
  AA_FTYPE(AA_FSIZE), intent(in) :: x(n*incx)
  AA_FTYPE(AA_FSIZE) :: sigma
  sigma = aa_la_std(x(1:n:incx),mu)
end function AA_FMOD_C_END(la,vecstd)

!! Colmean

!> Mean of columns
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

!> Mean of columns, C interface
pure subroutine AA_FMOD_C_BEGIN(la, colmean, m, n, A, lda, x )
  integer(C_SIZE_T), intent(in), value :: m,n,lda
  real(AA_FSIZE), intent(in) :: A(lda,n)
  real(AA_FSIZE), intent(out) :: x(m)
  call AA_FMOD(la,colmean)(A,x)
end subroutine AA_FMOD_C_END(la,colmean)


!> Mean of rows
pure subroutine AA_FMOD(la,rowmean)( A, x )
  real(AA_FSIZE), intent(in) :: A(:,:)
  real(AA_FSIZE), intent(out) :: x(:)
  integer :: i
  forall (i = 1:size(x))
     x(i) = sum(A(:,i)) / TOREAL(size(A,1))
  end forall
end subroutine AA_FMOD(la,rowmean)

!> Mean of rows, C interface
pure subroutine AA_FMOD_C_BEGIN(la, rowmean, m, n, A, lda, x )
  integer(C_SIZE_T), intent(in), value :: m,n,lda
  real(AA_FSIZE), intent(in) :: A(lda,n)
  real(AA_FSIZE), intent(out) :: x(m)
  call AA_FMOD(la,rowmean)(A,x)
end subroutine AA_FMOD_C_END(la,rowmean)

!! Colcov

!> Covariance of columns
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


!> Covariance of columns, C interface
pure subroutine AA_FMOD_C_BEGIN(la, colcov, m, n, A, lda, x, E, lde )
  integer(C_SIZE_T), intent(in), value :: m,n,lda,lde
  real(AA_FSIZE), intent(in) :: A(lda,n)
  real(AA_FSIZE), intent(in) :: x(m)
  real(AA_FSIZE), intent(out) :: E(lde,m)
  call AA_FMOD(la,colcov)(A,x,E)
end subroutine AA_FMOD_C_END(la,colcov)

!! Fits

!> Fit hyperplane to columns
subroutine AA_FMOD(la,colfit)( A, x )
  use amino_mem
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

!> Fit hyperplane to columns, C interface
subroutine AA_FMOD_C_BEGIN(la, colfit, m, n, A, lda, x)
  integer(c_size_t), intent(in), value :: m,n,lda
  real(AA_FSIZE), intent(out) :: A(lda,n), x(int(m)+1)
  call AA_FMOD(la,colfit)( A, x )
end subroutine AA_FMOD_C_END(la,colfit)

pure subroutine AA_FMOD(la,lerp) (u, v1, v2, vu)
  real(AA_FSIZE), intent(in)  :: v1(:), v2(:)
  real(AA_FSIZE), intent(in)  :: u
  real(AA_FSIZE), intent(out) :: vu(:)
  vu = v1 + u * (v2 - v1 )
end subroutine AA_FMOD(la,lerp)

subroutine AA_FMOD_C_BEGIN(la,lerp, n, u, v1, inc1, v2, inc2, vu, incu)
  integer(c_size_t), intent(in), value :: n, inc1, inc2, incu
  real(AA_FSIZE), intent(in)  :: v1(n*inc1), v2(n*inc2)
  real(AA_FSIZE), intent(in), value  :: u
  real(AA_FSIZE), intent(out) :: vu(n*incu)
  call aa_la_lerp( u, v1(1:size(v1):inc1), v2(1:size(v2):inc2), vu(1:size(vu):incu) )
end subroutine AA_FMOD_C_END(la,lerp)

!> Compute cubic spline parameters for time from x1 to x2
!> Note, a0 = x1 and a1 = dx1
pure subroutine AA_FMOD(la,3spline_param) (tf, x1, dx1, x2, dx2, a2, a3)
  real(AA_FSIZE), intent(in) :: tf
  real(AA_FSIZE), dimension(:), intent(in)  :: x1, x2, dx1, dx2
  real(AA_FSIZE), dimension(:), intent(out) :: a2, a3
  a2 = 3/tf**2 * (x2-x1) - 2/tf * dx1 - 1/tf * dx2
  a3 = -2/tf**3 * (x2-x1) + 1/tf**2 * (dx2+dx1)
end subroutine AA_FMOD(la,3spline_param)

!> Compute cubic spline values
pure subroutine AA_FMOD(la,3spline) (t, x1, dx1, a2, a3, x, dx, ddx)
  real(AA_FSIZE), dimension(:), intent(in)  :: x1, dx1, a2, a3
  real(AA_FSIZE), intent(in) :: t
  real(AA_FSIZE), dimension(:), intent(out)  :: x, dx, ddx
  x = x1 + t*dx1 + t**2*a2 + t**3*a3
  dx = dx1 + 2*t*a2 + 3*t**2*a3
  ddx = 2*a2 + 6*t*a3
end subroutine AA_FMOD(la,3spline)

subroutine AA_FMOD_C_BEGIN(la, 3spline_param, n, tf, x1, incx1, dx1, incdx1, x2, incx2, dx2, incdx2, a2, a3 )
  integer(c_size_t), intent(in), value :: n, incx1, incdx1, incx2, incdx2
  real(AA_FSIZE), value  :: tf
  real(AA_FSIZE), intent(in)  :: x1(n*incx1), x2(n*incx2), dx1(n*incdx1), dx2(n*incdx2)
  real(AA_FSIZE), intent(out) :: a2(n), a3(n)
  call aa_la_3spline_param(tf, &
       x1(1:size(x1):incx1), dx1(1:size(dx1):incdx1),  &
       x2(1:size(x2):incx2), dx2(1:size(dx2):incdx2),  &
       a2, a3 )
end subroutine AA_FMOD_C_END(la,3spline_param)

subroutine AA_FMOD_C_BEGIN(la, 3spline, n, tf, x1, incx1, dx1, incdx1, a2, a3, x, incx, dx, incdx, ddx, incddx )
  integer(c_size_t), intent(in), value :: n, incx1, incdx1, incx, incdx, incddx
  real(AA_FSIZE), value  :: tf
  real(AA_FSIZE), intent(in)  :: x1(n*incx1), dx1(n*incdx1), a2(n), a3(n)
  real(AA_FSIZE), intent(out) :: x(n*incx), dx(n*incdx), ddx(n*incddx)
  call aa_la_3spline(tf, &
       x1(1:size(x1):incx1), dx1(1:size(dx1):incdx1), a2, a3, &
       x(1:size(x):incx), dx(1:size(dx):incdx), ddx(1:size(ddx):incddx) )
end subroutine AA_FMOD_C_END(la,3spline)

!> Compute quintic spline parameters for time from x1 to x2
!> Note, a0 = x1, a1 = dx1, and a2 = ddx1/2
pure subroutine AA_FMOD(la,5spline_param) (tf, x1, dx1, ddx1, x2, dx2, ddx2, a3, a4, a5)
  real(AA_FSIZE), intent(in) :: tf
  real(AA_FSIZE), dimension(:), intent(in)  :: x1, x2, dx1, dx2, ddx1, ddx2
  real(AA_FSIZE), dimension(:), intent(out) :: a3, a4, a5
  a3 = ( 20*x2 - 20*x1 - (8*dx2 + 12*dx1)*tf - (3*ddx1-ddx2)*tf**2 ) / (2*tf**3)
  a4 = ( 30*x1 - 30*x2 + (14*dx2 + 16*dx1)*tf + (3*ddx1 - 2*ddx2)*tf**2 ) / (2*tf**4)
  a5 = ( 12*x2 - 12*x1 - (6*dx2 + 6*dx1)*tf - (ddx1 - ddx2)*tf**2 ) / (2*tf**5)
end subroutine AA_FMOD(la,5spline_param)

pure subroutine AA_FMOD(la,5spline) (t, x1, dx1, ddx1, a3, a4, a5, x, dx, ddx)
  real(AA_FSIZE), dimension(:), intent(in)  :: x1, dx1, ddx1, a3, a4, a5
  real(AA_FSIZE), intent(in) :: t
  real(AA_FSIZE), dimension(:), intent(out)  :: x, dx, ddx
  x = x1 + t*dx1 + 0.5*t**2*ddx1 + t**3*a3 + t**4*a4 + t**5*a5
  dx = dx1 + t*ddx1 + 3*t**2*a3 + 4*t**3*a4 + 5*t**4*a5
  ddx = ddx1 + 6*t*a3 + 12*t**2*a4 + 20*t**3*a5
end subroutine AA_FMOD(la,5spline)

subroutine AA_FMOD_C_BEGIN(la, 5spline_param, n, tf, x1, incx1, dx1, incdx1, ddx1, incddx1, x2, incx2, dx2, incdx2, ddx2, incddx2, a3, a4, a5 )
  integer(c_size_t), intent(in), value :: n, incx1, incdx1, incx2, incdx2, incddx1, incddx2
  real(AA_FSIZE), value  :: tf
  real(AA_FSIZE), intent(in)  :: x1(n*incx1), x2(n*incx2), dx1(n*incdx1), dx2(n*incdx2), ddx1(n*incddx1), ddx2(n*incddx2)
  real(AA_FSIZE), intent(out) :: a3(n), a4(n), a5(n)
  call aa_la_5spline_param(tf, &
       x1(1:size(x1):incx1), dx1(1:size(dx1):incdx1), ddx1(1:size(ddx1):incddx1),  &
       x2(1:size(x2):incx2), dx2(1:size(dx2):incdx2), ddx2(1:size(ddx2):incddx2),  &
       a3, a4, a5 )
end subroutine AA_FMOD_C_END(la,5spline_param)

subroutine AA_FMOD_C_BEGIN(la, 5spline, n, tf, x1, incx1, dx1, incdx1, ddx1, incddx1, a3, a4, a5, x, incx, dx, incdx, ddx, incddx )
  integer(c_size_t), intent(in), value :: n, incx1, incdx1, incx, incdx, incddx1, incddx
  real(AA_FSIZE), value  :: tf
  real(AA_FSIZE), intent(in)  :: x1(n*incx1), dx1(n*incdx1),  ddx1(n*incddx1)
  real(AA_FSIZE), intent(in) :: a3(n), a4(n), a5(n)
  real(AA_FSIZE), intent(out) :: x(n*incx), dx(n*incdx), ddx(n*incddx)
  call aa_la_5spline(tf, &
       x1(1:size(x1):incx1), dx1(1:size(dx1):incdx1), ddx1(1:size(ddx1):incddx1),  &
       a3, a4, a5, &
       x(1:size(x):incx), dx(1:size(dx):incdx), ddx(1:size(ddx):incddx) )
end subroutine AA_FMOD_C_END(la,5spline)

subroutine AA_FMOD_C_BEGIN(la, assign_hungarian, m, n, A, lda, row, col)
  use amino_mem
  integer(c_size_t), intent(in), value :: m,n,lda
  real(AA_FSIZE),intent(in) :: A(lda,n)
  integer(c_size_t),intent(out) :: row(m),col(n)

  integer, pointer :: rowi(:), coli(:)
  call aa_mem_region_alloc( m, rowi )
  call aa_mem_region_alloc( m, coli )

  call AA_FMOD(la,assign_hungarian)(A(1:m,1:n),rowi,coli)
  ! convert to C indices
  row = int(rowi-1,c_size_t)
  col = int(coli-1,c_size_t)

  call aa_mem_region_pop( rowi )
end subroutine AA_FMOD_C_END(la,assign_hungarian)


!> Solve assignment problem via Hungarian algorithm, padding if necessary
subroutine AA_FMOD(la,assign_hungarian)(A,row_assign,col_assign)
  use amino_mem
  real(AA_FSIZE), intent(in) :: A(:,:)
  integer, intent(out) :: row_assign(:),col_assign(:)

  integer :: m,n,p
  integer, pointer :: alt_assign( : )
  real(AA_FSIZE), pointer :: B(:,:)

  m = size(A,1)
  n = size(A,2)
  p = max(m,n)

  call aa_mem_region_alloc( p, p, b)
  B = real(0,AA_FSIZE)
  B(1:m,1:n) = A

  if ( m > n ) then
     ! more rows
     call aa_mem_region_alloc( p, alt_assign )
     call AA_FMOD(la,assign_hungarian_square)(B,row_assign,alt_assign)
     col_assign = alt_assign(1:n)
     where (row_assign > n)
        row_assign = -1
     end where
  elseif ( m < n ) then
     ! more cols
     call aa_mem_region_alloc( p, alt_assign )
     call AA_FMOD(la,assign_hungarian_square)(B,alt_assign,col_assign)
     row_assign = alt_assign(1:m)
     where (col_assign > m)
        col_assign = -1
     end where
  else
     ! equal rows / cols
     call AA_FMOD(la,assign_hungarian_square)(B,row_assign,col_assign)
  end if

  call aa_mem_region_pop(B)

end subroutine AA_FMOD(la,assign_hungarian)

!> Solve square assignment problem via Hungarian algorithm
subroutine AA_FMOD(la,assign_hungarian_square)(A,row_assign,col_assign)
  use amino_mem
  real(AA_FSIZE), intent(inout) :: A(:,:)
  integer, intent(out) :: row_assign(:),col_assign(:)

  integer, pointer :: path(:,:)
  logical(4), pointer :: star(:,:)
  logical(4), pointer :: prime(:,:)
  logical(4), pointer :: row(:),col(:)

  integer :: i,j,k,n
  real(AA_FSIZE) :: mv

  n = size(A,1)
  call aa_mem_region_alloc(2,n**2,path)
  call aa_mem_region_alloc(n,n,star)
  call aa_mem_region_alloc(n,n,prime)
  call aa_mem_region_alloc(n,row)
  call aa_mem_region_alloc(n,col)

  ! --- Step 1 ---
  ! subtract smallest from each col
  forall (j=1:n)
     A(:,j) = A(:,j) - minval(A(:,j))
  end forall
  ! subtract smallest from each row
  forall (i=1:n)
     A(i,:) = A(i,:) - minval(A(i,:))
  end forall

  ! --- Step 2 ---
  ! Star each zero in A
  star = .false.
  row = .false.
  do j=1,n
     do i=1,n
        if ( real(0,AA_FSIZE) >= A(i,j) .and. &
             .not. row(i) ) then
           row(i) = .true.
           star(i,j) = .true.
           exit ! one mark per column
        end if
     end do
  end do

  ! --- Step 3 ---
  ! Cover starred columns
  step3: do
     forall (j=1:n)
       col(j) = any(star(:,j))
    end forall
     ! check if all covered
     if ( count(col) == n ) then
        exit step3
     end if

     ! --- Step 4 ---
     ! Find uncovered zero and prime it or goto 6.
     ! If star in row, cover row, uncover stars column,
     ! repeat for all uncovered zeros.  Otherwise, goto 5.
     prime = .false.
     row = .false.
     step4: do
        do j=1,n
           if ( .not. col(j) ) then
              do i=1,n
                 if ( real(0,AA_FSIZE) >= A(i,j) .and. &
                      .not. row(i) ) then
                    ! prime the uncovered zero
                    prime(i,j) = .true.
                    do k=1,n
                       if ( star(i,k) ) then
                          row(i) = .true.  ! cover star row
                          col(k) = .false. ! uncover stars column
                          cycle step4
                       end if
                    end do
                    ! nothing starred in row
                    path(1,1) = i
                    path(2,1) = j
                    call make_path() ! step5
                    cycle step3
                 end if
              end do
           end if
        end do

        ! no uncovered zeros

        ! --- Step 6 ---
        mv = HUGE(A(1,1))
        ! find smallest uncovered value in A
        row = .not. row
        do j=1,n
           if ( .not. col(j) ) then
              mv = min( mv, minval( A(:,j), row ) )
           end if
        end do
        row = .not. row

        ! add minval to covered rows
        ! subtract minval from uncovered cols
        do j=1,n
           if( col(j) ) then
              where (row) A(:,j) = A(:,j) + mv
           else
              where (.not. row) A(:,j) = A(:,j) - mv
           end if
        end do
     end do  step4
  end do step3

  ! --- Step 7 (The End) ---
  ! Compute assignments
  do j=1,n
     do i=1,n
        if ( star(i,j) ) then
           row_assign(i) = j
           col_assign(j) = i
           exit ! one assignment per row/col
        end if
     end do
  end do

  ! deallocate
  call aa_mem_region_pop(path)

  contains

    ! --- Step 5 ---
    !! construct series of alternating primes and stars
    subroutine make_path()
      integer :: k,i,j,c
      c = 1
      ! loop till no starred column
      do k=1,n
         ! find starred column
         do i=1,n
            if ( star(i, path(2,c)) ) then
               c = c+1
               path(1,c) = i ! row of starred zero
               path(2,c) = path(2,c-1) ! col of starred zero
               do j=1,n
                  if ( prime( path(1,c), j ) ) then
                     c = c+1
                     path(1,c) = path(1,c-1) ! row of primed zero
                     path(2,c) = j ! col of primed zero
                     exit
                  end if
               end do
               exit
            end if
         end do
      end do
      ! convert path
      ! forall (k=1:c)
      !    star( path(1,k), path(2,k) ) = &
      !         .not. star( path(1,k), path(2,k) )
      ! end forall
      do k=1,c
         star( path(1,k), path(2,k) ) = &
              .not. star( path(1,k), path(2,k) )
      end do
    end subroutine make_path
end subroutine AA_FMOD(la,assign_hungarian_square)

subroutine AA_FMOD(la,assign_hungarian_max2min)(A)
  real(AA_FSIZE), intent(inout) :: A(:,:)
  A = maxval(A) - A
end subroutine AA_FMOD(la,assign_hungarian_max2min)

subroutine AA_FMOD_C_BEGIN(la, assign_hungarian_max2min, m, n, A, lda)
  integer(c_size_t), intent(in), value :: m,n,lda
  real(AA_FSIZE),intent(inout) :: A(lda,n)
  call AA_FMOD(la,assign_hungarian_max2min)( A(1:m,1:n) )
end subroutine AA_FMOD_C_END(la,assign_hungarian_max2min)

#endif
#if 0
!! Float
#endif


#include "amino/undef.F90"
