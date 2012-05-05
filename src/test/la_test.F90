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



program la_test
  use amino_la

  real,parameter :: pi=3.14159265

  print *,"Running la_test..."

  call test_ssd()
  call test_colssd()
  call test_angle()
  call test_norm2()
  call test_cross1()
  call test_cross2()
  call test_proj_orth()
  call test_std()
  call test_meancov()

  print *,"la_test completed."


contains


  subroutine test(name, tst)
    logical :: tst
    character :: name
    if( .not. tst ) then
       print *,"FAILED:", name
       call abort()
    end if
  end subroutine test

  subroutine ftest(name, a,b,tol)
    character :: name
    real :: a,b,tol
    call test( name, abs(a-b) <= abs(tol) )
  end subroutine ftest

  subroutine vtest(name, a,b,tol)
    character :: name
    real :: a(:),b(:),tol
    call test( name, sqrt(aa_la_ssd(a,b)) <= abs(tol) )
  end subroutine vtest

  subroutine mtest(name, a,b,tol)
    character :: name
    real :: a(:,:),b(:,:),tol
    real :: va(size(a)), vb(size(b))
    integer :: i,j
    forall (i=1:size(a,1))
       forall (j=1:size(a,2))
          va((j-1)*size(a,1)+i) = a(i,j)
       end forall
    end forall
    forall (i=1:size(b,1))
       forall (j=1:size(b,2))
          vb((j-1)*size(b,1)+i) = b(i,j)
       end forall
    end forall
    call vtest( name, va, vb, tol )
  end subroutine mtest

  subroutine test_cross1()
    real :: tmp(3)
    tmp = aa_la_cross( [1.,2.,3.], [2.,4.,8.])
    call vtest( "cross1", [4., -2., 0.], tmp, 0.)
  end subroutine test_cross1

  subroutine test_cross2()
    real :: tmp(3)
    call aa_la_cross_sub( [-1.,2.,-3.], [2.,4.,8.], tmp)
    call vtest( "cross2", [28., 2., -8.], tmp, 0.)
  end subroutine test_cross2

  subroutine test_ssd()
    call test("ssd1", &
         2.0 == aa_la_ssd([0.0, 0.0], [1.0, 1.0]) .and. &
         5.0 == aa_la_ssd([0.0, 0.0], [1.0, 2.0]) .and. &
         5.0 == aa_la_ssd([0.0, 0.0], [2.0, 1.0]) .and. &
         8.0 == aa_la_ssd([0.0, 0.0], [2.0, 2.0]) .and. &
         5.0 == aa_la_ssd([1.0, 0.0], [2.0, 2.0]) &
         )
  end subroutine test_ssd

  subroutine test_colssd()
    real :: a(2,2) = reshape([0.,0., 1.,0.], [2,2])
    real :: b(2,2) = reshape([1.,1., 2.,2.], [2,2])
    real :: c(size(a,2), size(b,2))
    integer :: i,j

    call aa_la_colssd(a,b,c)
    do i=1,size(a,2)
       do j=1,size(a,2)
          call ftest( "colssd", c(i,j),  aa_la_ssd(a(:,i),b(:,j)), 0. )
       end do
    end do
  end subroutine test_colssd

  subroutine test_angle()
    call ftest("angle1", pi/2., aa_la_angle( [1.,0.], [0., 1.] ), .001 )
    call ftest("angle2", pi, aa_la_angle( [1.,0.], [-1., 0.] ), .001 )
    call ftest("angle3", pi/2., aa_la_angle( [2.,0.], [0., 2.] ), .001 )
    call ftest("angle4", pi, aa_la_angle( [2.,0.], [-2., 0.] ), .001 )
  end subroutine test_angle

  subroutine test_norm2()
    call ftest("norm2-1", 0., aa_la_norm2( [0.,0.] ), .000 )
    call ftest("norm2-2", 5., aa_la_norm2( [5.,0.] ), .000 )
    call ftest("norm2-2", 1., aa_la_norm2( [cos(.5),sin(.5)] ), .001 )
  end subroutine test_norm2

  subroutine test_unit()
    integer :: i,j
    real :: theta,a(2),b(2)
    do i=1,360
       do j=1,100
          theta = real(i)*pi/180.
          a(1) = cos(theta)
          a(2) = sin(theta)
          b = real(j)*a
          call vtest("unit", a, b, .001 )
       end do
    end do
  end subroutine test_unit

  subroutine test_proj_orth()
    integer :: i,j
    real :: theta,a(2),p(2),o(2),rp(2),ro(2),s(2)
    do i=1,360
       do j=1,100
          theta = real(i)*pi/180.
          a(1) = real(j)*cos(theta)
          a(2) = real(j)*sin(theta)
          p=a
          o=a
          o(1) = 0.
          p(2) = 0.
          call aa_la_proj_sub(a, [1.,0.], rp)
          call aa_la_orth_sub(a, [1.,0.], ro)
          call vtest("proj_sub", p, rp, .001 )
          call vtest("orth_sub", o, ro, .001 )
          rp = aa_la_proj(a, [1.,0.])
          ro = aa_la_orth(a, [1.,0.])
          call vtest("proj_sub", p, rp, .001 )
          call vtest("orth_sub", o, ro, .001 )
          s = ro+rp
          call vtest("proj_orth", a, s, .001 )
       end do
    end do
  end subroutine test_proj_orth


  subroutine test_std()
    real :: a(10) = [1.,2.,3.,4.,5.,6.,7.,8.,9.,10.]
    real :: x,s
    x = sum(a)/real(size(a))
    s = aa_la_std(a,x)
    call ftest( "std", 3.0277, s, .001 )
  end subroutine test_std

  subroutine test_meancov()
    real :: a(2,3) = reshape( [1.,2., 3.,3., 8.,12.], [2,3] )
    real :: e(2,2),mu(2)
    real :: at(3,2)
    real :: mu_r(2) = [4., 5.666667]
    real :: e_r(2,2) = reshape( [13.,19.5, 19.5,30.333333], [2,2] )

    at = transpose(a)
    call aa_la_colmean(a,mu)
    call vtest("colmean", mu, mu_r, .001 )
    call aa_la_rowmean(at,mu)
    call vtest("rowmean", mu, mu_r, .001 )

    call aa_la_colcov(a,mu,e)
    call mtest("colcov", e, e_r, .001 )
  end subroutine test_meancov

  subroutine test_colfit()
    !! FIXME: write test
  end subroutine test_colfit
end program la_test
