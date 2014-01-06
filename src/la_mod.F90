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


!> \file la_mod.f90
!! \brief Linear Algebra Module
!! \author Neil T. Dantam
!!
!! This file is preprocessed to produce functions for various types.
!! Since gfortran's proprocessor doesn't support token pasting, we
!! need to explicitly preprocess the file with the C preprocessor.  It
!! should be sufficient, then, to include the preprocessed source file
!! in the tarball to sidestep preprocessing compatibilities with,
!! ie. ifort.
!!

!! To bind the fortran definitions to C, we need to touch three things:
!!   - The fortran function definition
!!   - A fortran wrapper function using the C calling convention
!!   - A C header file to declare the wrapper function to cc

#include "amino/mangle.h"

module amino_la
  use ISO_C_BINDING
  implicit none

  !> Linear Least Squares
  interface aa_la_lls
     !> Double float linear least squares
     subroutine aa_la_d_lls(m,n,p,A,lda,b,ldb,x,ldx) &
          bind(C,name="aa_la_d_lls")
       use ISO_C_BINDING
       integer(c_size_t), intent(in), value :: m,n,p,lda,ldb,ldx
       real(c_double), intent(in) :: A(lda,n), b(ldb,p)
       real(c_double), intent(out) :: x(ldx,p)
     end subroutine aa_la_d_lls
     !> Single float linear least squares
     subroutine aa_la_s_lls(m,n,p,A,lda,b,ldb,x,ldx) &
          bind(C,name="aa_la_s_lls")
       use ISO_C_BINDING
       integer(c_size_t), intent(in), value :: m,n,p,lda,ldb,ldx
       real(c_float), intent(in) :: A(lda,n), b(ldb,p)
       real(c_float), intent(out) :: x(ldx,p)
     end subroutine aa_la_s_lls
  end interface

  !> Singular Value Decomposition
  interface aa_la_svd
     !> Double float SVD
     subroutine aa_la_d_svd(m,n,A,lda,U,ldu,S,Vt,ldvt) &
          bind(C,name="aa_la_d_svd")
       use ISO_C_BINDING
       integer(c_size_t), intent(in), value :: m,n,lda,ldu,ldvt
       real(c_double), intent(in) :: A(lda,n)
       real(c_double), intent(out) :: U(ldu,m), S(min(m,n)), Vt(ldvt,n)
     end subroutine aa_la_d_svd
     !> Single float SVD
     subroutine aa_la_s_svd(m,n,A,lda,U,ldu,S,Vt,ldvt) &
          bind(C,name="aa_la_s_svd")
       use ISO_C_BINDING
       integer(c_size_t), intent(in), value :: m,n,lda,ldu,ldvt
       real(c_float), intent(in) :: A(lda,n)
       real(c_float), intent(out) :: U(ldu,m), S(min(m,n)), Vt(ldvt,n)
     end subroutine aa_la_s_svd
  end interface

  !! interface to inversion function (via lapack) in amino
  Interface
     Function aa_la_inv( n, A ) result(info)
       use ISO_C_BINDING
       integer(C_SIZE_T), intent(in), value :: n
       real(C_DOUBLE), intent(inout) :: A(n,n)
       integer :: info
     End Function aa_la_inv
  End Interface

  Interface
     Function aa_la_care_laub( m, n, p, A, B, C, X ) result(info)
       use ISO_C_BINDING
       integer(C_SIZE_T), intent(in), value :: m, n, p
       real(C_DOUBLE), intent(in) :: A(m,m), B(m,n), C(p,m)
       real(C_DOUBLE), intent(out) :: X(m,m)
       integer :: info
     End Function aa_la_care_laub
  End Interface

  !> Cross product
  interface aa_la_cross_sub
     module procedure &
          AA_MANGLE_FIFACE(la,cross_sub)
  end interface

  !> Cross product
  interface aa_la_cross
     module procedure &
          AA_MANGLE_FIFACE(la,cross_fun)
  end interface

  !> Sum Squared Differences
  interface aa_la_ssd
     module procedure AA_MANGLE_FIFACE(la,ssd)
  end interface

  !> Sum Squared Differences of matrix columns
  interface aa_la_colssd
     module procedure AA_MANGLE_FIFACE(la,colssd)
  end interface

  !> Angle between vectors
  interface aa_la_angle
     module procedure AA_MANGLE_FIFACE(la,angle)
  end interface

  !> Norm-2 of a vector
  interface aa_la_norm2
     module procedure AA_MANGLE_FIFACE(la,norm2)
  end interface

  !> Make a unit vector
  interface aa_la_unit_sub
     module procedure AA_MANGLE_FIFACE(la,unit_sub1), &
          AA_MANGLE_FIFACE(la,unit_sub2)
  end interface

  !> Make a unit vector
  interface aa_la_unit
     module procedure AA_MANGLE_FIFACE(la,unit_fun)
  end interface

  !> Linear Interpolation of vectors
  interface aa_la_lerp
     module procedure AA_MANGLE_FIFACE(la,lerp)
  end interface


  interface aa_la_3spline_param
     module procedure AA_MANGLE_FIFACE(la,3spline_param)
  end interface

  interface aa_la_3spline
     module procedure AA_MANGLE_FIFACE(la,3spline)
  end interface

  interface aa_la_5spline_param
     module procedure AA_MANGLE_FIFACE(la,5spline_param)
  end interface

  interface aa_la_5spline
     module procedure AA_MANGLE_FIFACE(la,5spline)
  end interface


  !> Vector projection
  interface aa_la_proj_sub
     module procedure AA_MANGLE_FIFACE(la,proj_sub)
  end interface

  !> Vector projection
  interface aa_la_proj
     module procedure AA_MANGLE_FIFACE(la,proj_fun)
  end interface

  !> Vector orthogonal projection
  interface aa_la_orth_sub
     module procedure AA_MANGLE_FIFACE(la,orth_sub)
  end interface

  !> Vector orthogonal projection
  interface aa_la_orth
     module procedure AA_MANGLE_FIFACE(la,orth_fun)
  end interface

  !> Standard deviation
  interface aa_la_std
     module procedure AA_MANGLE_FIFACE(la,vecstd)
  end interface

  !> Mean of matrix columns
  interface aa_la_colmean
     module procedure AA_MANGLE_FIFACE(la,colmean)
  end interface

  !> Mean of matrix rows
  interface aa_la_rowmean
     module procedure AA_MANGLE_FIFACE(la,rowmean)
  end interface

  !> Covariance of matrix columns
  interface aa_la_colcov
     module procedure AA_MANGLE_FIFACE(la,colcov)
  end interface

  !> Fit hyperplane to matrix columns
  interface aa_la_colfit
     module procedure AA_MANGLE_FIFACE(la,colfit)
  end interface

  !> Solve assignment problem
  interface aa_la_assign_hungarian
     module procedure AA_MANGLE_FIFACE(la,assign_hungarian)
  end interface

contains

  !! Preprocessor type generics hack

#define AA_TYPE_DOUBLE
#include "la_implf.F90"

#define AA_TYPE_FLOAT
#include "la_implf.F90"

end module amino_la
