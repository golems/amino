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


!> \file mod_la.F90
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

module amino_la
  use ISO_C_BINDING
  implicit none

  interface
     !! bind some C functions for fortran
     function aa_la_ssd(n, m, y) result(s)
       use ISO_C_BINDING
       integer(C_SIZE_T), intent(in) :: n
       real(C_DOUBLE), intent(in), dimension(n) :: m, y
       real(C_DOUBLE) :: s
     end function aa_la_ssd

  end interface

contains

  !! Preprocessor type generics hack

#define AA_LA_TYPE_DOUBLE
#include "la_implf.F90"

#define AA_LA_TYPE_FLOAT
#include "la_implf.F90"

end module amino_la
