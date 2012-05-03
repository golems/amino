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

#include "amino/mangle.h"

module amino_mem
  use ISO_C_BINDING
  implicit none

  interface
     function aa_mem_region_local_alloc_c( n )  result(p) &
          bind(C,name="aa_mem_region_local_alloc")
       use ISO_C_BINDING
       integer(c_size_t), intent(in), value :: n
       type(c_ptr) :: p
     end function aa_mem_region_local_alloc_c
  end interface

  interface
     subroutine aa_mem_region_local_pop_c( p ) &
          bind(C,name="aa_mem_region_local_pop")
       use ISO_C_BINDING
       type(c_ptr), intent(in), value :: p
     end subroutine aa_mem_region_local_pop_c
  end interface

  interface aa_mem_region_alloc
     module procedure &
          AA_MANGLE_FMOD(d,mem,array1_4), &
          AA_MANGLE_FMOD(s,mem,array1_4), &
          AA_MANGLE_FMOD(i32,mem,array1_4), &
          AA_MANGLE_FMOD(i64,mem,array1_4), &
          AA_MANGLE_FMOD(l8,mem,array1_4), &
          AA_MANGLE_FMOD(l32,mem,array1_4), &
          AA_MANGLE_FMOD(d,mem,array2_4), &
          AA_MANGLE_FMOD(s,mem,array2_4), &
          AA_MANGLE_FMOD(i64,mem,array2_4), &
          AA_MANGLE_FMOD(i32,mem,array2_4), &
          AA_MANGLE_FMOD(l8,mem,array2_4), &
          AA_MANGLE_FMOD(l32,mem,array2_4), &
          AA_MANGLE_FMOD(d,mem,array1_8), &
          AA_MANGLE_FMOD(s,mem,array1_8), &
          AA_MANGLE_FMOD(i32,mem,array1_8), &
          AA_MANGLE_FMOD(i64,mem,array1_8), &
          AA_MANGLE_FMOD(l8,mem,array1_8), &
          AA_MANGLE_FMOD(l32,mem,array1_8), &
          AA_MANGLE_FMOD(d,mem,array2_8), &
          AA_MANGLE_FMOD(s,mem,array2_8), &
          AA_MANGLE_FMOD(i64,mem,array2_8), &
          AA_MANGLE_FMOD(i32,mem,array2_8), &
          AA_MANGLE_FMOD(l8,mem,array2_8), &
          AA_MANGLE_FMOD(l32,mem,array2_8)
  end interface

  interface aa_mem_region_pop
     module procedure &
          AA_MANGLE_FMOD(d,mem,pop1), &
          AA_MANGLE_FMOD(s,mem,pop1), &
          AA_MANGLE_FMOD(i32,mem,pop1), &
          AA_MANGLE_FMOD(i64,mem,pop1), &
          AA_MANGLE_FMOD(l8,mem,pop1), &
          AA_MANGLE_FMOD(l32,mem,pop1), &
          AA_MANGLE_FMOD(d,mem,pop2), &
          AA_MANGLE_FMOD(s,mem,pop2), &
          AA_MANGLE_FMOD(i32,mem,pop2), &
          AA_MANGLE_FMOD(i64,mem,pop2), &
          AA_MANGLE_FMOD(l8,mem,pop2), &
          AA_MANGLE_FMOD(l32,mem,pop2)
  end interface

contains

#define AA_TYPE_DOUBLE
#include "mem_impl.F90"

#define AA_TYPE_FLOAT
#include "mem_impl.F90"

#define AA_TYPE_INT
#include "mem_impl.F90"

#define AA_TYPE_LONG
#include "mem_impl.F90"

#define AA_TYPE_FLOGICAL1
#include "mem_impl.F90"

#define AA_TYPE_FLOGICAL4
#include "mem_impl.F90"

end module amino_mem
