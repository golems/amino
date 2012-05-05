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


#include "amino/def.F90"


subroutine AA_FMOD(mem,array1_4)(n,a)
  integer(4), intent(in) :: n
  AA_FTYPE(AA_FSIZE), intent(out), pointer :: a(:)
  integer(4) :: s(1)
  type(c_ptr) :: p
  p = aa_mem_region_local_alloc_c(int(n*AA_FSIZE,c_size_t))
  s(1) = n
  call c_f_pointer( p, a, s )
end subroutine AA_FMOD(mem,array1_4)

subroutine AA_FMOD(mem,array2_4)(m,n,a)
  integer(4), intent(in) :: m,n
  AA_FTYPE(AA_FSIZE), intent(out), pointer :: a(:,:)
  type(c_ptr) :: p
  integer(4) :: s(2)
  p = aa_mem_region_local_alloc_c(int(m*n*AA_FSIZE,c_size_t))
  s(1) = m
  s(2) = n
  call c_f_pointer( p, a, s )
end subroutine AA_FMOD(mem,array2_4)

subroutine AA_FMOD(mem,array1_8)(n,a)
  integer(8), intent(in) :: n
  AA_FTYPE(AA_FSIZE), intent(out), pointer :: a(:)
  type(c_ptr) :: p
  integer(8) :: s(1)
  p = aa_mem_region_local_alloc_c(int(n*int(AA_FSIZE,8),c_size_t))
  s(1) = n
  call c_f_pointer( p, a, s )
end subroutine AA_FMOD(mem,array1_8)

subroutine AA_FMOD(mem,array2_8)(m,n,a)
  integer(8), intent(in) :: m,n
  AA_FTYPE(AA_FSIZE), intent(out), pointer :: a(:,:)
  type(c_ptr) :: p
  integer(8) :: s(2)
  p = aa_mem_region_local_alloc_c(int(m*n*int(AA_FSIZE,8),c_size_t))
  s(1) = m
  s(2) = n
  call c_f_pointer( p, a, s )
end subroutine AA_FMOD(mem,array2_8)

subroutine AA_FMOD(mem,pop1)(a)
  AA_FTYPE(AA_FSIZE), intent(inout), pointer :: a(:)
  call aa_mem_region_local_pop_c( c_loc(a(1)) )
  nullify(a)
end subroutine AA_FMOD(mem,pop1)

subroutine AA_FMOD(mem,pop2)(a)
  AA_FTYPE(AA_FSIZE), intent(inout), pointer :: a(:,:)
  call aa_mem_region_local_pop_c( c_loc(a(1,1)) )
  nullify(a)
end subroutine AA_FMOD(mem,pop2)

#include "amino/undef.F90"
