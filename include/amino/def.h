#if 0
/* Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */
#endif

#include "amino/mangle.h"

#if defined AA_TYPE_DOUBLE

#define AA_TYPE double
#define AA_CBLAS_NAME( name ) AA_MANGLE_CBLAS_NAME( d, name )
#define AA_LAPACK_NAME( name ) AA_MANGLE_LAPACK_NAME( d, name )
#define AA_CLA_NAME( name ) AA_MANGLE_CLA_NAME( d, name )
#define AA_LAPACK_PREFIX_STR "D"

#define AA_NAME( prefix, name ) AA_MANGLE_NAME( d, prefix, name )
#define AA_FMOD( prefix, name ) AA_MANGLE_FMOD( d, prefix, name )
#define AA_FMOD_F( prefix, name ) AA_MANGLE_FMOD_F( d, prefix, name )
#define AA_FMOD_C_BEGIN( prefix, name, ... ) AA_MANGLE_FMOD_BIND_C( d, prefix, name, __VA_ARGS__ )
#define AA_FMOD_C_END( prefix, name ) AA_MANGLE_FMOD_C( d, prefix, name )

#elif defined AA_TYPE_FLOAT

#define AA_TYPE float
#define AA_CBLAS_NAME( name ) AA_MANGLE_CBLAS_NAME( s, name )
#define AA_LAPACK_NAME( name ) AA_MANGLE_LAPACK_NAME( s, name )
#define AA_CLA_NAME( name ) AA_MANGLE_CLA_NAME( s, name )
#define AA_LAPACK_PREFIX_STR "S"

#define AA_NAME( prefix, name ) AA_MANGLE_NAME( s, prefix, name )
#define AA_FMOD( prefix, name ) AA_MANGLE_FMOD( s, prefix, name )
#define AA_FMOD_F( prefix, name ) AA_MANGLE_FMOD_F( s, prefix, name )
#define AA_FMOD_C_BEGIN( prefix, name, ... ) AA_MANGLE_FMOD_BIND_C( s, prefix, name, __VA_ARGS__ )
#define AA_FMOD_C_END( prefix, name ) AA_MANGLE_FMOD_C( s, prefix, name )

#elif defined AA_TYPE_INT

#define AA_TYPE int32_t
#define AA_NAME( prefix, name ) AA_MANGLE_NAME( i32, prefix, name )
#define AA_FMOD( prefix, name ) AA_MANGLE_FMOD( i32, prefix, name )
#define AA_FMOD_F( prefix, name ) AA_MANGLE_FMOD_F( i32, prefix, name )
#define AA_FMOD_C_BEGIN( prefix, name, ... ) AA_MANGLE_FMOD_BIND_C( i32, prefix, name, __VA_ARGS__ )
#define AA_FMOD_C_END( prefix, name ) AA_MANGLE_FMOD_C( i32, prefix, name )

#elif defined AA_TYPE_LONG

#define AA_TYPE int64_t
#define AA_NAME( prefix, name ) AA_MANGLE_NAME( i64, prefix, name )
#define AA_FMOD( prefix, name ) AA_MANGLE_FMOD( i64, prefix, name )
#define AA_FMOD_F( prefix, name ) AA_MANGLE_FMOD_F( i64, prefix, name )
#define AA_FMOD_C_BEGIN( prefix, name, ... ) AA_MANGLE_FMOD_BIND_C( i64, prefix, name, __VA_ARGS__ )
#define AA_FMOD_C_END( prefix, name ) AA_MANGLE_FMOD_C( i64, prefix, name )


#elif defined AA_TYPE_FLOGICAL1

#define AA_TYPE bool
#define AA_NAME( prefix, name ) AA_MANGLE_NAME( l8, prefix, name )
#define AA_FMOD( prefix, name ) AA_MANGLE_FMOD( l8, prefix, name )
#define AA_FMOD_F( prefix, name ) AA_MANGLE_FMOD_F( l8, prefix, name )
#define AA_FMOD_C_BEGIN( prefix, name, ... ) AA_MANGLE_FMOD_BIND_C( l8, prefix, name, __VA_ARGS__ )
#define AA_FMOD_C_END( prefix, name ) AA_MANGLE_FMOD_C( l8, prefix, name )

#elif defined AA_TYPE_FLOGICAL4

#define AA_TYPE bool
#define AA_NAME( prefix, name ) AA_MANGLE_NAME( l32, prefix, name )
#define AA_FMOD( prefix, name ) AA_MANGLE_FMOD( l32, prefix, name )
#define AA_FMOD_F( prefix, name ) AA_MANGLE_FMOD_F( l32, prefix, name )
#define AA_FMOD_C_BEGIN( prefix, name, ... ) AA_MANGLE_FMOD_BIND_C( l32, prefix, name, __VA_ARGS__ )
#define AA_FMOD_C_END( prefix, name ) AA_MANGLE_FMOD_C( l32, prefix, name )


#else

#error "Need to define AA_TYPE_?"

#endif


#if 0
/* Declarations for fortran-defined functions.
 */
#endif

#define AA_FDEC( rettype, prefix, name, ... )                           \
    AA_API rettype AA_NAME( prefix, name ) ( __VA_ARGS__ );
