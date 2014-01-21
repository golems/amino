/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
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

#ifndef AMINO_H
#define AMINO_H
/** \file amino.h */
/** \file amino.h
 *
 * \mainpage
 *
 * Amino is package of basic utilites for robotics software.  It
 * includes mathematical and linear algebra routines, memory
 * management, and time-handling.  Design goals are easy integration,
 * efficiency, and simplicity.
 *
 * \section mem Memory Allocation
 *
 * Amino provides two O(1), non-fragmenting allocators.  The region
 * allocator performs variable-sized allocation and frees all objects
 * with a single call.  The pooled allocator allocates and frees
 * fixed-sized blocks. \sa amino/mem.h
 *
 * \section la Linear Algebra
 *
 * Amino provides some light-weight wrappers over BLAS and LAPACK plus
 * a few other convenience routines.  Care is taken to avoid
 * heap-allocation in all calls, making performance suitable for
 * real-time operation.  \sa amino/math.h
 *
 * \section tf Euclidean Transforms
 *
 * Amino provides operations for transformation matrices, quaternions,
 * axis-angle representations, and some of the euler-angle
 * conventions. \sa amino/tf.h
 *
 * \section Legal
 *
 * \author Neil T. Dantam
 * \author Developed at the Georgia Tech Humanoid Robotics Lab
 * \author Under Direction of Professor Mike Stilman
 *
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation.
 * All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   - Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   - Redistributions in binary form must reproduce the above
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

// include everything we'll typically need
#ifdef __cplusplus
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <ctime>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <stack>
#include <string>
#else
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#include <stdarg.h>
#endif //__cplusplus

#include <cblas.h>
#include <time.h>

// for C symbols
#ifdef __cplusplus
#define AA_API extern "C"
#define AA_EXTERN extern "C"
#define AA_RESTRICT
#else
/// calling and name mangling convention for functions
#define AA_API
/// name mangling convention external symbols
#define AA_EXTERN extern
/// Defined restrict keyword based on language flavor
#define AA_RESTRICT restrict
#endif //__cplusplus

/// deprecated thing
#define AA_DEPRECATED  __attribute__ ((deprecated))

/// (int) 1e9
#define AA_IBILLION 1000000000
/// (int) 1e6
#define AA_IMILLION 1000000

/// Stringify X
#define AA_QUOTE(X) (#X)

/// Enum case sym that stores stringified sym in res
#define AA_ENUM_SYM_CASE( res, sym )  case sym: (res) = #sym; break;

/// SWAP a and b
#define AA_SWAP(a,b) \
    {                                                              \
        const typeof(a) aa_$_tmpa = (a);                           \
        const typeof(b) aa_$_tmpb = (b);                           \
        (a) = aa_$_tmpb;                                           \
        (b) = aa_$_tmpa;                                           \
    }


// include our own headers
#include "amino/mem.h"
#include "amino/math.h"
#include "amino/la.h"
#include "amino/tf.h"
#include "amino/lapack.h"
#include "amino/clapack.h"
#include "amino/time.h"
#include "amino/debug.h"
#include "amino/plot.h"
#include "amino/kin.h"
#include "amino/type.h"
#include "amino/endconv.h"
#include "amino/unit.h"
#include "amino/validate.h"
#include "amino/io.h"
#include "amino/cv.h"
#include "amino/const.h"
#include "amino/heap.h"


#endif //AMINO_H
