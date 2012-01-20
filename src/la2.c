/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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

/**
 * \file amino/la2.c
 */

#include "amino.h"

#define AA_LA_DEF( TYPE, prefix )                                       \
    void AA_LA_NAME(transpose, prefix) ( size_t m, size_t n,            \
                                         const TYPE *A, size_t lda,     \
                                         TYPE *B, size_t ldb ) {        \
        for( size_t i=0, ia=0; i < n; i++, ia+=lda ) {                  \
            for( size_t j=0, ib=0; j < m; j++, ib+=ldb ) {              \
                B[ib+i] = A[ia+j];                                      \
            }                                                           \
        }                                                               \
    }                                                                   \
    void AA_LA_NAME(cmean, prefix)                                      \
    ( size_t m, size_t n,                                               \
      const TYPE *A, size_t lda,                                        \
      TYPE *x)                                                          \
    {                                                                   \
        memset( x, 0, sizeof(x[0])*m );                                 \
        for( size_t i=0, j=0; i < n; i++, j+=lda ) {                    \
            AA_CBLAS_NAME(axpy, prefix)( (int)m, 1.0, A+j, (int)1,      \
                                         x, 1 );                        \
        }                                                               \
        AA_CBLAS_NAME(scal, prefix)((int)m, ((TYPE)1.0)/(TYPE)n,        \
                                    x, 1);                              \
    }                                                                   \

AA_LA_DEF( double, d )
AA_LA_DEF( float, s )
