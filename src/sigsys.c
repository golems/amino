/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "amino.h"
#include <cblas.h>

AA_API void aa_lsim_dstep( size_t m, size_t n,
                           const double *restrict A,
                           const double *restrict B,
                           const double *restrict x0, const double *restrict u,
                           double *restrict x1 ) {
    /*-- x1 := A*x0 + B*u -- */
    // 0:  x1 := A*x0
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)m, (int)m,
                 1.0, A, (int)m,
                 x0, 1,
                 0, x1, 1 );

    // 1:  x1 += B*u
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)m, (int)n,
                 1.0, B, (int)m,
                 u, 1,
                 1, x1, 1 );
}

AA_API void aa_lsim_estep( size_t m, size_t n,
                           double dt,
                           const double *restrict A,
                           const double *restrict B,
                           const double *restrict x0,
                           const double *restrict u,
                           double *restrict x1 ) {
    // dx := A*x_0 + B*u
    aa_lsim_dstep( m, n, A, B, x0, u, x1 );

    // \delta x := dx*dt
    cblas_dscal( m, dt, x1, 1 );

    // x_1 := \delta x + x_0
    cblas_daxpy( m, 1.0, x0, 1, x1, 1 );

    // The following method is ~20% slower on an Intel Core2Duo
    /* // x1 := x0 */
    /* cblas_dcopy( m, x0, 1, x1, 1 ); */

    /* // 0:  x1 += dt*A*x0 */
    /* cblas_dgemv( CblasColMajor, CblasNoTrans, */
    /*              (int)m, (int)m, */
    /*              dt, A, (int)m, */
    /*              x0, 1, */
    /*              1, x1, 1 ); */

    /* // 1:  x1 += dt*B*u */
    /* cblas_dgemv( CblasColMajor, CblasNoTrans, */
    /*              (int)m, (int)n, */
    /*              dt, B, (int)m, */
    /*              u, 1, */
    /*              1, x1, 1 ); */

}
