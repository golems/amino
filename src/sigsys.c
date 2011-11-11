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
    cblas_dscal( (int)m, dt, x1, 1 );

    // x_1 := \delta x + x_0
    cblas_daxpy( (int)m, 1.0, x0, 1, x1, 1 );

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



void aa_rk1_step( size_t n, double dt,
                  const double *restrict dx,
                  const double *restrict x0,
                  double *restrict x1 ) {
    // x1 = x0 + dt*dx
    cblas_dcopy( (int)n, x0, 1, x1, 1 );
    cblas_daxpy( (int)n, dt, dx, 1, x1, 1 );
}

void aa_rk4_step( size_t n, aa_sys_fun sys, const void *cx,
                  double t0, double dt,
                  const double *restrict x0, double *restrict x1 ) {
    double k1[n], k2[n], k3[n], k4[n];

    // k1
    sys( cx, t0, x0, k1 );

    // k2
    aa_rk1_step( n, dt/2, k1, x0, x1 );
    sys( cx, t0+dt/2, x1, k2 );

    // k3
    aa_rk1_step( n, dt/2, k2, x0, x1 );
    sys( cx, t0+dt/2, x1, k3 );

    // k4
    aa_rk1_step( n, dt, k3, x0, x1 );
    sys( cx, t0+dt, x1, k4 );

    for( size_t i = 0; i < n; i ++ ) {
        x1[i] = x0[i] + (dt/6) * (k1[i] + k4[i] + 2 * (k2[i]+k3[i]));
    }

    /* cblas_dcopy( (int) n, x0, 1, x1, 1 ); */
    /* cblas_daxpy( n, 1, k3, 1, k2, 1 ); */
    /* cblas_daxpy( n, 2, k2, 1, k4, 1 ); */
    /* cblas_daxpy( n, 1, k4, 1, k1, 1 ); */
    /* cblas_daxpy( n, dt/6, k1, 1, x1, 1 ); */
}

void aa_sys_affine( const aa_sys_affine_t *cx,
                    double t, const double *restrict x,
                    double *restrict dx ) {
    (void)t;
    // x1 = Ax + D
    cblas_dcopy( (int) cx->n, cx->D, 1, dx, 1 );
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)cx->n, (int)cx->n,
                 1, cx->A, (int)cx->n,
                 x, 1,
                 1, dx, 1 );

}
