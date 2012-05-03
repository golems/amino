/* -*- mode: C; c-basic-offset: 4 -*- */
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

#include "amino.h"


static int aa_la_care_laub_select( const double *real, const double *complex ) {
    (void)complex;
    return *real < 0;
}

static void aa_la_care_laub_hamiltonian( size_t m, size_t n, size_t p,
                                         const double *restrict A,
                                         const double *restrict B,
                                         const double *restrict C,
                                         double *restrict H ) {
    // copy in A
    // upper left gets A
    aa_cla_dlacpy( 0, (int)m, (int)m, A, (int)m, H, 2*(int)m );
    // lower right gets -A^T
    aa_la_d_transpose( m, m, A, m,
                       &AA_MATREF(H, 2*m, m, m), 2*m );
    aa_cla_dlascl( 'G', 0, 0,
                  -1.0, 1.0,
                   (int)m, (int)m,
                   &AA_MATREF(H, 2*m, m, m), 2*(int)m );

    // copy B
    if( n == m ) {
        aa_cla_dlacpy( 0, (int)m, (int)m, B, (int)m,
                       &AA_MATREF(H, 2*m, 0, m), 2*(int)m );
        aa_cla_dlascl( 'G', 0, 0,
                       -1.0, 1.0,
                       (int)m, (int)m,
                       &AA_MATREF(H, 2*m, 0, m), 2*(int)m );
    } else {
        cblas_dgemm( CblasColMajor, CblasNoTrans, CblasTrans,
                     (int)m, (int)m, (int)n, -1, B, (int)m, B, (int)m,
                     0, &AA_MATREF(H, 2*m, 0, m), 2*(int)m );
    }

    // copy C
    if( p == m ) {
        aa_cla_dlacpy( 0, (int)m, (int)m, C, (int)m,
                      &AA_MATREF(H, 2*m, m, 0), 2*(int)m );
        aa_cla_dlascl( 'G', 0, 0,
                       -1.0, 1.0,
                       (int)m, (int)m,
                       &AA_MATREF(H, 2*m, m, 0), 2*(int)m );
    } else {
        cblas_dgemm( CblasColMajor, CblasTrans, CblasNoTrans,
                     (int)m, (int)m, (int)p, -1, C, (int)p, C, (int)p,
                     0, &AA_MATREF(H, 2*m, m, 0), 2*(int)m );
    }

}

AA_API int aa_la_care_laub( size_t m, size_t n, size_t p,
                            const double *restrict A,
                            const double *restrict B,
                            const double *restrict C,
                            double *restrict X ) {
    /* See Laub, Alan. "A Schur Method for Solving Algebraic Riccati
     *  Equations".  IEEE Transactions on Automatic Control. Dec 1979.
     */
    // A: m*m
    // B: m*n
    // C: p*m
    int mi = (int)m;
    int mi2 = 2*mi;

    double *vs = (double*)
        aa_mem_region_local_alloc(sizeof(double)*2*m*2*m);

    {
        // shcur in lapack: dgees, (will balance the array itself)
        // build hamiltonian

        double *W = (double*)
            aa_mem_region_local_alloc( sizeof(double)*
                                   (2*m*2*m + 2*m + 2*m) );
        double *H = W;
        double *wi = H + 2*m*2*m;
        double *wr = wi + 2*m;
        int *bwork = (int*)
            aa_mem_region_local_alloc( sizeof(int)* 2*m );

        aa_la_care_laub_hamiltonian(m,n,p,A,B,C,H);

        int info;
        int lwork = -1;
        while(1) {
            int sdim;
            double *work = (double*)
                aa_mem_region_local_tmpalloc( sizeof(double)*
                                          (size_t)(lwork < 0 ? 1 : lwork) );
            dgees_("V", "S",
                   aa_la_care_laub_select,
                   &mi2, H, &mi2, &sdim, wr, wi,
                   vs, &mi2, work, &lwork, bwork, &info );
            if( lwork > 0 ) break;
            // got work array size
            assert( -1 == lwork );
            lwork = (int)work[0];
        };
        aa_mem_region_local_pop(W);
    }

    // solve the least squares problem
    // X * u11 = u21, but X is symmetric, so
    // u11^T X^T = u11^T X = u21^T

    // copy u21' to X for dgels
    aa_la_d_transpose( m, m,
                       &AA_MATREF(vs, 2*m, m, 0), 2*m, X, m );
    {
        int lwork = -1;
        int info;
        while(1) {
            double *work = (double*)
                aa_mem_region_local_tmpalloc( sizeof(double)*
                                          (size_t)(lwork < 0 ? 1 : lwork) );
            dgels_( "T", &mi, &mi, &mi,
                    vs, &mi2, /* u11' */
                    X, &mi,   /* u21' */
                    work, &lwork,
                    &info );
            if(lwork > 0) break;
            assert( -1 == lwork );
            lwork = (int)work[0];
        };
    }

    aa_mem_region_local_pop(vs);

    return 0;
}
