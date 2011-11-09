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
    for( size_t i = 0; i < m; i ++ ) {
        for( size_t j = 0; j < m; j ++ ) {
            // upper left gets A
            AA_MATREF(H, 2*m, i, j) = AA_MATREF(A, m, i, j);
            // lower right gets -A^T
            AA_MATREF(H, 2*m, m+i, m+j) = -AA_MATREF(A, m, j, i);
        }
    }
    // copy B
    if( n == m ) {
        for( size_t i = 0; i < m; i ++ ) {
            for( size_t j = 0; j < m; j ++ ) {
                AA_MATREF(H, 2*m, i, m+j) = -AA_MATREF(B, m, i, j);
            }
        }
    } else {
        cblas_dgemm( CblasColMajor, CblasNoTrans, CblasTrans,
                     (int)m, (int)m, (int)n, -1, B, (int)m, B, (int)m,
                     0, &AA_MATREF(H, 2*m, 0, m), 2*(int)m );
    }
    // copy C
    if( p == m ) {
        for( size_t i = 0; i < m; i ++ ) {
            for( size_t j = 0; j < m; j ++ ) {
                AA_MATREF(H, 2*m, i+m, j) = -AA_MATREF(C, m, i, j);
            }
        }
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

    double u11t[m*m];
    double *u21t = X;  // u21 overwritten with X by lapack solver
    {
        // shcur in lapack: dgees, (will balance the array itself)
        double vs[2*m*2*m];
        {
            // build hamiltonian
            double H[2*m*2*m];
            aa_la_care_laub_hamiltonian(m,n,p,A,B,C,H);

            int info;
            int lwork = -1;
            while(1) {
                int sdim;
                int bwork[2*m];
                double wr[2*m];
                double wi[2*m];
                double work [ lwork < 0 ? 1 : lwork ];
                dgees_("V", "S",
                       aa_la_care_laub_select,
                       &mi2, H, &mi2, &sdim, wr, wi,
                       vs, &mi2, work, &lwork, bwork, &info );
                if( lwork > 0 ) break;
                // got work array size
                assert( -1 == lwork && sizeof(work) == sizeof(double) );
                lwork = (int)work[0];
            };
        }

        // compute the resulting thing
        // vs = scal * vs

        // get u11
        for( size_t j = 0; j < m; j ++) {
            for( size_t i = 0; i < m; i ++) {
                //u11
                AA_MATREF(u11t, m, j, i) =
                    AA_MATREF(vs, 2*m, i, j);
                //u21
                AA_MATREF(u21t, m, j, i) =
                    AA_MATREF(vs, 2*m, i+m, j);
            }
        }
    }

    // solve the least squares problem
    // X * u11 = u21, but X is symmetric, so
    // u11^T X = u21^T
    // X = u21 * u11^(-1)
    {
        int lwork = -1;
        int info;
        while(1) {
            double work [ lwork < 0 ? 1 : lwork ];
            dgels_( "N", &mi, &mi, &mi, u11t, &mi, u21t, &mi, work, &lwork,
                    &info );
            if(lwork > 0 ) break;
            assert( -1 == lwork && sizeof(work) == sizeof(double) );
            lwork = (int)work[0];
        };
    }

    return 0;
}
