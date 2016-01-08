/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
#include "amino/opt/lp.h"

#include <coin/ClpSimplex.hpp>

int aa_opt_lp_clp (
    size_t m, size_t n,
    enum aa_opt_rel_type *types,
    const double *A, size_t ldA,
    const double *b,
    const double *c,
    const double *l, const double *u,
    double *x )
{
    ClpSimplex M;
    int rows[m];
    int mi = (int)m;
    int ni = (int)n;
    M.resize(mi,0);

    for( int i = 0; i < mi; i ++ ) rows[i] = i;

    /* b */
    for( int i = 0; i < mi; i ++ ) {
        double rl,ru;
        switch( types[i] ) {
        case AA_OPT_REL_EQ:
            rl = b[i];
            ru = b[i];
            break;
        case AA_OPT_REL_LEQ:
            ru = b[i];
            rl = -DBL_MAX;
            break;
        case AA_OPT_REL_GEQ:
            rl = b[i];
            ru = DBL_MAX;
            break;
        default: return -1;
        }
        M.setRowBounds(i,rl,ru);
    }

    /* A, c, l, u */
    for( size_t j = 0; j < n; j ++ ) {
        M.addColumn( mi, rows,
                     AA_MATCOL(A,ldA,j),
                     l[j], u[j], c[j] );
    }

    M.setOptimizationDirection( -1 );

    /* Solve */
    int r = M.initialSolve();

    // const double * columnPrimal = M.getColSolution();
    // const double * columnLower = M.columnLower();  // Alternatively getColUpper()
    // const double * columnUpper = M.columnUpper();
    // const double * columnObjective = M.objective();

    // printf("x: "); aa_dump_vec(stdout, columnPrimal, n );
    // printf("l: "); aa_dump_vec(stdout, columnLower, n );
    // printf("u: "); aa_dump_vec(stdout, columnUpper, n );
    // printf("c: "); aa_dump_vec(stdout, columnObjective, n );
    // printf("rl: "); aa_dump_vec(stdout, M.rowLower(), m );
    // printf("ru: "); aa_dump_vec(stdout, M.rowUpper(), m );
    // printf("ra: "); aa_dump_vec(stdout, M.getRowActivity(), m );

    /* Result */
    AA_MEM_CPY( x, M.getColSolution(), n );

    return 0;
}
