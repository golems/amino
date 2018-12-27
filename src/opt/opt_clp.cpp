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
#include "amino/opt/opt.h"
#include "opt_internal.h"

#include <coin/ClpSimplex.hpp>

typedef ClpSimplex SolverType;


int s_solve( struct aa_opt_cx *cx, size_t n, double *x )
{
    SolverType * M = static_cast<SolverType*>(cx->data);

    /* Solve */
    int r;


    try {
        r = M->initialSolve();
    } catch (CoinError e) {
        e.print();
        return -1;
    } catch(...) {
        return -1;
    }

    /* Result */
    AA_MEM_CPY( x, M->getColSolution(), n );

    return r;
}
int s_destroy( struct aa_opt_cx *cx )
{
    if( cx ) {
        SolverType * M = static_cast<SolverType*>(cx->data);
        if( M ) {
            delete M;
        }
        free(cx);
    }
    return 0;
}


static int
s_set_direction( struct aa_opt_cx *cx, enum aa_opt_direction dir )
{
    SolverType * M = static_cast<SolverType*>(cx->data);
    switch(dir) {
    case AA_OPT_MAXIMIZE:
        M->setOptimizationDirection( -1 );
        return 0;
    case AA_OPT_MINIMIZE:
        M->setOptimizationDirection( 1 );
        return 0;
    }
    return -1;
}

static int s_set_quad_obj_crs( struct aa_opt_cx *cx,
                               size_t n, const double *Q_values, int *Q_cols, int *Q_row_ptr )
{
    //int ni = (int)n;
    // std::vector<int> Q_rows;
    // for( int i = 0; i < ni; i ++ ) {
    //     int k0 = Q_row_ptr[i];
    //     int k1 = Q_row_ptr[i+1];
    //     for( int k = k0; k < k1; k++ ) {
    //         Q_rows.push_back(i);
    //     }
    // }

    // CoinPackedMatrix Q( false, &Q_rows[0], Q_cols, Q_values, (int)Q_rows.size() );


    // Q.dumpMatrix();

    //SolverType * M = static_cast<SolverType*>(cx->data);
    //int n = M->getNumCols();

    // for( int j = 0; j < ni; j ++ ) {
    //     int i0 = Q_row_ptr[j];
    //     int i1 = Q_row_ptr[j+1];
    //     fprintf(stderr, "%d: %d, %d\n", j, i0, i1 );
    //     if( i1 > i0 ) {
    //         Q.appendRow( i1-i0, Q_cols+i0, Q_values+i0 );
    //     }
    // }



    // M->loadQuadraticObjective(Q);
    // M->loadQuadraticObjective( (int)n, Q_row_ptr, Q_cols, Q_values );

    // return 0;
    return -1;
}

static int s_set_type( struct aa_opt_cx *cx, size_t i, enum aa_opt_type type ) {
    return -1;
}


static int
s_set_obj( struct aa_opt_cx *cx, size_t n, const double * c)
{
    SolverType * M = static_cast<SolverType*>(cx->data);
    int ni = (int) n;
    for( int i = 0; i < ni; i ++ ) {
        M->setObjectiveCoefficient( i, c[i] );
    }
}

static int
s_set_bnd( struct aa_opt_cx *cx, size_t n,
           const double * x_min, const double *x_max)
{
    SolverType * M = static_cast<SolverType*>(cx->data);
    int ni = (int) n;
    for( int i = 0; i < ni; i ++ ) {
        double lo = x_min[i];
        double hi = x_max[i];
        M->setColumnBounds( i,
                            aa_isfinite(lo) ? lo : -DBL_MAX,
                            aa_isfinite(hi) ? hi : DBL_MAX );
    }
}

static int
s_set_cstr_gm( struct aa_opt_cx *cx,
               size_t m, size_t n,
               const double *A, size_t lda )
{
    SolverType * M = static_cast<SolverType*>(cx->data);
    int mi = (int) m;
    int ni = (int) n;
    for( int j = 0; j < ni; j ++ ) {
        for( int i = 0; i < mi; i ++ ) {
            M->modifyCoefficient( i, j, AA_MATREF(A,lda, i,j) );
        }
    }
}

static int
s_set_cstr_bnd( struct aa_opt_cx *cx, size_t m,
                const double * b_min, const double *b_max )
{
    SolverType * M = static_cast<SolverType*>(cx->data);
    int mi = (int) m;
    for( int i = 0; i < mi; i ++ ) {
        double lo = b_min[i];
        double hi = b_max[i];
        M->setRowBounds(i,
                        aa_isfinite(lo) ? lo : -DBL_MAX,
                        aa_isfinite(hi) ? hi : DBL_MAX );
    }
}

static struct aa_opt_vtab s_vtab = {
    .solve = s_solve,
    .destroy = s_destroy,
    .set_direction = s_set_direction,
    .set_quad_obj_crs = s_set_quad_obj_crs,
    .set_type = s_set_type,
    .set_obj = s_set_obj,
    .set_bnd = s_set_bnd,
    .set_cstr_bnd = s_set_cstr_bnd,
    .set_cstr_gm = s_set_cstr_gm
};


// AA_API struct aa_opt_cx* aa_opt_clp_gmcreate (
//     size_t m, size_t n,
//     const double *A, size_t ldA,
//     const double *b_lower, const double *b_upper,
//     const double *c,
//     const double *x_lower, const double *x_upper
//     )
// {


//     ClpSimplex *pM = new ClpSimplex();
//     ClpSimplex &M = *pM;
//     pM->setLogLevel(0); // shut up

//     int rows[m];
//     int mi = (int)m;
//     int ni = (int)n;
//     M.resize(mi,0);

//     for( int i = 0; i < mi; i ++ ) rows[i] = i;

//     /* A, c, l, u */
//     for( size_t j = 0; j < n; j ++ ) {
//         M.addColumn( mi, rows,
//                      AA_MATCOL(A,ldA,j),
//                      x_lower[j], x_upper[j], c[j] );
//     }

//     /* b */
//     for( int i = 0; i < mi; i ++ ) {
//         M.setRowBounds(i,b_lower[i],b_upper[i]);
//     }


//     M.setOptimizationDirection( -1 );


//     return cx_finish( &s_vtab, pM );
// }

AA_API struct aa_opt_cx* aa_opt_clp_gmcreate (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper
    )
{
    ClpSimplex *pM = new ClpSimplex();
    ClpSimplex &M = *pM;

    struct aa_opt_cx *cx = cx_finish( &s_vtab, pM );

    pM->setLogLevel(0); // shut up

    int mi = (int)m;
    int ni = (int)n;
    M.resize(mi,0);


    /* A, c, l, u */
    int rows[m];
    for( int i = 0; i < mi; i ++ ) rows[i] = i;
    // TODO: is there a better way to initialize?
    for( size_t j = 0; j < n; j ++ ) {
        M.addColumn( mi, rows,
                     AA_MATCOL(A,ldA,j),
                     x_lower[j], x_upper[j], c[j] );
    }
    // s_set_cstr_gm( cx, m, n, A, ldA );


    s_set_bnd( cx, n, x_lower, x_upper );
    s_set_obj( cx, n, c );

    s_set_cstr_bnd( cx, m, b_lower, b_upper );


    M.setOptimizationDirection( -1 );


    return cx;
}


AA_API struct aa_opt_cx* aa_opt_clp_crscreate (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper )
{
    ClpSimplex *pM = new ClpSimplex();
    ClpSimplex &M = *pM;
    int rows[m];
    int mi = (int)m;
    int ni = (int)n;
    M.resize(0,ni);

    /* A, b */
    for( int i = 0; i < mi; i ++ ) {
        int start = A_row_ptr[i];
        int end = A_row_ptr[i + 1];
        M.addRow( end - start,
                  A_cols+start,
                  A_values+start,
                  b_lower[i],
                  b_upper[i] );
    }

    /* c, xl, xu */
    M.chgColumnLower(x_lower);
    M.chgColumnUpper(x_upper);
    M.chgObjCoefficients(c);

    /* Solve */
    M.setOptimizationDirection( -1 );


    return cx_finish( &s_vtab, pM );
}




int aa_opt_lp_clp (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{

    struct aa_opt_cx *cx = aa_opt_clp_gmcreate( m, n,
                                                A, ldA,
                                                b_lower, b_upper,
                                                c,
                                                x_lower, x_upper );
    int r = aa_opt_solve( cx, n, x);
    aa_opt_destroy(cx);

    return r;
}

AA_API int aa_opt_lp_crs_clp (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper,
    double *x )
{
    struct aa_opt_cx *cx = aa_opt_clp_crscreate( m, n,
                                                 A_values, A_cols, A_row_ptr,
                                                 b_lower, b_upper,
                                                 c,
                                                 x_lower, x_upper );
    int r = aa_opt_solve( cx, n, x);
    aa_opt_destroy(cx);

    return r;
}
