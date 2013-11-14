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

// uncomment to check that local allocs actually get freed
// #define AA_ALLOC_STACK_MAX 0

#include "amino.h"


/*--- Scalar Ops ---*/


double aa_la_min( size_t n, const double *x ) {
    double v = HUGE_VAL;
    for( size_t i = 0; i < n; i++ )
        v = AA_MIN(v,x[i]);
    return v;
}

double aa_la_max( size_t n, const double *x ) {
    double v = -HUGE_VAL;
    for( size_t i = 0; i < n; i++ )
        v = AA_MAX(v,x[i]);
    return v;
}

double aa_la_dot( size_t n, const double *x, const double *y ) {
    double a = 0;
    for( size_t i = 0; i < n; i ++ )
        a += x[i]*y[i];
    return a;
}

double aa_la_norm( size_t n, const double *x ) {
    return sqrt( aa_la_dot( n, x, x ) );
}

double aa_la_ssd( size_t n, const double *x, const double *y ) {
    return aa_la_d_ssd(n,x,1,y,1);
}

double aa_la_dist( size_t n, const double *x, const double *y ) {
    return sqrt( aa_la_ssd(n,x,y) );
}

/*--- Vector Ops ---*/

void aa_la_sinc( size_t n, double alpha, double *x  ) {
    for( size_t i = 0; i < n; i ++ )
        x[i] += alpha;
}

void aa_la_vinc( size_t n, const double *x, double *y  ) {
    for( size_t i = 0; i < n; i ++ )
        y[i] += x[i];

}

void aa_la_axpy( size_t n, double alpha, const double *x, double *y  ) {
    for( size_t i = 0; i < n; i ++ )
        y[i] += alpha*x[i];
}

AA_API void aa_la_axpy3( size_t n, double alpha,
                         const double *x, const double *y, double *z ) {
    for( size_t i = 0; i < n; i ++ )
        z[i] = alpha*x[i] + y[i];
}

void aa_la_scal( size_t n, double alpha, double *x  ) {
    for( size_t i = 0; i < n; i ++ )
        x[i] *= alpha;
}

void aa_la_sadd( size_t n, double alpha, const double *x, double *r ) {
    for( size_t i = 0; i < n; i ++ )
        r[i] = alpha + x[i];
}

void aa_la_smul( size_t n, double alpha, const double *x, double *r ) {
    for( size_t i = 0; i < n; i ++ )
        r[i] = alpha * x[i];
}

void aa_la_ssub( size_t n, double alpha, const double *x, double *r ) {
    for( size_t i = 0; i < n; i ++ )
        r[i] = alpha - x[i];
}

void aa_la_sdiv( size_t n, double alpha, const double *x, double *r ) {
    for( size_t i = 0; i < n; i ++ )
        r[i] = alpha / x[i];
}

void aa_la_vadd( size_t n, const double *x, const double *y, double *r ) {
    for( size_t i = 0; i < n; i ++ ) {
        r[i] = x[i] + y[i];
    }
}

void aa_la_vsub( size_t n, const double *x, const double *y, double *r ){
    for( size_t i = 0; i < n; i ++ ) {
        r[i] = x[i] - y[i];
    }
}

void aa_la_vmul( size_t n, const double *x, const double *y, double *r ){
    for( size_t i = 0; i < n; i ++ ) {
        r[i] = x[i] * y[i];
    }
}

void aa_la_vdiv( size_t n, const double *x, const double *y, double *r ){
    for( size_t i = 0; i < n; i ++ ) {
        r[i] = x[i] / y[i];
    }
}

void aa_la_cross( const double a[3], const double b[3], double c[3] ) {
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

void aa_la_normalize( size_t n, double *x ) {
    aa_la_scal( n, 1.0/aa_la_norm(n,x), x );
}

/*--- Matrix Ops --- */

AA_API void aa_la_transpose2( size_t m, size_t n, const double *A, double *At  ) {
    for( size_t i = 0; i < m; i ++ ) {
        for( size_t j = 0; j < n; j++ ) {
            AA_MATREF(At,n,j,i) = AA_MATREF(A,m,i,j);
        }
    }
}


AA_API void aa_la_transpose( size_t n, double *A  ) {
    for( size_t i = 0; i < n; i ++ ) {
        for( size_t j = i+1; j < n; j++ ) {
            double t = AA_MATREF(A,n,i,j);
            AA_MATREF(A,n,i,j) = AA_MATREF(A,n,j,i);
            AA_MATREF(A,n,j,i) = t;
        }
    }
}

double aa_la_trace( size_t n, const double *A ) {
    return cblas_dasum( (int)n, A, (int)(n+1) );
}

double aa_la_wdot( size_t n,
                   const double *x, const double *A, const double *y ) {

    double a = 0;
    for( size_t i = 0; i < n; i++ ) {
        a += y[i] * cblas_ddot( (int)n, &A[i*n], 1, x, 1 );
    }
    return a;
}

int aa_la_inv( size_t n, double *A ) {
    const int mi = (int) n;
    const int ni = (int) n;
    int info;

    int *ipiv = (int*)
        aa_mem_region_local_alloc(sizeof(int)*n);

    // LU-factor
    info = aa_cla_dgetrf( mi, ni, A, mi, ipiv );

    int lwork = -1;
    while(1) {
        double *work = (double*)
            aa_mem_region_local_tmpalloc( sizeof(double)*
                                      (size_t)(lwork < 0 ? 1 : lwork) );
        aa_cla_dgetri( ni, A, mi, ipiv, work, lwork );
        if( lwork > 0 ) break;
        assert( -1 == lwork );
        lwork = (int)work[0];
    }

    aa_mem_region_local_pop(ipiv);

    return info;
}

int aa_la_inv_( int *n, double *A ) {
    size_t ns = (size_t)*n;
    return aa_la_inv(ns,A);
}

void aa_la_dpinv( size_t m, size_t n, double k, const double *A, double *A_star ) {
    // A^T (AA^T + kI)^{-1}
    // A is m*n
    // x = Aq, x is m, q is n

    const int mi = (int)m;
    const int ni = (int)n;

    // this method uses an LU factorization
    // B = AA^T
    double *B = (double*)aa_mem_region_local_alloc( sizeof(double) *
                                                    (m*m) );
    cblas_dgemm( CblasColMajor, CblasNoTrans, CblasTrans, mi, mi, ni,
                 1, A, mi, A, mi, 0, B, mi );

    // B += kI
    for( size_t i = 0; i < m; i ++ )
       B[i*m+i] += k;

    // B = B^-1
    aa_la_inv(m,B);

    // A^* = A^T*B
    cblas_dgemm( CblasColMajor, CblasTrans, CblasNoTrans, ni, mi, mi,
                 1, A, mi, B, mi, 0, A_star, ni );

    aa_mem_region_local_pop( B );

    // This method uses the SVD
    /* double *W = (double*)aa_mem_region_local_alloc( sizeof(double) * */
    /*                                             (m*m + n*n + AA_MIN(m,n)) ); */
    /* double *U = W;        // size m*m */
    /* double *Vt = U + m*m; // size n*n */
    /* double *S = Vt + n*n; // size min(m,n) */

    /* // A = U S V^T */
    /* aa_la_svd(m,n,A,U,S,Vt); */

    /* memset( A_star, 0, sizeof(double)*m*n ); */
    /* // \sum s_i/(s_i**2+k) * v_i * u_i^T */
    /* for( size_t i = 0; i < AA_MIN(m,n); i ++ ) { */
    /*     cblas_dger( CblasColMajor, ni, mi, S[i] / (S[i]*S[i] + k), */
    /*             Vt + i, ni, */
    /*             U + m*i, 1, */
    /*             A_star, ni */
    /*             ); */
    /* } */
    /* aa_mem_region_local_pop( W ); */
}

/// Deadzone Damped Pseudoinverse
void aa_la_dzdpinv( size_t m, size_t n, double s2_min, const double *A, double *A_star ) {
    // A^T (AA^T + kI)^{-1}
    // A is m*n
    // x = Aq, x is m, q is n

    const int mi = (int)m;
    const int ni = (int)n;

    // This method uses the SVD
    double *W = (double*)aa_mem_region_local_alloc( sizeof(double) *
                                                (m*m + n*n + AA_MIN(m,n)) );
    double *U = W;        // size m*m
    double *Vt = U + m*m; // size n*n
    double *S = Vt + n*n; // size min(m,n)

    // A = U S V^T
    aa_la_svd(m,n,A,U,S,Vt);

    memset( A_star, 0, sizeof(double)*m*n );
    // \sum s_i/(s_i**2+k) * v_i * u_i^T
    for( size_t i = 0; i < AA_MIN(m,n); i ++ ) {
        double s2 = AA_MAX( (S[i]*S[i]), s2_min );
        cblas_dger( CblasColMajor, ni, mi, S[i] / s2,
                Vt + i, ni,
                U + m*i, 1,
                A_star, ni
                );
    }
    aa_mem_region_local_pop( W );
}


int aa_la_svd( size_t m, size_t n, const double *A, double *U, double *S, double *Vt ) {
    return aa_la_d_svd(m,n,A,m,U,m,S,Vt,n);
}

AA_API void aa_la_dls( size_t m, size_t n,
                       double k, const double *A,
                       const double *x, double *y ) {
    double *A_star = (double*)aa_mem_region_local_alloc( sizeof(double) *
                                                     (m*n) );
    aa_la_dpinv(m,n,k,A,A_star);
    aa_la_mvmul(n,m,A_star,x,y);

    aa_mem_region_local_pop( A_star );
}


AA_API void aa_la_xlsnp( size_t m, size_t n,
                         const double *A, const double *A_star, const double *x,
                         const double *yp, double *y ) {
    aa_la_mvmul(n,m,A_star,x,y);

    double *B = (double*)aa_mem_region_local_alloc( sizeof(double) * n*n );

    // B = A^* A
    cblas_dgemm( CblasColMajor, CblasNoTrans, CblasNoTrans,
                 (int)n, (int)n, (int)m,
                 1, A_star, (int)n, A, (int)m, 0, B, (int)n );

    // B =  A^* A - I
    for( size_t i = 0; i < n; i ++ )
        AA_MATREF(B,n,i,i) -= 1;

    // y = y + -B yp
    cblas_dgemv( CblasColMajor, CblasNoTrans, (int)n, (int)n,
                 -1.0, B, (int)n,
                 yp, 1,
                 1, y, 1 );

    aa_mem_region_local_pop( B );
}

AA_API void aa_la_dlsnp( size_t m, size_t n,
                         double k, const double *A, const double *x,
                         const double *yp, double *y ) {

    double *A_star = (double*)aa_mem_region_local_alloc( sizeof(double) * m*n );
    aa_la_dpinv(m,n,k,A,A_star);
    aa_la_xlsnp( m, n, A, A_star, x, yp, y );
    aa_mem_region_local_pop( A_star );
}


AA_API void aa_la_lls( size_t m, size_t n, size_t p, const double *A, const double *b, double *x ) {

    aa_la_d_lls(m,n,p,A,m,b,m,x,n);

}



AA_API void aa_la_linterp( size_t n,
                           double t0, const double *X0,
                           double t1, const double *X1,
                           double ti, double *Xi ) {
    aa_la_d_lerp( n, (ti-t0)/(t1-t0),
                  X0, 1,
                  X1, 1,
                  Xi, 1 );
}



AA_API void aa_la_quadterp( size_t n,
                            double t0, const double *X0,
                            double t1, const double *X1,
                            double t2, const double *X2,
                            double ti, double *Xi )
{
    /* x = a + b*t + c*t*t
     *
     * x0 = a + b*t0 + c*t0*t0
     * x1 = a + b*t1 + c*t1*t1
     * x2 = a + b*t2 + c*t2*t2
     *
     * [x0]    [ 1,  t0,  t0*t0 ]   [a]
     * [x1] =  [ 1,  t1,  t1*t1 ] = [b]
     * [x2]    [ 1,  t2,  t2*t2 ]   [c]
     *
     *  X = T*A
     *  A = T^-1 * X
     */

    /* M is T^-1*/
    // col 0
    double M00 = t1*t2/((t1-t0)*(t2-t0));
    double M10 = -(t2+t1)/((t1-t0)*(t2-t0));
    double M20 = 1/((t1-t0)*(t2-t0));
    // col 1
    double M01 = -t0*t2/((t1-t0)*(t2-t1));
    double M11 = (t2+t0)/((t1-t0)*(t2-t1));
    double M21 = -1/((t1-t0)*(t2-t1));
    // col 2
    double M02 = t0*t1/((t2-t0)*(t2-t1));
    double M12 = -(t1+t0)/((t2-t0)*(t2-t1));
    double M22 = 1/((t2-t0)*(t2-t1));

    double ti2 = ti*ti;

    for( size_t i = 0; i < n; i ++ ) {
        double a = M00*X0[i] + M01*X1[i] + M02*X2[i];
        double b = M10*X0[i] + M11*X1[i] + M12*X2[i];
        double c = M20*X0[i] + M21*X1[i] + M22*X2[i];
        Xi[i] = a + b*ti + c*ti2;
    }
}

AA_API void aa_la_quadterp_dx( size_t n,
                               double t0, const double *X0,
                               double t1, const double *X1,
                               double t2, const double *X2,
                               double ti, double *dXi )
{
    // col 0
    double M10 = -(t2+t1)/((t1-t0)*(t2-t0));
    double M20 = 1/((t1-t0)*(t2-t0));
    // col 1
    double M11 = (t2+t0)/((t1-t0)*(t2-t1));
    double M21 = -1/((t1-t0)*(t2-t1));
    // col 2
    double M12 = -(t1+t0)/((t2-t0)*(t2-t1));
    double M22 = 1/((t2-t0)*(t2-t1));

    double ti_2 = ti/2;

    for( size_t i = 0; i < n; i ++ ) {
        double b = M10*X0[i] + M11*X1[i] + M12*X2[i];
        double c = M20*X0[i] + M21*X1[i] + M22*X2[i];
        dXi[i] = b + c*ti_2;
    }
}



double aa_la_det3x3( const double R[restrict 9] ) {

    double d;
    d = AA_MATREF(R,3,0,0) *
        ( AA_MATREF(R,3,1,1) * AA_MATREF(R,3,2,2) -
          AA_MATREF(R,3,1,2) * AA_MATREF(R,3,2,1) ) -
        AA_MATREF(R,3,0,1) * ( AA_MATREF(R,3,1,0) * AA_MATREF(R,3,2,2) -
                               AA_MATREF(R,3,1,2) * AA_MATREF(R,3,2,0)) +
        AA_MATREF(R,3,0,2) * ( AA_MATREF(R,3,1,0) * AA_MATREF(R,3,2,1) -
                               AA_MATREF(R,3,1,1) * AA_MATREF(R,3,2,0) );
    return d;
}


double aa_la_point_plane( size_t n, const double *point, const double *plane ) {
    double d = cblas_ddot( (int)n, point, 1, plane, 1 ) + plane[n];
    return d / cblas_dnrm2( (int)(n), plane, 1 );
}

void aa_la_plane_hessian( size_t n, double *plane ) {
    double a = cblas_dnrm2( (int)n-1, plane, 1 );
    cblas_dscal( (int)n, 1/a, plane, 1 );
}

void aa_la_plane_fit( size_t m, size_t n, const double *points, double *plane ) {
    aa_la_d_colfit( m, n, points, m, plane );
}
