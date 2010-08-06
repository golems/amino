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

// uncomment to check that local allocs actually get freed
// #define AA_ALLOC_STACK_MAX 0

#include "amino.h"


/*--- Scalar Ops ---*/

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
    double a = 0;
    double t = 0;
    for( size_t i = 0; i < n; i ++ ) {
        t = x[i] - y[i];
        a += t*t;
    }
    return a;
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

void aa_la_unit( size_t n, double *x ) {
    aa_la_scal( n, 1.0/aa_la_norm(n,x), x );
}

/*--- Matrix Ops --- */

int aa_la_inv( size_t n, double *A ) {
    int ipiv[n];
    const int mi = (int) n;
    const int ni = (int) n;
    int info;

    // LU-factor
    dgetrf_( &mi, &ni, A, &mi, ipiv, &info );

    // find optimal size
    double swork[1];
    int lwork_query = -1;
    dgetri_( &ni, A, &mi, ipiv, swork, &lwork_query, &info );
    int lwork = (int) swork[0];

    // invert
    double *work = AA_NEW_LOCAL(double, (size_t)swork[0]);
    dgetri_( &ni, A, &mi, ipiv, work, &lwork, &info );
    AA_DEL_LOCAL(work, double, (size_t)swork[0]);

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

    // B = AA^T
    /* double *B = (double*)AA_ALLOCAL(sizeof(double)*m*m);
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
    aa_frlocal(B, sizeof(double)*m*m);
    */

    double *Ap = AA_NEW_LOCAL(double, m*n);
    double *U = AA_NEW_LOCAL(double, m*m);
    double *Vt = AA_NEW_LOCAL(double, n*n);
    double *S = AA_NEW_LOCAL(double, AA_MIN(m,n));

    aa_fcpy(Ap,A,m*n);
    aa_fset(A_star,0,m*n);

    // A = U S V^T
    aa_la_svd(m,n,Ap,U,S,Vt);

    // \sum s_i/(s_i**2+k) * v_i * u_i^T
    for( size_t i = 0; i < AA_MIN(m,n); i ++ ) {
        cblas_dger( CblasColMajor, ni, mi, S[i] / (S[i]*S[i] + k),
                Vt + i, ni,
                U + m*i, 1,
                A_star, ni
                );
    }

    AA_DEL_LOCAL(Ap, double, m*n);
    AA_DEL_LOCAL(U,  double, m*m);
    AA_DEL_LOCAL(Vt, double, n*n);
    AA_DEL_LOCAL(S,  double, AA_MIN(m,n));
}

int aa_la_svd( size_t m, size_t n, double *A, double *U, double *S, double *Vt ) {
    const char *jobu = U ? "A" : "O";
    const char *jobvt = Vt ? "A" : "O";
    int mi = (int)m, ni=(int)n;
    int lwork = -1;
    int info;

    // calculate work size
    double qwork[1];
    dgesvd_( jobu, jobvt, &mi, &ni,
             A, &mi,
             S, U, &mi,
             Vt, &ni,
             &qwork[0], &lwork, &info );

    // allocate work array
    lwork = (int) qwork[0];
    double *work =  AA_NEW_LOCAL( double, (size_t)lwork );
    //printf("size: %d\n", (int)qwork[0]);

    // calculate SVD
    dgesvd_( jobu, jobvt, &mi, &ni,
             A, &mi,
             S, U, &mi,
             Vt, &ni,
             &work[0], &lwork, &info );

    //finish
    AA_DEL_LOCAL(work, double, (size_t)lwork );
    return info;
}

AA_CDECL void aa_la_dls( size_t m, size_t n, double k, const double *A, const double *x, double *y ) {
    double *A_star = AA_NEW_LOCAL(double, m*n);
    aa_la_dpinv(m,n,k,A,A_star);
    aa_la_mvmul(m,n,A_star,x,y);
    AA_DEL_LOCAL(A_star, double, m*n);
}

AA_CDECL void aa_la_dlsnp( size_t m, size_t n, double k, const double *A, const double *x, const double *yp, double *y ) {
    double *A_star = AA_NEW_LOCAL(double, m*n);
    double *B = AA_NEW_LOCAL(double, n*n);

    aa_la_dpinv(m,n,k,A,A_star);
    aa_la_mvmul(m,n,A_star,x,y);

    // B = -A^* A
    cblas_dgemm( CblasColMajor, CblasNoTrans, CblasNoTrans, (int)n, (int)n, (int)m,
                 -1, A_star, (int)n, A, (int)m, 0, B, (int)n );

    // B = I - A^* A
    for( size_t i = 0; i < n; i ++ )
        B[i*n+i] -= 1;

    // y = y + B yp
    cblas_dgemv( CblasColMajor, CblasNoTrans, (int)m, (int)n,
            1.0, A, (int)n,
            yp, 1, 1, y, 1 );

    AA_DEL_LOCAL(A_star, double, m*n);
    AA_DEL_LOCAL(B, double, n*n);
}
