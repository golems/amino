/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
 * Copyright (c) 2019, Colorado School of Mines
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


AA_API void
aa_tf_duqu_matrix_l( const double S[AA_RESTRICT 8], double *AA_RESTRICT M, size_t ldm )
{
    const size_t r = AA_TF_DUQU_REAL;
    const size_t d = AA_TF_DUQU_DUAL;
    aa_tf_qmatrix_l(&S[r], &AA_MATREF(M,ldm,r,r), ldm);
    aa_tf_qmatrix_l(&S[d], &AA_MATREF(M,ldm,d,r), ldm);

    FOR_QUAT(i) AA_MEM_ZERO( &AA_MATREF(M,ldm,r,d+i), 4 );
    FOR_QUAT(i) AA_MEM_CPY( &AA_MATREF(M,ldm,d,d+i), &AA_MATREF(M,ldm,r,r+i), 4);
}

AA_API void
aa_tf_duqu_matrix_r( const double S[AA_RESTRICT 8], double *AA_RESTRICT M, size_t ldm )
{
    const size_t r = AA_TF_DUQU_REAL;
    const size_t d = AA_TF_DUQU_DUAL;
    aa_tf_qmatrix_r(&S[r], &AA_MATREF(M,ldm,r,r), ldm);
    aa_tf_qmatrix_r(&S[d], &AA_MATREF(M,ldm,d,r), ldm);

    FOR_QUAT(i) AA_MEM_ZERO( &AA_MATREF(M,ldm,r,d+i), 4 );
    FOR_QUAT(i) AA_MEM_CPY( &AA_MATREF(M,ldm,d,d+i), &AA_MATREF(M,ldm,r,r+i), 4);
}

AA_API void aa_tf_duqu_mat_l( const double *q, struct aa_dmat *M )
{
    aa_tf_duqu_matrix_l( q, M->data, M->ld );
}

AA_API void aa_tf_duqu_mat_r( const double *q, struct aa_dmat *M )
{
    aa_tf_duqu_matrix_r( q, M->data, M->ld );
}


AA_API void
aa_tf_duqu_mul( const double _a[AA_RESTRICT 8], const double _b[AA_RESTRICT 8],
                double _c[AA_RESTRICT 8] )
{
    /*
     * Mul: 8
     * FMA: 40
     */
    const struct aa_tf_duqu *A = (struct aa_tf_duqu *)_a;
    const struct aa_tf_duqu *B = (struct aa_tf_duqu *)_b;
    struct aa_tf_duqu *C = (struct aa_tf_duqu *)_c;
    aa_tf_qmul(A->real.data, B->real.data, C->real.data);
    aa_tf_qmul(A->real.data, B->dual.data, C->dual.data);
    aa_tf_qmul_a(A->dual.data, B->real.data, C->dual.data);
}


AA_API void aa_tf_duqu_smul( double alpha, const double x[AA_RESTRICT 8],
                             double y[AA_RESTRICT 8] )
{
    for(size_t i = 0; i < 8; i ++ ) {
        y[i] = x[i] * alpha;
    }
}

AA_API void aa_tf_duqu_add( const double a[AA_RESTRICT 8], const double b[AA_RESTRICT 8],
                            double c[AA_RESTRICT 8] )
{
    const struct aa_tf_duqu *A = (struct aa_tf_duqu *)a;
    const struct aa_tf_duqu *B = (struct aa_tf_duqu *)b;
    struct aa_tf_duqu *C = (struct aa_tf_duqu *)c;
    aa_tf_qadd(A->real.data, B->real.data, C->real.data);
    aa_tf_qadd(A->dual.data, B->dual.data, C->dual.data);
}

AA_API void aa_tf_duqu_sub( const double a[AA_RESTRICT 8], const double b[AA_RESTRICT 8],
                            double c[AA_RESTRICT 8] )
{
    const struct aa_tf_duqu *A = (struct aa_tf_duqu *)a;
    const struct aa_tf_duqu *B = (struct aa_tf_duqu *)b;
    struct aa_tf_duqu *C = (struct aa_tf_duqu *)c;
    aa_tf_qsub(A->real.data, B->real.data, C->real.data);
    aa_tf_qsub(A->dual.data, B->dual.data, C->dual.data);
}


AA_API void
aa_tf_duqu_conj( const double _s[AA_RESTRICT 8], double _c[AA_RESTRICT 8] )
{
    const struct aa_tf_duqu *S = (struct aa_tf_duqu *)_s;
    struct aa_tf_duqu *C = (struct aa_tf_duqu *)_c;
    aa_tf_qconj(S->real.data, C->real.data );
    aa_tf_qconj(S->dual.data, C->dual.data );
}


AA_API void
aa_tf_duqu_cmul( const double a[AA_RESTRICT 8], const double b[AA_RESTRICT 8],
                double c[AA_RESTRICT 8] )
{
    double x[8];
    aa_tf_duqu_conj(a,x);
    aa_tf_duqu_mul(x,b,c);
}

AA_API void
aa_tf_duqu_mulc( const double a[AA_RESTRICT 8], const double b[AA_RESTRICT 8],
                double c[AA_RESTRICT 8] )
{
    double x[8];
    aa_tf_duqu_conj(b,x);
    aa_tf_duqu_mul(a,x,c);
}


AA_API void
aa_tf_duqu_normalize( double S[8] )
{
    double n = aa_tf_qnorm(S+AA_TF_DUQU_REAL);
    FOR_DUQU(i) S[i] /= n;
}


AA_API void
aa_tf_duqu_norm( const double S[AA_RESTRICT 8],
                 double * AA_RESTRICT nreal, double * AA_RESTRICT  ndual )
{
    *nreal = aa_tf_qnorm(S+AA_TF_DUQU_REAL);
    *ndual = aa_tf_qdot(S+AA_TF_DUQU_REAL, S+AA_TF_DUQU_DUAL) / *nreal;
}


AA_API void
aa_tf_duqu_vnorm( const double S[AA_RESTRICT 8],
                 double * AA_RESTRICT nreal, double * AA_RESTRICT  ndual )
{
    *nreal = aa_tf_qvnorm(S+AA_TF_DUQU_REAL);
    *ndual = aa_tf_qdot(S+AA_TF_DUQU_REAL_X, S+AA_TF_DUQU_DUAL_X) / *nreal;
}

AA_API void
aa_tf_duqu_minimize( double S[8] )
{
    if( S[AA_TF_DUQU_REAL_W] < 0 ) {
        FOR_DUQU(i) S[i] *= -1;
    }
}

AA_API void
aa_tf_duqu_vel2twist( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                      double t[AA_RESTRICT 8] )
{
    double v[3];
    aa_tf_duqu_trans(d, v );
    aa_tf_qv_vel2twist( d+AA_TF_DUQU_REAL, v,
                        dx+AA_TF_DX_W, dx+AA_TF_DX_V,
                        t+AA_TF_DUQU_REAL_XYZ, t+AA_TF_DUQU_DUAL_XYZ );

    t[REAL_W] = 0;
    t[DUAL_W] = 0;
}

AA_API void
aa_tf_duqu_twist2vel( const double d[AA_RESTRICT 8], const double t[AA_RESTRICT 8],
                      double dx[AA_RESTRICT 6] )
{
    double p[3];
    aa_tf_duqu_trans(d,p);

    aa_tf_qv_twist2vel( d+AA_TF_DUQU_REAL, p,
                        t+REAL_XYZ, t+DUAL_XYZ,
                        dx + AA_TF_DX_W, dx + AA_TF_DX_V );
}

AA_API void
aa_tf_duqu_twist2diff( const double d[AA_RESTRICT 8], const double t[AA_RESTRICT 8],
                       double dd[AA_RESTRICT 8] )
{
    aa_tf_duqu_mul(t,d,dd);
    for( size_t i = 0; i < 8; i ++ ) dd[i] /= 2;
}

AA_API void
aa_tf_duqu_diff2twist( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                       double twist[AA_RESTRICT 8] )
{
    double dx[6];
    aa_tf_duqu_diff2vel(d, dd, dx);
    aa_tf_duqu_vel2twist(d, dx, twist);
}

AA_API void
aa_tf_duqu_vel2diff( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                     double dd[AA_RESTRICT 8] )
{
    double t[8];
    aa_tf_duqu_vel2twist( d, dx, t );
    aa_tf_duqu_twist2diff( d, t, dd );

}

AA_API void
aa_tf_duqu_diff2vel( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                     double dx[AA_RESTRICT 6] )
{
    double t1[3], t2[3];
    // rotation
    aa_tf_qdiff2vel( &d[REAL], &dd[REAL], &dx[OMEGA] );
    // translation
    // dx/dt = 2 * ( d_dual/dt conj(r) + d_dual conj(d_real/dt) )
    aa_tf_qmulc_v( &dd[DUAL], &d[REAL], t1 );
    aa_tf_qmulc_v( &d[DUAL], &dd[REAL], t2 );
    FOR_VEC(i) dx[V+i] = 2 * (t1[i] + t2[i]);
}

AA_API void
aa_tf_duqu_stwist( const double d0[AA_RESTRICT 8], const double twist[AA_RESTRICT 8],
                   double dt, double d1[AA_RESTRICT 6] )
{
    double twist1[8], etwist[8];
    FOR_DUQU(i) twist1[i] = dt/2 * twist[i];
    aa_tf_duqu_exp( twist1, etwist );
    aa_tf_duqu_mul( etwist, d0, d1 );
}

AA_API void
aa_tf_duqu_svel( const double d0[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                             double dt, double d1[AA_RESTRICT 6] )
{
    double twist[8];
    aa_tf_duqu_vel2twist( d0, dx, twist );
    aa_tf_duqu_stwist(d0, twist, dt, d1 );
}

AA_API void
aa_tf_duqu_sdiff( const double d0[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                  double dt, double d1[AA_RESTRICT 6] )
{
    double w[8];
    aa_tf_duqu_diff2twist(d0, dd, w);
    aa_tf_duqu_stwist(d0, w, dt, d1 );
}

AA_API void
aa_tf_duqu_trans( const double S[AA_RESTRICT 8], double v[AA_RESTRICT 3] )
{
    aa_tf_qmulc_v(S+AA_TF_DUQU_DUAL, S+AA_TF_DUQU_REAL, v);
    FOR_VEC(i) v[i] *= 2;
}

AA_API void
aa_tf_tf_duqu( const double S[AA_RESTRICT 8], const double p0[AA_RESTRICT 3],
               double p1[AA_RESTRICT 3] )
{
    aa_tf_duqu_tf(S,p0,p1);
}

AA_API void
aa_tf_duqu_tf( const double S[AA_RESTRICT 8], const double p0[AA_RESTRICT 3],
               double p1[AA_RESTRICT 3]  )
{
    double x[4];
    const double *r = S+AA_TF_DUQU_REAL;
    const double *d = S+AA_TF_DUQU_DUAL;

    aa_tf_qmul_qv(r, p0, x);     /* 12mul, 8add */
    FOR_QUAT(i) x[i] += 2*d[i];  /*  0mul, 4add */
    aa_tf_qmulc_v(x, r, p1);     /* 12mul, 9add */
    /* Total: 24 mul, 21 add */
}

static void
dual_mul( double ar, double ad, double br, double bd, double *cr, double *cd )
{
    *cr = ar*br;
    *cd = ar*bd + ad*br;
}

AA_API void
aa_tf_duqu_exp( const double S[AA_RESTRICT 8], double eS[AA_RESTRICT 8] )
{
    double nr, c, vv, vd, ar, ad, er, ed;

    vv = aa_tf_vdot( S+AA_TF_DUQU_REAL_XYZ, S+AA_TF_DUQU_REAL_XYZ );

    if( vv < DBL_EPSILON ) {
        ar = aa_tf_sinc_series2(vv);
        c = aa_tf_cos_series2(vv);
        /* Taylor series for cos(nr)/nr**2 - sin(nr)/nr**3 */
        ad = aa_horner3( vv, -1.0/3.0, 1.0/30.0, -1.0/840.0 );
    } else {
        nr = sqrt(vv);
        ar = sin(nr)/nr;
        c = cos(nr);
        ad = (c-ar)/vv;
    }

    vd = aa_tf_vdot( S+AA_TF_DUQU_REAL_XYZ, S+AA_TF_DUQU_DUAL_XYZ );
    ad *= vd;

    FOR_VEC(i) eS[AA_TF_DUQU_REAL_XYZ+i] = ar*S[AA_TF_DUQU_REAL_XYZ+i];
    eS[AA_TF_DUQU_REAL_W] = c;

    FOR_VEC(i) eS[AA_TF_DUQU_DUAL_XYZ+i] = ar*S[AA_TF_DUQU_DUAL_XYZ+i] + ad*S[AA_TF_DUQU_REAL_XYZ+i];
    eS[AA_TF_DUQU_DUAL_W] = -ar*vd;

    if( 0 < fabs(S[AA_TF_DUQU_REAL_W]) ||  0 < fabs(S[AA_TF_DUQU_DUAL_W]) ) {
        /* dual number exponential */
        er = exp(S[AA_TF_DUQU_REAL_W]);
        ed = er * S[AA_TF_DUQU_DUAL_W];
        /* scale */
        FOR_QUAT(i) {
            dual_mul(er, ed,
                     eS[AA_TF_DUQU_REAL_XYZ+i], eS[AA_TF_DUQU_DUAL_XYZ+i],
                     &eS[AA_TF_DUQU_REAL_XYZ+i], &eS[AA_TF_DUQU_DUAL_XYZ+i] );
        }
    }
}


AA_API void
aa_tf_duqu_ln( const double S[AA_RESTRICT 8], double lnS[AA_RESTRICT 8] )
{
    double vv = aa_tf_vdot(S+AA_TF_DUQU_REAL_XYZ, S+AA_TF_DUQU_REAL_XYZ);
    double gamma = aa_tf_vdot(S+AA_TF_DUQU_REAL_XYZ, S+AA_TF_DUQU_DUAL_XYZ);

    double mr2 = vv + S[AA_TF_DUQU_REAL_W]*S[AA_TF_DUQU_REAL_W];
    double mr = sqrt( mr2 );

    /* Scalar part */
    lnS[AA_TF_DUQU_REAL_W] = log(mr);
    lnS[AA_TF_DUQU_DUAL_W] = (gamma + S[AA_TF_DUQU_REAL_W]*S[AA_TF_DUQU_DUAL_W]) / mr2;

    /* Vector part */
    /* ! Dual number computation */
    /* ! call aa_tf_duqu_vnorm( d, nv%r, nv%d ) */
    /* ! a = atan2( nv, dh(W_INDEX) ) / nv */

    /* expanded dual computation */
    double nr = sqrt(vv);                   /* nr is positive */
    double theta = atan2( nr, S[AA_TF_DUQU_REAL_W] ); /* theta is always positive */

    /* Try to avoid small number division */
    double kappa, zeta;
    if( theta < DBL_EPSILON )  {
        /* ad = 1/mr * 1d0/sin(x)**2 * ( cos(x) - x/sin(x) ) */
        kappa = aa_horner3( theta*theta, -2.0/3.0, -1.0/5.0, -17.0/420.0 ) / mr;
        zeta = aa_tf_invsinc_series2(theta*theta)/(mr2*mr);
    } else {
        kappa = theta/nr;
        zeta = (S[AA_TF_DUQU_REAL_W]/mr2 - kappa) / vv;
    }
    double ad = gamma*zeta - S[AA_TF_DUQU_DUAL_W] / mr2;
    FOR_VEC(i)
        lnS[AA_TF_DUQU_REAL_XYZ+i] = kappa * S[AA_TF_DUQU_REAL_XYZ+i];
    FOR_VEC(i)
        lnS[AA_TF_DUQU_DUAL_XYZ+i] = (ad * S[AA_TF_DUQU_REAL_XYZ+i]
                                      + kappa * S[AA_TF_DUQU_DUAL_XYZ+i]);
}


AA_API void
aa_tf_duqu_lnv( const double S[AA_RESTRICT 8], double w[AA_RESTRICT 6] )
{
    double k[8];
    aa_tf_duqu_ln(S,k);
    aa_tf_duqu2pure( k, w );
}

AA_API void aa_tf_xyz2duqu (
    double x, double y, double z,
    double d[AA_RESTRICT 8] )
{
    // TODO: test case
    d[AA_TF_DUQU_REAL_X] = 0;
    d[AA_TF_DUQU_REAL_Y] = 0;
    d[AA_TF_DUQU_REAL_Z] = 0;
    d[AA_TF_DUQU_REAL_W] = 1;

    d[AA_TF_DUQU_DUAL_X] = x/2;
    d[AA_TF_DUQU_DUAL_Y] = y/2;
    d[AA_TF_DUQU_DUAL_Z] = z/2;
    d[AA_TF_DUQU_DUAL_W] = 0;

}

AA_API void
aa_tf_duqu_jac_vel2diff(const double S[8], const struct aa_dmat *Jvel,
                        struct aa_dmat *Js )
{
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    // SR := 0.5 * [S]^R
    struct aa_dmat *SR = aa_dmat_alloc(reg,8,8);
    double Sh[8]; aa_tf_duqu_smul(0.5,S,Sh);
    aa_tf_duqu_mat_r(Sh, SR);

    struct aa_dmat *VM = aa_dmat_alloc(reg,8,8);
    struct aa_dmat Vb;
    aa_dmat_zero(VM);
    {
        aa_dmat_view_block(&Vb, VM,
                           AA_TF_DUQU_DUAL_XYZ, AA_TF_DUQU_REAL_XYZ,
                           3, 3 );
        double v[3]; aa_tf_duqu_trans(S,v);
        aa_tf_cross_mat_l(v, &Vb);
    }

    // SR := SR * VM, VM is unit lower triangular
    // TODO: save a few multiplies by manually handling upper/lower blocks
    cblas_dtrmm( CblasColMajor, CblasRight, CblasLower, CblasNoTrans, CblasUnit,
                 8, 8, 1,
                 VM->data, 8,
                 SR->data, 8 );
    aa_mem_region_pop(reg,VM);


    // Expand velocity Jacobian with zeros for scalar (w) rows
    size_t n = Jvel->cols;
    struct aa_dmat *JE = aa_dmat_alloc(reg,8,n);
    {
        // copy vector rows
        struct aa_dmat Jm, JEm;
        aa_dmat_view_block( &Jm, Jvel, AA_TF_DX_W, 0, 3, n );
        aa_dmat_view_block( &JEm, JE, AA_TF_DUQU_REAL_XYZ, 0, 3, n );
        aa_dmat_copy(&Jm, &JEm);

        aa_dmat_view_block( &Jm, Jvel, AA_TF_DX_V, 0, 3, n );
        aa_dmat_view_block( &JEm, JE, AA_TF_DUQU_DUAL_XYZ, 0, 3, n );
        aa_dmat_copy(&Jm, &JEm);
    }

    {
        // zero vector rows
        struct aa_dvec JEw;
        aa_dmat_row_vec(JE, AA_TF_DUQU_REAL_W, &JEw );
        aa_dvec_zero(&JEw);
        aa_dmat_row_vec(JE, AA_TF_DUQU_DUAL_W, &JEw );
        aa_dvec_zero(&JEw);
    }


    aa_lb_dgemm( CblasNoTrans, CblasNoTrans,
                 1, SR, JE,
                 0, Js );

    aa_mem_region_pop(reg,ptrtop);
}

void
aa_tf_duqu2pure( const double S[AA_RESTRICT 8],
                 double v[AA_RESTRICT 6] )
{
    AA_MEM_CPY( v + AA_TF_DX_W, S+AA_TF_DUQU_REAL_XYZ, 3);
    AA_MEM_CPY( v + AA_TF_DX_V, S+AA_TF_DUQU_DUAL_XYZ, 3);
}

AA_API void
aa_tf_pure2duqu( const double v[AA_RESTRICT 6],
                 double S[AA_RESTRICT 8])
{
    AA_MEM_CPY( S+AA_TF_DUQU_REAL_XYZ, v + AA_TF_DX_W, 3 );
    S[AA_TF_DUQU_REAL_W] = 0;

    AA_MEM_CPY( S+AA_TF_DUQU_DUAL_XYZ, v + AA_TF_DX_V, 3 );
    S[AA_TF_DUQU_DUAL_W] = 0;
}
