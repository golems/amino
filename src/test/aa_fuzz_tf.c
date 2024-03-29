/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

//#define AA_ALLOC_STACK_MAX
#include "amino.h"
#include "amino/test.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>

#include "amino/diffeq.h"

static void rand_dh( double dh[4] ) {
    dh[0] = aa_frand_minmax(-M_PI, M_PI);
    dh[1] = aa_frand_minmax(-1, 1);
    dh[2] = aa_frand_minmax(-1, 1);
    dh[3] = aa_frand_minmax(-M_PI, M_PI);
}



static void rand_point(double p[3])
{
    aa_test_randv(-1,1,3,p);
}

static void rand_axis(double axis[3])
{
    rand_point(axis);
    aa_tf_vnormalize(axis);
}

static void tf_fill_aa(const double aa[4], const double v[3], double rv[3],
                       double E[7], double S[8], double T[12])
{
    aa_tf_axang2rotvec(aa, rv);

    aa_tf_axang2quat(aa, E);
    AA_MEM_CPY(E + 4, v, 3);

    aa_tf_qutr2duqu(E, S);
    aa_tf_qutr2tfmat(E, T);
}

static void rand_tf_aa(double aa[4], double rv[3], double E[7], double S[8],
                       double T[12])
{
    rand_axis(aa);
    aa[3] = aa_frand_minmax(-1.99 * M_PI, 1.99 * M_PI);

    double v[3];
    rand_point(v);

    tf_fill_aa(aa, v, rv, E, S, T);
}


static void rand_tf_q(double aa[4], double rv[3], double E[7], double S[8],
                       double T[12])
{
    aa_tf_qutr_rand(E);
    aa_tf_quat2axang(E, aa);
    aa_tf_quat2rotvec(E, rv);
    aa_tf_qutr2duqu(E, S);
    aa_tf_qutr2tfmat(E, T);
}

static void rand_tf_sing(double aa[4], double rv[3], double E[7], double S[8],
                         double T[12])
{
    static const double thetas[] = {
        0,
        1e-9, -1e-9,
        1e-6, -1e-6,
        -1e-3, 1e-3,
        (M_PI - 1e-3),
        (M_PI - 1e-6),
        (M_PI - 1e-9),
        M_PI, -M_PI,
        M_PI+1e-9, -M_PI-1e-9,
    };
    double theta = thetas[(size_t)rand() % (sizeof(thetas) / sizeof(thetas[0]))];
    aa[3] = theta;
    rand_axis(aa);

    double v[3];
    rand_point(v);

    tf_fill_aa(aa, v, rv, E, S, T);
}

static void rand_tf(double aa[4], double rv[3], double E[7], double S[8],
                    double T[12])
{
    switch (rand() % 3) {
    case 0:
        rand_tf_aa(aa, rv, E, S, T);
        break;
    case 1:
        rand_tf_q(aa, rv, E, S, T);
        break;
    case 2:
        rand_tf_sing(aa, rv, E, S, T);
        break;
    default:
        abort();
        exit(EXIT_FAILURE);
    }

    // Check Validity
    aa_test_isrotmat("isrotmat", T, 1e-9);
    test_feq("q norm", aa_tf_qnorm(E), 1, 1e-9);

    // Check Equivalences
    {  // check equivalence of conversion to matrices
        double Rq[9], Raa[9], Rrv[7];
        aa_tf_quat2rotmat(E, Rq);
        aa_tf_axang2rotmat(aa, Raa);
        aa_tf_rotvec2rotmat(rv, Rrv);

        aa_test_rotmat_cmp("q->R", T, Rq, 1e-8);
        aa_test_rotmat_cmp("rv->R", T, Rrv, 1e-8);
        //aa_test_rotmat_cmp("aa->R", T, Raa, 1e-9);
    }

    {  // check equivalence of conversion to quaternions

        double Eq[7], Es[7], Et[7], qrv[4], aaq[4], qaaq[4];
        aa_tf_rotvec2quat(rv, qrv);
        aa_tf_quat2axang(E, aaq);
        aa_tf_axang2quat(aaq, qaaq);
        aa_tf_tfmat2qutr(T, Et);
        aa_tf_duqu2qutr(S, Es);

        aa_tf_qminimize2(E, Eq);
        AA_MEM_CPY(Eq + 4, E + 4, 3);
        aa_tf_qminimize(Es);
        aa_tf_qminimize(Et);
        aa_tf_qminimize(qrv);
        aa_tf_qminimize(qaaq);

        /* fprintf(stderr, "aa:  "); */
        /* aa_dump_vec(stderr, aa, 4); */
        /* fprintf(stderr, "aaq: "); */
        /* aa_dump_vec(stderr, aaq, 4); */

        aa_test_quat_cmp("aa<->q", Eq, qaaq, 1e-6);
        aa_test_quat_cmp("rv<->q", Eq, qrv, 1e-6);
        aa_test_qutr_cmp("qutr<->duqu", Eq, Es, 1e-6);
        aa_test_qutr_cmp("qutr<->tfmat", Eq, Et, 1e-6);
    }
}

static void rotvec(const double e[3], const double aa[4], const double q[4],
                   const double R[9])
{
    {  // Angles

        /* Axis-angle angle might be negative (if axis is "flipped"), but we
         * cannot recover the negative angle from the other representations. */
        double aa_theta = fabs(aa[3]);
        double rv_theta = aa_tf_vnorm(e);
        double s_aa = sin(aa_theta), c_aa = cos(aa_theta);
        double s_rv = sin(rv_theta), c_rv = cos(rv_theta);
        aafeq("theta-aa", aa_theta, rv_theta, 1e-9);
        aafeq("s-aa", s_aa, s_rv, 1e-9);
        aafeq("c-aa", c_aa, c_rv, 1e-9);

        /* qangle is in the range [0,pi], which corresponds to rotations of
         * [0, 2*pi].  */
        double q_theta = aa_tf_qangle(q) * 2;
        double q_theta_rv = rv_theta;
        aafeq("qangle-rv", q_theta_rv, q_theta, 1e-9);

        /* rotation matrix angle is in the range [0,pi]; sin() is always
         * positive.
         */
        double n_theta = aa_theta;
        double s_n = fabs(sin(n_theta)), c_n = cos(n_theta);
        double s_r, c_r;
        aa_tf_rotmat_sincos(R, &s_r, &c_r);
        aafeq("c-R", c_n, c_r, 1e-9);
        aafeq("s-R", s_n, s_r, 1e-9);
    }

    {
        double qr[3];
        aa_tf_quat2rotvec(q, qr);
        arveq("quat2rotvec", e, qr, 1e-6);
    }

    {
        double vr[3];
        aa_tf_rotmat2rotvec(R, vr);
        aa_test_rotvec_cmp_pi("rotmat2rotvec", e, vr, 1e-6 );
    }


    {
        double ee[9], rln[3];
        aa_tf_rotmat_expv( e, ee );
        aa_test_rotmat_cmp("rotmat_expv", R, ee, 1e-8);

        aa_tf_rotmat_lnv( ee, rln );
        aa_test_rotvec_cmp_pi("rotmat_lnv", e, rln, 1e-6 );
    }

    {
        double ee[9];
        aa_tf_rotmat_exp_aa(aa, ee);
        aa_test_rotmat_cmp("rotmat_exp_aa", ee, R, 1e-9 );
    }
}


static void eulerzyx(const double q[4], const double R[4])
{
    double qm[4], e_q[3], e_R[3];

    aa_tf_quat2eulerzyx(q,e_q);
    aa_tf_qminimize2( q, qm );

    {
        double q_e[4];
        aa_tf_eulerzyx2quat( e_q[0], e_q[1], e_q[2], q_e );
        aa_tf_qminimize( q_e );
        aa_test_quat_cmp("quat->euler->quat", qm, q_e, .001 );
    }

    {
        double q_e[4];
        aa_tf_rotmat2eulerzyx(R,e_R);
        aa_tf_eulerzyx2quat( e_R[0], e_R[1], e_R[2], q_e );
        aa_tf_qminimize( q_e );
        aa_test_quat_cmp("quat->euler->quat/rotmat->euler->quat", qm, q_e, .001 );

        aveq("quat->euler/rotmat->quat", 3, e_q, e_R, .001 );
    }

    aveq("quat->euler/rotmat->quat", 3, e_q, e_R, .001 );

}



static void chain(double E[2][7],
                  double S[2][8],
                  double T[2][12] ) {
    double E3[7];

    // rotation
    {
        double q3[4]={0};
        double R3[9]={0};
        double qc[4]={0};
        aa_tf_qmul( E[0], E[1], q3 );
        aa_tf_9mul( T[0], T[1], R3 );
        aa_tf_rotmat2quat( R3, qc );
        aa_tf_qminimize( q3 );
        aa_tf_qminimize( qc );
        aveq("chain-rot q/9", 4, q3, qc, 1e-6 );
    }

    // Transformation
    aa_tf_qutr_mul( E[0], E[1], E3 );
    aa_tf_qminimize( E3 );

    // duqu
    {
        double S3[8]={0}, Ec[7]={0};
        aa_tf_duqu_mul( S[0], S[1], S3 );
        aa_tf_duqu2qutr( S3, Ec );
        aa_tf_qminimize( Ec );
        aveq("chain-tf qutr/duqu", 7, E3, Ec, 1e-6 );
    }

    // qutr_norm
    {
        double N3[7]={0};
        aa_tf_qutr_mulnorm(E[0], E[1], N3);
        aa_tf_qminimize( N3 );
        aveq("chain-tf qutr/qutrnorm", 7, E3, N3, 1e-6 );
    }

    // tfmat
    {
        double T3[12]={0}, Ec[7]={0};
        aa_tf_12chain( T[0], T[1], T3 );
        aa_tf_tfmat2qutr( T3, Ec );
        aa_tf_qminimize( Ec );
        aveq("chain-tf qutr/tfmat", 7, E3, Ec, 1e-6 );
    }
}

static void quat(const double q1[4], const double q2[4], const double R1[4],
                 const double R2[4], const double w[3], double u)
{

    {
        double qg[4], qa[4];
        aa_tf_qslerp( u, q1, q2, qg );
        aa_tf_qslerpalg( u, q1, q2, qa );
        aveq("slerp", 4, qg, qa, .001 );

        double dqg[4], dqa[4];
        aa_tf_qslerpdiff( u, q1, q2, dqg );
        aa_tf_qslerpdiffalg( u, q1, q2, dqa );
        aveq("slerpdiff", 4, dqg, dqa, .001 );
    }

    // mul
    {
        double Ql[16], Qr[16];
        double y0[4], y1[4], y2[4];
        aa_tf_qmatrix_l(q1, Ql, 4);
        aa_tf_qmatrix_r(q2, Qr, 4);
        aa_tf_qmul(q1,q2, y0);
        cblas_dgemv( CblasColMajor, CblasNoTrans, 4, 4,
                     1.0, Ql, 4,
                     q2, 1,
                     0, y1, 1 );
        cblas_dgemv( CblasColMajor, CblasNoTrans, 4, 4,
                     1.0, Qr, 4,
                     q1, 1,
                     0, y2, 1 );
        aveq( "qmul-1", 4, y0, y1, 1e-6 );
        aveq( "qmul-2", 4, y0, y2, 1e-6 );
    }


    // average
    {
        double p1 = aa_tf_qangle(q1);
        double p2 = aa_tf_qangle(q2);
        /* fprintf(stderr, "p1: %f\n", p1); */
        /* fprintf(stderr, "p2: %f\n", p2); */
        /* fprintf(stderr, "q1: \n"); */
        /* aa_dump_vec(stderr, q1, 4); */
        /* fprintf(stderr, "q1: \n"); */
        /* aa_dump_vec(stderr, q2, 4) */;

        if ((p1 < .49 * M_PI && p1 > .01 * M_PI) ||
            (p2 < .49 * M_PI && p2 > .01 * M_PI)) {
            // TODO: robustify near 0/pi?
            double qq[8], p[4], s[4];
            AA_MEM_CPY( qq, q1, 4 );
            AA_MEM_CPY( qq+4, q2, 4 );
            double weight[2] = {.5,.5};
            aa_tf_quat_davenport( 2, weight, qq, 4, p );
            aa_tf_qslerp( .5, q1, q2, s );
            aa_tf_qminimize( p );
            aa_tf_qminimize( s );
            aveq("davenport-2", 4, p, s, 1e-4 );
        }
    }

    double Rr[9], qr[4], qrr[4];
    aa_tf_9rel( R1, R2, Rr );
    aa_tf_qrel( q1, q2, qr );
    aa_tf_rotmat2quat( Rr, qrr );
    aa_tf_qminimize( qr );
    aa_tf_qminimize( qrr );
    aveq("qrel", 4, qr, qrr, .001 );

    // minimize
    {
        double qmin[4], axang[4];
        aa_tf_qminimize2( q1, qmin );
        test( "quat-minimize",  aa_feq( fabs(q1[3]), qmin[3], 0) );
        aa_tf_quat2axang( qmin, axang );
        test( "quat-minimize-angle",  fabs(axang[3]) <= M_PI );
    }

    // mulc
    {
        double q1c[4], q2c[4], t1[4], t2[4];
        aa_tf_qconj(q1, q1c);
        aa_tf_qconj(q2, q2c);

        aa_tf_qmul(q1,q2c,t1);
        aa_tf_qmulc(q1,q2,t2);
        aveq("qmulc", 4, t1, t2, .001 );

        aa_tf_qmul(q1c,q2,t1);
        aa_tf_qcmul(q1,q2,t2);
        aveq("qcmul", 4, t1, t2, .001 );
    }
    // conj. props
    {
        // p*q = conj(conj(q) * conj(p))
        double c1[4], c2[4], c2c1[4], cc2c1[4], q1q2[4];
        aa_tf_qconj(q1,c1);
        aa_tf_qconj(q2,c2);
        aa_tf_qmul(c2,c1,c2c1);
        aa_tf_qmul(q1,q2,q1q2);
        aa_tf_qconj(c2c1,cc2c1);
        aveq("conjprop", 4, q1q2, cc2c1, .0001);
    }
    // exp
    {
        double q1e[4], q1eln[4];
        aa_tf_qexp(q1, q1e);
        aa_tf_qln(q1e, q1eln);
        aveq("exp-log", 4, q1, q1eln, .00001 );
        aa_tf_qln(q1, q1eln);
        aa_tf_qexp(q1eln, q1e);
        aveq("log-exp", 4, q1, q1e, .00001 );
    }

    // diff
    double  dq[4], wdq[3];
    aa_tf_qvel2diff( q1, w, dq );
    aa_tf_qdiff2vel( q1, dq, wdq );
    aveq("qveldiff", 3, w, wdq, .000001);

    // integrate

    double qn_rk1[4], qn_vrk1[4], qn_vrk4[4], qn_vexp[4], qn_dq[4], w0[3] = {0};
    double dt = .02;


    aa_tf_qrk1( q1, dq, dt, qn_rk1 );
    aa_tf_qvelrk1( q1, w, dt, qn_vrk1 );
    aa_tf_qvelrk4( q1, w, dt, qn_vrk4 );
    aa_tf_qsvel( q1, w, dt, qn_vexp );
    aa_tf_qsdiff( q1, dq, dt, qn_dq );
    aveq("qvelrk1", 4, qn_rk1, qn_vrk1, .001 );
    aveq("qvelrk4", 4, qn_rk1, qn_vrk4, .001 );
    aveq("qvelexp", 4, qn_vrk4, qn_vexp, .0001);
    aveq("qvelsdiff", 4, qn_vexp, qn_dq, .001 );
    aa_tf_qsvel( q1, w0, dt, qn_vexp );
    aveq("qvelsvel0", 4, q1, qn_vexp, .000 );

    {
        double Rb[9], qR[4];
        aa_tf_qsvel( q1, w, dt, qn_vexp );
        aa_tf_rotmat_svel( R1, w, dt, Rb );
        aa_tf_rotmat2quat( Rb, qR );
        aa_tf_qminimize( qn_vexp);
        aa_tf_qminimize( qR );
        aveq("rotmat_svel", 4, qn_vexp, qR, 1e-4 );
    }
}

static void vecs2quat(const double v0[3], const double v1[3])
{
    // vectors
    // const double *v0 = E[0] + AA_TF_QUTR_T;
    // const double *v1 = E[1] + AA_TF_QUTR_T;
    double q[4], vp[3];

    // identify case
    aa_tf_vecs2quat(v0, v0, q);
    aveq("vecs2quat-ident", 4, q, aa_tf_quat_ident, 1e-6);

    // regular case
    aa_tf_vecs2quat(v0, v1, q);
    aa_tf_qrot(q, v0, vp);

    // normalize result
    {
        double n0 = sqrt(v0[0] * v0[0] + v0[1] * v0[1] + v0[2] * v0[2]);
        double n1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
        double vp1[3];
        for (size_t i = 0; i < 3; i++) {
            vp1[i] = n0 * v1[i] / n1;
        }

        aveq("vecs2quat", 3, vp, vp1, 1e-6);
    }
    // inverted case
    double v0n[3] = {-v0[0], -v0[1], -v0[2]};
    aa_tf_vecs2quat(v0, v0n, q);
    aa_tf_qrot(q, v0, vp);
    {
        double n0 = sqrt(v0[0] * v0[0] + v0[1] * v0[1] + v0[2] * v0[2]);
        double n1 = sqrt(v0n[0] * v0n[0] + v0n[1] * v0n[1] + v0n[2] * v0n[2]);
        double vp1[3];
        for (size_t i = 0; i < 3; i++) {
            vp1[i] = n0 * v0n[i] / n1;
        }

        aveq("vecs2quat-degenerate", 3, vp, vp1, 1e-6);
    }
}


static void duqu_gen(const double A[8], const double B[8]) {
    // mul
    {
        double A_L[8*8], B_R[8*8];
        double C[8], Cl[8], Cr[8];
        aa_tf_duqu_mul(A,B,C);

        aa_tf_duqu_matrix_l(A, A_L, 8);
        cblas_dgemv( CblasColMajor, CblasNoTrans, 8, 8,
                     1.0, A_L, 8,
                     B, 1,
                     0, Cl, 1 );

        aveq( "duqu-mul-L", 8, C, Cl, 1e-6 );

        aa_tf_duqu_matrix_r(B, B_R, 8);
        cblas_dgemv( CblasColMajor, CblasNoTrans, 8, 8,
                     1.0, B_R, 8,
                     A, 1,
                     0, Cr, 1 );
        aveq( "duqu-mul-R", 8, C, Cr, 1e-6 );
    }
    // add / sub
    {
        double Ca[8], Cs[8], mB[8];

        for( size_t i = 0; i < 8; i ++ ) mB[i] = -B[i];
        aa_tf_duqu_add(A,B,Ca);
        aa_tf_duqu_sub(A,mB,Cs);
        aveq( "duqu-add-sub", 8, Ca, Cs, 1e-6 );

        double Cra[4], Crs[4];
        double Cda[4], Cds[4];
        aa_tf_duqu_sub(A,B,Cs);
        aa_tf_qadd(A+AA_TF_DUQU_REAL, B+AA_TF_DUQU_REAL,Cra);
        aa_tf_qadd(A+AA_TF_DUQU_DUAL, B+AA_TF_DUQU_DUAL,Cda);
        aa_tf_qsub(A+AA_TF_DUQU_REAL, B+AA_TF_DUQU_REAL,Crs);
        aa_tf_qsub(A+AA_TF_DUQU_DUAL, B+AA_TF_DUQU_DUAL,Cds);

        aveq( "duqu-qadd-real", 4, Cra, Ca+AA_TF_DUQU_REAL, 1e-6);
        aveq( "duqu-qadd-dual", 4, Cda, Ca+AA_TF_DUQU_DUAL, 1e-6);

        aveq( "duqu-qsub-real", 4, Crs, Cs+AA_TF_DUQU_REAL, 1e-6);
        aveq( "duqu-qsub-dual", 4, Cds, Cs+AA_TF_DUQU_DUAL, 1e-6);
    }

}

static void duqu(const double E[7], const double S_[8], const double T_[12],
                 const double p0[3], const double dx[6])
{
    double S_ident[8] = AA_TF_DUQU_IDENT_INITIALIZER;
    double Q_ident[4] = AA_TF_QUAT_IDENT_INITIALIZER;
    double v_ident[3] = {0};

    aa_tf_tfmat_t T;
    aa_tf_duqu_t H;
    AA_MEM_CPY(H.data, S_, 8);
    AA_MEM_CPY(T.data, T_, 12);

    // check trans
    double hv[3];
    aa_tf_duqu_trans(H.data, hv);
    aveq("duqu-trans", 3, T.v.data, hv, .001 );

    //double nreal,ndual;
    //aa_tf_duqu_norm( H.data, &nreal, &ndual );
    //printf("norm: %f + %f \\epsilon \n", nreal, ndual );

    // transform points
    double p1H[3], p1qv[3], p1T[3];
    aa_tf_12( T.data, p0, p1T );
    aa_tf_tf_qv( H.real.data, T.v.data, p0, p1qv );
    aa_tf_tf_duqu(  H.data, p0, p1H );

    aveq( "tf-qv",   3, p1T, p1qv, .001 );
    aveq( "tf-duqu", 3, p1T, p1H, .001 );

    // conjugate
    {

        double S_conj[8];
        double qv_conj[7], E_conj[7];
        double SSc[8], EEc[7];
        double Scv[3];

        aa_tf_duqu_conj(H.data, S_conj);
        aa_tf_qv_conj(H.real.data, T.v.data, qv_conj, qv_conj+4);
        aa_tf_qutr_conj(E, E_conj);

        aa_tf_duqu_trans(S_conj, Scv);

        aveq( "duqu/qutr conj q", 4, S_conj, E_conj, 1e-6 );
        aveq( "duqu/qv conj q", 4, S_conj, qv_conj, 1e-6 );
        aveq( "duqu/qutr conj v", 3, Scv, E_conj+4, 1e-6 );
        aveq( "duqu/qv conj v", 3, Scv, qv_conj+4, 1e-6 );

        aa_tf_duqu_mul( H.data, S_conj, SSc );
        aa_tf_qv_chain( H.real.data, T.v.data, qv_conj, qv_conj+4, EEc, EEc+4 );

        aveq( "duqu conj", 8, SSc, S_ident, 1e-6 );
        aveq( "qv conj q", 4, EEc, Q_ident, 1e-6 );
        aveq( "qv conj v", 3, EEc+4, v_ident, 1e-6 );
    }

    // derivative
    {
        double dd[8], dq[4];
        double dt = aa_frand() / 100;
        aa_tf_duqu_vel2diff( H.data, dx, dd );
        aa_tf_qvel2diff( H.real.data, dx+3, dq );

        // back to velocity
        double dx1[6];
        aa_tf_duqu_diff2vel( H.data, dd, dx1 );
        aveq( "duqu-vel invert", 6, dx, dx1, .001 );

        // integrate
        double H1[8], q1[4], v1[3], H1qv[8];
        double H1_sdd[8], H1_sdx[8];
        for( size_t i = 0; i < 8; i ++ ) H1[i] = H.data[i] + dd[i]*dt; // some numerical error here...
        for( size_t i = 0; i < 3; i ++ ) v1[i] = T.v.data[i] + dx[i]*dt;
        aa_tf_duqu_normalize( H1 );
        aa_tf_qsvel( H.real.data, dx+3, dt, q1 );
        aa_tf_qv2duqu( q1, v1, H1qv );
        aveq( "duqu-vel_real", 4, dq, dd, .001 );
        aveq( "duqu-vel-int real", 4, H1, H1qv, .001 );
        aveq( "duqu-vel-int dual", 4, H1+4, H1qv+4, .001 );
        aa_tf_duqu_svel( H.data, dx, dt, H1_sdx );
        aa_tf_duqu_sdiff( H.data, dd, dt, H1_sdd );
        aveq( "duqu-int vel", 8, H1qv, H1_sdx, .001 );
        aveq( "duqu-int diff", 8, H1_sdx, H1_sdd, .0001 );

        /* // twist */
        double tw[8], dxtw[6];
        aa_tf_duqu_vel2twist(H.data, dx, tw );
        aa_tf_duqu_twist2vel(H.data, tw, dxtw );
        aveq( "duqu twist<->vel", 6, dx, dxtw, 1e-6 );


    }


    // exponential
    {
        double expd[8], lnexpd[8], sv[6], ev[6];
        aa_tf_duqu_exp(H.data, expd );
        aa_tf_duqu_ln( expd, lnexpd );
        aveq( "duqu-exp-ln", 8, H.data, lnexpd, .001 );


        aa_tf_duqu_ln( H.data, lnexpd );
        aa_tf_duqu_exp(lnexpd, expd );
        aa_tf_duqu_lnv( H.data, sv );
        aa_tf_qutr_lnv( E, ev );
        aveq( "duqu-ln-exp", 8, H.data, expd, .001 );
        aveq( "duqu-lnv 0", 3, lnexpd+AA_TF_DUQU_REAL_XYZ, sv+AA_TF_DX_W, 1e-7 );
        aveq( "duqu-lnv 1", 3, lnexpd+AA_TF_DUQU_DUAL_XYZ, sv+AA_TF_DX_V, 1e-7 );
        aveq( "duqu-qutr-lnv", 6, sv, ev, 1e-7 );
    }

    // Logarithm
    {
        double HI[8], HIln[8], dxi[6], dx0[6] = {0};
        aa_tf_duqu_mulc( H.data, H.data, HI );
        aa_tf_duqu_ln(HI, HIln);
        aa_tf_duqu_twist2vel(HI, HIln, dxi );
        aveq( "duqu ln 0 near", 6, dx0, dxi, .0001 );

        aa_tf_duqu_ln(aa_tf_duqu_ident, HIln);
        aa_tf_duqu_twist2vel(HI, HIln, dxi );
        aveq( "duqu ln 0 exact", 6, dx0, dxi, 0.0 );

    }
    // Pure translation
    {
        double S[8], v[3], v1[3];
        aa_vrand(3,v);
        aa_tf_xyz2duqu( v[0], v[1], v[2], S );
        aa_tf_duqu_trans(S, v1);
        aveq( "duqu trans orientation", 4, S, aa_tf_quat_ident, 0.0 );
        aveq( "duqu trans translation", 3, v, v1, 1e-6 );
    }
}

static void rel_q(const double q0[4], const double q1[4], const double dx0[3])
{
    // Relative
    double qrel[4];
    aa_tf_qcmul(q0, q1, qrel );

    // random velocity, point 0
    double dq0[4];
    //aa_vrand(3,dx0);
    aa_tf_qvel2diff( q0, dx0, dq0 );

    // computed velocity, point 1
    double dq1[4], dx1[3];
    aa_tf_qmul( dq0, qrel, dq1 );
    aa_tf_qdiff2vel( q1, dq1, dx1 );

    // integrate both velocities
    double q0_1[4], q1_1[4];
    aa_tf_qsvel( q0, dx0, .1, q0_1 );
    aa_tf_qsvel( q1, dx1, .1, q1_1 );

    // new relative orientation
    double qrel_1[4];
    aa_tf_qcmul( q0_1, q1_1, qrel_1 );

    // check
    aveq("relq", 4, qrel, qrel_1, .000001);
}

static void rel_d(const double d0[8], const double d1[8], const double dx0[6])
{
    // dual quat transforms
    double drel[8], d1p[8];

    // d0 * drel = d1
    // drel = conj(d0) * d1
    aa_tf_duqu_cmul( d0, d1, drel );
    aa_tf_duqu_mul( d0, drel, d1p );
    aveq("duqu-relmul", 8, d1, d1p, .001 );

    // velocity
    double dd0[8];
    aa_tf_duqu_vel2diff(d0, dx0, dd0);

    // second velocity
    // d1 = d0*drel
    // d1/dt = d0/dt * drel + d0 * drel/dt, and drel/dt = 0
    double dd1[8];
    aa_tf_duqu_mul( dd0, drel, dd1 );

    // integrate
    double d0_1[8], d1_1[8];
    double dt = .1;
    aa_tf_duqu_sdiff( d0, dd0, dt, d0_1 );
    aa_tf_duqu_sdiff( d1, dd1, dt, d1_1 );
    aa_tf_duqu_normalize( d0_1 );
    aa_tf_duqu_normalize( d1_1 );

    // new relative
    double drel_1[8];
    // drel = d0*inv(d1)
    aa_tf_duqu_cmul( d0_1, d1_1, drel_1 );

    // twist
    double d0_1t[8], d1_1t[8], drel_1t[8];
    aa_tf_duqu_svel( d0, dx0, dt, d0_1t );
    aa_tf_duqu_sdiff( d1, dd1, dt, d1_1t );
    aa_tf_duqu_cmul( d0_1t, d1_1t, drel_1t );

    // check
    aveq("rel_d", 8, drel, drel_1t, 1e-6);
}

static void slerp() {
    double q[4], qy[4], u, du;
    double dq1[4], dq2[4], dqy[4];
    aa_tf_qurand(q);
    u = aa_frand();
    du = aa_frand();
    aa_vrand(4,dq1);
    aa_vrand(4,dq2);

    aa_tf_qslerpchaindiff( u, du, q, dq1, q, dq2, qy, dqy );
    aveq("chaindiff equiv", 4, q, qy, 1e-6);
}


static void rotmat(const double R[9], const double w[3])
{
    double dR[9], dRw[3];
    aa_tf_rotmat_vel2diff( R, w, dR );
    aa_tf_rotmat_diff2vel( R, dR, dRw );
    aveq("rotmat-vel", 3, w, dRw, 1e-6 );


    {
        double Rtmp[9];
        aa_tf_rotmat_xy( R+0, R+3, Rtmp );
        aveq( "rotmat_xy", 9, R, Rtmp, 1e-6 );

        aa_tf_rotmat_yz( R+3, R+6, Rtmp );
        aveq( "rotmat_yz", 9, R, Rtmp, 1e-6 );

        aa_tf_rotmat_zx( R+6, R+0, Rtmp );
        aveq( "rotmat_zx", 9, R, Rtmp, 1e-6 );
    }
}

static void tfmat_exp_ln(const double v[6]) {
    double evR[9], ev[12], lneR[3], lne[6];

    aa_tf_tfmat_expv( v, ev );
    aa_tf_rotmat_expv( v+3, evR );
    aveq( "rotmat-tfmat-exp", 9, evR, ev, 1e-6 );
    aa_tf_tfmat_lnv( ev, lne );
    aa_tf_rotmat_lnv( evR, lneR );

    aveq( "rotmat-exp-ln", 3, v+3, lneR, 1e-6 );
    aveq( "tfmat-exp-ln", 6, v, lne, 1e-6 );
}

static void tfmat_inv(const double T[12]) {
    double Tc[12], Ti[12], Tr[12];

    // aa_tf_rotmat_inv1
    AA_MEM_CPY(Tc, T, 12);
    aa_tf_rotmat_inv1(Tc);
    aa_tf_rotmat_mul(T,Tc,Tr);
    aveq( "aa_tf_rotmat_inv1", 9, Tr, aa_tf_rotmat_ident, 1e-6 );

    // aa_tf_rotmat_inv2
    AA_MEM_CPY(Tc, T, 12);
    aa_tf_rotmat_inv2(Tc, Ti);
    aa_tf_rotmat_mul(T,Ti,Tr);
    aveq( "aa_tf_rotmat_inv2", 9, Tr, aa_tf_rotmat_ident, 1e-6 );

    // aa_tf_tfmat_inv1
    AA_MEM_CPY(Tc, T, 12);
    aa_tf_tfmat_inv1(Tc);
    aa_tf_tfmat_mul(T,Tc,Tr);
    aveq( "aa_tf_tfmat_inv1", 12, Tr, aa_tf_tfmat_ident, 1e-6 );

    // aa_tf_rotmat_inv2
    AA_MEM_CPY(Tc, T, 12);
    aa_tf_tfmat_inv2(Tc, Ti);
    aa_tf_tfmat_mul(T,Ti,Tr);
    aveq( "aa_tf_tfmat_inv2", 12, Tr, aa_tf_tfmat_ident, 1e-6 );
}

static void mzlook( const double eye[3],
                    const double target[3],
                    const double up[3] )
{

    {
        double R[9] = {0};
        aa_tf_rotmat_mzlook(eye,target,up,R);
        assert( aa_tf_isrotmat(R) );
    }

    double g_E_cam[7];
    double cam_E_g[7];
    aa_tf_qutr_mzlook(eye,target,up,g_E_cam);
    aa_tf_qutr_conj(g_E_cam, cam_E_g);
    {
        double x[3];
        aa_tf_qutr_tf(g_E_cam, aa_tf_vec_ident, x);
        aveq("mzlook eye 0", 3, eye, x, 1e-6 );
        aa_tf_qutr_tf(cam_E_g, eye, x);
        aveq("mzlook eye 1", 3, aa_tf_vec_ident, x, 1e-6 );
    }
    {
        double ell[3];
        for(size_t i=0; i<3; i++) ell[i] = -(target[i] - eye[i]);
        aa_tf_vnormalize(ell);
        double z[3];
        aa_tf_qrot(cam_E_g, ell, z);
        aveq("mzlook rot", 3, z, aa_tf_vec_z, 1e-6 );
    }

}

static void integrate(const double *E, const double *S, const double *T,
                      const double *dx, const double *ddx )
{
    const double *q = E+AA_TF_QUTR_Q;
    /* const double *v = E+AA_TF_QUTR_V; */
    /* const double *dv = dx+AA_TF_DX_V; */
    const double *w = dx+AA_TF_DX_W;
    const double *a = ddx+AA_TF_DX_W;
    const double *R = T+AA_TF_TFMAT_R;
    double dt = aa_frand() / 100;

    { /* quatenion */
        double dq[4], wp[3];
        aa_tf_qvel2diff(q, w, dq);
        aa_tf_qdiff2vel(q,dq,wp);
        aveq( "qdiff<->vel", 3, w, wp, 1e-5 );

    }


    { /* rotmat */
        double dR[9], wp[3];
        aa_tf_rotmat_vel2diff(R, w, dR);
        aa_tf_rotmat_diff2vel(R, dR, wp );
        aveq( "rotmat diff<->vel", 3, w, wp, 1e-5 );

    }

    { /* quaternion translation */
        double dE[7], dxp[6];
        aa_tf_qutr_vel2diff( E, dx, dE );
        aa_tf_qutr_diff2vel( E, dE, dxp );
        aveq( "qutr diff<->vel", 6, dx, dxp, 1e-5 );
    }

    { /* dual quaternion */
        double dS[8], dxp[6];
        aa_tf_duqu_vel2diff( S, dx, dS );
        aa_tf_duqu_diff2vel( S, dS, dxp );
        aveq( "duqu diff<->vel", 6, dx, dxp, 1e-5 );
    }

    // integrate velocity
    {
        double S1[8], q1[4], T1[12], R1[9], E1[7];
        aa_tf_duqu_svel( S, dx, dt, S1 );
        aa_tf_qsvel( q, dx+3, dt, q1 );
        aa_tf_rotmat_svel( T, dx+3, dt, R1 );
        aa_tf_tfmat_svel( T, dx, dt, T1 );
        aa_tf_qutr_svel( E, dx, dt, E1 );

        // normalize
        double R1q[4], T1q[8], E1q[8];
        aa_tf_rotmat2quat( R1, R1q );
        aa_tf_tfmat2duqu( T1, T1q );
        aa_tf_qutr2duqu(E1, E1q);
        aa_tf_duqu_minimize( S1 );
        aa_tf_duqu_minimize( T1q );
        aa_tf_duqu_minimize( E1q );
        aa_tf_qminimize( q1 );
        aa_tf_qminimize( R1q );

        // check
        aveq( "duqu-quat", 4, S, q, 0 );
        aveq( "int-duqu-quat", 4, S1, q1, 1e-8 );
        aveq( "int-rotmat-quat", 4, R1q, q1, 1e-8 );
        aveq( "int-duqu-tfmat", 8, S1, T1q, 1e-6 );
        aveq( "int-qutr", 8, S1, E1q, 1e-6 );


        // normalized check
        aa_tf_duqu_normalize( T1q );
        aa_tf_duqu_normalize( S1 );
        aveq( "int-duqu-tfmat-norm", 8, S1, T1q, 1e-7 );
    }
    // integrate acceleration
    {
        double q1[4], q1rk[4];
        aa_tf_qsacc_rk(q, w, a, dt, q1rk);
        aa_tf_qsacc(q, w, a, dt, q1);
        aa_tf_qnormalize(q1);
        aa_tf_qminimize(q1);
        aa_tf_qnormalize(q1rk);
        aa_tf_qminimize(q1rk);
        //aveq( "aa_tf_qsacc", 4, q1, q1rk, 1e-7 );
        aveq( "aa_tf_qsacc", 4, q1, q1rk, 1e-7 );
    }

}

static void qvmul(const double q[4], const double v_[3])  {
    double v[4], r1[4], r2[4];
    AA_MEM_CPY(v, v_, 3);
    v[3] = 0;

    aa_tf_qmul_qv( q, v, r1);
    aa_tf_qmul(  q, v, r2);
    aveq( "qmul_qv", 4, r1, r2, 1e-7 );

    aa_tf_qmul_vq( v, q, r1);
    aa_tf_qmul(  v, q, r2);
    aveq( "qmul_v", 4, r1, r2, 1e-7 );
}

static void tf_conj1(const double E[7], const double S[8])
{
    double S2[8], SE[7], E2[8];

    aa_tf_duqu_conj(S, S2);
    aa_tf_qutr_conj(E, E2);
    aa_tf_duqu2qutr(S2, SE);
    aveq( "duqu/qutr conj", 7, E2, SE, 1e-7 );
}

static void tf_conj(const double E[2][7], const double S[2][8])
{
    double S2[8], SE[7], E2[8];

    aa_tf_duqu_mul(S[0], S[1], S2);
    aa_tf_qutr_mul(E[0], E[1], E2);
    aa_tf_duqu2qutr(S2, SE);
    aveq( "duqu/qutr mul", 7, E2, SE, 1e-7 );

    aa_tf_duqu_mulc(S[0], S[1], S2);
    aa_tf_qutr_mulc(E[0], E[1], E2);
    aa_tf_duqu2qutr(S2, SE);
    aveq( "duqu/qutr mulc", 7, E2, SE, 1e-7 );

    aa_tf_duqu_cmul(S[0], S[1], S2);
    aa_tf_qutr_cmul(E[0], E[1], E2);
    aa_tf_duqu2qutr(S2, SE);
    aveq( "duqu/qutr cmul", 7, E2, SE, 1e-7 );
}

static void qdiff(const double E[2][7], const double dx[2][6] )
{
    const double *q0 = E[0];
    const double *q1 = E[1];
    const double *w0 = dx[0]+3;
    const double *w1 = dx[1]+3;

    // Velocity and Quaternion Derivatives
    double dq0[4], dq1[4];
    aa_tf_qvel2diff(q0, w0, dq0);
    aa_tf_qvel2diff(q1, w1, dq1);
    {
        double wt0[3], wt1[3];
        aa_tf_qdiff2vel( q0, dq0, wt0 );
        aa_tf_qdiff2vel( q1, dq1, wt1 );
        aveq("qvel/diff 0", 3, w0, wt0, 1e-7 );
        aveq("qvel/diff 1", 3, w1, wt1, 1e-7 );
    }

    // Log and Exponential Derivatives
    {

        double pp = aa_tf_qangle(q0);

        double u[3], du[3], duj[3];
        double e[4], de[4], dej[4];
        aa_tf_qln( q0, u );
        aa_tf_qexp(u, e);
        aveq( "qln/exp", 4, q0, e, 1e-7 );

        aa_tf_qduln( q0, dq0, du );
        if (pp > AA_TF_EPSILON) {
            // Jacobian isn't robust at 0
            aa_tf_qdulnj( q0, dq0, duj );
            aveq( "dln/dlnj", 3, du, duj, 1e-7 );
        }

        aa_tf_qdpexp( u, du, de );
        aa_tf_qdpexpj( u, du, dej );
        aveq( "dexp/dexpj", 4, de, dej, 1e-7 );

        aveq( "qdln/qdexpq", 4, dq0, de, 1e-7 );


        //aa_tf_qdpexpj( ln0, dlnj0, de0 );
        //aveq( "qdln/dexp", 4, dq0, de0, 1e-4 );
    }

}

static void normalize(const double *Rp, const double *qp) {
    double R[9], q[4], Rq[4];
    AA_MEM_CPY(R, Rp, 9);
    AA_MEM_CPY(q, qp, 4);

    aa_tf_qnormalize(q);
    aa_tf_rotmat_normalize(R);
    aa_tf_rotmat2quat( R, Rq );

    aa_tf_qminimize(q);
    aa_tf_qminimize(Rq);

    aa_test_quat_cmp( "normalize", q, Rq, 1e-7 );
}

static void rotate(const double q[4], const double aa[4], const double R[9],
                   const double p[3])
{
    double pq[3], Rq[3], aq[3];
    aa_tf_qrot(q,p,pq);
    aa_tf_rotmat_rot(R,p,Rq);
    aa_tf_axang_rot(aa,p,aq);

    aveq( "rotate quat-rotmat", 3, pq, Rq, 1e-9 );
    aveq( "rotate rotmat-axang",  3, Rq, aq, 1e-9 );
    aveq( "rotate quat-axang",  3, pq, aq, 1e-9 );
}

static void cross(double a[3], double b[3])
{
    double c[3], Mr[9], d[3];
    struct aa_dmat M;
    aa_dmat_view(&M,3,3,Mr,3);
    aa_tf_cross(a,b,c);

    aa_tf_cross_mat_l(a,&M);
    aa_tf_rotmat_rot(M.data,b,d);
    aveq( "cross-left",  3, c, d, 1e-9 );

    aa_tf_cross_mat_r(b,&M);
    aa_tf_rotmat_rot(M.data,a,d);
    aveq( "cross-right",  3, c, d, 1e-9 );
}

static void dhparam()
{
    double dh[4];
    rand_dh(dh);

    double T[12], E[7], S[8], ET[7], ES[7], SE[8];

    aa_tf_dhprox2tfmat(dh[0], dh[1], dh[2], dh[3], T);
    aa_tf_dhprox2qutr(dh[0], dh[1], dh[2], dh[3], E);
    aa_tf_dhprox2duqu(dh[0], dh[1], dh[2], dh[3], S);
    aa_tf_tfmat2qutr( T, ET );
    aa_tf_duqu2qutr( S, ES );
    aa_tf_qutr2duqu( E, SE );

    aa_tf_qminimize(E + AA_TF_QUTR_Q);
    aa_tf_qminimize(ES + AA_TF_QUTR_Q);
    aa_tf_qminimize(ET + AA_TF_QUTR_Q);
    aa_tf_duqu_minimize( S );
    aa_tf_duqu_minimize( SE );

    aveq( "dhprox tfmat qutr", 7, ET, E, 1e-5 );
    aveq( "dhprox qutr duqu (S)", 8, SE, S, 1e-5 );

    aveq( "dhprox tfmat duqu", 7, ET, ES, 1e-5 );

    aa_tf_dhdist2tfmat(dh[0], dh[1], dh[2], dh[3], T);
    aa_tf_dhdist2qutr(dh[0], dh[1], dh[2], dh[3], E);
    aa_tf_dhdist2duqu(dh[0], dh[1], dh[2], dh[3], S);
    aa_tf_tfmat2qutr( T, ET );
    aa_tf_duqu2qutr( S, ES );
    aa_tf_qminimize(E + AA_TF_QUTR_Q);
    aa_tf_qminimize(ES + AA_TF_QUTR_Q);
    aa_tf_qminimize(ET + AA_TF_QUTR_Q);

    aveq( "dhdist tfmat qutr", 7, ET, E, 1e-5 );
    aveq( "dhdist tfmat duqu", 7, ET, ES, 1e-5 );

}

typedef void (raw_vector_field_fun)(const double*,double*);

static void pde_j_helper( void *cx, const struct aa_dvec *x, struct aa_dvec *y)
{
    void (*f)(const double*,double*) = (raw_vector_field_fun*)cx;
    f(x->data, y->data);

}

static void q_pde( const double *q )
{
    double dJfd[4*4], dJ[4*4];
    struct aa_dmat Jfd = AA_DMAT_INIT(4,4,dJfd,4);
    struct aa_dmat J = AA_DMAT_INIT(4,4,dJ,4);

    aa_tf_qln_jac( q, &J );

    struct aa_dvec vq = AA_DVEC_INIT(4,(double*)q,1);
    aa_de_jac_fd( pde_j_helper, aa_tf_qln, &vq, 1e-6, &Jfd );


    aveq( "qln_jac / fd ", 4*4, dJfd, dJ, 1e-3 );
}

static void dq_pde( const double *S )
{
    double dJfd[8*8], dJ[8*8];
    struct aa_dmat Jfd = AA_DMAT_INIT(8,8,dJfd,8);
    struct aa_dmat J = AA_DMAT_INIT(8,8,dJ,8);
    struct aa_dvec vS = AA_DVEC_INIT(8,(double*)S,1);

    {
        aa_tf_duqu_ln_jac( S, &J );

        aa_de_jac_fd( pde_j_helper, aa_tf_duqu_ln, &vS, 1e-6, &Jfd );

        /* printf("\n--\nJ_fd:\n"); */
        /* aa_dump_mat(stdout,dJfd,8,8); */
        /* printf("J:\n"); */
        /* aa_dump_mat(stdout,dJ,8,8); */

        aveq( "duqu_ln_jac / fd ", 8*8, dJfd, dJ, 1e-3 );
    }
    {
        aa_tf_duqu_conj_jac( &J );
        aa_de_jac_fd( pde_j_helper, aa_tf_duqu_conj, &vS, 1e-6, &Jfd );
        aveq( "duau_conj_jac / fd ", 8*8, dJfd, dJ, 1e-3 );
    }
}

static void test_rotations(const double q[4], const double aa[4],
                           const double rv[3], const double R[9],
                           const double p[3])
{
    rotate(q, aa, R, p);
    rotvec(rv, aa, q, R);
    normalize(R, q);

    eulerzyx(q, R);

    rotmat(R, p);
}

static void test_rotations2(const double q1[4], const double q2[4],
                            const double R1[9], const double R2[9],
                            const double p1[3], const double p2[3])
{
    quat(q1, q2, R1, R2, p1, p2[0]);

    rel_q(q1, q2, p1);
}

static void test_tf(const double E[7], const double S[8], const double T[12],
                    const double p[3], const double dx[6])
{
    duqu(E, S, T, p, dx);
    tf_conj1(E,S);

    tfmat_inv(T);
}

static void test_tf2(double E[2][7], double S[2][8], double T[2][12],
                     const double dx[2][6],
                     const double p[2][3])
{
    for (size_t j = 0; j < 2; j++) {
        test_tf(E[j], S[j], T[j], p[j], dx[j]);

        integrate(E[j], S[j], T[j], dx[0], dx[1]);
    }
    chain(E, S, T);
    tf_conj(E, S);

    qdiff(E, dx);

    rel_d(S[0], S[1], dx[0]);
}

int main(int argc, char *argv[])
{
    // init
    aa_test_args(argc, argv);
    aa_test_ulimit();

    // TODO: also generate random rotation via quaternions
    // TODO: also generation rotations and random axes near singularities

    /* Not at singularities */
    for( size_t i = 0; i < 1000; i++ ) {
        /* Random Data */
        static const size_t k=2;
        double aa[2][4], rv[2][3], E[2][7], S[2][8], T[2][12], dx[2][6];
        double p[2][3];

        for (size_t j = 0; j < k; j++) {
            rand_tf(aa[j], rv[j], E[j], S[j], T[j]);
            rand_point(p[j]);
            aa_vrand(6,dx[j]);
            for( size_t l = 0; l < 6; l++ ) {
                dx[j][l] -= 0.5;
            }
        }

        /* Run Tests */
        for (size_t j = 0; j < k; j++) {
            test_rotations(E[j], aa[j], rv[j], T[j], p[j]);
        }

        test_tf2(E, S, T, dx, p);

        test_rotations2(E[0], E[1], T[0], T[1], p[0], p[1]);

        vecs2quat(p[0], p[1]);

        mzlook(dx[0]+0, dx[0]+3, dx[1]+0);

        cross(p[0], p[1]);

        q_pde( E[0]+AA_TF_QUTR_Q );
        dq_pde( S[0] );
    }

    // log and exp
    for( size_t i = 0; i < 1000; i++ ) {
        double v[6];
        aa_test_randv(-1, 1, 6, v); // TODO: range and near singularities
        tfmat_exp_ln(v);
    }

    // general quaternions
    for( size_t i = 0; i < 1000; i++ ) {
        double A[8], B[8];
        aa_test_randv(-1, 1, 8, A);
        aa_test_randv(-1, 1, 8, B);
        duqu_gen(A, B);

        qvmul(A, A+4);
    }

    // misc
    for( size_t i = 0; i < 1000; i++ ) {
        slerp();
        dhparam();
    }

    return 0;
}
