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

static void rotvec() {
    double e[3],  R[9], q[4], vr[3], vq[3];
    aa_vrand(3,e);

    aa_tf_rotvec2quat(e, q);
    aa_tf_quat2rotmat(q, R);
    assert( aa_tf_isrotmat(R) );


    aa_tf_rotmat2rotvec(R, vr);
    aa_tf_quat2rotvec(q, vq);

    aveq("rotvec", 3, vq, vr, .001 );

    {
        double ee[9], eln[3], qe[4];
        aa_tf_rotmat_exp_rv( e, ee );
        aa_tf_rotmat2quat( ee, qe );
        aa_tf_quat2rotvec( qe, eln );
        aveq("rotmat_exp_rv", 3, e, eln, 1e-6 );
    }
    {
        double aa[4], ee[9], eln[3], qe[4];
        aa_tf_rotvec2axang(e, aa);
        aa_tf_rotmat_exp_aa( aa, ee );
        aa_tf_rotmat2quat( ee, qe );
        aa_tf_quat2rotvec( qe, eln );
        aveq("rotmat_exp_aa", 3, e, eln, 1e-6 );
    }
}

typedef void (*fun_type)(double,double,double, double*b);

static void euler_helper( double e[4], fun_type e2r, fun_type e2q ) {
    double R[9],  q[4];
    e2r(e[0], e[1], e[2], R);
    aa_tf_isrotmat(R) ;

    e2q(e[0], e[1], e[2], q);

    double vq[3], vr[3];
    aa_tf_quat2rotvec(q, vq);
    aa_tf_rotmat2rotvec(R, vr);

    aveq("euler-vecs", 3, vq, vr, .001 );
}

static void euler() {
    double e[3];
    aa_vrand(3,e);

    euler_helper( e, &aa_tf_eulerxyz2rotmat, &aa_tf_eulerxyz2quat );
    euler_helper( e, &aa_tf_eulerxzy2rotmat, &aa_tf_eulerxzy2quat );

    euler_helper( e, &aa_tf_euleryxz2rotmat, &aa_tf_euleryxz2quat );
    euler_helper( e, &aa_tf_euleryzx2rotmat, &aa_tf_euleryzx2quat );

    euler_helper( e, &aa_tf_eulerzxy2rotmat, &aa_tf_eulerzxy2quat );
    euler_helper( e, &aa_tf_eulerzyx2rotmat, &aa_tf_eulerzyx2quat );


    euler_helper( e, &aa_tf_eulerxyx2rotmat, &aa_tf_eulerxyx2quat );
    euler_helper( e, &aa_tf_eulerxzx2rotmat, &aa_tf_eulerxzx2quat );

    euler_helper( e, &aa_tf_euleryxy2rotmat, &aa_tf_euleryxy2quat );
    euler_helper( e, &aa_tf_euleryzy2rotmat, &aa_tf_euleryzy2quat );

    euler_helper( e, &aa_tf_eulerzxz2rotmat, &aa_tf_eulerzxz2quat );
    euler_helper( e, &aa_tf_eulerzyz2rotmat, &aa_tf_eulerzyz2quat );

}

static void euler1() {
    double a[1];
    aa_vrand(1, a);
    double g = a[0];
    double Rx[9], Ry[9], Rz[9];
    double eRx[9], eRy[9], eRz[9];

    aa_tf_xangle2rotmat( g, Rx );
    aa_tf_yangle2rotmat( g, Ry );
    aa_tf_zangle2rotmat( g, Rz );

    aa_tf_eulerzyx2rotmat(g, 0, 0, eRz);
    aa_tf_eulerzyx2rotmat(0, g, 0, eRy);
    aa_tf_eulerzyx2rotmat(0, 0, g, eRx);

    aveq("euler-x", 9, Rx, eRx, .001 );
    aveq("euler-y", 9, Ry, eRy, .001 );
    aveq("euler-z", 9, Rz, eRz, .001 );
}


static void chain() {
    double e1[3], e2[3], R1[9], R2[9], q1[4], q2[4];
    aa_vrand(3,e1);
    aa_vrand(3,e2);

    aa_tf_eulerzyx2rotmat(e1[0], e1[1], e1[2], R1);
    aa_tf_eulerzyx2rotmat(e2[0], e2[1], e2[2], R2);
    aa_tf_eulerzyx2quat(e1[0], e1[1], e1[2], q1);
    aa_tf_eulerzyx2quat(e2[0], e2[1], e2[2], q2);


    double q[4], R[9];
    aa_tf_qmul( q1, q2, q );
    aa_tf_9mul( R1, R2, R );

    double vq[3], vr[3];
    aa_tf_rotmat2rotvec(R, vr);
    aa_tf_quat2rotvec(q, vq);

    aveq("chain-rot", 3, vr, vq, .001 );
}


static void quat() {
    double q1[4], q2[4], u;
    aa_test_qurand( q1 );
    aa_test_qurand( q2 );
    u = aa_frand();

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

    double R1[9], R2[9], Rr[9], qr[4], qrr[4];
    aa_tf_quat2rotmat(q1, R1);
    aa_tf_quat2rotmat(q2, R2);
    aa_tf_9rel( R1, R2, Rr );
    aa_tf_qrel( q1, q2, qr );
    aa_tf_rotmat2quat( Rr, qrr );
    if(qr[3] < 0 ) for( size_t i = 0; i < 4; i ++ ) qr[i] *= -1;
    if(qrr[3] < 0 ) for( size_t i = 0; i < 4; i ++ ) qrr[i] *= -1;
    aveq("qrel", 4, qr, qrr, .001 );

    // minimize
    {
        double qmin[4], axang[4];
        aa_tf_qminimize2( q1, qmin );
        test( "quat-minimize",  fabs(q1[3]) == qmin[3] );
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
    double w[3]={0}, dq[4], wdq[3];
    aa_vrand( 3, w );
    aa_tf_qvel2diff( q1, w, dq );
    aa_tf_qdiff2vel( q1, dq, wdq );
    aveq("qveldiff", 3, w, wdq, .000001);

    // integrate

    double qn_rk1[4], qn_vrk1[4], qn_vrk4[4], qn_vexp[4],w0[3] = {0};
    double dt = .02;


    aa_tf_qrk1( q1, dq, dt, qn_rk1 );
    aa_tf_qvelrk1( q1, w, dt, qn_vrk1 );
    aa_tf_qvelrk4( q1, w, dt, qn_vrk4 );
    aa_tf_qsvel( q1, w, dt, qn_vexp );
    aveq("qvelrk1", 4, qn_rk1, qn_vrk1, .001 );
    aveq("qvelrk4", 4, qn_rk1, qn_vrk4, .001 );
    aveq("qvelexp", 4, qn_vrk4, qn_vexp, .0001);
    aa_tf_qsvel( q1, w0, dt, qn_vexp );
    aveq("qvelsvel0", 4, q1, qn_vexp, .000 );

}


static void duqu() {

    // random tf
    double q[4], v[3], p0[3];
    aa_vrand( 3, v );
    aa_test_qurand( q );
    //AA_MEM_SET( v, 0, 3 );
    aa_vrand( 3, p0 );

    // tfmat
    aa_tf_tfmat_t T;
    aa_tf_quat2rotmat(q, T.R);
    AA_MEM_CPY( &T.t.x, v, 3 );

    // dual quat
    aa_tf_duqu_t H, H_tran;
    aa_tf_qv2duqu( q, v, H.data );
    aa_tf_qv2duqu( aa_tf_quat_ident, v, H_tran.data );

    // check trans
    double hv[3];
    aa_tf_duqu_trans(H.data, hv);
    aveq("duqu-trans", 3, v, hv, .001 );

    //double nreal,ndual;
    //aa_tf_duqu_norm( H.data, &nreal, &ndual );
    //printf("norm: %f + %f \\epsilon \n", nreal, ndual );

    // transform points
    double p1H[3], p1qv[3], p1T[3];
    aa_tf_12( T.data, p0, p1T );
    aa_tf_qv( q, v, p0, p1qv );
    aa_tf_duqu(  H.data, p0, p1H );

    aveq( "tf-qv",   3, p1T, p1qv, .001 );
    aveq( "tf-duqu", 3, p1T, p1H, .001 );

    // derivative
    {
        double dx[6], dd[8], dq[4];
        aa_vrand(6, dx);
        double dt = aa_frand() / 100;
        aa_tf_duqu_vel2diff( H.data, dx, dd );
        aa_tf_qvel2diff( q, dx+3, dq );

        // back to velocity
        double dx1[6];
        aa_tf_duqu_diff2vel( H.data, dd, dx1 );
        aveq( "duqu-vel invert", 6, dx, dx1, .001 );

        // integrate
        double H1[8], q1[4], v1[3], H1qv[8];
        double H1_sdd[8], H1_sdx[8];
        for( size_t i = 0; i < 8; i ++ ) H1[i] = H.data[i] + dd[i]*dt; // some numerical error here...
        for( size_t i = 0; i < 3; i ++ ) v1[i] = v[i] + dx[i]*dt;
        aa_tf_duqu_normalize( H1 );
        aa_tf_qsvel( q, dx+3, dt, q1 );
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
        double expd[8], lnexpd[8];
        aa_tf_duqu_exp(H.data, expd );
        aa_tf_duqu_ln( expd, lnexpd );
        aveq( "duqu-exp-ln", 8, H.data, lnexpd, .001 );
        aa_tf_duqu_ln( H.data, lnexpd );
        aa_tf_duqu_exp(lnexpd, expd );
        aveq( "duqu-ln-exp", 8, H.data, expd, .001 );
    }
    {
        double expd[8], lnexpd[8];
        aa_tf_duqu_exp(H_tran.data, expd );
        aa_tf_duqu_ln( expd, lnexpd );
        aveq( "duqu-tran exp", 8, H_tran.data, lnexpd, .001 );
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
}


void rel_q() {
    // random transforms
    double q0[4], qrel[4], q1[4];
    aa_vrand(4,q0);
    aa_vrand(4,q1);
    aa_tf_qnormalize(q0);
    aa_tf_qnormalize(q1);
    aa_tf_qcmul(q0, q1, qrel );

    // random velocity, point 0
    double dx0[3]={0}, dq0[4];
    aa_vrand(3,dx0);
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

void rel_d() {
    // random transforms
    double q0[4], v0[3], q1[4], v1[4];
    aa_vrand(4,q0);
    aa_vrand(4,q1);
    aa_tf_qnormalize(q0);
    aa_tf_qnormalize(q1);
    aa_vrand(3,v0);
    aa_vrand(3,v1);

    // dual quat transforms
    double d0[8], drel[8], d1[8], d1p[8];
    aa_tf_qv2duqu(q0,v0, d0);
    aa_tf_qv2duqu(q1,v1, d1);
    // d0 * drel = d1
    // drel = conj(d0) * d1
    aa_tf_duqu_cmul( d0, d1, drel );
    aa_tf_duqu_mul( d0, drel, d1p );
    aveq("duqu-relmul", 8, d1, d1p, .001 );

    // random velocity
    double dx0[6], dd0[8];
    aa_vrand(6,dx0);
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
    aa_test_qurand(q);
    u = aa_frand();
    du = aa_frand();
    aa_vrand(4,dq1);
    aa_vrand(4,dq2);

    aa_tf_qslerpchaindiff( u, du, q, dq1, q, dq2, qy, dqy );
    aveq("chaindiff equiv", 4, q, qy, 1e-6);
}


static void theta2quat() {
    double theta = (aa_frand() - 0.5) * 2 * M_PI;
    double qx[4], qy[4], qz[4];
    double Rx[9], Ry[9], Rz[9];
    double qRx[4], qRy[4], qRz[4];
    aa_tf_xangle2rotmat( theta, Rx );
    aa_tf_yangle2rotmat( theta, Ry );
    aa_tf_zangle2rotmat( theta, Rz );

    aa_tf_xangle2quat( theta, qx );
    aa_tf_yangle2quat( theta, qy );
    aa_tf_zangle2quat( theta, qz );

    aa_tf_rotmat2quat( Rx, qRx );
    aa_tf_rotmat2quat( Ry, qRy );
    aa_tf_rotmat2quat( Rz, qRz );

    aa_tf_qminimize( qx );
    aa_tf_qminimize( qRx );
    aa_tf_qminimize( qy );
    aa_tf_qminimize( qRy );
    aa_tf_qminimize( qz );
    aa_tf_qminimize( qRz );

    aveq("xangle2quat", 4, qx, qRx, 1e-6 );
    aveq("yangle2quat", 4, qy, qRy, 1e-6 );
    aveq("xangle2quat", 4, qz, qRz, 1e-6 );
}


static void rotmat() {
    double q[4], R[9], w[3], dR[9], dRw[3];
    aa_test_qurand( q );
    aa_vrand( 3, w );
    aa_tf_quat2rotmat(q, R);
    aa_tf_rotmat_vel2diff( R, w, dR );
    aa_tf_rotmat_diff2vel( R, dR, dRw );
    aveq("rotmat-vel", 3, w, dRw, 1e-6 );


}

int main( void ) {
    // init
    srand((unsigned int)time(NULL)); // might break in 2038
    aa_test_ulimit();

    for( size_t i = 0; i < 1000; i++ ) {
        rotvec();
        euler();
        euler1();
        chain();
        quat();
        duqu();
        rel_q();
        rel_d();
        slerp();
        theta2quat();
        rotmat();
    }

    return 0;
}
