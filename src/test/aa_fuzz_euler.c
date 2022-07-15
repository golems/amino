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


typedef void (*fun_type)(double,double,double, double*b);

static void euler_helper( const double e[4], fun_type e2r, fun_type e2q ) {
    double R[9],  q[4];
    e2r(e[0], e[1], e[2], R);
    aa_tf_isrotmat(R) ;

    e2q(e[0], e[1], e[2], q);

    double vq[3], vr[3];
    aa_tf_quat2rotvec(q, vq);
    aa_tf_rotmat2rotvec(R, vr);

    aveq("euler-vecs", 3, vq, vr, .001 );
}

static void euler(const double *e) {

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

static void euler1(const double g)
{
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

static void theta2quat(double theta)
{
    double qx[4], qy[4], qz[4];
    double Rx[9], Ry[9], Rz[9];
    double qRx[4], qRy[4], qRz[4];
    double ax[4], ay[4], az[4];
    double qax[4], qay[4], qaz[4];

    aa_tf_xangle2rotmat( theta, Rx );
    aa_tf_yangle2rotmat( theta, Ry );
    aa_tf_zangle2rotmat( theta, Rz );

    aa_tf_xangle2quat( theta, qx );
    aa_tf_yangle2quat( theta, qy );
    aa_tf_zangle2quat( theta, qz );

    aa_tf_xangle2axang( theta, ax );
    aa_tf_yangle2axang( theta, ay );
    aa_tf_zangle2axang( theta, az );

    aa_tf_rotmat2quat( Rx, qRx );
    aa_tf_rotmat2quat( Ry, qRy );
    aa_tf_rotmat2quat( Rz, qRz );

    aa_tf_axang2quat( ax, qax );
    aa_tf_axang2quat( ay, qay );
    aa_tf_axang2quat( az, qaz );

    aa_tf_qminimize( qx );
    aa_tf_qminimize( qRx );
    aa_tf_qminimize( qy );
    aa_tf_qminimize( qRy );
    aa_tf_qminimize( qz );
    aa_tf_qminimize( qRz );

    aveq("xangle2quat", 4, qx, qRx, 1e-6 );
    aveq("yangle2quat", 4, qy, qRy, 1e-6 );
    aveq("xangle2quat", 4, qz, qRz, 1e-6 );

    aveq("xangle2quat", 4, qx, qax, 1e-6 );
    aveq("yangle2quat", 4, qy, qay, 1e-6 );
    aveq("xangle2quat", 4, qz, qaz, 1e-6 );
}


int main(int argc, char *argv[])
{
    // init
    aa_test_args(argc, argv);
    aa_test_ulimit();

    for( size_t i = 0; i < 1000; i++ ) {
        double e[3];
        aa_test_randv(-.9*M_PI, .9*M_PI, 3, e);

        euler(e);
        euler1(e[0]);
        theta2quat(e[0]);
    }


    return 0;
}
