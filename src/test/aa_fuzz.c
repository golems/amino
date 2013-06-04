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
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>

static void aveq( const char * name,
                  size_t n, double *a, double *b, double tol ) {
    if( !aa_veq(n, a, b, tol) ) {
        fprintf( stderr, "FAILED: %s\n",name);
        fprintf( stderr, "a: ");
        aa_dump_vec( stderr, a, n );
        fprintf( stderr, "b: ");
        aa_dump_vec( stderr, b, n );

        assert( 0 );
    }
}

static void rotvec() {
    double e[3], R[9], q[4], vr[3], vq[3];
    aa_vrand(3,e);

    aa_tf_rotvec2quat(e, q);
    aa_tf_quat2rotmat(q, R);
    assert( aa_tf_isrotmat(R) );

    aa_tf_rotmat2rotvec(R, vr);
    aa_tf_quat2rotvec(q, vq);

    aveq("rotvec", 3, vq, vr, .001 );
}


static void euler() {
    double e[3], R[9],  q[4];
    aa_vrand(3,e);

    aa_tf_eulerzyx2rotmat(e, R);
    assert( aa_tf_isrotmat(R) );

    aa_tf_eulerzyx2quat(e, q);

    double vq[3], vr[3];
    aa_tf_quat2rotvec(q, vq);
    aa_tf_rotmat2rotvec(R, vr);

    aveq("euler-vecs", 3, vq, vr, .001 );
}

static void euler1() {
    double a[1];
    aa_vrand(1, a);
    double g = a[0];
    double Rx[9], Ry[9], Rz[9];
    double eRx[9], eRy[9], eRz[9];

    double ez[3] = {g, 0, 0};
    double ey[3] = {0, g, 0};
    double ex[3] = {0, 0, g};

    aa_tf_xangle2rotmat( a[0], Rx );
    aa_tf_yangle2rotmat( a[0], Ry );
    aa_tf_zangle2rotmat( a[0], Rz );

    aa_tf_eulerzyx2rotmat(ez, eRz);
    aa_tf_eulerzyx2rotmat(ey, eRy);
    aa_tf_eulerzyx2rotmat(ex, eRx);

    aveq("euler-x", 9, Rx, eRx, .001 );
    aveq("euler-y", 9, Ry, eRy, .001 );
    aveq("euler-z", 9, Rz, eRz, .001 );
}


static void chain() {
    double e1[3], e2[3], R1[9], R2[9], q1[4], q2[4];
    aa_vrand(3,e1);
    aa_vrand(3,e2);

    aa_tf_eulerzyx2rotmat(e1, R1);
    aa_tf_eulerzyx2rotmat(e2, R2);
    aa_tf_eulerzyx2quat(e1, q1);
    aa_tf_eulerzyx2quat(e2, q2);


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
    aa_vrand( 4, q1 );
    aa_vrand( 4, q2 );
    u = aa_frand();
    aa_tf_qnormalize(q1);
    aa_tf_qnormalize(q2);

    double qg[4], qa[4];
    aa_tf_qslerp( u, q1, q2, qg );
    aa_tf_qslerpalg( u, q1, q2, qa );
    aveq("slerp", 4, qg, qa, .001 );

    double dqg[4], dqa[4];
    aa_tf_qslerpdiff( u, q1, q2, dqg );
    aa_tf_qslerpdiffalg( u, q1, q2, dqa );
    aveq("slerpdiff", 4, dqg, dqa, .001 );

}

int main( void ) {
    // init
    srand((unsigned int)time(NULL)); // might break in 2038
    // some limits because
    {
        int r;
        struct rlimit lim;
        // address space
        lim.rlim_cur = (1<<30);
        lim.rlim_max = (1<<30);
        r = setrlimit( RLIMIT_AS, &lim );
        assert(0 == r );
        // cpu time
        lim.rlim_cur = 60;
        lim.rlim_max = 60;
        r = setrlimit( RLIMIT_CPU, &lim );
        assert(0 == r );
        // drop a core
        r = getrlimit( RLIMIT_CORE, &lim );
        assert(0==r);
        lim.rlim_cur = 100*1<<20;
        r = setrlimit( RLIMIT_CORE, &lim );
        assert(0==r);

    }

    for( size_t i = 0; i < 1000; i++ ) {
        rotvec();
        euler();
        euler1();
        chain();
        quat();
    }

    return 0;
}
