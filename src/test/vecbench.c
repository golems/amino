/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2012, Georgia Tech Research Corporation
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
#include "amino/vec.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>

#define N 10000

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

void qmul() {
    double a[4], b[4], c1[4], c2[4];
    double va[4], vb[4];
    aa_vrand( 4, a );
    aa_vrand( 4, b );
    aa_tf_qnormalize(a);
    aa_tf_qnormalize(b);

    aa_vec_d4_st( va, aa_vec_d4_ld(a) );
    aa_vec_d4_st( vb, aa_vec_d4_ld(b) );
    aveq( "lda", 4, a, va, .000 );
    aveq( "ldb", 4, b, vb, .000 );


    aa_tf_qmul(a,b,c1);
    aa_vecm_qmul(a,b,c2);
    aveq( "quat-equal", 4, c1, c2, .0001 );
    aa_tick("qmul non-vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_tf_qmul(a,b,c1);
    }
    aa_tock();

    aa_tick("qmul vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_vecm_qmul(a,b,c1);
    }
    aa_tock();

}

void qrot() {

    double a[4], b[4], c1[4], c2[4];
    aa_vrand( 4, a );
    aa_vrand( 4, b );
    aa_tf_qnormalize(a);
    aa_tf_qnormalize(b);


    aa_tf_qrot(a,b,c1);
    aa_vecm_qrot(a,b,c2);
    aveq( "qrot-equal", 3, c1, c2, .0001 );

    aa_tick("qrot non-vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_tf_qrot(a,b,c1);
    }
    aa_tock();

    aa_tick("qrot vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_vecm_qrot(a,b,c1);
    }
    aa_tock();

}

void cross() {

    double a[3], b[3], c1[3], c2[3];
    aa_vrand( 3, a );
    aa_vrand( 3, b );

    // cross
    aa_tf_cross(a,b,c1);
    aa_vecm_cross(a,b,c2);
    aveq( "cross-equal", 3, c1, c2, .0001 );

    aa_tick("cross non-vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_tf_cross(a,b,c1);
    }
    aa_tock();

    aa_tick("cross vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_vecm_cross(a,b,c1);
    }
    aa_tock();
}


void tfmul() {

    double q0[4], q1[4];
    double T0[12], T1[12];
    double Ua[12], Ub[12];
    aa_vrand( 4, q0 );
    aa_vrand( 4, q1 );
    aa_tf_qnormalize(q0);
    aa_tf_qnormalize(q1);
    aa_tf_quat2rotmat( q0, T0 );
    aa_tf_quat2rotmat( q1, T1 );
    aa_vrand( 3, T0+9 );
    aa_vrand( 3, T1+9 );

    // cross
    aa_tf_12chain(T0,T1,Ua);
    aa_vecm_tfmul(T0,T1,Ub);
    aveq( "tfmul-equal", 12, Ua, Ub, .0001 );

    aa_tick("tfmul non-vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_tf_12chain(T0,T1,Ua);
    }
    aa_tock();

    aa_tick("tfmul vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_vecm_tfmul(T0,T1,Ub);
    }
    aa_tock();
}

void duqumul() {

    double q0[4], q1[4];
    double v0[3], v1[3];
    double d0[8], d1[8];
    double ra[8], rb[8];
    aa_vrand( 4, q0 );
    aa_vrand( 4, q1 );
    aa_tf_qnormalize(q0);
    aa_tf_qnormalize(q1);
    aa_vrand( 3, v0 );
    aa_vrand( 3, v1 );
    aa_tf_qv2duqu( q0, v0, d0 );
    aa_tf_qv2duqu( q1, v1, d1 );

    aa_tf_duqu_mul(q0,q1,ra);
    aa_vecm_duqu_mul(q0,q1,rb);
    aveq( "duqu_mul-equal", 8, ra, rb, .0001 );

    aa_tick("duqu-mul non-vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_tf_duqu_mul(d0,d1,ra);
    }
    aa_tock();

    aa_tick("duqu-mul vec: ");
    for( size_t i = 0; i < N; i ++ ) {
        aa_vecm_duqu_mul(q0,q1,rb);
    }
    aa_tock();
}


int main( int argc, char **argv ) {
    (void) argc; (void) argv;


    // init
    srand((unsigned int)time(NULL)); // might break in 2038
    // some limits because linux (and sometimes our software) sucks
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

    cross();
    qmul();
    qrot();
    tfmul();
    duqumul();
}
