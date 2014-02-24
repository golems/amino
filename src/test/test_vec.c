/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2014, Georgia Tech Research Corporation
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
#include "amino/vec.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>

double test_qu[8][4];

static void quat(void)
{
    double vr[8], mr[8];
    aa_vec_4d q0 = aa_vec_4d_ld(test_qu[0]);
    aa_vec_4d q1 = aa_vec_4d_ld(test_qu[1]);
    aa_vec_4d q2 = aa_vec_4d_ld(test_qu[2]);
    aa_vec_4d q3 = aa_vec_4d_ld(test_qu[3]);

    // qmul
    aa_vec_4d_st(vr, aa_vec_qmul(q0,q1) );
    aa_tf_qmul( test_qu[0], test_qu[1], mr );
    aveq( "qmul", 4, vr, mr, 0 );

    // qconj
    aa_vec_4d_st(vr, aa_vec_qconj(q0) );
    aa_tf_qconj( test_qu[0], mr );
    aveq( "qconj", 4, vr, mr, 0 );

    // rotation
    aa_vec_4d_st(vr, aa_vec_qrot( q0, q1 ) );
    aa_tf_qrot( test_qu[0], test_qu[1], mr );
    aveq( "qrot", 3, vr, mr, 1e-9 );

    // cross
    aa_vec_4d_st(vr, aa_vec_cross( q0, q1 ) );
    aa_tf_cross( test_qu[0], test_qu[1], mr );
    aveq( "cross", 3, vr, mr, 1e-9 );

    /* QUATERNION-VECTOR */

    // tf
    aa_vec_4d_st(vr, aa_vec_qv_tf( q0, q1, q2 ) );
    aa_tf_tf_qv( test_qu[0], test_qu[1], test_qu[2], mr );
    aveq( "qv-tf", 3, vr, mr, 1e-9 );

    // mul
    {
        aa_vec_4d tmp1, tmp2;
        AA_VEC_QV_MUL( q0, q1, q2, q3, tmp1, tmp2 );
        aa_vec_4d_st( vr, tmp1 );
        aa_vec_4d_st( vr+4, tmp2 );
        aa_tf_qv_chain( test_qu[0], test_qu[1],
                        test_qu[2], test_qu[3],
                        mr, mr+4 );
        aveq( "qv-mul", 7, vr, mr, 1e-9 );
    }
}

static void mat(void)
{
    double T[12];
    aa_tf_quat2rotmat(test_qu[0], T);
    AA_MEM_CPY( T+9, test_qu[1], 3 );
    //aa_vec_4d q0 = aa_vec_4d_ld(test_qu[0]);
    aa_vec_4d q1 = aa_vec_4d_ld(test_qu[1]);
    aa_vec_4d q2 = aa_vec_4d_ld(test_qu[2]);

    double vr1[3], mr1[3];

    aa_vec_4d vR0, vR1, vR2;
    AA_VEC_ROTMAT_LD( vR0, vR1, vR2, T );

    aa_vec_4d Tc0, Tc1, Tc2, Tc3;
    AA_VEC_TFMAT_LD( Tc0, Tc1, Tc2, Tc3, T );

    // store
    {
        double T1[12];
        aa_vec_rotmat_st(T1, vR0, vR1, vR2 );
        aveq( "rotmat-st", 9, T, T1, 0 );

        aa_vec_tfmat_st(T1, Tc0, Tc1, Tc2, Tc3 );
        aveq( "tfmat-st", 12, T, T1, 0 );
    }

    // rotate
    aa_tf_9( T, test_qu[1], mr1 );
    aa_vec_3d_st( vr1, aa_vec_rotmat_tf( vR0, vR1, vR2, q1 ) );
    aveq( "rotmat-tf", 3, vr1, mr1, 1e-9 );

    // transform
    aa_tf_12( T, test_qu[2], mr1 );
    aa_vec_3d_st( vr1, aa_vec_tfmat_tf( Tc0, Tc1, Tc2, Tc3, q2 ) );
    aveq( "tfmat-tf", 3, vr1, mr1, 1e-9 );
}

int main( int argc, char **argv ) {
    (void) argc; (void) argv;


    // init
    srand((unsigned int)time(NULL)); // might break in 2038
    aa_test_ulimit();


    for( size_t i = 0; i < 1000; i++ ) {
        // random data
        for( size_t j = 0; j < sizeof(test_qu)/sizeof(test_qu[0]); j++ ) {
            aa_tf_qurand( test_qu[j] );
        }

        quat();
        mat();

    }

}
