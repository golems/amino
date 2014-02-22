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
    double vr[4], mr[4];
    aa_vec_4d q0 = aa_vec_4d_ld(test_qu[0]);
    aa_vec_4d q1 = aa_vec_4d_ld(test_qu[1]);

    // qmul
    aa_vec_4d_st(vr, aa_vec_qmul(q0,q1) );
    aa_tf_qmul( test_qu[0], test_qu[1], mr );
    aveq( "qmul", 4, vr, mr, 0 );

    // qconj
    aa_vec_4d_st(vr, aa_vec_qconj(q0) );
    aa_tf_qconj( test_qu[0], mr );
    aveq( "qconj", 4, vr, mr, 0 );

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

    }

}
