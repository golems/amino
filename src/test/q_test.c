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

#include "amino.h"
#include "amino/test.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>


static void quat_tiny(double q[4]) {
    aa_vrand(3, q);
    for(size_t i = 0; i < 3; i++) q[i] *= 1e-4;
    q[3] = 1;
    aa_tf_qnormalize(q);
}

static void duqu_tiny(double d[8], double sq, double sv) {
    double q[4], v[3];
    aa_vrand(3, q);
    aa_vrand(3, v);
    for(size_t i = 0; i < 3; i++){
        q[i] *= sq;
        v[i] *= sv;
    }

    q[3] = 1;
    aa_tf_qnormalize(q);
    aa_tf_qv2duqu(q,v,d);
}

static void qexp(void) {
    double qtiny[4];
    quat_tiny(qtiny);

    // exp ident
    {
        double e[4], ep[4] = {0,0,0, M_E};
        // exact
        aa_tf_qexp(aa_tf_quat_ident, e );
        aveq( "qexp-ident", 4, e, ep, 0);
        // tiny
        aa_tf_qexp(qtiny, e );
        aveq( "qexp-tiny", 4, e, ep, 1e-3);

    }

    // log ident
    {
        double e[4], ep[4] = {0,0,0, 0};
        // exact
        aa_tf_qln(aa_tf_quat_ident, e );
        aveq( "qln-ident", 4, e, ep, 0);
        // tiny
        aa_tf_qln(qtiny, e );
        aveq( "qln-tiny", 4, e, ep, 1e-3);
    }
}

static void duqu_exp(void) {
    double dtiny[8], dtinyq[8], dtinyv[8];
    duqu_tiny(dtiny, 1e-7, 1e-7);
    duqu_tiny(dtinyq, 1e-7, 0);
    duqu_tiny(dtinyv, 0, 1e-7);

    // exp ident
    {
        double e[8], ep[8] = {0,0,0, M_E, 0,0,0,0};
        // exact
        aa_tf_duqu_exp(aa_tf_duqu_ident, e );
        aveq( "duqu-exp-ident", 8, e, ep, 0);
        // tiny
        aa_tf_duqu_exp(dtiny, e );
        aveq( "duqu-exp-tiny", 8, e, ep, 1e-6);
        aa_tf_duqu_exp(dtinyq, e );
        aveq( "duqu-exp-tinyq", 8, e, ep, 1e-6);
        aa_tf_duqu_exp(dtinyv, e );
        aveq( "duqu-exp-tinyv", 8, e, ep, 1e-6);
    }

    // log ident
    {
        double e[8], ep[8] = {0,0,0,0, 0,0,0,0};
        // exact
        aa_tf_duqu_ln(aa_tf_duqu_ident, e );
        aveq( "duqu-ln-ident", 8, e, ep, 0);
        // tiny
        aa_tf_duqu_ln(dtiny, e );
        aveq( "duqu-ln-tiny", 8, e, ep, 1e-6);
    }
}


int main( int argc, char **argv ) {
    time_t seed = time(NULL);
    printf("Init q_test: %ld\n", seed);
    (void) argc; (void) argv;
    // init
    srand((unsigned)seed); // might break in 2038
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

    qexp();
    duqu_exp();
}
