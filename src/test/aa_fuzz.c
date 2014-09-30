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

void fuzz_sort(void) {
    static const size_t n = 512;
    double a0[n];
    aa_vrand(n, a0 );

    double ar_qsort[n];
    double ar_hsort[n];
    AA_MEM_CPY( ar_qsort, a0, n );
    AA_MEM_CPY( ar_hsort, a0, n );

    qsort( ar_qsort, n, sizeof(a0[0]), aa_la_d_compar );
    aa_aheap_sort( ar_hsort, n, sizeof(a0[0]), aa_la_d_compar );
    aveq( "heap-sort", n, ar_qsort, ar_hsort, 0 );

}

void mem(void) {
    static const size_t n = 1024;
    double a[n], b[n], a2[n], b2[n];
    aa_vrand(n, a);
    aa_vrand(n, b);
    AA_MEM_CPY(a2, a, n);
    AA_MEM_CPY(b2, b, n);

    aveq( "mem-cpy-a", n, a, a2, 0 );
    aveq( "mem-cpy-b", n, b, b2, 0 );

    AA_MEM_SWAP(a,b,n);

    aveq( "mem-cpy-a", n, b, a2, 0 );
    aveq( "mem-cpy-b", n, a, b2, 0 );
}

int main( void ) {
    // init
    srand((unsigned int)time(NULL)); // might break in 2038
    aa_test_ulimit();

    for( size_t i = 0; i < 1000; i++ ) {
        fuzz_sort();
        mem();
    }

    return 0;
}
