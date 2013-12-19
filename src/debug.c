/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
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
#include <stdarg.h>

static struct timespec aa_tick_tock_start;

// FIXME: should use monotonic clock

AA_API void
aa_tick(const char fmt[], ...) {
    va_list argp;
    va_start( argp, fmt );
    vfprintf( stderr, fmt, argp );
    va_end( argp );
    aa_tick_tock_start = aa_tm_now();
}

AA_API struct timespec
aa_tock(void) {
   struct timespec t = aa_tm_sub( aa_tm_now(), aa_tick_tock_start );
   fprintf( stderr, "%.6f ms\n", (double)t.tv_sec*1e3 + (double)t.tv_nsec / 1e6 );
   return t;
}


AA_API void
aa_dump_vec( FILE *file, const double *v, size_t n ) {
    for( size_t i = 0; i < n-1; i ++ )
        fprintf(file, "%f\t", v[i] );
    fprintf(file, "%f\n", v[n-1]);
}

AA_API void
aa_dump_mat( FILE *file, const double *A, size_t m, size_t n  ) {
    for( size_t i = 0; i < m; i ++ ) {
        for( size_t j = 0; j < n-1; j ++ ) {
            fprintf(file, "%f\t", AA_MATREF(A, m,i,j));
        }
        fprintf(file, "%f\n", AA_MATREF(A, m, i, n-1) );
    }
}


const char *aa_verbf_prefix = "amino";
int aa_opt_verbosity = 0;

void aa_verbf( int level, const char fmt[], ... ) {
    va_list argp;
    va_start( argp, fmt );
    if( level <= aa_opt_verbosity ) {
        fprintf(stderr, "[%s]\t", aa_verbf_prefix);
        vfprintf( stderr, fmt, argp );
    }
    va_end( argp );
}

void aa_hard_assert( int test, const char fmt[], ... ) {
    if( ! test ) {
        va_list argp;
        va_start( argp, fmt );
        fprintf(stderr, "ERROR: ");
        vfprintf( stderr, fmt, argp );
        va_end( argp );
        abort();
        exit(EXIT_FAILURE);
    }
}
