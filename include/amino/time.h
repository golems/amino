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
#ifndef AMINO_TIME_H
#define AMINO_TIME_H

/**
 * \file amino/time.h
 */

#define AA_CLOCK CLOCK_MONOTONIC

#include <errno.h>
/// create a struct timespec with given elements
static inline struct timespec
aa_tm_make( time_t sec, long nsec ) {
    struct timespec t;
    t.tv_sec = sec;
    t.tv_nsec = nsec;
    return t;
}

/// create a struct timespec with given elements, fixing things up if
/// nsec is negative or more than a billion
static inline struct timespec
aa_tm_make_norm( time_t sec, long nsec ) {
    long nsp = aa_lmodulo( nsec, AA_IBILLION );
    return aa_tm_make( sec + (nsec - nsp)/AA_IBILLION, nsp );
}

/// add two times: a + b
static inline struct timespec
aa_tm_add( struct timespec t1, struct timespec t0 ) {
    return aa_tm_make_norm( t1.tv_sec + t0.tv_sec,
                            t1.tv_nsec + t0.tv_nsec );
}

/// subtract two times: a - b
static inline struct timespec
aa_tm_sub( const struct timespec a, const struct timespec b ) {
    return aa_tm_make_norm( a.tv_sec - b.tv_sec,
                            a.tv_nsec - b.tv_nsec );
}

/// gets current time via AA_CLOCK
static inline struct timespec
aa_tm_now() {
    struct timespec t;
    clock_gettime( AA_CLOCK, &t );
    return t;
}

/** returns reltime + now */
static inline struct timespec
aa_tm_future( const struct timespec reltime ) {
    return aa_tm_add( reltime, aa_tm_now() );
}

/** t1 < t2: negative; t1 == t2: 0; t1 > t2: positive */
static inline long
aa_tm_cmp( const struct timespec t1, const struct timespec t2 ) {
    return ( t1.tv_sec != t2.tv_sec ) ?
        (t1.tv_sec - t2.tv_sec) :
        (t1.tv_nsec - t2.tv_nsec);
}

/// Return the earlier of t1, t2
static inline struct timespec
aa_tm_min( const struct timespec t1, const struct timespec t2 ) {
    return (aa_tm_cmp(t1,t2)<0) ? t1 : t2;
}

/// Return the latter of t1, t2
static inline struct timespec
aa_tm_max( const struct timespec t1, const struct timespec t2 ) {
    return (aa_tm_cmp(t1,t2)>0) ? t1 : t2;
}

/// is the current time later than abstime?
static inline int
aa_tm_isafter( const struct timespec abstime ) {
    return aa_tm_cmp(aa_tm_now(), abstime) > 0;
}

/// convert timespec t to microseconds
static inline int64_t
aa_tm_timespec2usec( const struct timespec t ) {
    return t.tv_sec*1000000 + t.tv_nsec/1000;
}

/// convert timespec t to seconds
static inline double
aa_tm_timespec2sec( const struct timespec t ) {
    return (double)t.tv_sec + (double)t.tv_nsec/1e9;
}

/// convert seconds t to timespec
static inline struct timespec
aa_tm_sec2timespec( double t ) {
    time_t sec = (time_t) t;
    long nsec = (long) ((t-(double)sec)*AA_IBILLION);
    return aa_tm_make_norm( sec, nsec );
}

/// sleep for specified time
static inline int
aa_tm_relsleep( struct timespec t ) {
    struct timespec rem;
    int r;
    do {
        r = nanosleep( &t, &rem );
        assert( 0 == r || EINTR == errno );
        t.tv_sec = rem.tv_sec;
        t.tv_nsec = rem.tv_nsec;
    } while( 0 != r ) ;
    return 0;
}


#if (defined(__GNUC__) || defined(__ICC)) && (defined(__i386__) || defined(__x86_64__))
#define AA_FEATURE_RDTSC
AA_API uint64_t aa_rdtsc(void);


#endif

#endif //AMINO_TIME_H
