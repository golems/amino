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

#include "config.h"
#include <math.h>
#include "amino.h"


#ifndef HAVE_SINCOS
void sincos(double b, double *s,  double *c ) {
    *s = sin(b);
    *c = cos(b);
}
#endif


int aa_veq(size_t n, const double *a, const double *b, double tol ) {
    for( size_t i = 0; i < n; i ++ ) {
        if( ! aa_feq( a[i], b[i], tol ) ) return 0;
    }
    return 1;
}

AA_API int aa_isfok( double x ) {
    int i = fpclassify(x);
    return (i != FP_NAN) && (i != FP_INFINITE );
}

AA_API size_t aa_fminloc( size_t n, double *v ) {
    size_t i_min = 0;
    double x_min = v[0];
    for( size_t i = 0; i < n; i ++ ) {
        if( v[i] < x_min ) {
            x_min = v[i];
            i_min = i;
        }
    }
    return i_min;
}

AA_API size_t aa_fmaxloc( size_t n, double *v ) {
    size_t i_max = 0;
    double x_max = v[0];
    for( size_t i = 0; i < n; i ++ ) {
        if( v[i] > x_max ) {
            x_max = v[i];
            i_max = i;
        }
    }
    return i_max;
}

AA_API double aa_frand() {
    return (double)rand() / (double)RAND_MAX;
}

AA_API void aa_vrand(size_t n, double *v) {
    for( size_t i = 0; i < n; i ++ )
        v[i] = aa_frand();
}

AA_API void aa_box_muller(double x1, double x2, double *z1, double *z2) {
    // z1 = sqrt( -2 * ln(x1) ) * cos( 2 * pi * x2 )
    // z1 = sqrt( -2 * ln(x1) ) * sin( 2 * pi * x2 )
    const double a = sqrt( -2.0 * log(x1) );
    double s,c;
    const double b = 2 * M_PI * x2;
    sincos(b, &s, &c );
    *z1 = a*c;
    *z2 = a*s;
}


AA_API double aa_mean( size_t n, const double *x) {
    return cblas_dasum( (int)n, x, 1 ) / (double)n;
}


AA_API double aa_stddev( size_t n, const double *x) {
    double mu = aa_mean(n,x);
    double a = 0;
    for( size_t i = 0; i < n; i++ ) {
        double t = x[i] - mu;
        a += t*t;
    }
    return sqrt( a / (double)(n-1) );
}
