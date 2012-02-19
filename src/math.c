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
    return aa_la_d_minloc( n, v, 1 );
}

AA_API size_t aa_fmaxloc( size_t n, double *v ) {
    return aa_la_d_maxloc( n, v, 1 );
}

AA_API double aa_frand() {
    return (double)rand() / (double)RAND_MAX;
}

AA_API void aa_vrand(size_t n, double *v) {
    for( size_t i = 0; i < n; i ++ )
        v[i] = aa_frand();
}

AA_API void aa_stat_box_muller(double x1, double x2, double *z1, double *z2) {
    // z1 = sqrt( -2 * ln(x1) ) * cos( 2 * pi * x2 )
    // z1 = sqrt( -2 * ln(x1) ) * sin( 2 * pi * x2 )
    const double a = sqrt( -2.0 * log(x1) );
    double s,c;
    const double b = 2 * M_PI * x2;
    sincos(b, &s, &c );
    *z1 = a*c;
    *z2 = a*s;
}


AA_API double aa_stat_mean( size_t n, const double *x) {
    double a = 0;
    for( size_t i = 0; i < n; i ++ ) a += x[i];
    return a / (double)n;
}


AA_API double aa_stat_std( size_t n, const double *x) {
    double mu = aa_stat_mean(n,x);
    return aa_la_d_vecstd(n, x, 1, mu );
}


AA_API double aa_stat_circ_mean( size_t n, const double *x) {
    double as = 0, ac = 0;
    for( size_t i = 0; i < n; i ++ ) {
        double s, c;
        sincos(x[i], &s, &c);
        as += s;
        ac += c;
    }
    return atan2( as/(double)n, ac/(double)n );
}

AA_API double aa_stat_circ_std( size_t n, const double *x) {
    double as = 0, ac = 0;
    for( size_t i = 0; i < n; i++ ) {
        double s,c;
        sincos( x[i], &s, &c );
        as += s;
        ac += c;
    }
    as = as / (double)n;
    ac = ac / (double)n;
    double r = (as*as + ac*ac);
    return sqrt( -2 * log(r) );
}


AA_API size_t aa_stat_excluded_mean_std( size_t n, const double *x,
                                      double *pmu, double *psigma,
                                      double zmin, double zmax,
                                      size_t max_iterations ) {
    double mu = aa_stat_mean(n,x);
    double sigma = aa_stat_std(n,x);
    size_t iter = 0;
    size_t jj,j=0;
    do {
        jj = j;
        j = 0;
        double am = 0;
        double as = 0;
        double dxmax = zmax*sigma;
        double dxmin = zmin*sigma;
        for( size_t i = 0; i < n; i++ ) {
            double dx = x[i] - mu;
            if( dx >= dxmin && dx <= dxmax ) {
                j++;
                as += dx*dx;
                am += x[i];
            }
        }
        assert(j > 1);
        mu = am / (double)j;
        sigma = sqrt(as / (double)(j-1));
    } while( jj != j &&  ++iter < max_iterations ) ;
    *pmu = mu;
    *psigma = sigma;
    return iter;
}

AA_API size_t aa_stat_excluded_circ_mean_std( size_t n, const double *x,
                                              double *pmu, double *psigma,
                                              double zmin, double zmax,
                                              size_t max_iterations ) {
    double mu = aa_stat_circ_mean(n,x);
    double sigma = aa_stat_circ_std(n,x);
    size_t iter = 0;
    size_t jj,j=0;
    do {
        jj = j;
        j = 0;
        double as = 0;
        double ac = 0;
        double dxmax = zmax*sigma;
        double dxmin = zmin*sigma;
        for( size_t i = 0; i < n; i++ ) {
            double dx = aa_ang_delta( x[i], mu );
            if( dx >= dxmin && dx <= dxmax ) {
                j++;
                double s, c;
                sincos(x[i], &s, &c);
                as += s;
                ac += c;
            }
        }
        assert(j > 1);
        as /= (double)j;
        ac /= (double)j;
        mu = atan2(as,ac);
        double r = (as*as + ac*ac);
        sigma = sqrt( -2 * log(r) );
    } while(jj != j && ++iter < max_iterations ) ;
    *pmu = mu;
    *psigma = sigma;
    return iter;
}

void aa_stat_vmean( size_t m, size_t n, const double *X, double *mu) {
    aa_la_d_colmean( m, n, X, m, mu );
}

void aa_stat_vmean_cov( size_t m, size_t n, const double *X,
                        double *mu, double *E) {
    aa_stat_vmean(m,n,X,mu);
    aa_la_d_colcov( m, n, X, m, mu, E, m );
}

double aa_stat_mahalanobis( size_t m, const double *x,
                            const double *mu, const double *E_inv) {
    double t[m];
    // t := - mu + x_i
    memcpy( t, x, sizeof(t[0])*m );
    cblas_daxpy( (int)m, -1, mu, 1, t, 1 );

    // sqrt( t' A t )
    return sqrt( aa_la_wdot( m, t, E_inv, t ) );
}
