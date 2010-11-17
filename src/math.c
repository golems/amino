/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "amino.h"


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
