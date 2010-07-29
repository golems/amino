/* -*- mode: C; c-basic-offset: 4  -*- */
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

int aa_feq( double a, double b, double tol ) {
    return fabs(a-b) <= tol;
}


int aa_imodulo( int a, int b ) {
    return ((a % b) + b) % b;
}

int aa_irem( int a, int b ) {
    return a % b;
}

double aa_fmodulo( double a, double b ) {
    return fmod(fmod(a , b) + b,  b);
}

double aa_frem( double a, double b ) {
    return fmod(a , b);
}

int aa_veq(size_t n, double *a, double *b, double tol ) {
    for( size_t i = 0; i < n; i ++ ) {
        if( ! aa_feq( a[i], b[i], tol ) ) return 0;
    }
    return 1;
}

double aa_an_rad2deg( double rad ) {
    return rad*180.0/M_PI;
}

double aa_an_deg2rad( double deg ) {
    return deg*M_PI/180;
}

double aa_an_norm_2pi( double an ) {
    return aa_fmodulo( an, 2*M_PI );
}

double aa_an_norm_pi( double an ) {
    return aa_fmodulo( an + M_PI, 2*M_PI ) - M_PI;
}
