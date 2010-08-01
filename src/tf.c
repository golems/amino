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




void aa_tf_qnorm( double q[4] ) {
    aa_la_unit( 4, q );
}

void aa_tf_qconj( const double q[4], double r[4] ) {
    r[0] = -q[0];
    r[1] = -q[1];
    r[2] = -q[2];
    r[3] =  q[3];
}

void aa_tf_qinv( const double q[4], double r[4] ) {
    aa_tf_qconj(q,r);
    aa_la_scal( 4, 1.0/aa_la_dot(4,r,r), r );
}

void aa_tf_qmul( const double a[4], const double b[4], double c[4] ) {
    c[3] = a[3]*b[3] - aa_la_dot(3, a, b);
    aa_la_cross( a, b, c );
    for( size_t i = 0; i < 3; i ++ )
        c[i] += a[3]*b[i] + b[3]*a[i];
}

void aa_tf_qadd( const double a[4], const double b[4], double c[4] );

void aa_tf_qadd( const double a[4], const double b[4], double c[4] );

void aa_tf_qslerp( double t, const double a[4], const double b[4], double c[4] );

void aa_tf_quat2axang( const double q[4], double axang[4] ) {
    double a = aa_la_norm(4,q);
    double w = q[3]/a;
    aa_la_smul( 3, 1.0 / (a*sqrt(1 - w*w)), q, axang );
    axang[3] = 2 * acos(w);
}
