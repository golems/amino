/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#define _GNU_SOURCE
#include "amino.h"
#include "amino_internal.h"


AA_API void
aa_tf_duqu2tfmat( const double d[AA_RESTRICT 8], double T[AA_RESTRICT 12] )
{
    aa_tf_quat2rotmat( &d[AA_TF_DUQU_REAL], T );
    aa_tf_duqu_trans( d, T+9);
}

AA_API void
aa_tf_tfmat2duqu( const double T[AA_RESTRICT 12], double d[AA_RESTRICT 8] )
{
    double q[4];
    aa_tf_rotmat2quat( T, q );
    aa_tf_qv2duqu( q, T+9, d );
}

AA_API void
aa_tf_xangle2rotmat( double theta, double R[AA_RESTRICT 9] )
{
    double s = sin(theta), c = cos(theta);
    R[0] = 1;
    R[1] = 0;
    R[2] = 0;

    R[3] = 0;
    R[4] = c;
    R[5] = s;

    R[6] = 0;
    R[7] = -s;
    R[8] = c;
}

AA_API void
aa_tf_yangle2rotmat( double theta, double R[AA_RESTRICT 9] )
{
    double s = sin(theta), c = cos(theta);
    R[0] = c;
    R[1] = 0;
    R[2] = -s;

    R[3] = 0;
    R[4] = 1;
    R[5] = 0;

    R[6] = s;
    R[7] = 0;
    R[8] = c;
}

AA_API void
aa_tf_zangle2rotmat( double theta, double R[AA_RESTRICT 9] )
{
    double s = sin(theta), c = cos(theta);
    R[0] = c;
    R[1] = s;
    R[2] = 0;

    R[3] = -s;
    R[4] = c;
    R[5] = 0;

    R[6] = 0;
    R[7] = 0;
    R[8] = 1;
}

AA_API void
aa_tf_quat2rotmat( const double q[AA_RESTRICT 4], double R[AA_RESTRICT 9] )
{
    double b[3] = { 2*q[0],
                    2*q[1],
                    2*q[2] };
    double a[3] = { q[3] * b[0],
                    q[3] * b[1],
                    q[3] * b[2] };
    aa_tf_skewsym_scal_c( q, a, b, R );
}


AA_API void
aa_tf_xangle2quat( double theta, double _q[AA_RESTRICT 4] )
{
    aa_tf_quat_t *q = (aa_tf_quat_t *)_q;
    q->x = sin(theta/2);
    q->y = 0;
    q->z = 0;
    q->w = cos(theta/2);
}

AA_API void
aa_tf_yangle2quat( double theta, double _q[AA_RESTRICT 4] )
{
    aa_tf_quat_t *q = (aa_tf_quat_t *)_q;
    q->x = 0;
    q->y = sin(theta/2);
    q->z = 0;
    q->w = cos(theta/2);
}

AA_API void
aa_tf_zangle2quat( double theta, double _q[AA_RESTRICT 4] )
{
    aa_tf_quat_t *q = (aa_tf_quat_t *)_q;
    q->x = 0;
    q->y = 0;
    q->z = sin(theta/2);
    q->w = cos(theta/2);
}

AA_API void
aa_tf_rotvec2quat( const double a[AA_RESTRICT 3], double _q[AA_RESTRICT 4] )
{
    aa_tf_quat_t *q = (aa_tf_quat_t *)_q;
    double sc;
    aa_tf_sinccos2( (a[0]*a[0]+a[1]*a[1]+a[2]*a[2])/4, &sc, &q->w );
    sc /= 2;
    FOR_VEC(i) q->v[i] = sc * a[i];
}

AA_API void
aa_tf_axang2quat2( const double axis[AA_RESTRICT 3], double angle,
                   double _q[AA_RESTRICT 4] )
{

    aa_tf_quat_t *q = (aa_tf_quat_t *)_q;
    angle /= 2;
    double s = sin(angle);
    q->w = cos(angle);
    FOR_VEC(i) q->v[i] = s * axis[i];
}

AA_API void
aa_tf_axang2quat( const double _aa[AA_RESTRICT 4],
                  double q[AA_RESTRICT 4] )
{
    const aa_tf_axang_t *aa = (aa_tf_axang_t *) _aa;
    aa_tf_axang2quat2( aa->v, aa->angle, q );
}


void aa_tf_quat2eulerzyx( const double q[restrict 4],
                          double e[restrict 3] )
{
    double n = aa_tf_qnorm( q );
    double x = q[AA_TF_QUAT_X] / n;
    double y = q[AA_TF_QUAT_Y] / n;
    double z = q[AA_TF_QUAT_Z] / n;
    double w = q[AA_TF_QUAT_W] / n;

    /* roll*/
    e[2] = atan2( 2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y) );
    /* pitch*/
    e[1] = asin( 2.0*(w*y - x*z) );
    /* yaw*/
    e[0] = atan2( 2*(w*z + x*y), 1.0 - 2.0*(y*y+z*z) );
}

AA_API void
aa_tf_quat2rotvec( const double q[AA_RESTRICT 4], double r[AA_RESTRICT 3] )
{
    double qmin[4], qrv[4];
    aa_tf_qminimize2( q, qmin );
    aa_tf_qln(qmin, qrv);
    FOR_VEC(i) r[i] = 2 * qrv[AA_TF_QUAT_V + i];
}
AA_API void
aa_tf_quat2axang( const double q[AA_RESTRICT 4], double a[AA_RESTRICT 4] )
{
    double rv[3];
    aa_tf_quat2rotvec( q, rv );
    a[3] = sqrt( rv[0]*rv[0]+rv[1]*rv[1]+rv[2]*rv[2] );
    if( 0 < fabs(a[3]) ) {
        FOR_VEC(i) a[i] = rv[i] / a[3];
    } else {
        FOR_VEC(i) a[i] = 0;
    }

}


void aa_tf_qv2duqu( const double q[4], const double v[3], double S[8] )
{
    AA_MEM_CPY(S+AA_TF_DUQU_REAL, q, 4 );

    double v_2[3];
    FOR_VEC(i) v_2[i] = v[i]/2;
    aa_tf_qmul_vq(v_2, q, S+AA_TF_DUQU_DUAL);
}

AA_API void
aa_tf_qutr2duqu( const double E[AA_RESTRICT 7], double S[AA_RESTRICT 8] )
{
    aa_tf_qv2duqu( E+AA_TF_QUTR_Q, E+AA_TF_QUTR_V, S );
}

AA_API void
aa_tf_duqu2qv( const double S[AA_RESTRICT 8], double q[AA_RESTRICT 4], double v[AA_RESTRICT 3] )
{
    AA_MEM_CPY(q, S+AA_TF_DUQU_REAL, 4);
    aa_tf_duqu_trans( S, v );
}


AA_API void
aa_tf_duqu2qutr( const double S[AA_RESTRICT 8], double E[AA_RESTRICT 7] )
{
    aa_tf_duqu2qv(S,  E+AA_TF_QUTR_Q, E+AA_TF_QUTR_V );
}


void aa_tf_qv2tfmat( const double q[4], const double v[3], double T[12] )
{
    aa_tf_quat2rotmat( q, T + AA_TF_TFMAT_R);
    AA_MEM_CPY(T+AA_TF_TFMAT_V, v, 3 );
}

AA_API void
aa_tf_qutr2tfmat( const double E[AA_RESTRICT 7], double T[AA_RESTRICT 12] )
{
    aa_tf_qv2tfmat( E+AA_TF_QUTR_Q, E+AA_TF_QUTR_V, T );
}

AA_API void
aa_tf_tfmat2qv( const double T[AA_RESTRICT 12], double q[AA_RESTRICT 4], double v[AA_RESTRICT 3] )
{
    aa_tf_rotmat2quat(T + AA_TF_TFMAT_R, q);
    AA_MEM_CPY( v, T+AA_TF_TFMAT_V, 3 );
}

AA_API void
aa_tf_tfmat2qutr( const double T[AA_RESTRICT 12], double E[AA_RESTRICT 7] )
{
    aa_tf_tfmat2qv( T, E+AA_TF_QUTR_Q, E+AA_TF_QUTR_V );
}

static double vdot (const double u[3],
                    const double v[3] )
{
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

/* static double vnorm(double v[3] ) */
/* { */

/*     return sqrt( vdot(v,v) ); */
/* } */

/* static void vnormalize (double v[3] ) */
/* { */
/*     double n = vnorm(v); */
/*     for( size_t i = 0; i < 3; i ++ ) v[i] /= n; */
/* } */

AA_API void
aa_tf_vecs2quat( const double u[AA_RESTRICT 3],
                 const double v[AA_RESTRICT 3],
                 double q[AA_RESTRICT 4] )
{
    aa_tf_cross( u, v, &q[AA_TF_QUAT_X] );
    q[AA_TF_QUAT_W] = vdot(u,v) + sqrt( vdot(u,u) * vdot(v,v) );
    aa_tf_qnormalize(q);
}
