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
    /* Operations:
     * -----------
     * Mul: 3+6  =  9
     * Add: 3+12 =  15
     */
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

AA_API void aa_tf_xangle2axang( double theta, double _a[AA_RESTRICT 4] )
{
    aa_tf_axang_t *a = (aa_tf_axang_t *)_a;
    a->axis.x = 1;
    a->axis.y = 0;
    a->axis.z = 0;
    a->angle = theta;
}

AA_API void aa_tf_yangle2axang( double theta, double _a[AA_RESTRICT 4] )
{
    aa_tf_axang_t *a = (aa_tf_axang_t *)_a;
    a->axis.x = 0;
    a->axis.y = 1;
    a->axis.z = 0;
    a->angle = theta;
}
AA_API void aa_tf_zangle2axang( double theta, double _a[AA_RESTRICT 4] )
{
    aa_tf_axang_t *a = (aa_tf_axang_t *)_a;
    a->axis.x = 0;
    a->axis.y = 0;
    a->axis.z = 1;
    a->angle = theta;
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


AA_API void
aa_tf_rotmat_mzlook( const double eye[AA_RESTRICT 3],
                     const double target[AA_RESTRICT 3],
                     const double up[AA_RESTRICT 3],
                     double R[AA_RESTRICT 9] )
{
    double *x = R, *y = R+3, *z = R+6;
    /* Find the -z vector in parent coords */
    FOR_VEC(i) z[i] = -target[i] + eye[i]; /* z is negated */
    /* Find the x vector */
    AA_TF_CROSS(up,z,x);
    /* Find the y vector */
    AA_TF_CROSS(z,x,y);

    aa_tf_vnormalize(x);
    aa_tf_vnormalize(y);
    aa_tf_vnormalize(z);
}


AA_API void
aa_tf_tfmat_mzlook( const double eye[AA_RESTRICT 3],
                    const double target[AA_RESTRICT 3],
                    const double up[AA_RESTRICT 3],
                    double T[AA_RESTRICT 12] )
{
    aa_tf_rotmat_mzlook(eye, target, up, T+AA_TF_TFMAT_R);
    AA_MEM_CPY(T+AA_TF_TFMAT_V, eye, 3 );
}

AA_API void
aa_tf_qmzlook( const double eye[AA_RESTRICT 3],
               const double target[AA_RESTRICT 3],
               const double up[AA_RESTRICT 3],
               double q[AA_RESTRICT 4] )
{
    double R[9];
    aa_tf_rotmat_mzlook(eye, target, up, R);
    aa_tf_rotmat2quat(R, q);

}

AA_API void
aa_tf_qv_mzlook( const double eye[AA_RESTRICT 3],
                 const double target[AA_RESTRICT 3],
                 const double up[AA_RESTRICT 3],
                 double q[AA_RESTRICT 4],
                 double v[AA_RESTRICT 3] )
{
    aa_tf_qmzlook(eye, target, up, q);
    AA_MEM_CPY(v, eye, 3);
}

AA_API void
aa_tf_qutr_mzlook( const double eye[AA_RESTRICT 3],
                   const double target[AA_RESTRICT 3],
                   const double up[AA_RESTRICT 3],
                   double E[AA_RESTRICT 12] )
{

    aa_tf_qv_mzlook( eye, target, up,
                     E+AA_TF_QUTR_Q, E+AA_TF_QUTR_T );
}

AA_API void
aa_tf_dhprox2tfmat( double alpha, double a, double d, double phi,
                    double T[AA_RESTRICT 12])
{
    double sp = sin(phi);
    double cp = cos(phi);
    double sa = sin(alpha);
    double ca = cos(alpha);

    T[0] = cp;
    T[1] = sp*ca;
    T[2] = sp*sa;

    T[3] = -sp;
    T[4] = cp*ca;
    T[5] = cp*sa;

    T[6] = 0;
    T[7] = -sa;
    T[8] = ca;

    T[9] = a;
    T[10] = -d*sa;
    T[11] = d*ca;
}

AA_API void
aa_tf_dhprox2duqu( double alpha, double a, double d, double phi,
                   double S[AA_RESTRICT 8])
{
    double v[3];
    double *H = S+AA_TF_DUQU_REAL;
    double *D = S+AA_TF_DUQU_DUAL;

    aa_tf_dhprox2qv(alpha, a, d, phi, H, v);

    FOR_VEC(i) v[i] *= 0.5;
    aa_tf_qmul_vq(v, H, D);
}

AA_API void
aa_tf_dhprox2qutr( double alpha, double a, double d, double phi,
                   double E[AA_RESTRICT 7])
{
    aa_tf_dhprox2qv( alpha, a, d, phi,
                     E+AA_TF_QUTR_Q,
                     E+AA_TF_QUTR_T );
}

AA_API void
aa_tf_dhprox2qv( double alpha, double a, double d, double phi,
                 double q[AA_RESTRICT 4], double v[3])
{
    double sp = sin(phi/2);
    double cp = cos(phi/2);
    double sa = sin(alpha/2);
    double ca = cos(alpha/2);

    q[AA_TF_QUAT_X] = sa*cp;
    q[AA_TF_QUAT_Y] = -sa*sp;
    q[AA_TF_QUAT_Z] = ca*sp;
    q[AA_TF_QUAT_W] = ca*cp;

    // double angle formulas
    double sa2 = 2*sa*ca;
    double ca2 = ca*ca-sa*sa;

    v[0] = a;
    v[1] = -d*sa2;
    v[2] = d*ca2;
}

AA_API void
aa_tf_dhdist2tfmat( double alpha, double a, double d, double phi,
                    double T[AA_RESTRICT 12])
{

    double sp = sin(phi);
    double cp = cos(phi);
    double sa = sin(alpha);
    double ca = cos(alpha);

    T[0] = cp;
    T[1] = sp;
    T[2] = 0;

    T[3] = -sp*ca;
    T[4] = cp*ca;
    T[5] = sa;

    T[6] = sp*sa;
    T[7] = -cp*sa;
    T[8] = ca;

    T[9] =  a*cp;
    T[10] = a*sp;
    T[11] = d;
}

AA_API void
aa_tf_dhdist2duqu( double alpha, double a, double d, double phi,
                   double S[AA_RESTRICT 8])
{
    double v[3];
    double *H = S+AA_TF_DUQU_REAL;
    double *D = S+AA_TF_DUQU_DUAL;

    aa_tf_dhdist2qv(alpha, a, d, phi, H, v);

    FOR_VEC(i) v[i] *= 0.5;
    aa_tf_qmul_vq(v, H, D);
}

AA_API void
aa_tf_dhdist2qutr( double alpha, double a, double d, double phi,
                   double E[AA_RESTRICT 7])
{
    aa_tf_dhdist2qv( alpha, a, d, phi,
                     E+AA_TF_QUTR_Q,
                     E+AA_TF_QUTR_T );
}

AA_API void
aa_tf_dhdist2qv( double alpha, double a, double d, double phi,
                 double q[AA_RESTRICT 4], double v[3])
{
    double sp = sin(phi/2);
    double cp = cos(phi/2);
    double sa = sin(alpha/2);
    double ca = cos(alpha/2);

    q[AA_TF_QUAT_X] = sa*cp;
    q[AA_TF_QUAT_Y] = sa*sp;
    q[AA_TF_QUAT_Z] = ca*sp;
    q[AA_TF_QUAT_W] = ca*cp;

    // double angle formulas
    double sp2 = 2*sp*cp;
    double cp2 = cp*cp-sp*sp;

    v[0] = a*cp2;
    v[1] = a*sp2;
    v[2] = d;
}
