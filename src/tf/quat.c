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

#include "amino.h"
#include "amino_internal.h"

/*----------------------*/
/* ORDINARY QUATERNIONS */
/*----------------------*/

AA_API double
aa_tf_qnorm( const double q[AA_RESTRICT 4] )
{
    return sqrt( (q[0]*q[0] + q[1]*q[1]) + (q[2]*q[2] + q[3]*q[3]) );
}

AA_API double
aa_tf_qvnorm( const double q[AA_RESTRICT 4] )
{
    const double *v = q + AA_TF_QUAT_XYZ;
    return sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
}

AA_API void
aa_tf_qnormalize( double q[AA_RESTRICT 4] )
{
    double n = aa_tf_qnorm(q);
    size_t i;
    for( i = 0; i < 4; i ++ ) q[i] /= n;
}

AA_API void
aa_tf_qnormalize2( const double q[AA_RESTRICT 4], double qn[AA_RESTRICT 4] )
{
    double n = aa_tf_qnorm(q);
    size_t i;
    for( i = 0; i < 4; i ++ ) qn[i] = q[i] / n;
}

AA_API void
aa_tf_qminimize( double q[AA_RESTRICT 4] )
{
    size_t i;
    if( q[AA_TF_QUAT_W] < 0 )
        for( i = 0; i < 4; i ++ ) q[i] *= -1;
}

AA_API void
aa_tf_qminimize2( const double q[AA_RESTRICT 4], double m[AA_RESTRICT 4] )
{
    size_t i;
    if( q[AA_TF_QUAT_W] < 0 )
        for( i = 0; i < 4; i ++ ) m[i] = -q[i];
    else
        AA_MEM_CPY(m,q,4);
}

AA_API void
aa_tf_normalize2( const double q[AA_RESTRICT 4], double qn[AA_RESTRICT 4] )
{
    size_t i;
    double n = aa_tf_qnorm(q);
    for( i = 0; i < 4; i ++ ) qn[i] = q[i]/n;
}


AA_API void
aa_tf_qconj( const double q[AA_RESTRICT 4], double qc[AA_RESTRICT 4] )
{
    size_t i;
    const double *v = q+AA_TF_QUAT_XYZ;
    double *vc = qc+AA_TF_QUAT_XYZ;
    for( i = 0; i < 3; i ++ ) vc[i] = -v[i];
    qc[AA_TF_QUAT_W] = q[AA_TF_QUAT_W];
}

AA_API void
aa_tf_qmul( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4], double c[AA_RESTRICT 4] )
{
    const size_t x = AA_TF_QUAT_X;
    const size_t y = AA_TF_QUAT_Y;
    const size_t z = AA_TF_QUAT_Z;
    const size_t w = AA_TF_QUAT_W;

    c[x] =    a[x]*b[w] + a[y]*b[z] + a[w]*b[x] - a[z]*b[y];
    c[y] =    a[z]*b[x] + a[w]*b[y] + a[y]*b[w] - a[x]*b[z];
    c[z] =    a[w]*b[z] + a[z]*b[w] + a[x]*b[y] - a[y]*b[x];
    c[w] = - (a[y]*b[y] + a[x]*b[x] + a[z]*b[z] - a[w]*b[w]);
}

void aa_tf_qv2tfmat( const double q[4], const double v[3], double T[12] )
{
    aa_tf_quat2rotmat( q, T + AA_TF_TFMAT_R);
    AA_MEM_CPY(T+AA_TF_TFMAT_V, v, 3 );
}


AA_API void
aa_tf_tf_qv( const double quat[AA_RESTRICT 4],
             const double v[AA_RESTRICT 3],
             const double p0[AA_RESTRICT 3],
             double p1[AA_RESTRICT 4] )
{
    aa_tf_qrot( quat, p0, p1 );
    for( size_t i = 0; i < 3; i ++ ) p1[i] += v[i];
}

AA_API void
aa_tf_qv_chain( const double q1[AA_RESTRICT 4], const double v1[AA_RESTRICT 3],
                const double q2[AA_RESTRICT 4], const double v2[AA_RESTRICT 3],
                double q3[AA_RESTRICT 4], double v3[AA_RESTRICT 3] )
{
    aa_tf_qmul( q1, q2, q3 );
    aa_tf_tf_qv( q1, v1, v2, v3 );
}

AA_API void
aa_tf_qv_conj( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
               double qc[AA_RESTRICT 4], double vc[AA_RESTRICT 3] )
{
    aa_tf_qconj( q, qc );
    aa_tf_qrot( qc, v, vc );
    FOR_VEC(i) vc[i] = -vc[i];
}


void aa_tf_quat2eulerzyx( const double q[restrict 4],
                          double e[restrict 3] )
{

    double x = q[AA_TF_QUAT_X];
    double y = q[AA_TF_QUAT_Y];
    double z = q[AA_TF_QUAT_Z];
    double w = q[AA_TF_QUAT_W];

    /* roll*/
    e[2] = atan2( 2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y) );
    /* pitch*/
    e[1] = asin( 2.0*(w*y - x*z) );
    /* yaw*/
    e[0] = atan2( 2*(w*z + x*y), 1.0 - 2.0*(y*y+z*z) );
}

/********************/
/* DUAL QUATERNIONS */
/********************/

AA_API void
aa_tf_duqu_vel2twist( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                      double t[AA_RESTRICT 8] )
{
    AA_MEM_CPY( &t[REAL_XYZ], &dx[OMEGA], 3 );
    t[REAL_W] = 0;

    double p[3];
    aa_tf_duqu_trans(d, p );
    aa_tf_cross( p, &dx[OMEGA], &t[DUAL_XYZ] );
    for( size_t i = 0; i < 3; i ++ ) t[DUAL_XYZ+i] += dx[V+i];
    t[DUAL_W] = 0;
}

AA_API void
aa_tf_duqu_twist2vel( const double d[AA_RESTRICT 8], const double t[AA_RESTRICT 8],
                                  double dx[AA_RESTRICT 6] )
{
    double p[3], pxw[3];
    AA_MEM_CPY( &dx[OMEGA], &t[XYZ], 3 );
    aa_tf_duqu_trans(d,p);
    aa_tf_cross(p, &t[REAL_XYZ], pxw );
    for( size_t i = 0; i < 3; i ++ ) dx[V+i] = t[DUAL_XYZ+i] - pxw[i];
}

AA_API void
aa_tf_duqu_twist2diff( const double d[AA_RESTRICT 8], const double t[AA_RESTRICT 8],
                       double dd[AA_RESTRICT 8] )
{
    aa_tf_duqu_mul(t,d,dd);
    for( size_t i = 0; i < 8; i ++ ) dd[i] /= 2;
}

AA_API void
aa_tf_duqu_diff2twist( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                       double twist[AA_RESTRICT 8] )
{
    double dx[6];
    aa_tf_duqu_diff2vel(d, dd, dx);
    aa_tf_duqu_vel2twist(d, dx, twist);
}

AA_API void
aa_tf_duqu_vel2diff( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                     double dd[AA_RESTRICT 8] )
{
    double t[8];
    aa_tf_duqu_vel2twist( d, dx, t );
    aa_tf_duqu_twist2diff( d, t, dd );

}

AA_API void
aa_tf_duqu_diff2vel( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                     double dx[AA_RESTRICT 6] )
{
    double t1[4], t2[4];
    // rotation
    aa_tf_qdiff2vel( &d[REAL], &dd[REAL], &dx[OMEGA] );
    // translation
    // dx/dt = 2 * ( d_dual/dt conj(r) + d_dual conj(d_real/dt) )
    aa_tf_qmulc( &dd[DUAL], &d[REAL], t1 );
    aa_tf_qmulc( &d[DUAL], &dd[REAL], t2 );
    for( size_t i = 0; i < 3; i ++ ) dx[V+i] = 2 * (t1[XYZ+i] + t2[XYZ+i]);
}

AA_API void
aa_tf_duqu_stwist( const double d0[AA_RESTRICT 8], const double twist[AA_RESTRICT 8],
                   double dt, double d1[AA_RESTRICT 6] )
{
    double twist1[8], etwist[8];
    for( size_t i = 0; i < 8; i ++ ) twist1[i] = dt/2 * twist[i];
    aa_tf_duqu_exp( twist1, etwist );
    aa_tf_duqu_mul( etwist, d0, d1 );
}

AA_API void
aa_tf_duqu_svel( const double d0[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                             double dt, double d1[AA_RESTRICT 6] )
{
    double twist[8];
    aa_tf_duqu_vel2twist( d0, dx, twist );
    aa_tf_duqu_stwist(d0, twist, dt, d1 );
}

AA_API void
aa_tf_duqu_sdiff( const double d0[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                  double dt, double d1[AA_RESTRICT 6] )
{
    double w[8];
    aa_tf_duqu_diff2twist(d0, dd, w);
    aa_tf_duqu_stwist(d0, w, dt, d1 );
}
