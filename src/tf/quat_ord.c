/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
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

#define DECLARE_QUAT_XYZW          \
    const size_t x = AA_TF_QUAT_X; \
    const size_t y = AA_TF_QUAT_Y; \
    const size_t z = AA_TF_QUAT_Z; \
    const size_t w = AA_TF_QUAT_W;


#define WITH_QUAT_XYZW(var,x,y,z,w)   \
    double x = var[AA_TF_QUAT_X];     \
    double y = var[AA_TF_QUAT_Y];     \
    double z = var[AA_TF_QUAT_Z];     \
    double w = var[AA_TF_QUAT_W];

static inline double
aa_tf_qdot( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4] )
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
}

static inline double
aa_tf_vdot( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3] )
{
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

AA_API double
aa_tf_qnorm( const double q[AA_RESTRICT 4] )
{
    return sqrt( aa_tf_qdot(q,q) );
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
    FOR_QUAT(i) q[i] /= n;
}

AA_API void
aa_tf_qminimize( double q[AA_RESTRICT 4] )
{
    if( q[AA_TF_QUAT_W] < 0 )
        FOR_QUAT(i) q[i] *= -1;
}

AA_API void
aa_tf_qminimize2( const double q[AA_RESTRICT 4], double m[AA_RESTRICT 4] )
{
    if( q[AA_TF_QUAT_W] < 0 )
        FOR_QUAT(i) m[i] = -q[i];
    else
        AA_MEM_CPY(m,q,4);
}

AA_API void
aa_tf_normalize2( const double q[AA_RESTRICT 4], double qn[AA_RESTRICT 4] )
{
    double n = aa_tf_qnorm(q);
    FOR_QUAT(i) qn[i] = q[i]/n;
}


AA_API void
aa_tf_qconj( const double q[AA_RESTRICT 4], double qc[AA_RESTRICT 4] )
{
    const double *v = q+AA_TF_QUAT_XYZ;
    double *vc = qc+AA_TF_QUAT_XYZ;
    FOR_VEC(i) vc[i] = -v[i];
    qc[AA_TF_QUAT_W] = q[AA_TF_QUAT_W];
}


AA_API void
aa_tf_qinv( const double q[AA_RESTRICT 4], double r[AA_RESTRICT 4] )
{
    aa_tf_qconj(q,r);
    double n = aa_tf_qdot(q,q);
    FOR_QUAT(i) r[i] /= n;
}

AA_API void
aa_tf_qmul( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4], double c[AA_RESTRICT 4] )
{
    DECLARE_QUAT_XYZW;

    c[x] =    a[x]*b[w] + a[y]*b[z] + a[w]*b[x] - a[z]*b[y];
    c[y] =    a[z]*b[x] + a[w]*b[y] + a[y]*b[w] - a[x]*b[z];
    c[z] =    a[w]*b[z] + a[z]*b[w] + a[x]*b[y] - a[y]*b[x];
    c[w] = - (a[y]*b[y] + a[x]*b[x] + a[z]*b[z] - a[w]*b[w]);
}
AA_API void
aa_tf_qmulnorm( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4], double c[AA_RESTRICT 4] )
{
    aa_tf_qmul(a,b,c);
    aa_tf_qnormalize(c);
}


AA_API void
aa_tf_qcmul( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4], double c[AA_RESTRICT 4] )
{
    DECLARE_QUAT_XYZW;

    c[x] = ( a[z]*b[y] - a[y]*b[z] ) + ( a[w]*b[x] - a[x]*b[w] );
    c[y] = ( a[x]*b[z] - a[z]*b[x] ) + ( a[w]*b[y] - a[y]*b[w] );
    c[z] = ( a[y]*b[x] - a[x]*b[y] ) + ( a[w]*b[z] - a[z]*b[w] );
    c[w] = ( a[w]*b[w] + a[z]*b[z] ) + ( a[x]*b[x] + a[y]*b[y] );
}

AA_API void
aa_tf_qmulc( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4], double c[AA_RESTRICT 4] )
{
    DECLARE_QUAT_XYZW;

    c[x] = ( a[z]*b[y] - a[y]*b[z] ) + ( a[x]*b[w] - a[w]*b[x] );
    c[y] = ( a[x]*b[z] - a[z]*b[x] ) + ( a[y]*b[w] - a[w]*b[y] );
    c[z] = ( a[y]*b[x] - a[x]*b[y] ) + ( a[z]*b[w] - a[w]*b[z] );
    c[w] = ( a[x]*b[x] + a[y]*b[y] ) + ( a[z]*b[z] + a[w]*b[w] );
}

AA_API void
aa_tf_qmul_vq( const double v[AA_RESTRICT 3], const double q[AA_RESTRICT 4], double r[AA_RESTRICT 4] )
{
    DECLARE_QUAT_XYZW;

    r[x] = +  v[y]*q[z] - v[z]*q[y] + v[x]*q[w];
    r[y] = +  v[z]*q[x] - v[x]*q[z] + v[y]*q[w];
    r[z] = +  v[x]*q[y] - v[y]*q[x] + v[z]*q[w];
    r[w] = -  v[x]*q[x] - v[y]*q[y] - v[z]*q[z];
}

AA_API void
aa_tf_qmul_qv( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 2], double r[AA_RESTRICT 4] )
{
    DECLARE_QUAT_XYZW;

    r[x] = + q[y]*v[z] - q[z]*v[y] + q[w]*v[x];
    r[y] = + q[z]*v[x] - q[x]*v[z] + q[w]*v[y];
    r[z] = + q[x]*v[y] - q[y]*v[x] + q[w]*v[z];
    r[w] = - q[x]*v[x] - q[y]*v[y] - q[z]*v[z];
}


AA_API void
aa_tf_qrot( const double _q[AA_RESTRICT 4], const double v[AA_RESTRICT 3], double r[AA_RESTRICT 3] )
{
    const double *q_v  = _q+AA_TF_QUAT_XYZ;
    const double q_w  = _q[AA_TF_QUAT_W];;
    double a[3];
    aa_tf_cross( q_v, v, a);
    FOR_VEC(i) a[i] += q_w * v[i];
    aa_tf_cross( q_v, a, r);
    FOR_VEC(i) r[i] += r[i] + v[i];
}


AA_API void
aa_tf_qpexp( const double q[AA_RESTRICT 3], double r[AA_RESTRICT 4] )
{
    double sc, c;
    aa_tf_sinccos2( aa_tf_vdot(q,q), &sc, &c );
    r[AA_TF_QUAT_W] = c;
    FOR_VEC(i) r[AA_TF_QUAT_V + i] = sc*q[i];
}

AA_API void
aa_tf_qexp( const double q[AA_RESTRICT 4], double r[AA_RESTRICT 4] )
{
    double sc, c, ew;
    const double *q_v = q + AA_TF_QUAT_V;
    double *r_v = r + AA_TF_QUAT_V;

    ew = exp(q[AA_TF_QUAT_W]);
    aa_tf_sinccos2( aa_tf_vdot(q_v,q_v), &sc, &c );
    r[AA_TF_QUAT_W] = ew*c;

    FOR_VEC(i) r_v[i] = ew*sc*q_v[i];
}

AA_API void
aa_tf_qln( const double q[AA_RESTRICT 4], double r[AA_RESTRICT 4] )
{
    double vv, vnorm, qnorm, theta, a;
    const double *q_v = q + AA_TF_QUAT_V;
    double q_w = q[AA_TF_QUAT_W];

    vv = aa_tf_vdot(q_v, q_v );
    qnorm = sqrt(vv + q_w*q_w);
    vnorm = sqrt(vv);          /* for unit quaternions, vnorm = sin(theta) */
    theta = atan2(vnorm, q_w); /* always positive */

    if( theta < sqrt(sqrt(DBL_EPSILON)) ) {
        a = aa_tf_invsinc_series(theta)/qnorm; /* approx. 1/qnorm */
    } else {
        a = theta/vnorm;
    }

    FOR_VEC(i) r[AA_TF_QUAT_V + i] = a*q_v[i];
    r[AA_TF_QUAT_W] = log(qnorm);
}

AA_API void
aa_tf_qpow( const double q[AA_RESTRICT 4], double a, double r[AA_RESTRICT 4] )
{
    double qln[4];
    aa_tf_qln(q,qln);
    FOR_QUAT(i) qln[i] *= a;
    aa_tf_qexp(qln, r);
}

AA_API double
aa_tf_qangle( const double _q[AA_RESTRICT 4] )
{
    aa_tf_quat_t *q = (aa_tf_quat_t *)_q;
    return atan2( sqrt( aa_tf_vdot(q->v, q->v) ),
                  q->w );
}

AA_API void
aa_tf_qsvel( const double q0[AA_RESTRICT 4], const double w[AA_RESTRICT 3], double dt,
             double q1[AA_RESTRICT 4] )
{
    double wq[3], e[4];
    FOR_VEC(i) wq[i] = w[i] * dt/2;
    aa_tf_qpexp(wq, e);
    aa_tf_qmul(e, q0, q1);
}

AA_API void
aa_tf_qsdiff( const double q0[AA_RESTRICT 4], const double dq[AA_RESTRICT 4], double dt,
              double q1[AA_RESTRICT 4] )
{
    double w[4], r[4];
    aa_tf_qmulc( dq, q0, w );
    FOR_QUAT(i) w[i] *= dt;
    aa_tf_qpexp(w,r);
    aa_tf_qmul(r,q0,q1);
}
