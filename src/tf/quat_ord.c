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


#define WITH_VEC_XYZ(var,x,y,z)         \
    double x = (var)[AA_TF_QUAT_X];     \
    double y = (var)[AA_TF_QUAT_Y];     \
    double z = (var)[AA_TF_QUAT_Z];

#define WITH_QUAT_XYZW(var,x,y,z,w)             \
    WITH_VEC_XYZ((var)+AA_TF_QUAT_XYZ,x,y,z);   \
    double w = (var)[AA_TF_QUAT_W];

#define sqrt_epsilon sqrt(DBL_EPSILON)
#define sqrt_sqrt_epsilon sqrt(sqrt(DBL_EPSILON))

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
aa_tf_qnormalize2( const double q[AA_RESTRICT 4], double r[AA_RESTRICT 4] )
{
    double n = aa_tf_qnorm(q);
    FOR_QUAT(i) r[i] = q[i] / n;
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
aa_tf_qmatrix_l( const double q[AA_RESTRICT 4], double *AA_RESTRICT M, size_t ldm )
{
    DECLARE_QUAT_XYZW;

    AA_MATREF(M,ldm, x,x) =  q[w];
    AA_MATREF(M,ldm, y,x) =  q[z];
    AA_MATREF(M,ldm, z,x) = -q[y];
    AA_MATREF(M,ldm, w,x) = -q[x];

    AA_MATREF(M,ldm, x,y) = -q[z];
    AA_MATREF(M,ldm, y,y) =  q[w];
    AA_MATREF(M,ldm, z,y) =  q[x];
    AA_MATREF(M,ldm, w,y) = -q[y];

    AA_MATREF(M,ldm, x,z) =  q[y];
    AA_MATREF(M,ldm, y,z) = -q[x];
    AA_MATREF(M,ldm, z,z) =  q[w];
    AA_MATREF(M,ldm, w,z) = -q[z];

    AA_MATREF(M,ldm, x,w) =  q[x];
    AA_MATREF(M,ldm, y,w) =  q[y];
    AA_MATREF(M,ldm, z,w) =  q[z];
    AA_MATREF(M,ldm, w,w) =  q[w];
}

AA_API void
aa_tf_qmatrix_r( const double q[AA_RESTRICT 4], double *AA_RESTRICT M, size_t ldm )
{
    DECLARE_QUAT_XYZW;

    AA_MATREF(M,ldm, x,x) =  q[w];
    AA_MATREF(M,ldm, y,x) = -q[z];
    AA_MATREF(M,ldm, z,x) =  q[y];
    AA_MATREF(M,ldm, w,x) = -q[x];

    AA_MATREF(M,ldm, x,y) =  q[z];
    AA_MATREF(M,ldm, y,y) =  q[w];
    AA_MATREF(M,ldm, z,y) = -q[x];
    AA_MATREF(M,ldm, w,y) = -q[y];

    AA_MATREF(M,ldm, x,z) = -q[y];
    AA_MATREF(M,ldm, y,z) =  q[x];
    AA_MATREF(M,ldm, z,z) =  q[w];
    AA_MATREF(M,ldm, w,z) = -q[z];

    AA_MATREF(M,ldm, x,w) =  q[x];
    AA_MATREF(M,ldm, y,w) =  q[y];
    AA_MATREF(M,ldm, z,w) =  q[z];
    AA_MATREF(M,ldm, w,w) =  q[w];
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
aa_tf_qmul_a( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4], double c[AA_RESTRICT 4] )
{
    DECLARE_QUAT_XYZW;

    c[x] =  c[x] + a[x]*b[w] + a[y]*b[z] + a[w]*b[x] - a[z]*b[y];
    c[y] =  c[y] + a[z]*b[x] + a[w]*b[y] + a[y]*b[w] - a[x]*b[z];
    c[z] =  c[z] + a[w]*b[z] + a[z]*b[w] + a[x]*b[y] - a[y]*b[x];
    c[w] =  c[w] - a[y]*b[y] - a[x]*b[x] - a[z]*b[z] + a[w]*b[w];
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


/* compute vector part of a*conj(b) */
AA_API void
aa_tf_qmulc_v( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4],
               double v[AA_RESTRICT 3] )
{
    DECLARE_QUAT_XYZW;
    /* 12 multiplies, 9 adds */
    v[0] = a[z]*b[y] - a[y]*b[z] + a[x]*b[w] - a[w]*b[x];
    v[1] = a[x]*b[z] - a[z]*b[x] + a[y]*b[w] - a[w]*b[y];
    v[2] = a[y]*b[x] - a[x]*b[y] + a[z]*b[w] - a[w]*b[z];
}

AA_API void
aa_tf_qmulc( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4],
             double c[AA_RESTRICT 4] )
{
    DECLARE_QUAT_XYZW;
    aa_tf_qmulc_v(a,b,c+x);
    c[w] = ( a[x]*b[x] + a[y]*b[y] ) + ( a[z]*b[z] + a[w]*b[w] );
}

AA_API void
aa_tf_qmul_vq( const double v[AA_RESTRICT 3], const double q[AA_RESTRICT 4], double r[AA_RESTRICT 4] )
{
    DECLARE_QUAT_XYZW;

    r[x] = +  v[y]*q[z] - v[z]*q[y] + v[x]*q[w];
    r[y] = +  v[z]*q[x] - v[x]*q[z] + v[y]*q[w];
    r[z] = +  v[x]*q[y] - v[y]*q[x] + v[z]*q[w];

    r[AA_TF_QUAT_W] = -aa_tf_vdot(q+AA_TF_QUAT_V,v);
}

AA_API void
aa_tf_qmul_qv_v( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3], double r[AA_RESTRICT 3] )
{
    DECLARE_QUAT_XYZW;
    r[0] = + q[y]*v[z] - q[z]*v[y] + q[w]*v[x];
    r[1] = + q[z]*v[x] - q[x]*v[z] + q[w]*v[y];
    r[2] = + q[x]*v[y] - q[y]*v[x] + q[w]*v[z];
}

AA_API void
aa_tf_qmul_qv( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 2], double r[AA_RESTRICT 4] )
{
    aa_tf_qmul_qv_v(q,v,r+AA_TF_QUAT_V);
    r[AA_TF_QUAT_W] = -aa_tf_vdot(q+AA_TF_QUAT_V,v);
}


AA_API void
aa_tf_qrot( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
            double r[AA_RESTRICT 3] )
{
    double a[3];

    aa_tf_qmul_qv_v(q,v,a);
    FOR_VEC(i) a[i] *= 2;

    FOR_VEC(i) r[i] = v[i];
    aa_tf_cross_a( q+AA_TF_QUAT_V, a, r);
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
    double wq[3], e[4];
    aa_tf_qmulc_v( dq, q0, wq );
    FOR_VEC(i) wq[i] *= dt;

    aa_tf_qpexp(wq,e);
    aa_tf_qmul(e,q0,q1);
}

AA_API void
aa_tf_qdiff2vel( const double q[AA_RESTRICT 4], const double dq[AA_RESTRICT 4], double w[AA_RESTRICT 3] )
{
    aa_tf_qmulc_v(dq,q,w);
    FOR_VEC(i) w[i] *= 2;
}

AA_API void
aa_tf_qvel2diff( const double q[AA_RESTRICT 4], const double w[AA_RESTRICT 3], double dq[AA_RESTRICT 4] )
{
    double w2[3];
    FOR_VEC(i) w2[i] = w[i]/2;
    aa_tf_qmul_vq(w2,q,dq);
}

AA_API double
aa_tf_qangle_rel( const double a[AA_RESTRICT 4], const double b[AA_RESTRICT 4] )
{
    double r[4];
    aa_tf_qcmul(a,b,r);
    aa_tf_qminimize(r);
    return aa_tf_qangle(r);
}

AA_API void
aa_tf_qslerp_param( const double q1[AA_RESTRICT 4], const double q2[AA_RESTRICT 4],
                    double *theta, double *d1, double *d2 )
{
    *theta = fabs(aa_tf_quhypangle2(q1,q2));
    *d1 = sin(*theta);
    if( *theta > M_PI_2 ) {
        *theta = M_PI - *theta;
        *d2 = -*d1;
    } else {
        *d2 = *d1;
    }
}


AA_API void
aa_tf_qslerp_eval( double tau, const double q1[AA_RESTRICT 4], const double q2[AA_RESTRICT 4],
                   double theta, double d1, double d2,
                   double r[AA_RESTRICT 4] )
{
    if( 0 < fabs(theta) ) {
        double s0 = tau*theta;
        double s1 = sin(theta-s0);
        double s2 = sin(s0);
        double a1 = s1/d1;
        double a2 = s2/d2;
        FOR_QUAT(i) r[i] = a1*q1[i] + a2*q2[i];
    } else {
        AA_MEM_CPY(r,q1,4);
    }
}

AA_API void
aa_tf_qslerp( double tau, const double q1[AA_RESTRICT 4], const double q2[AA_RESTRICT 4],
              double r[AA_RESTRICT 4] )
{
    double theta, d1, d2;
    aa_tf_qslerp_param( q1, q2, &theta, &d1, &d2 );
    aa_tf_qslerp_eval( tau, q1, q2, theta, d1, d2, r );
}

AA_API void
aa_tf_qslerpalg( double tau, const double q1[AA_RESTRICT 4], const double q2[AA_RESTRICT 4],
              double r[AA_RESTRICT 4] )
{
    /* q1 * (conj(q1) * q2) ^ t  */
    double q_rel[4], qp[4];
    aa_tf_qcmul( q1, q2, q_rel );
    aa_tf_qminimize(q_rel);        /* go the short way */
    aa_tf_qpow( q_rel, tau, qp );
    aa_tf_qmul( q1, qp, r );
    aa_tf_qnormalize(r);
}


AA_API void aa_tf_qrk1(
    const double q0[AA_RESTRICT 4],
    const double dq[AA_RESTRICT 4],
    double dt,
    double q1[AA_RESTRICT 4] )
{
    FOR_QUAT(i) q1[i] = q0[i] + dt*dq[i];
}



AA_API void aa_tf_qvelrk1(
    const double q0[AA_RESTRICT 4],
    const double v[AA_RESTRICT 3],
    double dt,
    double q1[AA_RESTRICT 4] )
{
    double dq[4];
    aa_tf_qvel2diff(q0,v,dq);
    aa_tf_qrk1(q0,dq,dt,q1);
}


static void qvelrk4_term (
    const double q0[4],
    const double v[3],
    const double dq0[4],
    double dt,
    double dq1[4]
    )
{
    double qtmp[4];
    aa_tf_qrk1( q0, dq0, dt, qtmp );
    aa_tf_qvel2diff(qtmp, v, dq1 );
}

AA_API void aa_tf_qvelrk4(
    const double q0[AA_RESTRICT 4],
    const double v[AA_RESTRICT 3],
    double dt,
    double q1[AA_RESTRICT 4] )
{
    double dq[4][4], qtmp[4];
    aa_tf_qvel2diff(q0, v, dq[0]);
    qvelrk4_term( q0, v, dq[0], dt/2, dq[1] );
    qvelrk4_term( q0, v, dq[1], dt/2, dq[2] );
    qvelrk4_term( q0, v, dq[2], dt, dq[3] );

    FOR_QUAT(i) qtmp[i] = dq[0][i] + dq[3][i] + 2*(dq[1][i]+dq[2][i]);
    aa_tf_qrk1(q0, qtmp, dt/6, q1 );
}



/* Jacobian of unit quaternion logarithm */
static void qjuln(
    const double q[AA_RESTRICT 4],
    double J[AA_RESTRICT 3*4] )
{
    WITH_QUAT_XYZW(q, x,y,z,w );

    double s2 = aa_tf_vdot(q,q);
    double s = sqrt(s2);
    double phi = atan2(s, w); // always positive
    double isc = phi/s;
    double a = isc/s2;

    J[0]  = isc * (-x/s2 * x + 1);
    J[1]  = -a*x*y;
    J[2]  = -a*x*z;

    J[3]  = -a*x*y;
    J[4]  = isc * (-y/s2 * y + 1);
    J[5]  = -a*y*z;

    J[6]  = -a*x*z;
    J[7]  = -a*y*z;
    J[8]  = isc * (-z/s2 * z + 1);

    J[9]  = -x/s2;
    J[10] = -y/s2;
    J[11] = -z/s2;
}

AA_API void aa_tf_qdulnj(
    const double q[AA_RESTRICT 4],
    const double dq[AA_RESTRICT 4],
    double dln[AA_RESTRICT 3] )
{
    double J[3*4];
    qjuln(q,J);
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 3, 4,
                 1.0, J, 3,
                 dq, 1,
                 0, dln, 1 );
}

AA_API void aa_tf_qduln(
    const double q[AA_RESTRICT 4],
    const double dq[AA_RESTRICT 4],
    double dln[AA_RESTRICT 3] )
{

    double s2 = aa_tf_vdot(q, q);
    double s = sqrt(s2);
    double c = q[AA_TF_QUAT_W];
    double phi = atan2(s, c); // always positive

    double alpha, beta;
    if( phi < sqrt_sqrt_epsilon ) {
        alpha = aa_tf_invsinc_series(phi);
        beta = aa_horner3( phi*phi, -1.0/3, 2.0/15, -2.0/63 );
    } else {
        alpha = phi/s;
        beta = (alpha*c-1) / s2;
    }

    FOR_VEC(i) {
        dln[i] = alpha * dq[AA_TF_QUAT_XYZ+i] + dq[AA_TF_QUAT_W]*beta * q[AA_TF_QUAT_XYZ+i];
    }
}


static void qjpexp(
    const double u[3],
    double J[4*3] )
{
    double vv = aa_tf_vdot(u,u);
    WITH_VEC_XYZ(u,x,y,z);
    double sc, a;
    if( vv < DBL_EPSILON ) {
        sc = aa_tf_sinc_series2(vv);
        a = aa_horner3( vv, -1.0/3, 1.0/30, -1.0/840 );
    } else {
        double v = sqrt(vv);
        sc = sin(v)/v;
        a = (cos(v) - sc)/vv;
    }

    J[0]  = a * x*x + sc;
    J[1]  = a * x*y;
    J[2]  = a * x*z;
    J[3]  = -sc*x;

    J[4]  = a * x*y;
    J[5]  = a * y*y + sc;
    J[6]  = a * y*z;
    J[7]  = -sc*y;

    J[8]  = a * x*z;
    J[9]  = a * y*z;
    J[10] = a * z*z + sc;
    J[11] = -sc*z;
}

AA_API void aa_tf_qdpexpj(
    const double e[AA_RESTRICT 3],
    const double de[AA_RESTRICT 3],
    double dq[AA_RESTRICT 4] )
{
    double J[4*3];
    qjpexp(e,J);
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 4, 3,
                 1.0, J, 4,
                 de, 1,
                 0, dq, 1 );
}

AA_API void aa_tf_qdpexp(
    const double e[AA_RESTRICT 3],
    const double de[AA_RESTRICT 3],
    double dq[AA_RESTRICT 4] )
{
    const double vv = aa_tf_vdot(e,e);
    const double phi = sqrt(vv);
    const double dd = aa_tf_vdot(e, de);
    double sinc, k;

    if( phi < sqrt_sqrt_epsilon ) {
        sinc = aa_tf_sinc_series(phi);
        k = aa_horner3( vv, -1.0/3, 1.0/30, -1.0/840 );
    } else {
        sinc = sin(phi)/phi;
        k = cos(phi)/(phi*phi) - sin(phi)/(phi*phi*phi);
    }
    FOR_VEC (i) {
        dq[AA_TF_QUAT_XYZ+i] = sinc*de[i] + k*dd*e[i];
    }
    dq[AA_TF_QUAT_W] = -dd*sinc;
}

double aa_tf_quhypangle2
( const double x[AA_RESTRICT 4], const double y[AA_RESTRICT 4] )
{
    double a[4],b[4];
    FOR_QUAT(i) a[i] = x[i] - y[i];
    FOR_QUAT(i) b[i] = x[i] + y[i];
    return 2 * atan2( aa_tf_qnorm(a),
                      aa_tf_qnorm(b) );
}


static void qslerpdiff_param(
    const double q1[AA_RESTRICT 4],
    const double q2[AA_RESTRICT 4],
    double *theta, double *d1, double *d2 )
{

    *theta = fabs(aa_tf_quhypangle2(q1,q2));
    if( *theta > M_PI_2 ) {
        *theta = M_PI - *theta;
        *d1 = aa_tf_sinc(*theta);
        *d2 = -*d1;
    } else {
        *d1 = aa_tf_sinc(*theta);
        *d2 = *d1;
    }
}

AA_API void aa_tf_qslerpdiff(
    double tau, const double q1[AA_RESTRICT 4],
    const double q2[AA_RESTRICT 4],
    double r[AA_RESTRICT 4] )
{
    double theta, d1, d2;
    qslerpdiff_param( q1, q2, &theta, &d1, &d2 );
    const double s1 = cos(theta - tau*theta);
    const double s2 = cos(tau*theta);
    FOR_QUAT(i)
        r[i] = -s1/d1 * q1[i] + s2/d2 * q2[i];
}

AA_API void aa_tf_qslerpdiffalg(
    double tau, const double q1[AA_RESTRICT 4],
    const double q2[AA_RESTRICT 4],
    double dq[AA_RESTRICT 4] )
{
    double qm[4], ql[4], q[4];
    aa_tf_qslerpalg(tau, q1, q2, q);
    aa_tf_qmulc(q2, q1, qm);
    aa_tf_qminimize(qm);
    aa_tf_qln(qm,ql);
    aa_tf_qmul(ql,q,dq);
}


AA_API void aa_tf_qslerpchaindiff(
    double u, double du,
    const double q1[AA_RESTRICT 4], const double dq1[AA_RESTRICT 4],
    const double q2[AA_RESTRICT 4], const double dq2[AA_RESTRICT 4],
    double q[AA_RESTRICT 4], double dq[AA_RESTRICT 4] )
{
    // TODO: test case
    double theta, d1, d2;
    aa_tf_qslerp_param(q1, q2, &theta, &d1, &d2);
    if( aa_feq(theta, 0, 0) ) {
        AA_MEM_ZERO(dq, 4);
        AA_MEM_CPY(q, q1, 4);
        return;
    }
    //const double s = sin(theta);
    const double c = cos(theta);
    const double sa = sin((1-u)*theta);
    const double ca = cos((1-u)*theta);
    const double sb = sin(u*theta);
    const double cb = cos(u*theta);

    const double dtheta_c = aa_tf_qdot(q1, dq2) + aa_tf_qdot(dq1, q2);
    const double dtheta =  dtheta_c / c;

    const double a = sa / d1;
    const double b = sb / d2;

    const double da = ( ca * (dtheta*(1-u) - du*theta) ) / d1
        - ( dtheta_c * sa ) / (d1*d1);

    const double db = ( (dtheta*u + theta*du) * cb ) / d2
        - ( dtheta_c * b ) / d1;

    FOR_QUAT(i) {
        q[i] = q1[i]*a + q2[i]*b;
        dq[i] = (dq1[i]*a + q1[i]*da) + (dq2[i]*b + q2[i]*db);
    }
}

AA_API void aa_tf_quat_davenport_matrix (
    size_t n, const double *w,
    const double *qq, size_t ldqq,
    double *M )
{
    AA_MEM_ZERO(M,4*4);
    for( size_t i = 0; i < n; i ++ ) {
        const double *q = AA_MATCOL(qq,ldqq,i);
        cblas_dgemm( CblasColMajor,
                     CblasNoTrans, CblasTrans,
                     4, 4, 1,
                     w[i], q, 4,
                     q, 4,
                     1.0, M, 4 );
        /* cblas_dsyrk( CblasColMajor, (enum CBLAS_UPLO)0, CblasNoTrans, */
        /*              4, 1, */
        /*              w[i], q, 4, */
        /*              1.0, M, 4 ); */
    }
    /* printf("\n"); */
    /* aa_dump_mat( stdout, M, 4, 4 ); */
}
