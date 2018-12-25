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

/*******/
/* OPS */
/*******/

AA_API void
aa_tf_tf_qv( const double q[AA_RESTRICT 4],
             const double v[AA_RESTRICT 3],
             const double p0[AA_RESTRICT 3],
             double p1[AA_RESTRICT 4] )
{
    //aa_tf_qrot( quat, p0, p1 );

    double a[3];
    double k[3];

    k[0] = p0[0] + p0[0];
    k[1] = p0[1] + p0[1];
    k[2] = p0[2] + p0[2];

    DECLARE_QUAT_XYZW;
    a[0] =  (q[y]*k[z] - q[z]*k[y]) + q[w]*k[x];
    a[1] =  (q[z]*k[x] - q[x]*k[z]) + q[w]*k[y];
    a[2] =  (q[x]*k[y] - q[y]*k[x]) + q[w]*k[z];

    p1[0] = p0[0] + v[0] + q[1]*a[2] - q[2]*a[1];
    p1[1] = p0[1] + v[1] + q[2]*a[0] - q[0]*a[2];
    p1[2] = p0[2] + v[2] + q[0]*a[1] - q[1]*a[0];

    /* p1[0] = (p0[0] +  q[1]*a[2]) + (v[0] - q[2]*a[1]); */
    /* p1[1] = (p0[1] +  q[2]*a[0]) + (v[1] - q[0]*a[2]); */
    /* p1[2] = (p0[2] +  q[0]*a[1]) + (v[2] - q[1]*a[0]); */

    /* p1[0] = p0[0] + v[0]; */
    /* p1[1] = p0[1] + v[1]; */
    /* p1[2] = p0[2] + v[2]; */

    /* aa_tf_cross_a(q,a,p1); */

}

AA_API void
aa_tf_qutr_tf( const double E[AA_RESTRICT 7], const double p0[AA_RESTRICT 3],
               double p1[AA_RESTRICT 3] )
{
    aa_tf_tf_qv( E+AA_TF_QUTR_Q, E+AA_TF_QUTR_V,
                 p0, p1 );
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
aa_tf_qutr_mul( const double A[AA_RESTRICT 7], const double B[AA_RESTRICT 7],
                double C[AA_RESTRICT 7] )
{
    aa_tf_qv_chain( A+AA_TF_QUTR_Q, A+AA_TF_QUTR_V,
                    B+AA_TF_QUTR_Q, B+AA_TF_QUTR_V,
                    C+AA_TF_QUTR_Q, C+AA_TF_QUTR_V );
}

AA_API void
aa_tf_qv_chainnorm( const double q1[AA_RESTRICT 4], const double v1[AA_RESTRICT 3],
                    const double q2[AA_RESTRICT 4], const double v2[AA_RESTRICT 3],
                    double q3[AA_RESTRICT 4], double v3[AA_RESTRICT 3] )
{
    aa_tf_qmulnorm( q1, q2, q3 );
    aa_tf_tf_qv( q1, v1, v2, v3 );
}

AA_API void
aa_tf_qutr_mulnorm( const double A[AA_RESTRICT 7], const double B[AA_RESTRICT 7],
                double C[AA_RESTRICT 7] )
{
    aa_tf_qv_chainnorm( A+AA_TF_QUTR_Q, A+AA_TF_QUTR_V,
                        B+AA_TF_QUTR_Q, B+AA_TF_QUTR_V,
                        C+AA_TF_QUTR_Q, C+AA_TF_QUTR_V );
}

AA_API void
aa_tf_qv_conj( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
               double qc[AA_RESTRICT 4], double vc[AA_RESTRICT 3] )
{
    aa_tf_qconj( q, qc );
    aa_tf_qrot( qc, v, vc );
    FOR_VEC(i) vc[i] = -vc[i];
}

AA_API void
aa_tf_qutr_conj( const double A[AA_RESTRICT 7], double B[AA_RESTRICT 7] )
{
    aa_tf_qv_conj( A+AA_TF_QUTR_Q, A+AA_TF_QUTR_V,
                   B+AA_TF_QUTR_Q, B+AA_TF_QUTR_V );
}

AA_API void
aa_tf_qutr_cmul( const double A[AA_RESTRICT 7], const double B[AA_RESTRICT 7], double C[AA_RESTRICT 7] )
{
    double x[7];
    aa_tf_qutr_conj(A,x);
    aa_tf_qutr_mul(x,B,C);
}

AA_API void
aa_tf_qutr_mulc( const double A[AA_RESTRICT 7], const double B[AA_RESTRICT 7], double C[AA_RESTRICT 7] )
{
    double x[7];
    aa_tf_qutr_conj(B,x);
    aa_tf_qutr_mul(A,x,C);
}


/***********/
/* EXP/LOG */
/***********/

void aa_tf_qv_expv( const double w[3],  const double dv[3],
                    double q[4], double v[3] )
{
    double phi2 = aa_tf_vdot(w,w);
    double sc, c, k, sc2, csc;
    if( phi2 < sqrt(DBL_EPSILON) ) {
        sc = aa_tf_sinc_series2(phi2);
        c = aa_tf_cos_series2(phi2);
        sc2 = 2*sc;
        csc = c*sc2;
        k = aa_horner3( phi2, 4.0/3, -4.0/15, 8.0/315 );
    } else {
        double phi = sqrt(phi2);
        double s = sin(phi);
        c = cos(phi);
        sc = s/phi;
        sc2 = 2*sc;
        csc = c*sc2;
        k = (2-csc)/phi2;
    }

    // real
    q[AA_TF_QUAT_W] = c;
    FOR_VEC(i) q[AA_TF_QUAT_V + i] = sc*w[i];

    // dual
    const double *q_v = q + AA_TF_QUAT_V;
    double gammak = aa_tf_vdot(w,dv) * k;
    double cr[3];
    aa_tf_cross(q_v, dv, cr);
    FOR_VEC(i) v[i] =  csc*dv[i] + gammak*w[i] + sc2*cr[i];
}


void aa_tf_qutr_expv
( const double w[6], double e[7] )
{
    aa_tf_qv_expv(w+AA_TF_DX_W, w+AA_TF_DX_V,
                  e+AA_TF_QUTR_Q, e+AA_TF_QUTR_T);
}

void aa_tf_qv_lnv
( const double q[4], const double v[4], double w[3], double dv[3] )
{
    double isc, cisc, k;
    const double *q_v = q + AA_TF_QUAT_V;
    double v2 = aa_tf_vdot(q_v, q_v);
    double s = sqrt(v2);
    double c = q[AA_TF_QUAT_W];
    double phi = atan2(s,c);
    double phi2 = phi*phi;

    if( phi < sqrt(DBL_EPSILON) ) {
        isc = aa_tf_invsinc_series2(phi2);
        k = aa_horner3( phi2, 1.0/3, 1.0/45, 2.0/945 );
        cisc = 1 - k * phi2;
    } else {
        isc = phi/s;
        cisc = c*isc;
        k = (1 - cisc)/phi2;
    }

    /* Real */
    FOR_VEC(i) w[i] = isc * q_v[i];

    /* Dual  */
    double v_2[3], cr[3];
    FOR_VEC(i) v_2[i] = v[i]/2;
    double a = k*aa_tf_vdot(v_2,w);
    aa_tf_cross(v_2, w, cr);

    FOR_VEC(i) dv[i] =  cr[i] + cisc*v_2[i] + a*w[i];
}

void aa_tf_qutr_lnv
( const double e[7], double w[6] )
{
    aa_tf_qv_lnv(
        e+AA_TF_QUTR_Q, e+AA_TF_QUTR_T,
        w+AA_TF_DX_W, w+AA_TF_DX_V
        );
}

void aa_tf_qutr_twist2vel
( const double e[7], const double w[6], double dx[6] )
{
    aa_tf_qv_twist2vel( e+AA_TF_QUTR_Q, e+AA_TF_QUTR_V,
                        w+AA_TF_DX_W, w+AA_TF_DX_V,
                        dx+AA_TF_DX_W, dx+AA_TF_DX_V );
}

/************/
/* CALCULUS */
/************/


AA_API void
aa_tf_qv_vel2twist( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                    const double w[AA_RESTRICT 3], const double dv[AA_RESTRICT 3],
                    double tw[AA_RESTRICT 3], double tv[AA_RESTRICT 3] )
{
    (void)q;
    // rotational
    AA_MEM_CPY( tw, w, 3 );

    // translation
    aa_tf_cross( v, w, tv );
    FOR_VEC(i) tv[i] += dv[i];
}

AA_API void
aa_tf_qv_twist2vel( const double q[AA_RESTRICT 4], const double v[AA_RESTRICT 3],
                    const double tw[AA_RESTRICT 3], const double tv[AA_RESTRICT 3],
                    double w[AA_RESTRICT 3], double dv[AA_RESTRICT 3] )
{
    (void)q;
    // rotational
    AA_MEM_CPY( w, tw, 3 );

    // translation
    FOR_VEC(i) dv[i] = tv[i];
    aa_tf_cross_a(tw, v, dv );
}

AA_API void
aa_tf_qv_svel( const double q0[AA_RESTRICT 4], const double v0[AA_RESTRICT 3],
               const double w[AA_RESTRICT 3], const double dv[AA_RESTRICT 3],
               double dt,
               double q1[AA_RESTRICT 4], double v1[AA_RESTRICT 3] )
{
    double tw[3], tv[3], ew[4], ev[3];
    aa_tf_qv_vel2twist(q0,v0,w,dv,tw,tv);
    FOR_VEC(i) {
        tw[i] *= dt/2;
        tv[i] *= dt/2;
    }
    aa_tf_qv_expv(tw,tv,ew,ev);
    aa_tf_qv_chain( ew, ev, q0, v0, q1, v1 );

}

AA_API void
aa_tf_qv_sdiff( const double q0[AA_RESTRICT 4], const double v0[AA_RESTRICT 3],
                const double dq[AA_RESTRICT 4], const double dv[AA_RESTRICT 3],
                double dt,
                double q1[AA_RESTRICT 4], double v1[AA_RESTRICT 3] )
{
    double w[3];
    aa_tf_qdiff2vel( q0, dq, w );
    aa_tf_qv_svel( q0, v0, w, dv, dt, q1, v1 );
}

AA_API void
aa_tf_qutr_svel( const double E0[AA_RESTRICT 7], const double dx[AA_RESTRICT 6], double dt,
                 double E1[AA_RESTRICT 7] )
{
    aa_tf_qv_svel( E0+AA_TF_QUTR_Q, E0+AA_TF_QUTR_V,
                   dx+AA_TF_DX_W, dx+AA_TF_DX_V,
                   dt,
                   E1+AA_TF_QUTR_Q, E1+AA_TF_QUTR_V );
}

AA_API void
aa_tf_qutr_sdiff( const double E0[AA_RESTRICT 7], const double dE[AA_RESTRICT 7], double dt,
                  double E1[AA_RESTRICT 7] )
{
    aa_tf_qv_sdiff( E0+AA_TF_QUTR_Q, E0+AA_TF_QUTR_V,
                    dE+AA_TF_QUTR_Q, dE+AA_TF_QUTR_V,
                    dt,
                    E1+AA_TF_QUTR_Q, E1+AA_TF_QUTR_V );
}

AA_API void
aa_tf_qutr_diff2vel( const double E[AA_RESTRICT 7], const double dE[AA_RESTRICT 7], double dx[AA_RESTRICT 6] )
{
    aa_tf_qdiff2vel(E+AA_TF_QUTR_Q, dE+AA_TF_QUTR_Q, dx+AA_TF_DX_W );
    AA_MEM_CPY( dx+AA_TF_DX_V, dE+AA_TF_QUTR_V, 3 );
}

AA_API void
aa_tf_qutr_vel2diff( const double E[AA_RESTRICT 7], const double dx[AA_RESTRICT 6], double dE[AA_RESTRICT 7] )
{
    aa_tf_qvel2diff(E+AA_TF_QUTR_Q, dx+AA_TF_DX_W, dE+AA_TF_QUTR_Q );
    AA_MEM_CPY( dE+AA_TF_QUTR_V, dx+AA_TF_DX_V, 3 );
}
