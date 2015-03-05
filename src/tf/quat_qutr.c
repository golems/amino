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

/**************/
/* CONVERSION */
/**************/

AA_API void
aa_tf_qutr2duqu( const double E[AA_RESTRICT 7], double S[AA_RESTRICT 8] )
{
    aa_tf_qv2duqu( E+AA_TF_QUTR_Q, E+AA_TF_QUTR_V, S );
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

/*******/
/* OPS */
/*******/

AA_API void
aa_tf_tf_qv( const double quat[AA_RESTRICT 4],
             const double v[AA_RESTRICT 3],
             const double p0[AA_RESTRICT 3],
             double p1[AA_RESTRICT 4] )
{
    aa_tf_qrot( quat, p0, p1 );
    FOR_VEC(i) p1[i] += v[i];
}

AA_API void
aa_tf_qutr_tf( const double E[AA_RESTRICT 7], const double p0[AA_RESTRICT 3],
               double p1[AA_RESTRICT 3] )
{
    aa_tf_qrot( E+AA_TF_QUTR_Q, p0, p1 );
    FOR_VEC( i ) p1[i] += E[AA_TF_QUTR_V+i];
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

/************/
/* CALCULUS */
/************/

AA_API void
aa_tf_qv_svel( const double q0[AA_RESTRICT 4], const double v0[AA_RESTRICT 3],
               const double w[AA_RESTRICT 3], const double dv[AA_RESTRICT 3],
               double dt,
               double q1[AA_RESTRICT 4], double v1[AA_RESTRICT 3] )
{
    double dx[6], S0[8], S1[8];
    /* TODO: avoid the memcpy()s */
    AA_MEM_CPY( dx+AA_TF_DX_W, w, 3 );
    AA_MEM_CPY( dx+AA_TF_DX_V, dv, 3 );
    aa_tf_qv2duqu( q0, v0, S0 );
    aa_tf_duqu_svel( S0, dx, dt, S1 );
    aa_tf_duqu2qv(S1, q1, v1);
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
