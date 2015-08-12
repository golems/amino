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
#include <float.h>


AA_API void
aa_tf_9( const double R[AA_RESTRICT 9],
         const double p0[AA_RESTRICT 3],
         double p1[AA_RESTRICT 4] )
{
    aa_tf_rotmat_rot( R, p0, p1 );
}

AA_API void
aa_tf_9mul( const double R0[AA_RESTRICT 9],
            const double R1[AA_RESTRICT 9],
            double R[AA_RESTRICT 9] )
{
    aa_tf_rotmat_mul(R0,R1,R);
}

AA_API void
aa_tf_93( const double R[AA_RESTRICT 9],
          const double v[AA_RESTRICT 3],
          const double p0[AA_RESTRICT 3],
          double p1[AA_RESTRICT 4] )
{
    aa_tf_tfmat2_tf(R, v, p0, p1 );
}


AA_API void
aa_tf_93chain( const double R0[AA_RESTRICT 9],
               const double v0[AA_RESTRICT 3],
               const double R1[AA_RESTRICT 9],
               const double v1[AA_RESTRICT 3],
               double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] )
{
    aa_tf_tfmat2_mul( R0, v0,
                      R1, v1,
                      R, v );
}


/*************/
/* ROTATIONS */
/*************/
static void
vnormalize2( const double x[AA_RESTRICT 3],
             double x_norm[AA_RESTRICT 3] )
{
    double n = sqrt( x[0]*x[0] + x[1]*x[1] + x[2]*x[2] );
    for( size_t i = 0; i < 3; i ++ ) x_norm[i] = x[i] / n;
}


AA_API void
aa_tf_rotmat_xy( const double x_axis[AA_RESTRICT 3],
                 const double y_axis[AA_RESTRICT 3],
                 double R[AA_RESTRICT 9] )
{
    vnormalize2( x_axis, R+0 );
    vnormalize2( y_axis, R+3 );
    aa_tf_cross( R+0, R+3, R+6 );
}

AA_API void
aa_tf_rotmat_yz( const double y_axis[AA_RESTRICT 3],
                 const double z_axis[AA_RESTRICT 3],
                 double R[AA_RESTRICT 9] )
{
    vnormalize2( y_axis, R+3 );
    vnormalize2( z_axis, R+6 );
    aa_tf_cross( R+3, R+6, R+0 );

}

AA_API void
aa_tf_rotmat_zx( const double z_axis[AA_RESTRICT 3],
                 const double x_axis[AA_RESTRICT 3],
                 double R[AA_RESTRICT 9] )
{
    vnormalize2( z_axis, R+6 );
    vnormalize2( x_axis, R+0 );
    aa_tf_cross( R+6, R+0, R+3 );

}


#define RREF(R,row,col) ((R)[(col)*3 + (row)])

AA_API void
aa_tf_skew_sym( const double u[AA_RESTRICT 3],
                double R[AA_RESTRICT 9] )
{
    double x=u[0], y=u[1], z=u[2];
    RREF(R,0,0) = 0;
    RREF(R,1,0) = z;
    RREF(R,2,0) = -y;

    RREF(R,0,1) = -z;
    RREF(R,1,1) = 0;
    RREF(R,2,1) = x;

    RREF(R,0,2) = y;
    RREF(R,1,2) = -x;
    RREF(R,2,2) = 0;
}
AA_API void
aa_tf_skew_sym1( double a, const double u[AA_RESTRICT 3],
                 double R[AA_RESTRICT 9] )
{
    double au[3] = { a*u[0], a*u[1], a*u[2] };
    aa_tf_skew_sym( au, R );


}


AA_API void
aa_tf_skewsym_scal_c( const double u[AA_RESTRICT 3],
                      const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3],
                      double R[AA_RESTRICT 9] )
{
    double  bu[3] = { b[0]*u[0],
                      b[1]*u[1],
                      b[2]*u[2] };

    double bc[3] = { b[0]*u[1],
                     b[1]*u[2],
                     b[2]*u[0] };

    double cx = 1 - bu[2] - bu[1];
    double cy = 1 - bu[2] - bu[0];
    double cz = 1 - bu[1] - bu[0];

    double dx = a[2] + bc[0];
    double dy = a[0] + bc[1];
    double dz = a[1] + bc[2];

    double ex = bc[2] - a[1];
    double ey = bc[0] - a[2];
    double ez = bc[1] - a[0];

    RREF(R,0,0) = cx;
    RREF(R,1,0) = dx;
    RREF(R,2,0) = ex;

    RREF(R,0,1) = ey;
    RREF(R,1,1) = cy;
    RREF(R,2,1) = dy;

    RREF(R,0,2) = dz;
    RREF(R,1,2) = ez;
    RREF(R,2,2) = cz;
}


AA_API void
aa_tf_skewsym_scal2( double a, double b, const double u[AA_RESTRICT 3],
                     double R[AA_RESTRICT 9] )
{
    double au[3] = {a*u[0], a*u[1], a*u[2]};
    double bu[3] = {b*u[0], b*u[1], b*u[2]};
    aa_tf_skewsym_scal_c( u, au, bu, R );
}

AA_API void
aa_tf_unskewsym_scal( double c, const double R[AA_RESTRICT 9], double u[AA_RESTRICT 3] )
{
    double a[3] = {RREF(R,2,1), RREF(R,0,2), RREF(R,1,0)};
    double b[3] = {RREF(R,1,2), RREF(R,2,0), RREF(R,0,1)};

    FOR_VEC(i) u[i] = c * ( a[i] - b[i] );
}

AA_API void
aa_tf_unskewsym( const double R[AA_RESTRICT 9], double u[AA_RESTRICT 3] )
{
    double tr = RREF(R,0,0) + RREF(R,1,1) + RREF(R,2,2);
    double c = sqrt( tr + 1 ) / 2;
    aa_tf_unskewsym_scal( c, R, u );
}

AA_API void
aa_tf_rotmat_exp_aa( const double aa[AA_RESTRICT 4], double E[9] )
{
    double s = sin(aa[W]), c = cos(aa[W]);
    aa_tf_skewsym_scal2( s, 1-c, aa, E );
}

AA_API void
aa_tf_rotmat_expv( const double rv[AA_RESTRICT 4], double E[9] )
{
    double theta2 = dot3(rv,rv);
    double theta = sqrt(theta2);
    double sc,cc;
    if( theta2*theta2 < DBL_EPSILON ) {
        sc = aa_tf_sinc_series2(theta2);
        cc = theta * aa_horner3( theta2, 1./2, -1./24, 1./720 );
    } else {
        double s = sin(theta), c = cos(theta);
        sc = s/theta;
        cc = (1-c) / theta2;
    }
    aa_tf_skewsym_scal2( sc, cc, rv, E );
}

AA_API void
aa_tf_rotmat_angle( const double R[AA_RESTRICT 9], double *c, double *s, double *theta )
{
    *c = (RREF(R,0,0) + RREF(R,1,1) + RREF(R,2,2) - 1) / 2;
    *s = sqrt( 1 - (*c)*(*c) );
    *theta = atan2(*s, *c);
}

AA_API void
aa_tf_rotmat_lnv( const double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] )
{
    double isinc, c, s, theta;
    aa_tf_rotmat_angle( R, &c, &s, &theta );
    double theta2 = theta*theta;
    if( theta2*theta2 < DBL_EPSILON ) {
        isinc = aa_tf_invsinc_series2( theta2 );
    } else {
        isinc = theta/s;
    }
    aa_tf_unskewsym_scal( isinc/2, R, v );
}

AA_API void
aa_tf_rotmat_vel2diff( const double R[AA_RESTRICT 9],
                       const double w[AA_RESTRICT 3],
                       double dR[AA_RESTRICT 9] )
{
    double K[9];
    aa_tf_skew_sym( w, K );
    aa_tf_9mul( K, R, dR );
}

AA_API void
aa_tf_rotmat_diff2vel( const double R[AA_RESTRICT 9],
                       const double dR[AA_RESTRICT 9],
                       double w[AA_RESTRICT 3] )
{
    double K[9], Rt[9];
    aa_la_transpose2( 3, 3, R, Rt );
    aa_tf_9mul( dR, Rt, K );
    aa_tf_unskewsym(K, w);
}

AA_API void
aa_tf_rotmat_svel( const double R0[9], const double w[3], double dt, double R1[9] )
{
    double delta[3], e[9];
    size_t i;
    for( i = 0; i < 3; i ++ ) delta[i] = w[i]*dt;
    aa_tf_rotmat_expv(delta,e);
    aa_tf_9mul(e,R0,R1);
}

AA_API void
aa_tf_tfmat_svel( const double T0[12], const double dx[6], double dt, double T1[12] )
{
    double delta[6];
    size_t i;

    for( i = 0; i < 6; i ++ )
        delta[i] = dx[i] * dt;

    {
        double cross[3];
        aa_tf_cross( delta + AA_TF_DX_W, T0+AA_TF_TFMAT_V, cross );
        for( i = 0; i < 3; i ++ )
            (delta+AA_TF_DX_V)[i] -= cross[i];
    }

    {
        double E[12];
        aa_tf_tfmat_expv( delta, E );
        aa_tf_12chain( E, T0, T1 );
    }
}

/* Transform */

AA_API void
aa_tf_tfmat_diff2vel( const double T[12], const double dT[12], double dx[6] )
{
    aa_tf_rotmat_diff2vel( T+AA_TF_TFMAT_R, dT+AA_TF_TFMAT_R, dx+AA_TF_DX_W );
    AA_MEM_CPY( dx+AA_TF_DX_V,  dT+AA_TF_TFMAT_V, 3 );
}

AA_API void
aa_tf_tfmat_vel2diff( const double T[12], const double dx[6], double dT[12] )
{
    aa_tf_rotmat_vel2diff( T+AA_TF_TFMAT_R, dx+AA_TF_DX_W, dT+AA_TF_TFMAT_R );
    AA_MEM_CPY( dT+AA_TF_TFMAT_V, dx+AA_TF_DX_V, 3 );
}


AA_API void aa_tf_rotmat_rot( const double R[AA_RESTRICT 9],
                              const double p0[AA_RESTRICT 3],
                              double p1[AA_RESTRICT 3] )
{
    p1[0] =  R[0]*p0[0] + R[3]*p0[1] + R[6]*p0[2];
    p1[1] =  R[1]*p0[0] + R[4]*p0[1] + R[7]*p0[2];
    p1[2] =  R[2]*p0[0] + R[5]*p0[1] + R[8]*p0[2];
}

AA_API void aa_tf_tfmat_tf( const double T[AA_RESTRICT 12],
                            const double p0[AA_RESTRICT 3],
                            double p1[AA_RESTRICT 3] )
{
    aa_tf_tfmat2_tf( T,T+9, p0, p1 );
}

AA_API void aa_tf_12( const double T[AA_RESTRICT 12],
                      const double p0[AA_RESTRICT 3],
                      double p1[AA_RESTRICT 3] )
{
    aa_tf_tfmat2_tf( T,T+9, p0, p1 );
}

AA_API void aa_tf_tfmat2_tf( const double R[AA_RESTRICT 9],
                             const double v[AA_RESTRICT 3],
                             const double p0[AA_RESTRICT 3],
                             double p1[AA_RESTRICT 4] )
{

    p1[0] =  R[0]*p0[0] + R[3]*p0[1] + R[6]*p0[2] + v[0];
    p1[1] =  R[1]*p0[0] + R[4]*p0[1] + R[7]*p0[2] + v[1];
    p1[2] =  R[2]*p0[0] + R[5]*p0[1] + R[8]*p0[2] + v[2];
}


/* Multiply */
AA_API void aa_tf_rotmat_mul( const double R1[AA_RESTRICT 9],
                              const double R2[AA_RESTRICT 9],
                              double R3[AA_RESTRICT 9] )
{
    aa_tf_rotmat_rot( R1, R2, R3 );
    aa_tf_rotmat_rot( R1, R2+3, R3+3 );
    aa_tf_rotmat_rot( R1, R2+6, R3+6 );
}

AA_API void aa_tf_tfmat_mul( const double T0[AA_RESTRICT 12],
                             const double T1[AA_RESTRICT 12],
                             double T[AA_RESTRICT 12] )
{
    aa_tf_tfmat2_mul( T0, T0+9,
                      T1, T1+9,
                      T,  T+9 );
}

AA_API void
aa_tf_12chain( const double T0[AA_RESTRICT 12],
               const double T1[AA_RESTRICT 12],
               double T2[AA_RESTRICT 12] )
{
    aa_tf_tfmat_mul( T0, T1, T2 );
}

AA_API void aa_tf_tfmat2_mul( const double R0[AA_RESTRICT 9],
                              const double v0[AA_RESTRICT 3],
                              const double R1[AA_RESTRICT 9],
                              const double v1[AA_RESTRICT 3],
                              double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] )
{
    aa_tf_rotmat_mul(R0,R1,R);
    aa_tf_tfmat2_tf( R0, v0, v1, v );
}


/* Inverting Multiply */

/* AA_API void aa_tf_rotmat_imul( const double R0[AA_RESTRICT 9], */
/*                                const double R1[AA_RESTRICT 9], */
/*                                double R[AA_RESTRICT 9] ); */

/* AA_API void aa_tf_rotmat_muli( const double R0[AA_RESTRICT 9], */
/*                                const double R1[AA_RESTRICT 9], */
/*                                double R[AA_RESTRICT 9] ); */

/* AA_API void aa_tf_tfmat_imul( const double T0[AA_RESTRICT 12], */
/*                               const double T1[AA_RESTRICT 12], */
/*                               double T[AA_RESTRICT 12] ); */

/* AA_API void aa_tf_tfmat_muli( const double T0[AA_RESTRICT 12], */
/*                               const double T1[AA_RESTRICT 12], */
/*                               double T[AA_RESTRICT 12] ); */

/* AA_API void aa_tf_tfmat2_imul( const double R0[AA_RESTRICT 9], */
/*                                const double v0[AA_RESTRICT 3], */
/*                                const double R1[AA_RESTRICT 9], */
/*                                const double v1[AA_RESTRICT 3], */
/*                                double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] ); */

/* AA_API void aa_tf_tfmat2_muli( const double R0[AA_RESTRICT 9], */
/*                                const double v0[AA_RESTRICT 3], */
/*                                const double R1[AA_RESTRICT 9], */
/*                                const double v1[AA_RESTRICT 3], */
/*                                double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] ); */

AA_API void
aa_tf_tfmat_expv( const double v[AA_RESTRICT 6],
                  double T[AA_RESTRICT 12] )
{
    double theta2 = dot3(v+3, v+3);
    double theta = sqrt(theta2);
    double sc, cc, ssc;
    if( theta2*theta2 < DBL_EPSILON ) {
        sc = aa_tf_sinc_series2(theta2);
        cc = theta * aa_horner3( theta2, 1./2, -1./24, 1./720 );
        ssc = aa_horner3( theta2, 1./6, -1./120, 1./5040 );
    } else {
        double s = sin(theta), c = cos(theta);
        sc = s/theta;
        cc = (1-c)/theta2;
        ssc = (1-sc)/theta2;
    }

    double K[9];
    aa_tf_skewsym_scal2( sc, cc, v+3, T );
    aa_tf_skewsym_scal2( cc, ssc, v+3, K );
    aa_tf_9(K,v, T+9);
}

AA_API void
aa_tf_tfmat_lnv( const double T[AA_RESTRICT 12],
                 double v[AA_RESTRICT 6] )
{
    double c,s,theta, a,b;
    aa_tf_rotmat_angle( T, &c, &s, &theta );
    double theta2 = theta*theta;
    if( theta2 < DBL_EPSILON ) {
        a = aa_tf_invsinc_series2(theta2);
        b = aa_horner3( theta2, 1./12, 1./720, 1./30240 );
    } else {
        a = theta/s;
        b = (2*s - theta*(1+c)) / (2*theta2*s);
    }
    double K[9];
    aa_tf_unskewsym_scal( a/2, T, v+3 );
    aa_tf_skewsym_scal2( -.5, b, v+3, K );
    aa_tf_9( K, T+9, v );
}
