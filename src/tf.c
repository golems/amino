/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
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

void aa_tf_93inv_( const double R[restrict 9],
                   const double v[restrict 3],
                   double Ri[restrict 9],
                   double vi[restrict 3] );

void aa_tf_12inv( const double T[restrict 12], double Ti[restrict 12] ) {
    aa_tf_93inv( T, T+9, Ti, Ti+9 );
}

void aa_tf_93inv( const double R[restrict 9], const double v[restrict 3],
                  double Ri[restrict 9], double vi[restrict 3] ) {
    aa_la_transpose2( 3, 3, R, Ri );
    aa_tf_9rot( Ri, (double[]){-v[0],-v[1],-v[2]}, vi );
}


void aa_tf_93( const double R[restrict 9], const double v[restrict 3],
               const double p0[restrict 3], double pp[restrict 3] ) {
    pp[0] = v[0] +
        p0[0] * AA_MATREF(R,3,0,0) +
        p0[1] * AA_MATREF(R,3,0,1) +
        p0[2] * AA_MATREF(R,3,0,2);
    pp[1] = v[1] +
        p0[0] * AA_MATREF(R,3,1,0) +
        p0[1] * AA_MATREF(R,3,1,1) +
        p0[2] * AA_MATREF(R,3,1,2);
    pp[2] = v[2] +
        p0[0] * AA_MATREF(R,3,2,0) +
        p0[1] * AA_MATREF(R,3,2,1) +
        p0[2] * AA_MATREF(R,3,2,2);
}

void aa_tf_12( const double T[restrict 12], const double p0[restrict 3],
               double p1[restrict 3] ) {
    aa_tf_93( T, T+9, p0, p1 );
}


void aa_tf_9( const double R[restrict 9], const double p0[restrict 3],
              double p1[restrict 3] ) {
    aa_tf_93( R, AA_FAR(0,0,0), p0, p1 );
}

void aa_tf_93chain_( const double R0[restrict 9], const double v0[restrict 3],
                     const double R1[restrict 9], const double v1[restrict 3],
                     double R[restrict 9], double v[restrict 3] );

void aa_tf_93chain( const double R0[restrict 9], const double v0[restrict 3],
                    const double R1[restrict 9], const double v1[restrict 3],
                    double R[restrict 9], double v[restrict 3] ) {
    aa_tf_93chain_( R0, v0, R1, v1, R, v );
}

void aa_tf_12chain( const double T1[restrict 12], const double T2[restrict 12],
                    double T[restrict 12] ) {
    aa_tf_93chain( T1, T1+9, T2, T2+9, T, T+9 );
}


void aa_tf_93rel( const double R1[restrict 9], const double v1[restrict 3],
                  const double R2[restrict 9], const double v2[restrict 3],
                  double Rrel[restrict 9], double vrel[restrict 3] ) {
    double R1i[9], v1i[3];
    aa_tf_93inv( R1, v1, R1i, v1i );
    aa_tf_93chain( R1i, v1i, R2, v2, Rrel, vrel );
}

void aa_tf_12rel( const double T1[restrict 12], const double T2[restrict 12],
                  double Trel[restrict 12] ) {
    aa_tf_93rel( T1, T1+9, T2, T2+9, Trel, Trel+9 );
}


// from maxima
void aa_tf_9mul_( const double R0[restrict 9], const double R1[restrict 9],
                  double R[restrict 9] );
void aa_tf_9mul( const double R0[restrict 9], const double R1[restrict 9],
                 double R[restrict 9] ) {
    aa_tf_9mul_(R0, R1, R);
}

void aa_tf_9rot( const double R[restrict 9], const double p0[restrict 3],
                 double pp[restrict 3] ) {
    pp[0] =
        p0[0] * AA_MATREF(R,3,0,0) +
        p0[1] * AA_MATREF(R,3,0,1) +
        p0[2] * AA_MATREF(R,3,0,2);
    pp[1] =
        p0[0] * AA_MATREF(R,3,1,0) +
        p0[1] * AA_MATREF(R,3,1,1) +
        p0[2] * AA_MATREF(R,3,1,2);
    pp[2] =
        p0[0] * AA_MATREF(R,3,2,0) +
        p0[1] * AA_MATREF(R,3,2,1) +
        p0[2] * AA_MATREF(R,3,2,2);
}

void aa_tf_qnormalize( double q[restrict 4] ) {
    aa_la_normalize( 4, q );
}

void aa_tf_qconj( const double q[restrict 4], double r[restrict 4] ) {
    r[0] = -q[0];
    r[1] = -q[1];
    r[2] = -q[2];
    r[3] =  q[3];
}

void aa_tf_qinv( const double q[restrict 4], double r[restrict 4] ) {
    aa_tf_qconj(q,r);
    aa_la_scal( 4, 1.0/aa_la_dot(4,r,r), r );
}

void aa_tf_qmul( const double a[restrict 4], const double b[restrict 4],
                 double c[restrict 4] ) {
    c[3] = a[3]*b[3] - aa_la_dot(3, a, b);
    aa_la_cross( a, b, c );
    for( size_t i = 0; i < 3; i ++ )
        c[i] += a[3]*b[i] + b[3]*a[i];
}

void aa_tf_qrel( const double q1[restrict 4], const double q2[restrict 4],
                 double q_rel[restrict 4]) {
    double inv[4];
    aa_tf_qinv(q2,inv);
    aa_tf_qmul(q1,inv,q_rel);
}

void aa_tf_qadd( const double a[restrict 4], const double b[restrict 4],
                 double c[restrict 4] );

void aa_tf_qslerp( double t, const double a[restrict 4],
                   const double b[restrict 4],
                   double c[restrict 4] );

void aa_tf_quat2axang( const double q[restrict 4], double axang[restrict 4] ) {
    double a = aa_la_norm(4,q);
    double w = q[3]/a;
    axang[3] = aa_ang_norm_pi(2 * acos(w));
    /* aa_la_smul( 3,  */
    /*             ( aa_feq( axang[3], 0, AA_TF_EPSILON ) /\* ident check *\/ ? */
    /*               0 : 1.0 / (a*sqrt(1 - w*w)) ),  */
    /*             q, axang ); */

    if(  aa_feq( axang[3], 0, AA_TF_EPSILON ) ) {
        aa_fzero( axang, 3 );
    } else {
        aa_la_smul( 3, 1.0 / (a*sqrt(1 - w*w)), q, axang );
    }
}

void aa_tf_axang_make( double x, double y, double z, double theta,
                       double axang[restrict 4] ) {
    double n = sqrt(x*x + y*y + z*z);
    // FIXME: zeros
    axang[0] = x/n;
    axang[1] = y/n;
    axang[2] = z/n;
    axang[3] = aa_ang_norm_pi(theta);
}

void aa_tf_axang_permute2( const double aa[restrict 4],
                           double aa_plus[restrict 4],
                           double aa_minus[restrict 4] ) {
    aa_fcpy( aa_plus, aa, 3 );
    aa_plus[3] = aa[3] + 2*M_PI;
    aa_fcpy( aa_minus, aa, 3 );
    aa_minus[3] = aa[3] - 2*M_PI;
}


void aa_tf_axang_permute( const double ra[restrict 4], int k,
                          double ra_p[restrict 4] ) {
    aa_fcpy( ra_p, ra, 3 );
    ra_p[3] = ra[3] + k*2*M_PI;
}

void aa_tf_rotvec_permute( const double rv[restrict 3], int k,
                           double rv_p[restrict 3] ) {
    double ra[4], ra_p[4];
    aa_tf_rotvec2axang(rv,ra);
    aa_tf_axang_permute(ra, k, ra_p);
    aa_tf_axang2rotvec(ra_p, rv_p);
}

void aa_tf_rotvec_near( const double rv[restrict 3],
                        const double rv_near[restrict 3],
                        double rv_p[restrict 3] ) {
    // FIXME: probably a more efficient solution
    double ssd[7];
    double arv[sizeof(ssd)/sizeof(double)][3];

    aa_fcpy(arv[0],rv,3);
    aa_tf_rotvec_permute( rv, 1, arv[1] );
    aa_tf_rotvec_permute( rv, 2, arv[2] );
    aa_tf_rotvec_permute( rv, 3, arv[3] );
    aa_tf_rotvec_permute( rv, -1, arv[4] );
    aa_tf_rotvec_permute( rv, -2, arv[5] );
    aa_tf_rotvec_permute( rv, -3, arv[6] );

    for( size_t i = 0; i < sizeof(ssd)/sizeof(double); i++ ) {
        ssd[i] = aa_la_ssd( 3, rv_near, arv[i] );
    }
    aa_fcpy( rv_p, arv[ aa_fminloc(7, ssd) ], 3 );
}

void aa_tf_axang2rotvec( const double axang[restrict 4],
                         double rotvec[restrict 3] ) {
    aa_la_smul( 3, axang[3], axang, rotvec );
}

void aa_tf_rotvec2axang( const double rotvec[restrict 3],
                         double axang[restrict 4] ) {
    axang[3] = aa_la_norm(3,rotvec);
    if( aa_feq( axang[3], 0, AA_TF_EPSILON ) ) {
        aa_fzero( axang, 3 );
    } else {
        aa_la_smul( 3, 1.0 / axang[3] , rotvec, axang );
    }
    /* aa_la_smul( 3,  */
    /*             ( aa_feq( axang[3], 0, AA_TF_EPSILON ) /\* ident check *\/ ?  */
    /*               0 : */
    /*               1.0 / axang[3] ), */
    /*             rotvec, axang ); */
}

void aa_tf_axang2quat( const double axang[restrict 4],
                       double q[restrict 4] ) {
    double s,c;
    sincos( axang[3]/2, &s, &c );
    q[3] = c;
    aa_la_smul( 3, s, axang, q );
}

void aa_tf_quat2rotmat( const double q[restrict 4],
                        double R[restrict 9] ) {
    double w,x,y,z;
    double d,s,xs,ys,zs;
    double wx,wy,wz;
    double xx,xy,xz;
    double yy,yz,zz;

    w = q[3];
    x = q[0];
    y = q[1];
    z = q[2];

    d = aa_la_dot(4, q, q);
    s = 2/d;
    xs = x*s;
    ys = y*s;
    zs = z*s;
    wx = w*xs;
    wy = w*ys;
    wz = w*zs;
    xx = x*xs;
    xy = x*ys;
    xz = x*zs;
    yy = y*ys;
    yz = y*zs;
    zz = z*zs;

    AA_MATREF(R,3,0,0) = 1 - yy - zz;
    AA_MATREF(R,3,1,0) = xy + wz;
    AA_MATREF(R,3,2,0) = xz - wy;
    AA_MATREF(R,3,0,1) = xy - wz;
    AA_MATREF(R,3,1,1) = 1 - xx - zz;
    AA_MATREF(R,3,2,1) = yz + wx;
    AA_MATREF(R,3,0,2) = xz + wy;
    AA_MATREF(R,3,1,2) = yz - wx;
    AA_MATREF(R,3,2,2) = 1 - xx - yy;
}

void aa_tf_rotmat2quat( const double R[restrict 9],
                        double q[restrict 4] ) {

    double x =  AA_MATREF(R,3,0,0);
    double y =  AA_MATREF(R,3,1,1);
    double z =  AA_MATREF(R,3,2,2);
    /* double trace = x+y+z; */
    /* if( trace > 0 ) { */
    /*     double r = sqrt(trace + 1); */
    /*     q[3] = r / 2; */
    /*     double s = 0.5 / r; */
    /*     q[0] = (AA_MATREF(R,3,2,1) - AA_MATREF(R,3,1,2))*s; */
    /*     q[1] = (AA_MATREF(R,3,0,2) - AA_MATREF(R,3,2,0))*s; */
    /*     q[2] = (AA_MATREF(R,3,1,0) - AA_MATREF(R,3,0,1))*s; */
    /* } else  */
    {
        size_t u = (x < y) ?
            (y < z) ? 2 : 1 :
            (x < z) ? 2 : 0;
        size_t v = (u + 1) % 3;
        size_t w = (u + 2) % 3;
        double r = sqrt( 1 + AA_MATREF(R,3,u,u) - AA_MATREF(R,3,v,v) - AA_MATREF(R,3,w,w) );
        q[u] = r / 2;
        double s = 0.5 / r;
        q[3] = (AA_MATREF(R,3,w,v) - AA_MATREF(R,3,v,w)) * s;
        q[v] = (AA_MATREF(R,3,u,v) + AA_MATREF(R,3,v,u)) * s;
        q[w] = (AA_MATREF(R,3,w,u) + AA_MATREF(R,3,u,w)) * s;
    }

    aa_tf_qnormalize(q);
}

void aa_tf_rotvec2quat( const double rotvec[restrict 3],
                        double q[restrict 4] ) {
    double aa[4];
    aa_tf_rotvec2axang(rotvec, aa);
    aa_tf_axang2quat(aa,q);
}

void aa_tf_quat2rotvec( const double q[restrict 4],
                        double rotvec[restrict 3] ) {
    double aa[4];
    aa_tf_quat2axang(q,aa);
    aa_tf_axang2rotvec(aa,rotvec);
}


void aa_tf_quat2rotvec_near( const double q[restrict 4],
                             const double rv_near[restrict 3],
                             double rv[restrict 3] ) {
    double aa[4], rv0[3];
    aa_tf_quat2axang(q,aa);
    aa_tf_axang2rotvec(aa,rv0);
    aa_tf_rotvec_near( rv0, rv_near, rv );
}

/* void aa_tf_tfv2tfq( const double vrv[6],  */
/*                            double x[3], double quat[4] ) { */
/*     aa_fcpy(x,vrv,3); */
/*     aa_tf_rotvec2quat( vrv+3, quat ); */
/* } */

/* void aa_tf_tfq2tfv( const double x[3], const double quat[4],  */
/*                            double vrv[6] ) { */
/*     aa_fcpy(vrv,x,3); */
/*     aa_tf_quat2rotvec( quat, vrv+3 ); */
/* } */


int aa_tf_isrotmat( const double R[restrict 9] ) {
    double Rt[9], Ri[9], d;
    aa_la_transpose2( 3, 3, R, Rt );
    aa_la_inverse3x3( R, Ri );
    d = aa_la_det3x3( R );
    return aa_veq( 9, Rt, Ri, .0001 ) && aa_feq( d, 1, .0001 );
}


void aa_tf_rotmat2axang( const double R[restrict 9],
                         double ra[restrict 4] ) {
    double rv[3];
    aa_tf_rotmat2rotvec(R,rv);
    aa_tf_rotvec2axang(rv,ra);
}

void aa_tf_rotmat2rotvec( const double R[restrict 9],
                          double rv[restrict 3] ) {
    rv[0] = 0.5 * (AA_MATREF(R,3,2,1) - AA_MATREF(R,3,1,2));
    rv[1] = 0.5 * (AA_MATREF(R,3,0,2) - AA_MATREF(R,3,2,0));
    rv[2] = 0.5 * (AA_MATREF(R,3,1,0) - AA_MATREF(R,3,0,1));

    double s = aa_la_norm( 3, rv );
    double c = (aa_la_trace(3,R) - 1) / 2 ;

    aa_la_scal( 3, ( (s > AA_TF_EPSILON) ? atan2(s,c)/s : 1 ), rv );
}

void aa_tf_axang2rotmat( const double ra[restrict 4],
                         double R[restrict 9] ) {
    double quat[4];
    aa_tf_axang2quat(ra,quat);
    aa_tf_quat2rotmat(quat,R);
}

/* void aa_tf_rotvec2rotmat( const double rv[3], double R[9] ) { */

/* } */

// Craig 3rd Ed., p44
void aa_tf_eulerzyx2rotmat( const double e[restrict 3],
                            double R[restrict 9] ) {
    double ca, cb, cg, sa, sb, sg;
    sincos( e[0], &sa, &ca );
    sincos( e[1], &sb, &cb );
    sincos( e[2], &sg, &cg );

    R[0] = ca*cb;
    R[1] = sa*cb;
    R[2] = -sb;

    R[3] = ca*cb*sg - sa*cg;
    R[4] = sa*sb*sg + ca*cg;
    R[5] = cb*sg;

    R[6] = ca*sb*cg + sa*sg;
    R[7] = sa*sb*cg - ca*sg;
    R[8] = cb*cg;
}

// Craig 3rd Ed., p43 (same as fixed XYZ)
void aa_tf_rotmat2eulerzyx( const double R[restrict 9],
                            double e[restrict 3] ) {
    double a, b, g, cb;
    b = atan2( -AA_MATREF(R,3,2,0),
               sqrt( aa_fsq( AA_MATREF(R,3,0,0) ) +
                     aa_fsq( AA_MATREF(R,3,1,0) ) ) );
    cb = cos(b);
    a = atan2( AA_MATREF( R,3,1,0) / cb,
               AA_MATREF( R,3,0,0) / cb );
    g = atan2( AA_MATREF( R,3,2,1) / cb,
               AA_MATREF( R,3,2,2) / cb );
    e[0] = a;
    e[1] = b;
    e[2] = g;
}

void aa_tf_xangle2rotmat( double theta_x, double R[restrict 9] ) {
    double s,c;
    sincos( theta_x, &s, &c);
    R[0] = 1;    R[3] = 0;    R[6] = 0;
    R[1] = 0;    R[4] = c;    R[7] = -s;
    R[2] = 0;    R[5] = s;    R[8] = c;
}
void aa_tf_yangle2rotmat( double theta_y, double R[restrict 9] ) {
    double s,c;
    sincos( theta_y, &s, &c);
    R[0] = c;    R[3] = 0;    R[6] = s;
    R[1] = 0;    R[4] = 1;    R[7] = 0;
    R[2] = -s;   R[5] = 0;    R[8] = c;

}
void aa_tf_zangle2rotmat( double theta_z, double R[restrict 9] ) {
    double s,c;
    sincos( theta_z, &s, &c);
    R[0] = c;    R[3] = -s;   R[6] = 0;
    R[1] = s;    R[4] = c;    R[7] = 0;
    R[2] = 0;    R[5] = 0;    R[8] = 1;
}
