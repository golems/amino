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
    double vv[3] = {-v[0],-v[1],-v[2]};
    aa_tf_9rot( Ri, vv, vi );
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


void aa_tf_9rot( const double R[restrict 9], const double p0[restrict 3],
                 double pp[restrict 3] ) {
    aa_tf_9(R,p0,pp);
}

void aa_tf_9rel( const double R1[restrict 9], const double R2[restrict 9],
                 double Ri[restrict 9] ) {
    double R2i[9];
    aa_la_transpose2( 3, 3, R2, R2i );
    aa_tf_9mul( R1, R2i, Ri );
}

/* void aa_tf_qinv( const double q[restrict 4], double r[restrict 4] ) { */
/*     aa_tf_qconj(q,r); */
/*     aa_la_scal( 4, 1.0/aa_la_dot(4,r,r), r ); */
/* } */

void aa_tf_qrel( const double q1[restrict 4], const double q2[restrict 4],
                 double q_rel[restrict 4]) {
    double inv[4];
    aa_tf_qinv(q2,inv);
    aa_tf_qmul(q1,inv,q_rel);
}

void aa_tf_qadd( const double a[restrict 4], const double b[restrict 4],
                 double c[restrict 4] ) {
    for( size_t i = 0; i < 4; i ++ ) c[i] = a[i] + b[i];
}

void aa_tf_qsub( const double a[restrict 4], const double b[restrict 4],
                 double c[restrict 4] ) {
    for( size_t i = 0; i < 4; i ++ ) c[i] = a[i] - b[i];
}

/* void aa_tf_quat2axang( const double q[restrict 4], double axang[restrict 4] ) { */
/*     double a = aa_la_norm(4,q); */
/*     double w = q[3]/a; */
/*     axang[3] = aa_ang_norm_pi(2 * acos(w)); */
/*     /\* aa_la_smul( 3,  *\/ */
/*     /\*             ( aa_feq( axang[3], 0, AA_TF_EPSILON ) /\\* ident check *\\/ ? *\/ */
/*     /\*               0 : 1.0 / (a*sqrt(1 - w*w)) ),  *\/ */
/*     /\*             q, axang ); *\/ */

/*     if(  aa_feq( axang[3], 0, AA_TF_EPSILON ) ) { */
/*         aa_fzero( axang, 3 ); */
/*     } else { */
/*         aa_la_smul( 3, 1.0 / (a*sqrt(1 - w*w)), q, axang ); */
/*     } */
/* } */

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
    AA_MEM_CPY( aa_plus, aa, 3 );
    aa_plus[3] = aa[3] + 2*M_PI;
    AA_MEM_CPY( aa_minus, aa, 3 );
    aa_minus[3] = aa[3] - 2*M_PI;
}


void aa_tf_axang_permute( const double ra[restrict 4], int k,
                          double ra_p[restrict 4] ) {
    AA_MEM_CPY( ra_p, ra, 3 );
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

    AA_MEM_CPY(arv[0],rv,3);
    aa_tf_rotvec_permute( rv, 1, arv[1] );
    aa_tf_rotvec_permute( rv, 2, arv[2] );
    aa_tf_rotvec_permute( rv, 3, arv[3] );
    aa_tf_rotvec_permute( rv, -1, arv[4] );
    aa_tf_rotvec_permute( rv, -2, arv[5] );
    aa_tf_rotvec_permute( rv, -3, arv[6] );

    for( size_t i = 0; i < sizeof(ssd)/sizeof(double); i++ ) {
        ssd[i] = aa_la_ssd( 3, rv_near, arv[i] );
    }
    AA_MEM_CPY( rv_p, arv[ aa_fminloc(7, ssd) ], 3 );
}

void aa_tf_axang2rotvec( const double axang[restrict 4],
                         double rotvec[restrict 3] ) {
    aa_la_smul( 3, axang[3], axang, rotvec );
}

void aa_tf_rotvec2axang( const double rotvec[restrict 3],
                         double axang[restrict 4] ) {
    axang[3] = aa_la_norm(3,rotvec);
    if( aa_feq( axang[3], 0, AA_TF_EPSILON ) ) {
        AA_MEM_ZERO( axang, 3 );
    } else {
        aa_la_smul( 3, 1.0 / axang[3] , rotvec, axang );
    }
    /* aa_la_smul( 3,  */
    /*             ( aa_feq( axang[3], 0, AA_TF_EPSILON ) /\* ident check *\/ ?  */
    /*               0 : */
    /*               1.0 / axang[3] ), */
    /*             rotvec, axang ); */
}

/* void aa_tf_axang2quat( const double axang[restrict 4], */
/*                        double q[restrict 4] ) { */
/*     double s,c; */
/*     sincos( axang[3]/2, &s, &c ); */
/*     q[3] = c; */
/*     aa_la_smul( 3, s, axang, q ); */
/*     aa_tf_qnormalize(q); */
/* } */

void aa_tf_rotmat2quat( const double R[restrict 9],
                        double q[restrict 4] ) {

    // Ken Shoemake. "Quaternion Calculus and Fast Animation".
    // SIGGRAPH course notes. 1987.
    //
    // as implemeted in Bullet and Eigen

    double x =  AA_MATREF(R,3,0,0);
    double y =  AA_MATREF(R,3,1,1);
    double z =  AA_MATREF(R,3,2,2);
    double trace = x+y+z;
    if( trace > 0 ) {
        double r = sqrt(trace + 1);
        q[3] = r / 2;
        double s = 0.5 / r;
        q[0] = (AA_MATREF(R,3,2,1) - AA_MATREF(R,3,1,2))*s;
        q[1] = (AA_MATREF(R,3,0,2) - AA_MATREF(R,3,2,0))*s;
        q[2] = (AA_MATREF(R,3,1,0) - AA_MATREF(R,3,0,1))*s;
    } else {
        size_t u = (x < y) ?
            ( (y < z) ? 2 : 1 ) :
            ( (x < z) ? 2 : 0 );
        size_t v = (u + 1) % 3;
        size_t w = (u + 2) % 3;
        double r = sqrt( 1 + AA_MATREF(R,3,u,u) -
                         AA_MATREF(R,3,v,v) -
                         AA_MATREF(R,3,w,w) );
        q[u] = r / 2;
        double s = 0.5 / r;
        q[3] = (AA_MATREF(R,3,w,v) - AA_MATREF(R,3,v,w)) * s;
        q[v] = (AA_MATREF(R,3,u,v) + AA_MATREF(R,3,v,u)) * s;
        q[w] = (AA_MATREF(R,3,w,u) + AA_MATREF(R,3,u,w)) * s;
    }

    aa_tf_qnormalize(q);
}

/* void aa_tf_rotvec2quat( const double rotvec[restrict 3], */
/*                         double q[restrict 4] ) { */
/*     double aa[4]; */
/*     aa_tf_rotvec2axang(rotvec, aa); */
/*     aa_tf_axang2quat(aa,q); */
/* } */

/* void aa_tf_quat2rotvec( const double q[restrict 4], */
/*                         double rotvec[restrict 3] ) { */
/*     double aa[4]; */
/*     aa_tf_quat2axang(q,aa); */
/*     aa_tf_axang2rotvec(aa,rotvec); */
/* } */


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
    //aa_la_inverse3x3( R, Ri );
    AA_MEM_CPY(Ri, R, 9); aa_la_inv( 3, Ri );
    d = aa_la_det3x3( R );
    return aa_veq( 9, Rt, Ri, .0001 ) && aa_feq( d, 1, .0001 );
}


void aa_tf_rotmat2axang( const double R[restrict 9],
                         double ra[restrict 4] ) {
    double q[4];
    aa_tf_rotmat2quat( R, q );
    aa_tf_quat2axang(q,ra);
}

void aa_tf_rotmat2rotvec( const double R[restrict 9],
                          double rv[restrict 3] ) {

    /* Numerically Unstable */
    /* rv[0] = 0.5 * (AA_MATREF(R,3,2,1) - AA_MATREF(R,3,1,2)); */
    /* rv[1] = 0.5 * (AA_MATREF(R,3,0,2) - AA_MATREF(R,3,2,0)); */
    /* rv[2] = 0.5 * (AA_MATREF(R,3,1,0) - AA_MATREF(R,3,0,1)); */

    /* double s = aa_la_norm( 3, rv ); */
    /* double c = (aa_la_trace(3,R) - 1) / 2 ; */

    /* aa_la_scal( 3, ( (s > AA_TF_EPSILON) ? atan2(s,c)/s : 1 ), rv ); */

    double q[4];
    aa_tf_rotmat2quat( R, q );
    aa_tf_quat2rotvec(q,rv);
}

void aa_tf_v9mul( double R[AA_RESTRICT 9], const double R1[AA_RESTRICT 9], const double R2[AA_RESTRICT 9], ... ) {
    va_list argp;

    double tmp1[9], tmp2[9];
    double *Ri, *Rp = tmp1, *Rc = tmp2;

    aa_tf_9mul( R1, R2, Rp );

    va_start(argp, R2);
    while( NULL != (Ri = va_arg(argp, double *)) ) {
        aa_tf_9mul( Rp, Ri, Rc );
        double *r = Rp;
        Rp = Rc;
        Rc = r;
    }

    AA_MEM_CPY( R, Rp, 9 );
    va_end(argp);
}


void aa_tf_v12chain( double T[AA_RESTRICT 12], const double T1[AA_RESTRICT 12], const double T2[AA_RESTRICT 12], ... ) {
    va_list argp;

    double tmp1[12], tmp2[12];
    double *Ti, *Tp = tmp1, *Tc = tmp2;

    aa_tf_12chain( T1, T2, Tp );

    va_start(argp, T2);
    while( NULL != (Ti = va_arg(argp, double *)) ) {
        aa_tf_12chain( Tp, Ti, Tc );
        double *t = Tp;
        Tp = Tc;
        Tc = t;
    }

    AA_MEM_CPY( T, Tp, 12 );
    va_end(argp);
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
/* void aa_tf_eulerzyx2rotmat( const double e[restrict 3], */
/*                             double R[restrict 9] ) { */
/*     double ca, cb, cg, sa, sb, sg; */
/*     sincos( e[0], &sa, &ca ); */
/*     sincos( e[1], &sb, &cb ); */
/*     sincos( e[2], &sg, &cg ); */

/*     R[0] = ca*cb; */
/*     R[1] = sa*cb; */
/*     R[2] = -sb; */

/*     R[3] = ca*sb*sg - sa*cg; */
/*     R[4] = sa*sb*sg + ca*cg; */
/*     R[5] = cb*sg; */

/*     R[6] = ca*sb*cg + sa*sg; */
/*     R[7] = sa*sb*cg - ca*sg; */
/*     R[8] = cb*cg; */
/* } */


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


/* AA_API void aa_tf_eulerzyx2quat( const double e[AA_RESTRICT 3], */
/*                                  double q[AA_RESTRICT 4] ) { */
/*     double ca, cb, cg, sa, sb, sg; */
/*     sincos( e[0]/2, &sa, &ca ); */
/*     sincos( e[1]/2, &sb, &cb ); */
/*     sincos( e[2]/2, &sg, &cg ); */

/*     q[0] = ca*cb*sg - sa*sb*cg; */
/*     q[1] = sa*cb*sg + ca*sb*cg; */
/*     q[2] = sa*cb*cg - ca*sb*sg; */
/*     q[3] = sa*sb*sg + ca*cb*cg; */

/*     aa_tf_qnormalize(q); */
/* } */

/* void aa_tf_xangle2rotmat( double theta_x, double R[restrict 9] ) { */
/*     double s,c; */
/*     sincos( theta_x, &s, &c); */
/*     R[0] = 1;    R[3] = 0;    R[6] = 0; */
/*     R[1] = 0;    R[4] = c;    R[7] = -s; */
/*     R[2] = 0;    R[5] = s;    R[8] = c; */
/* } */
/* void aa_tf_yangle2rotmat( double theta_y, double R[restrict 9] ) { */
/*     double s,c; */
/*     sincos( theta_y, &s, &c); */
/*     R[0] = c;    R[3] = 0;    R[6] = s; */
/*     R[1] = 0;    R[4] = 1;    R[7] = 0; */
/*     R[2] = -s;   R[5] = 0;    R[8] = c; */

/* } */
/* void aa_tf_zangle2rotmat( double theta_z, double R[restrict 9] ) { */
/*     double s,c; */
/*     sincos( theta_z, &s, &c); */
/*     R[0] = c;    R[3] = -s;   R[6] = 0; */
/*     R[1] = s;    R[4] = c;    R[7] = 0; */
/*     R[2] = 0;    R[5] = 0;    R[8] = 1; */
/* } */

AA_API void aa_tf_quat_davenport
( size_t n, const double *w, const double *qq, size_t ldqq, double *p )
{
    double M[16];
    aa_tf_quat_davenport_matrix( n,w,qq,ldqq,M);
    double wr[4]={0}, wi[4]={0}, Vr[16];
    aa_la_d_eev( 4, M, 4, wr, wi,
                 NULL, 0, Vr, 4 );
    /* printf("wr: "); aa_dump_vec(stdout, wr, 4 ); */
    /* printf("wi: "); aa_dump_vec(stdout, wi, 4 ); */
    /* printf("Vr\n"); */
    /* aa_dump_mat(stdout, Vr, 4, 4 ); */

    size_t i = aa_fmaxloc( 4, wr );
    AA_MEM_CPY( p, Vr+4*i, 4 );
}


void aa_tf_qurand( double q[4] ) {
    aa_vrand(4, q);
    for( size_t i = 0; i < 4; i ++ ) q[i] -= 0.5;
    aa_tf_qnormalize(q);
}

void aa_tf_qutr_rand( double E[7] )
{
    aa_vrand(7, E);
    for( size_t i = 0; i < 7; i ++ ) E[i] -= 0.5;
    aa_tf_qnormalize(E);
}

void aa_tf_relx( size_t n, const double *R,
                 const double *X, size_t ldx,
                 const double *Y, size_t ldy,
                 double *Z, size_t ldz )
{
    aa_cla_dlacpy( ' ', 3, (int)n,
                   Y, (int)ldy,
                   Z, (int)ldz );
    for( size_t j = 0; j < n; j ++ ) {
        double xp[3];
        aa_tf_9rot( R, &X[ldx*j], xp );
        for( size_t i = 0; i < 3; i++ )
            AA_MATREF(Z, ldz, i, j) -=  xp[i];
    }
}

void aa_tf_relx_mean( size_t n, const double *R,
                      const double *X, size_t ldx,
                      const double *Y, size_t ldy,
                      double rel[3])
{
    double *yp = AA_MEM_REGION_LOCAL_NEW_N(double, 3*n);
    aa_tf_relx(n,R, X, ldx, Y, ldy, yp, 3 );
    aa_la_d_colmean( 3, n, yp, 3, rel );
    aa_mem_region_local_pop(yp);
}

void aa_tf_relx_median( size_t n, const double *R,
                        const double *X, size_t ldx,
                        const double *Y, size_t ldy,
                        double rel[3])
{
    double *yp = AA_MEM_REGION_LOCAL_NEW_N(double, 3*n);
    aa_tf_relx(n,R, X, ldx, Y, ldy, yp, 3 );
    for( size_t i = 0; i < 3; i ++ )
        rel[i] = aa_la_d_median( n, yp+i, 3 );
    aa_mem_region_local_pop(yp);
}

double aa_tf_qangle_rel( const double *q, const double *p )
{
    double qrel[4];
    aa_tf_qcmul(q, p, qrel);
    aa_tf_qminimize(qrel);
    return  fabs(aa_tf_qangle(qrel));
}
