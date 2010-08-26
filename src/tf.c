/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define _GNU_SOURCE
#include "amino.h"




void aa_tf_qnormalize( double q[4] ) {
    aa_la_normalize( 4, q );
}

void aa_tf_qconj( const double q[4], double r[4] ) {
    r[0] = -q[0];
    r[1] = -q[1];
    r[2] = -q[2];
    r[3] =  q[3];
}

void aa_tf_qinv( const double q[4], double r[4] ) {
    aa_tf_qconj(q,r);
    aa_la_scal( 4, 1.0/aa_la_dot(4,r,r), r );
}

void aa_tf_qmul( const double a[4], const double b[4], double c[4] ) {
    c[3] = a[3]*b[3] - aa_la_dot(3, a, b);
    aa_la_cross( a, b, c );
    for( size_t i = 0; i < 3; i ++ )
        c[i] += a[3]*b[i] + b[3]*a[i];
}

void aa_tf_qadd( const double a[4], const double b[4], double c[4] );

void aa_tf_qadd( const double a[4], const double b[4], double c[4] );

void aa_tf_qslerp( double t, const double a[4], const double b[4], double c[4] );

void aa_tf_quat2axang( const double q[4], double axang[4] ) {
    double a = aa_la_norm(4,q);
    double w = q[3]/a;
    axang[3] = 2 * acos(w);
    aa_la_smul( 3,
                ( aa_feq( axang[3], 0, AA_TF_EPSILON ) /* ident check */ ?
                  0 : 1.0 / (a*sqrt(1 - w*w)) ),
                q, axang );

    /* if(  ) { */
    /*     aa_fset( axang, 0, 3 ); */
    /* } else { */
    /*     aa_la_smul( 3, 1.0 / (a*sqrt(1 - w*w)), q, axang ); */
    /* } */
}

void aa_tf_axang_make( double x, double y, double z, double theta, double axang[4] ) {
    double n = sqrt(x*x + y*y + z*z);
    // FIXME: zeros
    axang[0] = x/n;
    axang[1] = y/n;
    axang[2] = z/n;
    axang[3] = aa_an_norm_pi(theta);
}

void aa_tf_axang_permute2( const double aa[4], double aa_plus[4], double aa_minus[4] ) {
   aa_fcpy( aa_plus, aa, 3 );
   aa_plus[3] = aa[3] + 2*M_PI;
   aa_fcpy( aa_minus, aa, 3 );
   aa_minus[3] = aa[3] - 2*M_PI;
}


void aa_tf_axang2rotvec( const double axang[4], double rotvec[3] ) {
    aa_la_smul( 3, axang[3], axang, rotvec );
}

void aa_tf_rotvec2axang( const double rotvec[3], double axang[4] ) {
    axang[3] = aa_la_norm(3,rotvec);
    aa_la_smul( 3,
                ( aa_feq( axang[3], 0, AA_TF_EPSILON ) /* ident check */ ?
                  0 :
                  1.0 / axang[3] ),
                rotvec, axang );
}

void aa_tf_axang2quat( const double axang[4], double q[4] ) {
    double s,c;
    sincos( axang[3]/2, &s, &c );
    q[3] = c;
    aa_la_smul( 3, s, axang, q );
}

AA_API void aa_tf_quat2rotmat( const double q[4], double R[9] ) {
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

AA_API void aa_tf_rotmat2quat( const double R[9], double q[4] ) {

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

    fflush(stderr);
}

AA_API void aa_tf_rotvec2quat( const double rotvec[3], double q[4] ) {
    double aa[4];
    aa_tf_rotvec2axang(rotvec, aa);
    aa_tf_axang2quat(aa,q);
}

AA_API void aa_tf_quat2rotvec( const double q[4], double rotvec[3] ) {
    double aa[4];
    aa_tf_quat2axang(q,aa);
    aa_tf_axang2rotvec(aa,rotvec);
}

/* AA_API void aa_tf_tfv2tfq( const double vrv[6],  */
/*                            double x[3], double quat[4] ) { */
/*     aa_fcpy(x,vrv,3); */
/*     aa_tf_rotvec2quat( vrv+3, quat ); */
/* } */

/* AA_API void aa_tf_tfq2tfv( const double x[3], const double quat[4],  */
/*                            double vrv[6] ) { */
/*     aa_fcpy(vrv,x,3); */
/*     aa_tf_quat2rotvec( quat, vrv+3 ); */
/* } */

