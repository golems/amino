/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
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

static void qln_jac(const double *q, double qq,
                    double zeta, double kappa,
                    struct aa_dmat *J )
{
    WITH_QUAT_XYZW(q, x, y, z, w);
    double xzeta = x*zeta;
    double yzeta = y*zeta;
    double zzeta = z*zeta;
    double iqq = 1 / qq;

    AA_DMAT_REF(J,0,0) = x * xzeta + kappa;
    AA_DMAT_REF(J,1,0) = x * yzeta;
    AA_DMAT_REF(J,2,0) = x * zzeta;
    AA_DMAT_REF(J,3,0) = x * iqq;

    AA_DMAT_REF(J,0,1) = AA_DMAT_REF(J,1,0);
    AA_DMAT_REF(J,1,1) = y * yzeta + kappa;
    AA_DMAT_REF(J,2,1) = y * zzeta;
    AA_DMAT_REF(J,3,1) = y * iqq;

    AA_DMAT_REF(J,0,2) = AA_DMAT_REF(J,2,0);
    AA_DMAT_REF(J,1,2) = AA_DMAT_REF(J,2,1);
    AA_DMAT_REF(J,2,2) = z * zzeta + kappa;
    AA_DMAT_REF(J,3,2) = z * iqq;

    AA_DMAT_REF(J,0,3) = -AA_DMAT_REF(J,3,0);
    AA_DMAT_REF(J,1,3) = -AA_DMAT_REF(J,3,1);
    AA_DMAT_REF(J,2,3) = -AA_DMAT_REF(J,3,2);
    AA_DMAT_REF(J,3,3) =  w/qq;
}

AA_API void
aa_tf_qln_jac(const double q[4], struct aa_dmat *J )
{
    aa_dmat_check_size(4,4,J);
    double w = q[AA_TF_QUAT_W];
    const double *q_v = q + AA_TF_QUAT_V;
    double vv = aa_tf_vdot(q_v,q_v);
    double qq = vv + w*w;
    double v_norm = sqrt(vv);
    double q_norm = sqrt(qq);
    double q3 = qq*q_norm;
    double theta = atan2(v_norm,w);

    double zeta, kappa;
    if( theta < DBL_EPSILON )  {
        zeta = aa_horner3( theta*theta, -2.0/3.0, -1.0/5.0, -17.0/420.0 ) / q3;
        kappa = aa_tf_invsinc_series2(theta*theta)/q_norm;
    } else {
        kappa = theta / v_norm;
        zeta = w / (qq*vv)  - kappa / vv ;
    }

    qln_jac(q,qq,zeta,kappa,J);


    /* /\* Fill xyz columns *\/ */
    /* const double *v = q + AA_TF_QUAT_XYZ; */
    /* double vzeta[4]; */
    /* for( size_t i = 0; i < 4; i ++ ) { */
    /*     vzeta[i] = v[i]*zeta; */
    /* } */
    /* vzeta[3] = 1/qq; */
    /* for( size_t j = 0; j < 3; j ++ ) { */
    /*     double *col = &AA_DMAT_REF(Js,0,j); */
    /*     for( size_t i = 0; i < 4; i ++ ) { */
    /*         col[i] = v[j] * vzeta[i]; */
    /*     } */
    /*     col[j] += eta; */
    /* } */

    /* /\* Fill w column *\/ */
    /* for( size_t i = 0; i < 4; i ++ ) { */
    /*     AA_DMAT_REF(Js,i,3) = -q[i] / qq; */
    /* } */

}

static inline double
s_ff( const double *h, const double *d,
      double zeta, double *kv,
      int a, int b)
{
    return (h[a]*d[b]+h[b]*d[a]) * zeta + h[a]*kv[b];
}

static inline double
s_ff_diag( const double *h, const double *d,
           double zeta, double *kv,
           int a )
{
    return h[a]*d[a]*(2*zeta) + h[a]*kv[a];
}


AA_API void
aa_tf_duqu_ln_jac(const double S[8], struct aa_dmat *J )
{
    aa_dmat_check_size(8,8,J);

    struct aa_dmat J_rr, J_rd, J_dr, J_dd;
    aa_dmat_view_block(&J_rd, J, AA_TF_DUQU_REAL, AA_TF_DUQU_DUAL, 4, 4);
    aa_dmat_zero(&J_rd);

    const double *q = S + AA_TF_DUQU_REAL;

    double w = q[AA_TF_QUAT_W];
    const double *q_v = q + AA_TF_QUAT_V;
    double vv = aa_tf_vdot(q_v,q_v);
    double qq = vv + w*w;
    double v_norm = sqrt(vv);
    double q_norm = sqrt(qq);
    double q4 = qq*qq;
    double theta = atan2(v_norm,w);

    double zeta, kappa, mu;
    if( theta < DBL_EPSILON )  {
        double q3 = qq*q_norm;
        double q5 = q4*q_norm;
        kappa = aa_tf_invsinc_series2(theta*theta)/q_norm;
        zeta = aa_horner3( theta*theta, -2.0/3.0, -1.0/5.0, -17.0/420.0 ) / q3;
        mu = aa_horner3( theta*theta, 8./5., 4./7., 1./7.)/q5;
    } else {
        kappa = theta / v_norm;
        zeta = w / (qq*vv)  - kappa / vv ;
        mu = (-2*w/q4 -3*zeta)/vv;
    }

    aa_dmat_view_block(&J_rr, J, AA_TF_DUQU_REAL, AA_TF_DUQU_REAL, 4, 4);
    qln_jac(q,qq,zeta,kappa,&J_rr);
    aa_dmat_view_block(&J_dd, J, AA_TF_DUQU_DUAL, AA_TF_DUQU_DUAL, 4, 4);
    aa_dmat_copy(&J_rr,&J_dd);

    const double *d = S + AA_TF_DUQU_DUAL;
    double dw = d[AA_TF_QUAT_W];
    double gamma = aa_tf_vdot(q,d);
    double tau = gamma*zeta - dw/qq;
    double ks = gamma*mu + 2*(dw/q4);
    double kv[3];
    for( int i = 0; i < 3; i ++ ) kv[i] = q[i]*ks;
    aa_dmat_view_block(&J_dr, J, AA_TF_DUQU_DUAL, AA_TF_DUQU_REAL, 4, 4);

    AA_DMAT_REF(&J_dr,0,0) = s_ff_diag(q,d,zeta,kv,0) + tau;
    AA_DMAT_REF(&J_dr,1,0) = s_ff(q,d,zeta,kv,1,0);
    AA_DMAT_REF(&J_dr,2,0) = s_ff(q,d,zeta,kv,2,0);

    AA_DMAT_REF(&J_dr,0,1) = AA_DMAT_REF(&J_dr,1,0);
    AA_DMAT_REF(&J_dr,1,1) = s_ff_diag(q,d,zeta,kv,1) + tau;
    AA_DMAT_REF(&J_dr,2,1) = s_ff(q,d,zeta,kv,2,1);

    AA_DMAT_REF(&J_dr,0,2) = AA_DMAT_REF(&J_dr,2,0);
    AA_DMAT_REF(&J_dr,1,2) = AA_DMAT_REF(&J_dr,2,1);
    AA_DMAT_REF(&J_dr,2,2) = s_ff_diag(q,d,zeta,kv,2) + tau;

    double gamma2 = 2*(gamma + w*dw)/q4;
    for(size_t i = 0; i < 4; i ++ ) {
        AA_DMAT_REF(&J_dr,3,i) = d[i]/qq - q[i]*gamma2;
    }
    for(size_t i = 0; i < 3; i ++ ) {
        AA_DMAT_REF(&J_dr,i,3) = -AA_DMAT_REF(&J_dr,3,i);
    }
}

AA_API void
aa_tf_duqu_conj_jac( struct aa_dmat *J )
{
    aa_dmat_zero(J);
    for( size_t i = 0; i < 3; i ++ ) {
        size_t kr = i + AA_TF_DUQU_REAL_XYZ;
        size_t kd = i + AA_TF_DUQU_DUAL_XYZ;
        AA_DMAT_REF(J,kr,kr) = -1;
        AA_DMAT_REF(J,kd,kd) = -1;
    }
    {
        size_t kr = AA_TF_DUQU_REAL_W;
        size_t kd = AA_TF_DUQU_DUAL_W;
        AA_DMAT_REF(J,kr,kr) = 1;
        AA_DMAT_REF(J,kd,kd) = 1;
    }
}
