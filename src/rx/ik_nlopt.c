/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
 * Copyright (c) 2018-2019, Colorado School of Mines
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
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#include <nlopt.h>
#include "amino.h"
#include "amino/diffeq.h"



/* static double */
/* s_nlobj_jpinv(unsigned n, const double *q, double *dq, void *vcx) */
/* { */
/*     struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx; */
/*     void *ptrtop = aa_mem_region_ptr(cx->reg); */

/*     struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1); */
/*     aa_rx_fk_sub(cx->fk, cx->ssg, &vq); */
/*     double *E_act = aa_rx_fk_ref(cx->fk, cx->frame); */


/*     if( dq ) { */
/*         struct aa_dvec v_dq = AA_DVEC_INIT(n,dq,1); */
/*         s_ksol_jpinv(cx,q, &v_dq); */
/*         aa_dvec_scal(-1,&v_dq); */
/*     } */

/*     double x = s_serr( E_act, cx->TF_ref->data ); */

/*     aa_mem_region_pop(cx->reg, ptrtop); */
/*     return x; */

/* } */

static double s_nlobj_dq_fd_helper( void *vcx, const struct aa_dvec *x)
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);
    assert(1 == x->inc);

    aa_rx_fk_sub(cx->fk, cx->ssg, x);
    double *E_act = aa_rx_fk_ref(cx->fk, cx->frame);

    double S_act[8], S_ref[8], S_err[8], S_ln[8];
    aa_tf_qutr2duqu(E_act, S_act);
    aa_tf_qutr2duqu(cx->TF_ref->data, S_ref);
    aa_tf_duqu_cmul(S_act,S_ref,S_err);
    aa_tf_duqu_minimize(S_err);
    aa_tf_duqu_ln(S_err, S_ln);
    double result = aa_la_dot(8,S_ln,S_ln);

    aa_mem_region_pop(cx->reg, ptrtop);
    return result;
}

AA_API double
aa_rx_ik_opt_err_dqln_fd( void *vcx, const double *q, double *dq )
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    size_t n = aa_rx_sg_sub_config_count(cx->ssg);

    struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1);
    double x = s_nlobj_dq_fd_helper(vcx, &vq);

    if( dq ) {
        struct aa_dvec vdq = AA_DVEC_INIT(n,dq,1);
        // TODO: make epsilon a parameter
        double eps = 1e-6;
        aa_de_grad_fd( s_nlobj_dq_fd_helper, vcx,
                       &vq, eps, &vdq );
    }

    return x;
}

static void
mv_block_helper (const double *A, const double *x, double *y)
{
    cblas_dgemv(CblasColMajor, CblasTrans, 8, 4,
                1, A, 8, x, 1,
                0, y, 1);
    cblas_dgemv(CblasColMajor, CblasTrans, 4, 4,
                1, A, 8, x+4, 1,
                0, y+4, 1);
}

static void
duqu_rmul_helper( const double *Sr, const double *x, double *y )
{
    double M[8*8];
    aa_tf_duqu_matrix_r(Sr,M,8);
    mv_block_helper(M, x, y);
}


AA_API double
aa_rx_ik_opt_err_dqln( void *vcx, double *q, double *dq ) {
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);

    size_t n = aa_rx_sg_sub_config_count(cx->ssg);
    struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1);
    aa_rx_fk_sub(cx->fk, cx->ssg, &vq );
    double *E_act = aa_rx_fk_ref(cx->fk, cx->frame);


    double S_act[8], S_ref[8], S_err[8], S_ln[8];
    aa_tf_qutr2duqu(E_act, S_act);
    aa_tf_qutr2duqu(cx->TF_ref->data, S_ref);
    aa_tf_duqu_cmul(S_act,S_ref,S_err);
    // Apply a negative factor in gradient when we have to minimze the
    // error quaternion
    int needs_min = (S_err[AA_TF_DUQU_REAL_W] < 0 );
    if( needs_min ) {
        for( size_t i = 0; i < 8; i ++ ) S_err[i] *= -1;
    } else {
    }
    aa_tf_duqu_ln(S_err, S_ln);
    double result = aa_la_dot(8,S_ln,S_ln);

    if( dq ) {
        if( needs_min ) {
            for( size_t i = 0; i < 8; i ++ ) S_ln[i] *= -1;
        }
        struct aa_dvec v_dq = AA_DVEC_INIT(n,dq,1);

        /*
         * g_sumsq * J_ln*[S_ref]_r*J_conj*J_S
         * -> J_ln^T g_sumsq * [S_ref]_r*J_conj*J_S
         * -> [S_ref]_r^T (J_ln^T g_sumsq) *J_conj*J_S
         * -> J_conj^T*[S_ref]_r^T (J_ln^T g_sumsq) * J_S
         * -> J_S^T J_conj^T * [S_ref]_r^T * J_ln^T g_sumsq
         */

        double a[8], b[8], dJ[8*8];
        struct aa_dmat vJ_8x8 = AA_DMAT_INIT(8,8,dJ,8);
        struct aa_dmat *J_8x8 = &vJ_8x8;

        {
            struct aa_dmat *J_ln = J_8x8;
            aa_tf_duqu_ln_jac(S_err,J_ln);
            /*
             *  struct aa_dvec v_S_ln = AA_DVEC_INIT(8,S_ln,1);
             * //aa_dmat_gemv( CblasTrans, 2, J_8x8, &v_S_ln, 0, &va );
             *
             * Avoid multiplying by the zero block:
             *
             * J_ln = [J_r   0]     J_ln^T = [J_r^T J_d^T]
             *        [J_d J_r]              [  0   J_r^T]
             *
             * J_ln^T * del(S_ln^T*S_ln)^T = 2*J_ln^T S_ln
             *
             * 2*J_ln^T S_ln = 2*[J_r^T  J_d^T] (S_ln_r)
             *                   [   0   J_r^T] (S_ln_d)
             *
             */
            mv_block_helper(dJ, S_ln, a);
        }


        /*
         * dS/dphi = [S]_R V J
         * g [S]_R V J
         * -> [S]_R^T g V J
         * -> V^T [S]_R^T g J
         * -> J^T V [S]_R^T g
         */

        duqu_rmul_helper( S_ref, a, b );
        aa_tf_duqu_conj1(b);

        duqu_rmul_helper( S_act, b, a );  /* TODO: Pure result, some
                                           * extra multiplies here. */
        // a is now a pure dual quaternion
        {
            double c[6];
            struct aa_dvec vc = AA_DVEC_INIT(6,c,1);
            // Using Twist Jacobian
            aa_tf_duqu2pure(a,c);
            struct aa_dmat *Jtw = aa_rx_sg_sub_jac_twist_get(cx->ssg, cx->reg, cx->fk);
            aa_dmat_gemv( CblasTrans, 1, Jtw, &vc, 0, &v_dq );

            // Using Velocity Jacobian
            /* aa_tf_cross_a(a+AA_TF_DUQU_DUAL_XYZ,E_act+AA_TF_QUTR_V,a); */
            /* aa_tf_duqu2pure(a,c); */
            /* struct aa_dmat *J_vel = aa_rx_sg_sub_get_jacobian(cx->ssg,cx->reg,cx->TF); */
            /* aa_dmat_gemv( CblasTrans, 1, J_vel, &vc, 0, &v_dq ); */

        }


        /* { */
        /*     struct aa_dvec *v_dq_fd = aa_dvec_alloc(cx->reg,n); */
        /*     double eps = 1e-6; */
        /*     aa_de_grad_fd( s_nlobj_dq_fd_helper, vcx, */
        /*                    &vq, eps, v_dq_fd ); */


        /*     printf("--\n"); */
        /*     printf("needs min: %d\n", needs_min); */
        /*     printf("ad: "); */
        /*     aa_dump_vec(stdout,v_dq_fd->data,n); */
        /*     printf("an: "); */
        /*     aa_dump_vec(stdout,dq,n); */

        /*     assert( aa_dvec_ssd(v_dq_fd, &v_dq) < 1e-3 ); */
        /* } */
    }

    aa_mem_region_pop(cx->reg, ptrtop);
    return result;
}

static double s_nlobj_trans_fd_helper( void *vcx, const struct aa_dvec *x)
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    assert(1 == x->inc);

    aa_rx_fk_sub(cx->fk, cx->ssg, x);
    double *E_act = aa_rx_fk_ref(cx->fk, cx->frame);
    double *v_act = E_act + AA_TF_QUTR_V;

    double *E_ref = cx->TF_ref->data;
    double *v_ref = E_ref + AA_TF_QUTR_V;

    double E_err[7];
    double *v_err = E_err + AA_TF_QUTR_V;


    for(size_t i = 0; i < 3; i ++ ) v_err[i] = v_act[i] - v_ref[i];

    double result = aa_tf_vdot(v_err,v_err);

    return result;
}

AA_API double
aa_rx_ik_opt_err_trans_fd( void *vcx, const double *q, double *dq )
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    size_t n = aa_rx_sg_sub_config_count(cx->ssg);

    struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1);
    double x = s_nlobj_trans_fd_helper(vcx, &vq);

    if( dq ) {
        struct aa_dvec vdq = AA_DVEC_INIT(n,dq,1);
        // TODO: make epsilon a parameter
        double eps = 1e-6;
        aa_de_grad_fd( s_nlobj_trans_fd_helper, vcx,
                       &vq, eps, &vdq );
    }
    /* fprintf(stdout, "fd error: %2.2f\n", x);  */
    return x;
}

static double s_nlobj_qv_fd_helper( void *vcx, const struct aa_dvec *x)
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);
    assert(1 == x->inc);

    aa_rx_fk_sub(cx->fk, cx->ssg, x);
    double *E_act = aa_rx_fk_ref(cx->fk, cx->frame);
    double *v_act = E_act + AA_TF_QUTR_V;
    double *q_act = E_act + AA_TF_QUTR_Q;

    double *E_ref = cx->TF_ref->data;
    double *v_ref = E_ref + AA_TF_QUTR_V;
    double *q_ref = E_ref + AA_TF_QUTR_Q;

    double E_err[7];
    double *v_err = E_err + AA_TF_QUTR_V;
    double *q_err = E_err + AA_TF_QUTR_Q;

    aa_tf_qcmul(q_act,q_ref,q_err);

    double q_ln[4];
    aa_tf_qminimize(q_err);
    aa_tf_duqu_ln(q_err, q_ln);

    for(size_t i = 0; i < 3; i ++ ) v_err[i] = v_act[i] - v_ref[i];

    double result = ( aa_tf_qdot(q_ln,q_ln)
                      +
                      aa_tf_vdot(v_err,v_err) );

    aa_mem_region_pop(cx->reg, ptrtop);
    return result;
}


AA_API double
aa_rx_ik_opt_err_qlnpv_fd( void *vcx, const double *q, double *dq )
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    size_t n = aa_rx_sg_sub_config_count(cx->ssg);

    struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1);
    double x = s_nlobj_qv_fd_helper(vcx, &vq);

    if( dq ) {
        struct aa_dvec vdq = AA_DVEC_INIT(n,dq,1);
        // TODO: make epsilon a parameter
        double eps = 1e-6;
        aa_de_grad_fd( s_nlobj_qv_fd_helper, vcx,
                       &vq, eps, &vdq );
    }

    return x;
}

AA_API double
aa_rx_ik_opt_err_trans( void *vcx, const double *q, double *dq )
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);

    size_t n = aa_rx_sg_sub_config_count(cx->ssg);
    struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1);
    aa_rx_fk_sub(cx->fk, cx->ssg, &vq );

    double *E_act = aa_rx_fk_ref(cx->fk, cx->frame);
    double *v_act = E_act + AA_TF_QUTR_V;

    double *E_ref = cx->TF_ref->data;
    double *v_ref = E_ref + AA_TF_QUTR_V;

    double E_err[7];
    double *v_err = E_err + AA_TF_QUTR_V;

    for(size_t i = 0; i < 3; i ++ ) v_err[i] = v_act[i] - v_ref[i];

    double result = aa_tf_vdot(v_err,v_err);

    if( dq ) {

        struct aa_dvec v_dq = AA_DVEC_INIT(n,dq,1);

        struct aa_dmat *Jvel = aa_rx_sg_sub_jac_vel_get(cx->ssg, cx->reg, cx->fk);
        struct aa_dmat Jv;
        aa_dmat_view_block(&Jv, Jvel, AA_TF_DX_V, 0, 3, Jvel->cols);

        // Translational Part
        struct aa_dvec v_v_err = AA_DVEC_INIT(3,v_err,1);
        aa_dmat_gemv(CblasTrans, 2, &Jv, &v_v_err, 0, &v_dq);
    }

    aa_mem_region_pop(cx->reg, ptrtop);
    /* printf("err: %f\n", result); */
    return result;
}

static void
q_rmul_helper( const double *q, const struct aa_dvec *x, struct aa_dvec *y )
{
    double dM[4*4];
    struct aa_dmat M = AA_DMAT_INIT(4,4,dM,4);
    aa_tf_qmat_r(q,&M);
    aa_dmat_gemv(CblasTrans, 1, &M, x, 0, y);
}


AA_API double
aa_rx_ik_opt_err_qlnpv( void *vcx, const double *q, double *dq )
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);

    size_t n = aa_rx_sg_sub_config_count(cx->ssg);
    struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1);
    aa_rx_fk_sub(cx->fk, cx->ssg, &vq );

    double *E_act = aa_rx_fk_ref(cx->fk, cx->frame);
    double *v_act = E_act + AA_TF_QUTR_V;
    double *q_act = E_act + AA_TF_QUTR_Q;

    double *E_ref = cx->TF_ref->data;
    double *v_ref = E_ref + AA_TF_QUTR_V;
    double *q_ref = E_ref + AA_TF_QUTR_Q;

    double E_err[7];
    double *v_err = E_err + AA_TF_QUTR_V;
    double *q_err = E_err + AA_TF_QUTR_Q;

    aa_tf_qcmul(q_act,q_ref,q_err);

    // Apply a negative factor in gradient when we have to minimze the
    // error quaternion
    int needs_min = (E_err[AA_TF_QUAT_W] < 0 );
    if( needs_min ) {
        for( size_t i = 0; i < 4; i ++ ) E_err[i] *= -1;
    }

    double q_lnv[3];
    aa_tf_qulnv(E_err, q_lnv);

    for(size_t i = 0; i < 3; i ++ ) v_err[i] = v_act[i] - v_ref[i];

    double result = ( aa_tf_vdot(q_lnv,q_lnv)
                      + aa_tf_vdot(v_err,v_err) );

    if( dq ) {

        struct aa_dvec v_dq = AA_DVEC_INIT(n,dq,1);

        struct aa_dmat *Jvel = aa_rx_sg_sub_jac_vel_get(cx->ssg, cx->reg, cx->fk);
        struct aa_dmat Jr, Jv;
        aa_dmat_view_block(&Jv, Jvel, AA_TF_DX_V, 0, 3, Jvel->cols);
        aa_dmat_view_block(&Jr, Jvel, AA_TF_DX_W, 0, 3, Jvel->cols);

        // Translational Part
        struct aa_dvec v_v_err = AA_DVEC_INIT(3,v_err,1);
        aa_dmat_gemv(CblasTrans, 2, &Jv, &v_v_err, 0, &v_dq);

        // Rotational Part
        if( needs_min ) {
            for( size_t i = 0; i < 3; i ++ ) q_lnv[i] *= -1;
        }

        double a[4], b[4], dJ[4*4];
        struct aa_dvec va = AA_DVEC_INIT(4,a,1);
        struct aa_dvec vb = AA_DVEC_INIT(4,b,1);
        struct aa_dmat vJ_4x4 = AA_DMAT_INIT(4,4,dJ,4);
        struct aa_dmat *J_4x4 = &vJ_4x4;

        {
            struct aa_dvec vln = AA_DVEC_INIT(3,q_lnv,1);
            struct aa_dmat *J_ln = J_4x4;
            aa_tf_qln_jac(q_err,J_ln); // TODO: omit a row for unit quat
            struct aa_dmat J_lnb = AA_DMAT_INIT(3,4,dJ,4);
            aa_dmat_gemv(CblasTrans, 1, &J_lnb, &vln, 0, &va);
        }

        q_rmul_helper( q_ref, &va, &vb );
        aa_tf_qconj1(b);

        q_rmul_helper( q_act, &vb, &va ); /* TODO: Pure result, some
                                           * extra multiplies here. */

        // a is now a pure dual quaternion */
        {
            struct aa_dvec va3 = AA_DVEC_INIT(3,a,1);
            aa_dmat_gemv( CblasTrans, 1, &Jr, &va3, 1, &v_dq );
        }


        /* { */
        /*     struct aa_dvec *v_dq_fd = aa_dvec_alloc(cx->reg,n); */
        /*     double eps = 1e-6; */
        /*     aa_de_grad_fd( s_nlobj_qv_fd_helper, vcx, */
        /*                    &vq, eps, v_dq_fd ); */


        /*     printf("--\n"); */
        /*     printf("needs min: %d\n", needs_min); */
        /*     printf("ad: "); */
        /*     aa_dump_vec(stdout,v_dq_fd->data,n); */
        /*     printf("an: "); */
        /*     aa_dump_vec(stdout,dq,n); */

        /*     assert( aa_dvec_ssd(v_dq_fd, &v_dq) < 1e-3 ); */
        /* } */
    }

    aa_mem_region_pop(cx->reg, ptrtop);
    //printf("err: %f\n", result);
    return result;
}


// TODO: weighted error

AA_API double
aa_rx_ik_opt_err_jcenter( void *vcx, const double *q, double *dq )
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    struct aa_mem_region *reg = cx->reg;
    void *ptrtop = aa_mem_region_ptr(reg);

    size_t n = aa_rx_sg_sub_config_count(cx->ssg);
    struct aa_dvec *q_center = aa_dvec_alloc(reg,n);
    aa_rx_sg_sub_center_configv(cx->ssg,q_center);

    double result = 0;

    if( dq ) { // error and gradient
        for( size_t i = 0; i < n; i ++ ) {
            double d = q[i] - AA_DVEC_REF(q_center,i);
            result += (d*d);
            dq[i] = d;
        }
        //printf("--\n");
        //printf("q:  "); aa_dump_vec(stdout,q,n);
        //printf("qc: "); aa_dump_vec(stdout,q_center->data,n);
        //printf("dq: "); aa_dump_vec(stdout,dq,n);
    } else { // error only
        for( size_t i = 0; i < n; i ++ ) {
            double d = q[i] - AA_DVEC_REF(q_center,i);
            result += (d*d);
        }
    }

    aa_mem_region_pop(cx->reg, ptrtop);

    return result / 2;
}

struct err_cx {
    void *cx;
    aa_rx_ik_opt_fun *fun;
};

static double
s_err_cx_dispatch(unsigned n, const double *q, double *dq, void *vcx)
{
    (void)n;
    struct err_cx *cx = (struct err_cx *)vcx;
    return cx->fun(cx->cx, q, dq);
}

static int
s_ik_nlopt( struct kin_solve_cx *cx,
            struct aa_dvec *q )
{
    struct aa_mem_region *reg = cx->reg;
    void *ptrtop = aa_mem_region_ptr(reg);

    const struct aa_rx_sg_sub *ssg = cx->ssg;
    const struct aa_rx_sg *sg = cx->ssg->scenegraph;

    if( 1 != cx->q_sub->inc || 1 != cx->q_all->inc ) {
        return AA_RX_INVALID_PARAMETER;
    }

    size_t n_sub = cx->q_sub->len;
    nlopt_algorithm alg = NLOPT_LD_SLSQP;
    nlopt_opt opt = nlopt_create(alg, (unsigned)n_sub); /* algorithm and dimensionality */

    struct err_cx ocx, eqctcx;
    ocx.cx = cx;
    ocx.fun = cx->ik_cx->opts->obj_fun;
    nlopt_set_min_objective(opt, s_err_cx_dispatch, &ocx);

    if( cx->ik_cx->opts->eqct_fun ) {
        eqctcx.cx = cx;
        eqctcx.fun = cx->ik_cx->opts->eqct_fun;
        nlopt_add_equality_constraint(opt, s_err_cx_dispatch, &eqctcx,
                                      cx->ik_cx->opts->eqct_tol);
    }


    //nlopt_set_xtol_rel(opt, 1e-4); // TODO: make a parameter
    if( cx->opts->tol_obj_abs >= 0 )
        nlopt_set_ftol_abs(opt, cx->opts->tol_obj_abs );
    if( cx->opts->tol_obj_rel >= 0 )
        nlopt_set_ftol_rel(opt, cx->opts->tol_obj_rel );
    if( cx->opts->tol_dq >= 0 )
        nlopt_set_xtol_rel(opt, cx->opts->tol_dq );

    double *lb = AA_MEM_REGION_NEW_N(reg,double,n_sub);
    double *ub = AA_MEM_REGION_NEW_N(reg,double,n_sub);
    { // bounds
        for( size_t i = 0; i < n_sub; i ++ ) {
            aa_rx_config_id id = aa_rx_sg_sub_config(ssg,i);
            if( aa_rx_sg_get_limit_pos(sg, id, lb+i, ub+i) )
            {
                lb[i] = -DBL_MAX;
                ub[i] = DBL_MAX;
            }
        }
    }


    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);
    double minf;
    nlopt_result ores = nlopt_optimize(opt, cx->q_sub->data, &minf);
    if( cx->opts->debug ) {
        /* fprintf("AMINO IK NLOPT RESULT: %s (%d)", */
        /*         nlopt_result_to_string(ores), ores); */

        fprintf(stderr, "AMINO IK NLOPT RESULT: %d\n", ores);
    }
    aa_dvec_copy( cx->q_sub, q );


    aa_rx_fk_sub(cx->fk, cx->ssg, q);
    double *E_act = aa_rx_fk_ref(cx->fk, cx->frame);
    int result = s_check(cx->ik_cx, cx->TF_ref, E_act );
    nlopt_destroy(opt);
    aa_mem_region_pop(reg,ptrtop);

    return result;

}
