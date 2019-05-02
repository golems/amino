/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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
#include "amino/diffeq.h"


static double
s_nlobj_jpinv(unsigned n, const double *q, double *dq, void *vcx)
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);

    double  E_act[7];
    struct aa_dmat *TF_abs;
    s_tf( cx, q, &TF_abs, E_act );

    if( dq ) {
        struct aa_dvec v_dq = AA_DVEC_INIT(n,dq,1);
        s_ksol_jpinv(cx,q,TF_abs, E_act, &v_dq);
        aa_lb_dscal(-1,&v_dq);
    }

    double x = s_serr( E_act, cx->E1 );

    aa_mem_region_pop(cx->reg, ptrtop);
    return x;

}

static double s_nlobj_dq_fd_helper( void *vcx, const struct aa_dvec *x)
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);
    double *q = x->data;
    assert(1 == x->inc);

    double  E_act[7];
    struct aa_dmat *TF_abs;
    s_tf( cx, q, &TF_abs, E_act );

    double S_act[8], S_ref[8], S_err[8], S_ln[8];
    aa_tf_qutr2duqu(E_act, S_act);
    aa_tf_qutr2duqu(cx->E1, S_ref);
    aa_tf_duqu_cmul(S_act,S_ref,S_err);
    aa_tf_duqu_minimize(S_err);
    aa_tf_duqu_ln(S_err, S_ln);
    double result = aa_la_dot(8,S_ln,S_ln);

    aa_mem_region_pop(cx->reg, ptrtop);
    return result;
}

static double
s_nlobj_dq_fd(unsigned n, const double *q, double *dq, void *vcx)
{
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


static double
s_nlobj_dq_an(unsigned n, const double *q, double *dq, void *vcx)
{

    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);

    double  E_act[7];
    struct aa_dmat *TF_abs;
    s_tf( cx, q, &TF_abs, E_act );
    //double result = s_serr( E_act, cx->E1 );

    double S_act[8], S_ref[8], S_err[8], S_ln[8];
    aa_tf_qutr2duqu(E_act, S_act);
    aa_tf_qutr2duqu(cx->E1, S_ref);
    aa_tf_duqu_cmul(S_act,S_ref,S_err);
    aa_tf_duqu_minimize(S_err);
    aa_tf_duqu_ln(S_err, S_ln);
    double result = aa_la_dot(8,S_ln,S_ln);


    if( dq ) {
        //struct aa_dvec v_S_err = AA_DVEC_INIT(8,S_err,1);

        // TODO: make epsilon a parameter
        struct aa_dvec v_dq = AA_DVEC_INIT(n,dq,1);

        /* struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1); */
        /* double eps = 1e-6; */
        /* aa_de_grad_fd( s_nlobj_dq_fd_helper, vcx, */
        /*                &vq, eps, &v_dq ); */
        /* printf("--\nfd: "); */
        /* aa_dump_vec(stdout,dq,n); */


        struct aa_dvec *g_sumsq = aa_dvec_alloc(cx->reg,8);
        struct aa_dvec v_S_ln = AA_DVEC_INIT(8,S_ln,1);
        aa_lb_dcopy(&v_S_ln,g_sumsq);
        aa_lb_dscal(2,g_sumsq);

        struct aa_dmat *J_ln = aa_dmat_alloc(cx->reg,8,8);
        aa_tf_duqu_ln_jac(S_err,J_ln);

        struct aa_dmat *J_conj = aa_dmat_alloc(cx->reg,8,8);
        aa_tf_duqu_conj_jac(J_conj);

        struct aa_dmat *J_S = aa_dmat_alloc(cx->reg,8,n);
        struct aa_dmat *J_vel = aa_rx_sg_sub_get_jacobian(cx->ssg,cx->reg,TF_abs);
        aa_tf_duqu_jac_vel2diff(S_act,J_vel, J_S);

        struct aa_dmat *J_conj_phi = aa_dmat_alloc(cx->reg,8,n);
        aa_lb_dgemm(CblasNoTrans, CblasNoTrans, 1, J_conj, J_S, 0, J_conj_phi);

        struct aa_dmat *M_S_ref = aa_dmat_alloc(cx->reg,8,8);
        aa_tf_duqu_mat_r(S_ref,M_S_ref);
        struct aa_dmat *J_conj_phi_ref = aa_dmat_alloc(cx->reg,8,n);
        aa_lb_dgemm(CblasNoTrans, CblasNoTrans, 1, M_S_ref, J_conj_phi, 0, J_conj_phi_ref);

        struct aa_dmat *J_ln_phi = aa_dmat_alloc(cx->reg,8,n);
        aa_lb_dgemm(CblasNoTrans, CblasNoTrans, 1, J_ln, J_conj_phi_ref, 0, J_ln_phi);

        struct aa_dmat *J_ln_phi_t = aa_dmat_alloc(cx->reg,n,8);
        aa_dmat_trans(J_ln_phi, J_ln_phi_t);
        aa_lb_dgemv( CblasNoTrans, 1, J_ln_phi_t, g_sumsq, 0, &v_dq );

        /* printf("an: "); */
        /* aa_dump_vec(stdout,dq,n); */

    }

    aa_mem_region_pop(cx->reg, ptrtop);
    return result;
}



static int
s_ik_nlopt( struct kin_solve_cx *cx,
            double (*obj)(unsigned n, const double *q, double *dq, void *vcx),
            struct aa_dvec *q )
{

    struct aa_mem_region *reg = cx->reg;
    void *ptrtop = aa_mem_region_ptr(reg);

    const struct aa_rx_sg_sub *ssg = cx->ssg;
    const struct aa_rx_sg *sg = cx->ssg->scenegraph;

    if( 1 != cx->q_sub->inc || 1 != cx->q0_sub->inc ) {
        return AA_RX_INVALID_PARAMETER;
    }

    size_t n_sub = cx->n;


    aa_lb_dcopy(cx->q0_sub, cx->q_sub);

    //nlopt_algorithm alg = NLOPT_LN_BOBYQA;
    nlopt_algorithm alg = NLOPT_LD_SLSQP;
    nlopt_opt opt = nlopt_create(alg, (unsigned)n_sub); /* algorithm and dimensionality */
    nlopt_set_min_objective(opt, obj, cx);
    nlopt_set_xtol_rel(opt, 1e-4);
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
    int r = -1;
    double minf;
    if (nlopt_optimize(opt, cx->q_sub->data, &minf) < 0) {
        // no solution
        // printf("nlopt failed!\n");
    } else {
        r = 0;
        aa_lb_dcopy( cx->q_sub, q );
        //printf("found minimum\n");
    }

    nlopt_destroy(opt);

    aa_mem_region_pop(reg,ptrtop);

    if( r ) {
        return AA_RX_NO_SOLUTION | AA_RX_NO_IK;
    } else {
        return 0;
    }
}
