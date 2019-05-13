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
    struct aa_dvec vq = AA_DVEC_INIT(n,(double*)q,1);
    s_tf_update( cx, &vq, cx->TF, E_act );


    if( dq ) {
        struct aa_dvec v_dq = AA_DVEC_INIT(n,dq,1);
        s_ksol_jpinv(cx,q,cx->TF, E_act, &v_dq);
        aa_lb_dscal(-1,&v_dq);
    }

    double x = s_serr( E_act, cx->TF_ref->data );

    aa_mem_region_pop(cx->reg, ptrtop);
    return x;

}

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

static double
s_nlobj_dq_an(unsigned n, const double *q, double *dq, void *vcx)
{
    struct kin_solve_cx *cx = (struct kin_solve_cx*)vcx;
    void *ptrtop = aa_mem_region_ptr(cx->reg);

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
             * //aa_lb_dgemv( CblasTrans, 2, J_8x8, &v_S_ln, 0, &va );
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
            aa_lb_dgemv( CblasTrans, 1, Jtw, &vc, 0, &v_dq );

            // Using Velocity Jacobian
            /* aa_tf_cross_a(a+AA_TF_DUQU_DUAL_XYZ,E_act+AA_TF_QUTR_V,a); */
            /* aa_tf_duqu2pure(a,c); */
            /* struct aa_dmat *J_vel = aa_rx_sg_sub_get_jacobian(cx->ssg,cx->reg,cx->TF); */
            /* aa_lb_dgemv( CblasTrans, 1, J_vel, &vc, 0, &v_dq ); */

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



static int
s_ik_nlopt( struct kin_solve_cx *cx,
            double (*obj)(unsigned n, const double *q, double *dq, void *vcx),
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

    nlopt_set_min_objective(opt, obj, cx);
    nlopt_set_xtol_rel(opt, 1e-4); // TODO: make a parameter
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
    if (nlopt_optimize(opt, cx->q_sub->data, &minf) > 0) {
        // found miniumum
    }
    aa_lb_dcopy( cx->q_sub, q );


    double E[7];
    s_tf_update( cx, cx->q_sub, cx->TF, E);
    int result = s_check(cx->ik_cx, cx->TF_ref, E );
    nlopt_destroy(opt);
    aa_mem_region_pop(reg,ptrtop);

    return result;

}
