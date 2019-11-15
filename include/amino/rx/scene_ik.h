/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder the names of its
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

#ifndef AMINO_RX_SCENE_IK_H
#define AMINO_RX_SCENE_IK_H

#include "scenegraph.h"

/**
 * @file scene_ik.h
 * @brief Inverse Position Kinematics
 */

struct aa_rx_sg_sub;

/*-- Kinematic Solvers --*/

/**
 * General type for an IK solver function
 */
typedef int aa_rx_ik_fun( void *context,
                          size_t n_tf, const double *TF, size_t ld_TF,
                          size_t n_q, double *q );


/**
 * IK parameters opaque structure.
 */
struct aa_rx_ik_parm;

/**
 * Inverse Kinematics Algorithm
 */
enum aa_rx_ik_algo {
    /**
     * Sequential Quadratic Program.
     *
     * @sa aa_rx_ik_parm_set_obj
     * @sa aa_rx_ik_parm_set_eqct
     */
    AA_RX_IK_SQP,

    /**
     * Jacobian pseudo-inverse w/ adapative integration  (kludgey)
     */
    AA_RX_IK_JPINV,

    /**
     * Levenberg-Marquardt (unimplemented)
     */
    AA_RX_IK_LMA

};

/**
 * Create options struct for kinematic solver.
 *
 * The default options should offer good performance and robustness.
 */
AA_API struct aa_rx_ik_parm*
aa_rx_ik_parm_create();

/**
 * Destroy options struct for kinematic solver.
 */
AA_API void
aa_rx_ik_parm_destroy( struct aa_rx_ik_parm *parm);

/**
 * Set integration step.
 */
AA_API void
aa_rx_ik_parm_set_dt( struct aa_rx_ik_parm *parm, double dt);

/**
 * Set inverse kinematics algorithm
 *
 * @sa aa_rx_ik_algorithm
 */
AA_API void
aa_rx_ik_parm_set_algo( struct aa_rx_ik_parm *parm,
                        enum aa_rx_ik_algo algo );






/**
 * Set angular tolerance.
 */
AA_API void
aa_rx_ik_parm_set_tol_angle( struct aa_rx_ik_parm *parm, double tol);

/**
 * Set translational tolerance.
 */
AA_API void
aa_rx_ik_parm_set_tol_trans( struct aa_rx_ik_parm *parm, double tol);

/**
 * Set angular tolerance to switch to SVD.
 */
AA_API void
aa_rx_ik_parm_set_tol_angle_svd( struct aa_rx_ik_parm *parm, double tol);

/**
 * Set translational tolerance to switch to SVD.
 */
AA_API void
aa_rx_ik_parm_set_tol_trans_svd( struct aa_rx_ik_parm *parm, double tol);

/**
 * Set tolerance on joint motion.
 */
AA_API void
aa_rx_ik_parm_set_tol_dq( struct aa_rx_ik_parm *parm, double tol);

/**
 * Set absolute tolerance on objective function.
 */
AA_API void
aa_rx_ik_parm_set_tol_obj_abs( struct aa_rx_ik_parm *parm, double tol );

/**
 * Set relative tolerance on objective function.
 */
AA_API void
aa_rx_ik_parm_set_tol_obj_rel( struct aa_rx_ik_parm *parm, double tol );

/**
 * Set damping constant for damped least squares (LU decompisition).
 */
AA_API void
aa_rx_ik_parm_set_k_dls( struct aa_rx_ik_parm *parm, double s2min);

/**
 * Set minimum square singular value for damped least squares (SVD).
 */
AA_API void
aa_rx_ik_parm_set_s2min( struct aa_rx_ik_parm *parm, double s2min);

/**
 * Set angular gain.
 */
AA_API void
aa_rx_ik_parm_set_gain_angle( struct aa_rx_ik_parm *parm, double k );

/**
 * Set translational gain.
 */
AA_API void
aa_rx_ik_parm_set_gain_trans( struct aa_rx_ik_parm *parm, double k );

/**
 * Set maximum interations.
 */
AA_API void
aa_rx_ik_parm_set_max_iterations( struct aa_rx_ik_parm *parm, size_t n );

/**
 * Set frame to solve.
 */
AA_API void
aa_rx_ik_parm_set_frame( struct aa_rx_ik_parm *parm, aa_rx_frame_id frame );

/**
 * Set a reference configuration.
 */
AA_API void
aa_rx_ik_parm_take_config( struct aa_rx_ik_parm *parm, size_t n_q,
                             double *q, enum aa_mem_refop refop );

/**
 * Set a configuration gain (nullspace).
 */
AA_API void
aa_rx_ik_parm_take_gain_config( struct aa_rx_ik_parm *parm, size_t n_q,
                                  double *q, enum aa_mem_refop refop );

/**
 * Set a configuration seed (initial) value.
 */
AA_API void
aa_rx_ik_parm_take_seed( struct aa_rx_ik_parm *parm, size_t n_q,
                           double *q_all, enum aa_mem_refop refop );


/**
 * Set the configuration seed (initial) to be the center position.
 */
AA_API void
aa_rx_ik_parm_center_seed( struct aa_rx_ik_parm *parm,
                             const struct aa_rx_sg_sub *ssg );

/**
 * Convenience function to set IK options to center joints
 */
AA_API void
aa_rx_ik_parm_center_configs( struct aa_rx_ik_parm *parm,
                                const struct aa_rx_sg_sub *ssg,
                                double gain );

/**
 * Print debugging output for IK solver.
 */
AA_API void
aa_rx_ik_parm_set_debug( struct aa_rx_ik_parm *parm, int debug );



/*-- Jacobian IK Solver --*/

AA_API int
aa_rx_ik_jac_x2dq ( const struct aa_rx_ik_parm *parm, size_t n_q,
                    const double *AA_RESTRICT q_act, const double *AA_RESTRICT E_act,
                    const double E_ref[7], const double dx_ref[6],
                    const double *J, double *AA_RESTRICT dq );





/**
 * IK solver context, opaque structure.
 */
struct aa_rx_ik_cx;

/**
 * Create an IK solver context.
 */
AA_API struct aa_rx_ik_cx *
aa_rx_ik_cx_create(const struct aa_rx_sg_sub *ssg, const struct aa_rx_ik_parm *parm );

/**
 * Destroy an IK solver context.
 */
AA_API void
aa_rx_ik_cx_destroy(struct aa_rx_ik_cx *cx);

/**
 * Run the IK solver.
 */
AA_API int
aa_rx_ik_solve( const struct aa_rx_ik_cx *context,
                const struct aa_dmat *TF,
                struct aa_dvec *q );

/**
 * Return reference to the start state used by the IK solver.
 *
 * The start state holds the full scene configuration.
 */
AA_API struct aa_dvec *
aa_rx_ik_get_start( const struct aa_rx_ik_cx *context );

/**
 * Return reference to the seed state used by the IK solver.
 *
 * The seed state holds the sub-scenegraph configuration.
 */
AA_API struct aa_dvec *
aa_rx_ik_get_seed( const struct aa_rx_ik_cx *context );

/**
 * Set the start state used by the IK solver.
 *
 * The start state holds the full scene configuration.
 */
AA_API void
aa_rx_ik_set_start( struct aa_rx_ik_cx *context, const struct aa_dvec *q_start );

/**
 * Set seed used by the IK solver.
 *
 * The seed state holds the sub-scenegraph configuration.
 */
AA_API void
aa_rx_ik_set_seed( struct aa_rx_ik_cx *context, const struct aa_dvec *q_seed );

/**
 * Convenience function to set the seed to the centered configuration.
 */
AA_API void
aa_rx_ik_set_seed_center( struct aa_rx_ik_cx *context );

/**
 * Convenience function to set a random seed.
 */
AA_API void
aa_rx_ik_set_seed_rand( struct aa_rx_ik_cx *context );

/**
 * Set the maximum time limit for IK restarts.
 */
AA_API void
aa_rx_ik_set_restart_time( struct aa_rx_ik_cx *context, double t );

/**
 * Get the maximum time limit for IK restarts.
 */
AA_API double
aa_rx_ik_get_restart_time( struct aa_rx_ik_cx *context );

/**
 * Set the frame to solve for.
 */
AA_API void
aa_rx_ik_set_frame_name( struct aa_rx_ik_cx *context, const char *name );

/**
 * Set the frame to solve for.
 */
AA_API void
aa_rx_ik_set_frame_id( struct aa_rx_ik_cx *context, aa_rx_frame_id id );

/**
 * Function type for optimization objectives and contstraints.
 *
 *
 * @sa aa_rx_ik_opt_err_dqln
 * @sa aa_rx_ik_opt_err_qlnpv
 * @sa aa_rx_ik_opt_err_jcenter
 */
typedef double aa_rx_ik_opt_fun(void *cx, const double *q, double *dq);

/**
 * Set an error objective function for optimization (e.g., SQP) IK.
 *
 * If using a workspace objective, no additional constraints are
 * needed (position limit bound constraints are automatiically added).
 * If using a jointspace objective, a workspace constraint may be
 * necessary.
 *
 * @param parm     options structure
 * @param fun      error function
 * @param tol_abs  absolute tolerance on error
 *
 * @sa aa_rx_ik_opt_err_dqln
 * @sa aa_rx_ik_opt_err_qlnpv
 * @sa aa_rx_ik_opt_err_jcenter
 */
AA_API void
aa_rx_ik_parm_set_obj( struct aa_rx_ik_parm *parm,
                         aa_rx_ik_opt_fun *fun );

/**
 * Set the equality constraint for optimization (e.g., SQP) IK.
 *
 * Consider using a workspace constraint and a jointspace objective.
 *
 * @sa aa_rx_ik_err_dqln
 * @sa aa_rx_ik_err_qlnpv
 */
AA_API void
aa_rx_ik_parm_set_eqct( struct aa_rx_ik_parm *parm,
                          aa_rx_ik_opt_fun *fun,
                          double tol);



/**
 * IK Workspace error
 *
 *
 * Logarithm of the pose error:
 *
 * \f[ f(q) = \| \ln S_{\rm act}(q) \otimes S_{\rm ref} \|^2 \f]
 *
 * @sa aa_rx_ik_parm_set_obj
 * @sa aa_rx_ik_parm_set_eqct
 */
AA_API double
aa_rx_ik_opt_err_dqln( void *cx, double *q, double *dq );

/**
 * IK workspace error (for testing only)
 *
 * This function uses finite difference and will be slow.
 *
 * Logarithm of the pose error:
 *
 * \f[ f(q) = \| \ln S_{\rm act}(q) \otimes S_{\rm ref} \|^2 \f]
 *
 * From: Beeson, Patrick, and Barrett Ames. "TRAC-IK: An open-source
 * library for improved solving of generic inverse kinematics."
 * 2015 IEEE-RAS 15th International Conference on Humanoid Robots
 * (Humanoids). IEEE, 2015.
 *
 * @sa aa_rx_ik_parm_set_obj
 * @sa aa_rx_ik_parm_set_eqct
 */
AA_API double
aa_rx_ik_opt_err_dqln_fd( void *cx, const double *q, double *dq );

/**
 * IK workspace error.
 *
 * Logarithm of the quaternion error, plus translation error
 *
 * \f[
 *    f(q) =    \| \ln h_{\rm act}(q) \otimes h_{\rm ref} \|^2
 *            + \|v_{\rm act}(q) - v_{\rm ref}\|^2
 * \f]
 *
 * @sa aa_rx_ik_parm_set_obj
 * @sa aa_rx_ik_parm_set_eqct
 */
AA_API double
aa_rx_ik_opt_err_qlnpv( void *cx, const double *q, double *dq );

/**
 * IK workspace error (for testing only).
 *
 * This function uses finite difference and will be slow.
 *
 * Logarithm of the quaternion error, plus translation error
 *
 * \f[
 *   f(q) =   \| \ln h_{\rm act}(q) \otimes h_{\rm ref} \|^2
 *          + \|v_{\rm act}(q) - v_{\rm ref}\|^2
 * \f]
 *
 * From: Beeson, Patrick, and Barrett Ames. "TRAC-IK: An open-source
 * library for improved solving of generic inverse kinematics."
 * 2015 IEEE-RAS 15th International Conference on Humanoid Robots
 * (Humanoids). IEEE, 2015.
 *
 * @sa aa_rx_ik_parm_set_obj
 * @sa aa_rx_ik_parm_set_eqct
 *
 */
AA_API double
aa_rx_ik_opt_err_qlnpv_fd( void *cx, const double *q, double *dq );


/**
 * Error from the joint center.
 *
 * Use this function as the objective in conjunction with a workspace
 * error function as the constraint
 *
 * \f[
 *      f(q) = \frac{1}{2} \| \ln q_{\rm act} - q_{\rm center}\|^2
 * \f]
 *
 * @sa aa_rx_ik_parm_set_obj
 */
AA_API double
aa_rx_ik_opt_err_jcenter( void *cx, const double *q, double *dq );


/**
 * Check an IK solution.
 *
 * @return 0 when the IK solution is within tolerances.
 */
AA_API int
aa_rx_ik_check( const struct aa_rx_ik_cx *context,
                const struct aa_dmat *TF,
                struct aa_dvec *q_sub );


struct aa_rx_ik_jac_cx;

/**
 * Create a Jacobian IK solver.
 */
AA_API struct aa_rx_ik_jac_cx *
aa_rx_ik_jac_cx_create(const struct aa_rx_sg_sub *ssg, const struct aa_rx_ik_parm *parm );

/**
 * Destroy a Jacobian IK solver.
 */
AA_API void
aa_rx_ik_jac_cx_destroy( struct aa_rx_ik_jac_cx *cx );


/**
 * Solve the Jacobian IK.
 */
AA_API int aa_rx_ik_jac_solve( const struct aa_rx_ik_jac_cx *context,
                               size_t n_tf, const double *TF, size_t ld_TF,
                               size_t n_q, double *q );

/**
 * Convenience function for Jacobian IK solver.
 */
AA_API int aa_rx_ik_jac_fun( void *context,
                             size_t n_tf, const double *TF, size_t ld_TF,
                             size_t n_q, double *q );



/* AA_API int */
/* aa_rx_sg_sub_ksol_dls( const struct aa_rx_sg_sub *ssg, */
/*                        const struct aa_rx_ik_parm *parm, */
/*                        size_t n_tf, const double *TF, size_t ld_TF, */
/*                        size_t n_q_all, const double *q_start_all, */
/*                        size_t n_q, double *q_subset ); */

/* static inline int */
/* aa_rx_sg_chain_ksol_dls( const struct aa_rx_sg_sub *ssg, */
/*                          const struct aa_rx_ik_parm *parm, */
/*                          const double *TF, */
/*                          size_t n_q_all, const double *q_start_all, */
/*                          size_t n_qs, double *q_subset ) */
/* { */
/*     return aa_rx_sg_sub_ksol_dls( ssg, parm, 1, TF, 7, */
/*                                   n_q_all, q_start_all, */
/*                                   n_qs, q_subset ); */
/* } */


#endif /*AMINO_RX_SCENE_IK_H*/
