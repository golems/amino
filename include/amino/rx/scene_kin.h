/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
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

#ifndef AMINO_RX_SCENE_KIN_H
#define AMINO_RX_SCENE_KIN_H

#include "scenegraph.h"

/**
 * @file scene_kin.h
 * @brief Scenegraph kinematics
 */

struct aa_rx_sg_sub;

/*-- Kinematic Solvers --*/

/**
 * General type for an IK solver function
 */
typedef int aa_rx_ik_fun( void *context,
                          size_t n_tf, const double *TF, size_t ld_TF,
                          size_t n_q, double *q );


struct aa_rx_ksol_opts;

/**
 * Create options struct for kinematic solver.
 */
AA_API struct aa_rx_ksol_opts*
aa_rx_ksol_opts_create();

/**
 * Destroy options struct for kinematic solver.
 */
AA_API void
aa_rx_ksol_opts_destroy( struct aa_rx_ksol_opts *opts);

/**
 * Set integration step.
 */
AA_API void
aa_rx_ksol_opts_set_dt( struct aa_rx_ksol_opts *opts, double dt);

/**
 * Set angular tolerance.
 */
AA_API void
aa_rx_ksol_opts_set_tol_angle( struct aa_rx_ksol_opts *opts, double tol);

/**
 * Set translational tolerance.
 */
AA_API void
aa_rx_ksol_opts_set_tol_trans( struct aa_rx_ksol_opts *opts, double tol);

/**
 * Set angular tolerance to switch to SVD.
 */
AA_API void
aa_rx_ksol_opts_set_tol_angle_svd( struct aa_rx_ksol_opts *opts, double tol);

/**
 * Set translational tolerance to switch to SVD.
 */
AA_API void
aa_rx_ksol_opts_set_tol_trans_svd( struct aa_rx_ksol_opts *opts, double tol);

/**
 * Set tolerance on joint motion.
 */
AA_API void
aa_rx_ksol_opts_set_tol_dq( struct aa_rx_ksol_opts *opts, double tol);

/**
 * Set damping constant for damped least squares (LU decompisition).
 */
AA_API void
aa_rx_ksol_opts_set_tol_k_dls( struct aa_rx_ksol_opts *opts, double s2min);

/**
 * Set minimum square singular value for damped least squares (SVD).
 */
AA_API void
aa_rx_ksol_opts_set_tol_s2min( struct aa_rx_ksol_opts *opts, double s2min);

/**
 * Set angular gain.
 */
AA_API void
aa_rx_ksol_opts_set_gain_angle( struct aa_rx_ksol_opts *opts, double k );

/**
 * Set translational gain.
 */
AA_API void
aa_rx_ksol_opts_set_gain_trans( struct aa_rx_ksol_opts *opts, double k );

/**
 * Set maximum interations.
 */
AA_API void
aa_rx_ksol_opts_set_max_iterations( struct aa_rx_ksol_opts *opts, size_t n );

/**
 * Set frame to solve.
 */
AA_API void
aa_rx_ksol_opts_set_frame( struct aa_rx_ksol_opts *opts, aa_rx_frame_id frame );

/**
 * Set a reference configuration.
 */
AA_API void
aa_rx_ksol_opts_take_config( struct aa_rx_ksol_opts *opts, size_t n_q,
                             double *q, enum aa_mem_refop refop );

/**
 * Set a configuration gain (nullspace).
 */
AA_API void
aa_rx_ksol_opts_take_gain_config( struct aa_rx_ksol_opts *opts, size_t n_q,
                                  double *q, enum aa_mem_refop refop );

/**
 * Set a configuration seed (initial) value.
 */
AA_API void
aa_rx_ksol_opts_take_seed( struct aa_rx_ksol_opts *opts, size_t n_q,
                           double *q_all, enum aa_mem_refop refop );


/**
 * Set the configuration seed (initial) to be the center position.
 */
AA_API void
aa_rx_ksol_opts_center_seed( struct aa_rx_ksol_opts *opts,
                             const struct aa_rx_sg_sub *ssg );

/**
 * Convenience function to set IK options to center joints
 */
AA_API void
aa_rx_ksol_opts_center_configs( struct aa_rx_ksol_opts *opts,
                                const struct aa_rx_sg_sub *ssg,
                                double gain );



/*-- Jacobian IK Solver --*/

AA_API int
aa_rx_ik_jac_x2dq ( const struct aa_rx_ksol_opts *opts, size_t n_q,
                    const double *AA_RESTRICT q_act, const double *AA_RESTRICT E_act,
                    const double E_ref[7], const double dx_ref[6],
                    const double *J, double *AA_RESTRICT dq );


struct aa_rx_ik_jac_cx;

/**
 * Create a Jacobian IK solver.
 */
AA_API struct aa_rx_ik_jac_cx *
aa_rx_ik_jac_cx_create(const struct aa_rx_sg_sub *ssg, const struct aa_rx_ksol_opts *opts );

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



/*-- NLOPT IK Solver --*/
AA_API int
aa_rx_ik_lopt_solve(const struct aa_rx_sg_sub *ssg, const struct aa_rx_ksol_opts *opts,
                    size_t n_tf, const double *TF, size_t ld_TF,
                    size_t n_qs, double *q_sol);



/* AA_API int */
/* aa_rx_sg_sub_ksol_dls( const struct aa_rx_sg_sub *ssg, */
/*                        const struct aa_rx_ksol_opts *opts, */
/*                        size_t n_tf, const double *TF, size_t ld_TF, */
/*                        size_t n_q_all, const double *q_start_all, */
/*                        size_t n_q, double *q_subset ); */

/* static inline int */
/* aa_rx_sg_chain_ksol_dls( const struct aa_rx_sg_sub *ssg, */
/*                          const struct aa_rx_ksol_opts *opts, */
/*                          const double *TF, */
/*                          size_t n_q_all, const double *q_start_all, */
/*                          size_t n_qs, double *q_subset ) */
/* { */
/*     return aa_rx_sg_sub_ksol_dls( ssg, opts, 1, TF, 7, */
/*                                   n_q_all, q_start_all, */
/*                                   n_qs, q_subset ); */
/* } */


#endif /*AMINO_RX_SCENE_KIN_H*/
