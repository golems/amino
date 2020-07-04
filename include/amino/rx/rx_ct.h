/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2017, Rice University
 * Copyright (c) 2018, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@mines.edu>
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

#ifndef AMINO_RX_CT_H
#define AMINO_RX_CT_H

/**
 * @file rx_ct.h
 * @brief Integration between scene graphs and control
 */

#include "amino/ct/state.h"
#include "amino/ct/traj.h"

/**
 * Return a point list for a path.
 *
 * @param region   Memory region to allocate point list from
 * @param n_q      Number of configuration variables
 * @param n_path   Number of waypoints in the path
 * @param path     Path data
 */
AA_API struct aa_ct_pt_list *
aa_rx_ct_pt_list( struct aa_mem_region *region,
                     size_t n_q, size_t n_path, double *path );


/**
 * Extract the limits from a scene graph.
 *
 * @param region  Memory region to allocate limits from
 * @param sg      The scene graph
 */
AA_API struct aa_ct_limit *
aa_rx_ct_limits( struct aa_mem_region *region, const struct aa_rx_sg *sg );


AA_API int
aa_rx_ct_tjx_path( struct aa_mem_region *region,
                   const struct aa_rx_ik_parm *opts,
                   const struct aa_rx_sg_sub *ssg,
                   struct aa_ct_seg_list *segs,
                   size_t n_q_all, const double *q_start_all,
                   size_t *n_points, double **path );


AA_API struct aa_ct_limit *
aa_rx_ct_sg_limits( struct aa_mem_region *region, const struct aa_rx_sg *sg );

/**
 * @struct aa_rx_ct_wk_opts
 * Opaque structure for workspace control options.
 */
struct aa_rx_ct_wk_opts;

/**
 * Create workspce control options.
 */
AA_API struct aa_rx_ct_wk_opts *
aa_rx_ct_wk_opts_create(void);

/**
 * Destroy workspce control options.
 */
AA_API void
aa_rx_ct_wk_opts_destroy( struct aa_rx_ct_wk_opts * );


/**
 * Proportional control on pose error.
 *
 * The computed reference term is ADDED to dx.
 *
 * @param[in] opts Workspace control options
 * @param[in] E_act Actual pose (quaternion-translation)
 * @param[in] E_ref Reference pose (quaternion-translation)
 * @param[inout] dx Reference workspace velocity
 */
AA_API void
aa_rx_ct_wk_dx_pos( const struct aa_rx_ct_wk_opts * opts,
                    const double *E_act, const double *E_ref,
                    double *dx );

/**
 * Convert workspace (Cartesian) velocity to joint velocity.
 *
 * @param[in] ssg the subscenegraph to control
 * @param[in] opts workspace control options
 * @param[in] n_tf number of frames in TF_abs
 * @param[in] TF_abs absolute frames (quaternion-translations) for the entire scenegraph
 * @param[in] ld_tf Leading dimension of TF_abs
 * @param[in] n_x size of dx
 * @param[in] dx reference workspace velocity
 * @param[in] n_q size of dq
 * @param[out] dq reference joint velocity for subscenegraph
 */
AA_API int
aa_rx_ct_wk_dx2dq( const struct aa_rx_sg_sub *ssg,
                   const struct aa_rx_ct_wk_opts * opts,
                   size_t n_tf, const double *TF_abs, size_t ld_tf,
                   size_t n_x, const double *dx,
                   size_t n_q, double *dq );

/**
 * Convert workspace (Cartesian) velocity to joint velocity, with
 * nullspace projection
 *
 * @see aa_rx_ct_wk_dqcenter()
 *
 * @param[in] ssg the subscenegraph to control
 * @param[in] opts workspace control options
 * @param[in] n_tf number of frames in TF_abs
 * @param[in] TF_abs absolute frames (quaternion-translations) for the entire scenegraph
 * @param[in] ld_tf Leading dimension of TF_abs
 * @param[in] n_x size of dx
 * @param[in] dx reference workspace velocity
 * @param[in] n_q size of dq
 * @param[in] dq_r reference joint velocity (nullspace projected) for subscenegraph
 * @param[out] dq computed reference joint velocity for subscenegraph
 */
AA_API int
aa_rx_ct_wk_dx2dq_np( const struct aa_rx_sg_sub *ssg,
                     const struct aa_rx_ct_wk_opts * opts,
                     size_t n_tf, const double *TF_abs, size_t ld_tf,
                     size_t n_x, const double *dx,
                     size_t n_q, const double *dq_r, double *dq );


/**
 * @struct aa_rx_ct_wk_lc3_cx;
 *
 * Opaque context struct for LC3.
 *
 *  Z. Kingston, N. Dantam, and L. Kavraki.
 * [Kinematically Constrained  Workspace Control via Linear  Optimization]
 * (http://dx.doi.org/10.1109/HUMANOIDS.2015.7363455). International
 *  Conference on Humanoid Robots (Humanoids), IEEE. 2015.
 *
 */
struct aa_rx_ct_wk_lc3_cx;

AA_API  struct aa_rx_ct_wk_lc3_cx *
aa_rx_ct_wk_lc3_create ( const struct aa_rx_sg_sub *ssg,
                         const struct aa_rx_ct_wk_opts * opts );

AA_API int
aa_rx_ct_wk_dx2dq_lc3( const struct aa_rx_ct_wk_lc3_cx *lc3,
                       double dt,
                       size_t n_tf, const double *TF_abs, size_t ld_tf,
                       size_t n_x, const double *dx_r,
                       size_t n_q,
                       const double *q_a, const double *dq_a,
                       const double *dq_r, double *dq );

/**
 * Find joint-centering reference velocity.
 *
 * @param[in] ssg the subscenegraph to control
 * @param[in] opts workspace control options
 * @param[in] n_q size of q
 * @param[in] q actual joint positions for the subscenegraph
 * @param[out] dq_r centering velocities for the subscenegraph
 */
AA_API void
aa_rx_ct_wk_dqcenter( const struct aa_rx_sg_sub *ssg,
                      const struct aa_rx_ct_wk_opts * opts,
                      size_t n_q, const double *q, double *dq_r );


#endif /*AMINO_RX_CT_H*/
