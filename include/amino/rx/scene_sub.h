/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015-2016, Rice University
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

#ifndef AMINO_RX_SCENE_SUB_H
#define AMINO_RX_SCENE_SUB_H

#include "scenegraph.h"

/**
 * @file scene_sub.h
 * @brief Sub-scenegraphs
 */




/**
 * A scenegraph subset.
 */
struct aa_rx_sg_sub;

/**
 * Destroy the scengraph subset.
 *
 * The original scenegraph will remain valid.
 */
AA_API void
aa_rx_sg_sub_destroy( struct aa_rx_sg_sub *sg );

/**
 * Return the original scene graph for the sub-scenegraph
 */
AA_API const struct aa_rx_sg *
aa_rx_sg_sub_sg( const struct aa_rx_sg_sub *sg_sub );

/**
 * Return the number of configuration variables in the scenegraph subset.
 */
AA_API size_t
aa_rx_sg_sub_config_count( const struct aa_rx_sg_sub *sg_sub );


/**
 * Return the number of configuration variables in the full scenegraph.
 */
AA_API size_t
aa_rx_sg_sub_all_config_count( const struct aa_rx_sg_sub *sg_sub );

/**
 * Return the number of frames in the scenegraph subset.
 */
AA_API size_t
aa_rx_sg_sub_frame_count( const struct aa_rx_sg_sub *sg_sub );

/**
 * Return the number of frames in the full scenegraph.
 */
AA_API size_t
aa_rx_sg_sub_all_frame_count( const struct aa_rx_sg_sub *sg_sub );

/**
 * Return the full scenegraph config id for the i'th configuration of the sub-scenegraph.
 */
AA_API aa_rx_config_id
aa_rx_sg_sub_config( const struct aa_rx_sg_sub *sg_sub, size_t i );

/**
 * Return the full scenegraph frame id for the i'th frame of the sub-scenegraph.
 */
AA_API aa_rx_frame_id
aa_rx_sg_sub_frame( const struct aa_rx_sg_sub *sg_sub, size_t i );


/**
 * Return the end-effector frame id, if any
 */
AA_API aa_rx_frame_id
aa_rx_sg_sub_frame_ee( const struct aa_rx_sg_sub *sg_sub );


/**
 * Return the array of full scenegraph config ids contained in the sub-scenegraph.
 */
AA_API aa_rx_config_id*
aa_rx_sg_sub_configs( const struct aa_rx_sg_sub *sg_sub );

/**
 * Return the array of full scenegraph frame ids contained in the sub-scenegraph.
 */
AA_API aa_rx_frame_id*
aa_rx_sg_sub_frames( const struct aa_rx_sg_sub *sg_sub );


AA_API void
aa_rx_sg_sub_config_get(
    const struct aa_rx_sg_sub *ssg,
    size_t n_all, const double *config_all,
    size_t n_subset, double *config_subset );

AA_API void
aa_rx_sg_sub_config_set(
    const struct aa_rx_sg_sub *ssg,
    size_t n_sub, const double *config_subset,
    size_t n_all, double *config_all
    );


/**
 * Create a sub-scenegraph for the kinematic chain starting at root and ending a tip.
 */
AA_API struct aa_rx_sg_sub *
aa_rx_sg_chain_create( const struct aa_rx_sg *sg,
                       aa_rx_frame_id root, aa_rx_frame_id tip );


/**
 * Fill q with the centered positions of each configuration.
 */
AA_API void
aa_rx_sg_sub_center_configs( const struct aa_rx_sg_sub *ssg,
                             size_t n, double *q );

/*-- Jacobians --*/

/**
 * Determine the size of the Jacobian matrix for the sub-scenegraph.
 */
AA_API void
aa_rx_sg_sub_jacobian_size( const struct aa_rx_sg_sub *ssg,
                            size_t *rows, size_t *cols );

/**
 * Compute the Jacobian matrix for the sub-scenegraph.
 */
AA_API void
aa_rx_sg_sub_jacobian( const struct aa_rx_sg_sub *ssg,
                       size_t n_tf, const double *TF_abs, size_t ld_TF,
                       double *J, size_t ld_J );

/**
 * Allocate and computer the Jacobian matrix for the sub-scenegraph.
 */
AA_API struct aa_dmat *
aa_rx_sg_sub_jacobian_alloc( const struct aa_rx_sg_sub *ssg,
                             struct aa_mem_region *reg,
                             struct aa_dmat *TF_abs );

/**
 * Allocate jacobian matrix for sub scene graph.
 */
AA_API double *
aa_rx_sg_sub_alloc_jacobian( const struct aa_rx_sg_sub *ssg, struct aa_mem_region *region );

/**
 * Allocate sub scene graph config array.
 */
AA_API double *
aa_rx_sg_sub_alloc_config( const struct aa_rx_sg_sub *ssg, struct aa_mem_region *region );


/**
 * Expand sub path to full config path.
 *
 * @param ssg       Sub scene graph for the given path
 * @param n_pts     Number of points in the path
 * @param q_start   A start configuration for joints not in the
 *                  sub-scenegraph, or NULL to ignore
 * @param path_sub  Path for the sub-scenegraph
 * @param path_all  The full scene path
 *
 */
AA_API void
aa_rx_sg_sub_expand_path( const struct aa_rx_sg_sub *ssg, size_t n_pts,
                          const double *q_start,
                          const double *path_sub,
                          double *path_all );

#endif /*AMINO_RX_SCENE_SUB_H*/
