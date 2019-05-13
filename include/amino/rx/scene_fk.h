/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
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

#ifndef AMINO_SCENE_FK_H
#define AMINO_SCENE_FK_H

#include "scenegraph.h"
#include "rxtype.h"

/**
 * @file scene_fk.h
 * @brief Forward kinematics
 */


/**
 * Opaque type for forward kinematics result.
 */
struct aa_rx_fk;

/**
 * Region allocate a forward kinematics struct.
 */
AA_API struct aa_rx_fk *
aa_rx_fk_alloc(const struct aa_rx_sg *scene_graph, struct aa_mem_region *reg);

/**
 * Heap allocate a forward kinematics struct.
 */
AA_API struct aa_rx_fk *
aa_rx_fk_malloc(const struct aa_rx_sg *scene_graph);

/**
 * Copy a forward kinematics struct.
 */
AA_API void
aa_rx_fk_cpy(struct aa_rx_fk *dst, const struct aa_rx_fk *src);

/** Pointer to FK data */
AA_API double *
aa_rx_fk_data( const struct aa_rx_fk *fk );

/** Leading dimension of FK data */
AA_API size_t
aa_rx_fk_ld( const struct aa_rx_fk *fk );

/** Number of frames in the FK */
AA_API size_t
aa_rx_fk_cnt( const struct aa_rx_fk *fk );

/**
 * Destroy a malloc'ed struct aa_rx_fk.
 */
AA_API void
aa_rx_fk_destroy(struct aa_rx_fk * fk);

/**
 * Compute the forward kinematics.
 */
AA_API void
aa_rx_fk_all( struct aa_rx_fk *fk,
              const struct aa_dvec *q );

/**
 * Reference to internal storage for a transform
 */
AA_API double *
aa_rx_fk_ref(const struct aa_rx_fk *fk, aa_rx_frame_id id);

/**
 * Copy an absolute TF out of fk.
 */
AA_API void
aa_rx_fk_get_abs_qutr(const struct aa_rx_fk *fk, aa_rx_frame_id id, double E[7]);

/**
 * Update relative transform in the FK.
 */
AA_API void
aa_rx_fk_set_rel(struct aa_rx_fk *fk, aa_rx_frame_id id, const double E[AA_RX_TF_LEN]);

/**
 * Rotate a point to absolute coordinates.
 */
AA_API void
aa_rx_fk_rot_abs( const struct aa_rx_fk *fk, aa_rx_frame_id id,
                  const double *v_id, double *v_abs );

#endif /*AMINO_SCENE_FK_H*/
