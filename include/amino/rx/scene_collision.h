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

#ifndef AMINO_RX_SCENE_COLLISION_H
#define AMINO_RX_SCENE_COLLISION_H

/**
 * @file scene_collision.h
 * @brief Collision checking
 */

/**
 * Initialize collision handling.
 */
AA_API void
aa_rx_cl_init( );

/**
 * Opaque type for a set of collisions.
 *
 * This type represents a set of pairs of frames.
 */
struct aa_rx_cl_set;

/**
 * Create a collision set.
 */
AA_API struct aa_rx_cl_set*
aa_rx_cl_set_create( const struct aa_rx_sg *sg );

/**
 * Destroy a collision set.
 */
AA_API void
aa_rx_cl_set_destroy(struct aa_rx_cl_set *cl_set);

/**
 * Set the value of collision between frames i and j.
 */
AA_API void
aa_rx_cl_set_set( struct aa_rx_cl_set *cl_set,
                  aa_rx_frame_id i,
                  aa_rx_frame_id j,
                  int is_colliding );

/**
 * Fill dst with all true entries in src.
 *
 * False entries in src have no effect corresponding entries in dst.
 */
AA_API void
aa_rx_cl_set_fill( struct aa_rx_cl_set *dst,
                   const struct aa_rx_cl_set *src );

/**
 * Return the value of collision between frames i and j.
 */
AA_API int
aa_rx_cl_set_get( const struct aa_rx_cl_set *cl_set,
                  aa_rx_frame_id i,
                  aa_rx_frame_id j );

/**
 * Fill set `into' with all elements in set `from'.
 *
 * This is a union operation, with the result stored in `into'.
 */
AA_API void
aa_rx_cl_set_merge(struct aa_rx_cl_set* into, const struct aa_rx_cl_set* from);

/**
 * Opaque type for collision detection context.
 */
struct aa_rx_cl;

/**
 * Initialize the collision structures within scene_graph.
 */
AA_API void
aa_rx_sg_cl_init( struct aa_rx_sg *scene_graph );

/**
 * Create a new collision detection context for scene_graph.
 */
AA_API struct aa_rx_cl *
aa_rx_cl_create( const struct aa_rx_sg *scene_graph );

/**
 * Destroy a collision detection context.
 */
AA_API void
aa_rx_cl_destroy( struct aa_rx_cl *cl );

/**
 * Allow (ignore) collisions between frames i and j if allowed is true.
 */
AA_API void
aa_rx_cl_allow( struct aa_rx_cl *cl,
                aa_rx_frame_id i,
                aa_rx_frame_id j,
                int allowed );

/**
 * Allow collisions between all frame pairs in set.
 */
AA_API void
aa_rx_cl_allow_set( struct aa_rx_cl *cl,
                    const struct aa_rx_cl_set *set );

/**
 * Allow (ignore) collisions between frames0 and frame1 if allowed is true.
 */
AA_API void
aa_rx_cl_allow_name( struct aa_rx_cl *cl,
                     const char *frame0,
                     const char *frame1,
                     int allowed );

/**
 * Detect collisions.
 *
 * If cl_set is non-NULL, it will be filled in with all detected collisions.
 * If cl_set is NULL, collision checking may short-circuit after the first collision is detected.
 *
 * @returns 0 if no collisions are detected and non-zero if any collisions are detected.
 */
AA_API int
aa_rx_cl_check( struct aa_rx_cl *cl,
                size_t n_tf,
                const double *TF, size_t ldTF,
                struct aa_rx_cl_set *cl_set );

/**
 * Allow all collisions at configuration q.
 */
AA_API void
aa_rx_sg_allow_config( struct aa_rx_sg* scene_graph, size_t n_q, const double* q);

/**
 * Retrieve the set of allowed collisions.
 */
AA_API void
aa_rx_sg_cl_set_copy(const struct aa_rx_sg* sg, struct aa_rx_cl_set * cl_set);

/**
 * Check the collisions at q.
 */
AA_API void
aa_rx_sg_get_collision(const struct aa_rx_sg* scene_graph, size_t n_q, const double* q, struct aa_rx_cl_set* cl_set);

#endif /*AMINO_RX_SCENE_COLLISION_H*/
