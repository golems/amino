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


struct aa_rx_cl_set;

AA_API struct aa_rx_cl_set*
aa_rx_cl_set_create( const struct aa_rx_sg *sg );

AA_API void
aa_rx_cl_set_destroy(struct aa_rx_cl_set *cl_set);

AA_API void
aa_rx_cl_set_set( struct aa_rx_cl_set *cl_set,
                  aa_rx_frame_id i,
                  aa_rx_frame_id j,
                  int is_colliding );

AA_API void
aa_rx_cl_set_fill( struct aa_rx_cl_set *dst,
                   const struct aa_rx_cl_set *src );


AA_API int
aa_rx_cl_set_get( const struct aa_rx_cl_set *cl_set,
                  aa_rx_frame_id i,
                  aa_rx_frame_id j );


struct aa_rx_cl;

AA_API void
aa_rx_sg_cl_init( struct aa_rx_sg *scene_graph );

AA_API struct aa_rx_cl *
aa_rx_cl_create( const struct aa_rx_sg *scene_graph );

AA_API void
aa_rx_cl_destroy( struct aa_rx_cl *cl );

AA_API void
aa_rx_cl_allow( struct aa_rx_cl *cl,
                aa_rx_frame_id id0,
                aa_rx_frame_id id1,
                int allowed );

AA_API void
aa_rx_cl_allow_set( struct aa_rx_cl *cl,
                    const struct aa_rx_cl_set *set );

AA_API void
aa_rx_cl_allow_name( struct aa_rx_cl *cl,
                     const char *frame0,
                     const char *frame1,
                     int allowed );

AA_API int
aa_rx_cl_check( struct aa_rx_cl *cl,
                size_t n_tf,
                const double *TF, size_t ldTF,
                struct aa_rx_cl_set *cl_set );

#endif /*AMINO_RX_SCENE_COLLISION_H*/
