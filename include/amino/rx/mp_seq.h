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

#ifndef AMINO_RX_SCENE_MP_SEQ_H
#define AMINO_RX_SCENE_MP_SEQ_H


struct aa_rx_sg;;
struct aa_rx_mp_seq;

AA_API struct aa_rx_mp_seq *
aa_rx_mp_seq_create();


AA_API void
aa_rx_mp_seq_destroy( struct aa_rx_mp_seq * );

AA_API void
aa_rx_mp_seq_append_all( struct aa_rx_mp_seq * mp_seq,
                         const struct aa_rx_sg *sg,
                         size_t n_path, const double *q_all_path );


AA_API size_t
aa_rx_mp_seq_count( struct aa_rx_mp_seq * mp_seq );


AA_API size_t
aa_rx_mp_seq_point_count( struct aa_rx_mp_seq * mp_seq );

AA_API void
aa_rx_mp_seq_elt( struct aa_rx_mp_seq * mp_seq, size_t i,
                  const struct aa_rx_sg **sg_ptr,
                  size_t *n_path_ptr,
                  const double **q_all_path_ptr );


AA_API void
aa_rx_mp_seq_qref( struct aa_rx_mp_seq * mp_seq, size_t i,
                   const struct aa_rx_sg **sg_ptr,
                   const double **q_all_ptr );

#endif /*AMINO_RX_SCENE_MP_SEQ_H */
