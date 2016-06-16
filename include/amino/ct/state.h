/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
 * All rights reserved.
 *
 * Author(s): Zachary K. Kingston <zak@rice.edu>
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

#ifndef AMINO_CT_STATE_H
#define AMINO_CT_STATE_H

/**
 * @file state.h
 */

#define AA_CT_ST_Q     (1 << 0)
#define AA_CT_ST_DQ    (1 << 1)
#define AA_CT_ST_DDQ   (1 << 2)
#define AA_CT_ST_EFF   (1 << 3)
#define AA_CT_ST_TFABS (1 << 4)
#define AA_CT_ST_TFREL (1 << 5)

#define AA_CT_ST_QS    AA_CT_ST_Q
#define AA_CT_ST_TFS   AA_CT_ST_TFABS

#define AA_CT_ST_ON(active, f) (active & (uint32_t) f)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * State description of a robot
 */
struct aa_ct_state {
    size_t n_q;             ///< Number of configuration variables
    union {
        struct {
            double *q;      ///< Position
            double *dq;     ///< Velocity
            double *ddq;    ///< Acceleration
            double *eff;    ///< Efforts
        };
        double *qs[4];      ///< Jointspace vectors
    };

    size_t n_tf;            ///< Number of frames
    union {
        struct {
            double *TF_abs; ///< Absolute frame transforms
            double *TF_rel; ///< Relative frame transforms
        };
        double *tfs[2];     /// Frame transforms
    };

    uint32_t active;        ///< Active fields
};

void aa_ct_state_clone(struct aa_mem_region *reg, struct aa_ct_state *dest,
                       struct aa_ct_state *src);

struct aa_ct_state *aa_ct_state_create(struct aa_mem_region *reg, size_t n_q,
                                       size_t n_tf, uint32_t active);
#ifdef __cplusplus
}
#endif

#endif /*AMINO_CT_STATE_H*/
