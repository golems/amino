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

#include <string.h>

#include <amino.h>
#include <amino/ct/state.h>

AA_API void
aa_ct_state_clone(struct aa_mem_region *reg, struct aa_ct_state *dest,
                  struct aa_ct_state *src)
{
    dest->n_q = src->n_q;
    dest->n_tf = src->n_tf;

    if (src->q) {
        dest->q = AA_MEM_REGION_NEW_N(reg, double, src->n_q);
        AA_MEM_CPY(dest->q, src->q, src->n_q);
    }

    if (src->dq) {
        dest->dq = AA_MEM_REGION_NEW_N(reg, double, src->n_q);
        AA_MEM_CPY(dest->dq, src->dq, src->n_q);
    }

    if (src->ddq) {
        dest->ddq = AA_MEM_REGION_NEW_N(reg, double, src->n_q);
        AA_MEM_CPY(dest->ddq, src->ddq, src->n_q);
    }

    if (src->eff) {
        dest->eff = AA_MEM_REGION_NEW_N(reg, double, src->n_q);
        AA_MEM_CPY(dest->eff, src->eff, src->n_q);
    }

    if (src->TF_abs) {
        dest->TF_abs = AA_MEM_REGION_NEW_N(reg, double, src->n_tf);
        AA_MEM_CPY(dest->TF_abs, src->TF_abs, src->n_tf);
    }

    if (src->TF_rel) {
        dest->TF_rel = AA_MEM_REGION_NEW_N(reg, double, src->n_tf);
        AA_MEM_CPY(dest->TF_rel, src->TF_rel, src->n_tf);
    }
}
