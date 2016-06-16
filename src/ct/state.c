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

#include <stdint.h>
#include <string.h>

#include <amino.h>
#include <amino/ct/state.h>


AA_API void
aa_ct_state_clone(struct aa_mem_region *reg, struct aa_ct_state *dest,
                  struct aa_ct_state *src)
{
    dest->n_q = src->n_q;
    for (size_t i = 0; i < 4; i++) {
        if (AA_CT_ST_ON(src->active, (1 << i))) {
            dest->qs[i] = AA_MEM_REGION_NEW_N(reg, double, src->n_q);
            AA_MEM_CPY(dest->qs[i], src->qs[i], src->n_q);
        }
    }

    dest->n_tf = src->n_tf;
    for (size_t i = 4; i < 6; i++) {
        if (AA_CT_ST_ON(src->active, (1 << i))) {
            dest->tfs[i] = AA_MEM_REGION_NEW_N(reg, double, 7 * src->n_tf);
            AA_MEM_CPY(dest->tfs[i], src->tfs[i], 7 * src->n_tf);
        }
    }

    dest->active = src->active;
}

AA_API struct aa_ct_state *
aa_ct_state_create(struct aa_mem_region *reg, size_t n_q, size_t n_tf,
                   uint32_t active)
{
    struct aa_ct_state *st = AA_MEM_REGION_NEW(reg, struct aa_ct_state);

    st->n_q = n_q;
    if (n_q)
        for (size_t i = 0; i < 4; i++) {
            if (AA_CT_ST_ON(active, (1 << i))) {
                st->qs[i] = AA_MEM_REGION_NEW_N(reg, double, n_q);
                bzero(st->qs[i], sizeof(double) * n_q);
            }
        }

    st->n_tf = n_tf;
    if (n_tf)
        for (size_t i = 4; i < 6; i++) {
            if (AA_CT_ST_ON(active, (1 << i))) {
                st->tfs[i] = AA_MEM_REGION_NEW_N(reg, double, 7 * n_tf);
                bzero(st->tfs[i], sizeof(double) * 7 * n_tf);
            }
        }

    return st;
}
