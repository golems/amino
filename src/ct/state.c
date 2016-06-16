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

// Names of fields in order
static const char * const aa_ct_st_name[] = \
{ "Q",
  "dQ",
  "ddQ",
  "Eff",
  "TFabs",
  "TFrel",
  "X",
  "dX",
  "ddX" };

AA_API void
aa_ct_state_clone(struct aa_mem_region *reg, struct aa_ct_state *dest,
                  struct aa_ct_state *src)
{
    dest->active = src->active;
    dest->n_q = src->n_q;
    dest->n_tf = src->n_tf;

    for (size_t i = 0; i < 4; i++)
        if (AA_CT_ST_ON(src->active, (1 << (i + 0)))) {
            dest->qs[i] = AA_MEM_REGION_NEW_N(reg, double, src->n_q);
            AA_MEM_CPY(dest->qs[i], src->qs[i], src->n_q);
        }

    for (size_t i = 0; i < 2; i++)
        if (AA_CT_ST_ON(src->active, (1 << (i + 4)))) {
            dest->tfs[i] = AA_MEM_REGION_NEW_N(reg, double, 7 * src->n_tf);
            AA_MEM_CPY(dest->tfs[i], src->tfs[i], 7 * src->n_tf);
        }

    for (size_t i = 0; i < 3; i++) 
        if (AA_CT_ST_ON(src->active, (1 << (i + 6)))) {
            dest->Xs[i] = AA_MEM_REGION_NEW_N(reg, double, 7);
            AA_MEM_CPY(dest->Xs[i], src->Xs[i], 7);
        }
}

AA_API struct aa_ct_state *
aa_ct_state_create(struct aa_mem_region *reg, size_t n_q, size_t n_tf,
                   uint32_t active)
{
    struct aa_ct_state *st = AA_MEM_REGION_NEW(reg, struct aa_ct_state);
    st->active = active;
    st->n_q = n_q;
    st->n_tf = n_tf;

    for (size_t i = 0; i < 4; i++)
        if (AA_CT_ST_ON(active, (1 << (i + 0)))) {
            st->qs[i] = AA_MEM_REGION_NEW_N(reg, double, n_q);
            bzero(st->qs[i], sizeof(double) * n_q);
        }

    for (size_t i = 0; i < 2; i++)
        if (AA_CT_ST_ON(active, (1 << (i + 4)))) {
            st->tfs[i] = AA_MEM_REGION_NEW_N(reg, double, 7 * n_tf);
            bzero(st->tfs[i], sizeof(double) * 7 * n_tf);
        }

    for (size_t i = 0; i < 3; i++)
        if (AA_CT_ST_ON(active, (1 << (i + 6)))) {
            st->Xs[i] = AA_MEM_REGION_NEW_N(reg, double, 7);
            bzero(st->Xs[i], sizeof(double) * 7);
        }

    return st;
}

AA_API void
aa_ct_state_dump(FILE *stream, struct aa_ct_state *st)
{
    for (size_t i = 0; i < 4; i++)
        if (AA_CT_ST_ON(st->active, (1 << (i + 0)))) {
            fprintf(stream, "%5s: ", aa_ct_st_name[i]);
            aa_dump_vec(stream, st->qs[i], st->n_q);
        }

    for (size_t i = 0; i < 2; i++)
        if (AA_CT_ST_ON(st->active, (1 << (i + 4)))) {
            fprintf(stream, "%5s: ", aa_ct_st_name[i]);
            aa_dump_vec(stream, st->tfs[i], st->n_tf * 7);
        }

    if (AA_CT_ST_ON(st->active, (1 << 6))) {
        fprintf(stream, "%5s: ", aa_ct_st_name[6]);
        aa_dump_vec(stream, st->X, 7);
    }

    for (size_t i = 1; i < 3; i++) 
        if (AA_CT_ST_ON(st->active, (1 << (i + 6)))) {
            fprintf(stream, "%5s: ", aa_ct_st_name[i]);
            aa_dump_vec(stream, st->Xs[i], 6);
        }
}

AA_API int
aa_ct_state_eq(struct aa_ct_state *s1, struct aa_ct_state *s2)
{
    if (s1->n_q != s2->n_q || s1->n_tf != s2->n_tf || s1->active != s2->active)
        return 0;

    for (size_t i = 0; i < 4; i++)
        if (AA_CT_ST_ON(s1->active, (1 << (i + 0))))
            if (!aa_veq(s1->n_q, s1->qs[i], s2->qs[i], AA_EPSILON))
                return 0;

    for (size_t i = 0; i < 2; i++)
        if (AA_CT_ST_ON(s1->active, (1 << (i + 4))))
            if (!aa_veq(s1->n_tf * 7, s1->tfs[i], s2->tfs[i], AA_EPSILON))
                return 0;

    if (AA_CT_ST_ON(s1->active, (1 << 6)))
        if (!aa_veq(7, s1->X, s2->X, AA_EPSILON))
            return 0;

    for (size_t i = 1; i < 3; i++) 
        if (AA_CT_ST_ON(s1->active, (1 << (i + 6))))
            if (!aa_veq(6, s1->Xs[i], s2->Xs[i], AA_EPSILON))
                return 0;

    return 1;
}
