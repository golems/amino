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


AA_API struct aa_ct_state *
aa_ct_state_alloc(struct aa_mem_region *reg, size_t n_q, size_t n_tf )
{
    struct aa_ct_state *s = AA_MEM_REGION_NEW(reg, struct aa_ct_state);
    AA_MEM_ZERO(s,1);
    s->n_q = n_q;
    s->n_tf = n_tf;

    s->q = AA_MEM_REGION_NEW_N(reg, double, s->n_q);
    s->dq = AA_MEM_REGION_NEW_N(reg, double, s->n_q);
    s->ddq = AA_MEM_REGION_NEW_N(reg, double, s->n_q);

    s->eff = AA_MEM_REGION_NEW_N(reg, double, s->n_q);

    s->X = AA_MEM_REGION_NEW_N(reg, double, 7);
    s->dX = AA_MEM_REGION_NEW_N(reg, double, 6);
    s->ddX = AA_MEM_REGION_NEW_N(reg, double, 6);

    s->TF_abs = AA_MEM_REGION_NEW_N(reg, double, s->n_tf);
    s->TF_rel = AA_MEM_REGION_NEW_N(reg, double, s->n_tf);

    return s;
}

static double *state_clone_helper( struct aa_mem_region *reg, const double *src, size_t cnt )
{
    return (src
            ? AA_MEM_REGION_DUP(reg, double, src, cnt)
            : NULL
        );

}

AA_API void
aa_ct_state_clone(struct aa_mem_region *reg, struct aa_ct_state *dest,
                  struct aa_ct_state *src)
{
    dest->n_q = src->n_q;
    dest->n_tf = src->n_tf;

    dest->q = state_clone_helper(reg, src->q, src->n_q);

    dest->dq = state_clone_helper(reg, src->dq, src->n_q);

    dest->ddq = state_clone_helper(reg, src->ddq, src->n_q);

    dest->X = state_clone_helper(reg, src->X, 7);

    dest->dX = state_clone_helper(reg, src->dX, 6);

    dest->ddX = state_clone_helper(reg, src->ddX, 6);

    dest->eff = state_clone_helper(reg, src->eff, src->n_q);

    dest->TF_abs = state_clone_helper(reg, src->TF_abs, src->n_tf);

    dest->TF_rel = state_clone_helper(reg, src->TF_rel, src->n_tf);
}

// TODO: Finish this
AA_API void
aa_ct_state_dump(FILE *stream, struct aa_ct_state *state)
{
    if (state->q) {
        fputs("q:   ", stream);
        aa_dump_vec(stream, state->q, state->n_q);
    }

    if (state->dq) {
        fputs("dq:  ", stream);
        aa_dump_vec(stream, state->dq, state->n_q);
    }

    if (state->ddq) {
        fputs("ddq: ", stream);
        aa_dump_vec(stream, state->ddq, state->n_q);
    }

    if (state->eff) {
        fputs("eff: ", stream);
        aa_dump_vec(stream, state->ddq, state->n_q);
    }

    if (state->X) {
        fputs("X:   ", stream);
        aa_dump_vec(stream, state->X, 7);
    }

    if (state->dX) {
        fputs("dX:  ", stream);
        aa_dump_vec(stream, state->dX, 6);
    }

    if (state->ddX) {
        fputs("ddX: ", stream);
        aa_dump_vec(stream, state->ddX, 6);
    }
}

// TODO: Finish this
AA_API int
aa_ct_state_eq(struct aa_ct_state *s1, struct aa_ct_state *s2)
{
    if (s1->n_q != s2->n_q || s1->n_tf != s2->n_tf)
        return 0;

    if ((s1->q && !s2->q) || (!s1->q && s2->q))
        return 0;
    else if (s1->q && s2->q)
        if (!aa_veq(s1->n_q, s1->q, s2->q, AA_EPSILON))
            return 0;

    if ((s1->dq && !s2->dq) || (!s1->dq && s2->dq))
        return 0;
    else if (s1->dq && s2->dq)
        if (!aa_veq(s1->n_q, s1->dq, s2->dq, AA_EPSILON))
            return 0;

    if ((s1->ddq && !s2->ddq) || (!s1->ddq && s2->ddq))
        return 0;
    else if (s1->ddq && s2->ddq)
        if (!aa_veq(s1->n_q, s1->ddq, s2->ddq, AA_EPSILON))
            return 0;

    if ((s1->X && !s2->X) || (!s1->X && s2->X))
        return 0;
    else if (s1->X && s2->X)
        if (!aa_veq(7, s1->X, s2->X, AA_EPSILON))
            return 0;

    if ((s1->dX && !s2->dX) || (!s1->dX && s2->dX))
        return 0;
    else if (s1->dX && s2->dX)
        if (!aa_veq(6, s1->dX, s2->dX, AA_EPSILON))
            return 0;

    if ((s1->ddX && !s2->ddX) || (!s1->ddX && s2->ddX))
        return 0;
    else if (s1->ddX && s2->ddX)
        if (!aa_veq(6, s1->ddX, s2->ddX, AA_EPSILON))
            return 0;

    return 1;
}

void aa_ct_state_set_qutr( struct aa_ct_state *state, const double E[7] )
{
    AA_MEM_CPY(state->X, E, 7);
}
