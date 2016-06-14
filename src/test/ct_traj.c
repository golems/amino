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

#include "amino.h"
#include "amino/test.h"

#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"

#include "amino/ct/state.h"
#include "amino/ct/traj.h"

void
test_tjX(void)
{
    struct aa_mem_region reg;
    aa_mem_region_init(&reg, 512);

    struct aa_ct_pt_list *pt_list = aa_ct_pt_list_create(&reg);

    struct aa_ct_state pt1 = {0}, pt2 = {0}, pt3 = {0};
    double x1[7] = {0}, x2[7] = {0}, x3[7] = {0};
    pt1.X = x1;
    pt2.X = x2;
    pt3.X = x3;

    x1[0] = 1;
    x2[1] = 1;
    x3[0] = 1;

    x1[4] = 0;
    x2[4] = 1;
    x3[4] = 0;

    aa_ct_pt_list_add(pt_list, &pt1);
    aa_ct_pt_list_add(pt_list, &pt2);
    aa_ct_pt_list_add(pt_list, &pt3);

    double dXlim[6], ddXlim[6];
    struct aa_ct_state limits;
    limits.dX = dXlim;
    limits.ddX = ddXlim;

    for (size_t j = 0; j < 6; j++) {
        limits.dX[j] = 1;
        limits.ddX[j] = 1;
    }

    struct aa_ct_seg_list *seg_list =
        aa_ct_tjX_pb_generate(&reg, pt_list, &limits);

    struct aa_ct_state state;
    double X[7], dX[6];
    state.X = X;
    state.dX = dX;

    for (double t = 0; aa_ct_seg_list_eval(seg_list, &state, t); t += 0.1) {
        aa_ct_seg_list_eval(seg_list, &state, t);
        // aa_dump_vec(stdout, state.X, 7);
    }

    aa_ct_seg_list_destroy(seg_list);
    aa_ct_pt_list_destroy(pt_list);
    aa_mem_region_destroy(&reg);
}

void
test_tjq(void)
{
    size_t n_q = 4;

    struct aa_mem_region reg;
    aa_mem_region_init(&reg, 512);

    struct aa_ct_pt_list *pt_list = aa_ct_pt_list_create(&reg);

    for (size_t i = 0; i < 4; i++) {
        struct aa_ct_state state = {0};
        double q[n_q];
        state.n_q = n_q;
        state.q = q;

        for (size_t j = 0; j < n_q; j++)
            state.q[j] = (double) rand() / RAND_MAX;

        aa_ct_pt_list_add(pt_list, &state);
    }

    double dqlim[n_q], ddqlim[n_q];
    struct aa_ct_state limits;
    limits.n_q = n_q;
    limits.dq = dqlim;
    limits.ddq = ddqlim;

    for (size_t j = 0; j < n_q; j++) {
        limits.dq[j] = 1;
        limits.ddq[j] = 1;
    }

    struct aa_ct_seg_list *seg_list =
        aa_ct_tjq_pb_generate(&reg, pt_list, &limits);

    aa_ct_seg_list_destroy(seg_list);
    aa_ct_pt_list_destroy(pt_list);
    aa_mem_region_destroy(&reg);
}

/**
 * Test making a parabolic blend trajectory
 */
int
main(void)
{
    test_tjX();
    test_tjq();
}
