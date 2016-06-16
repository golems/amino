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
    aa_mem_region_init(&reg, 1024);

    struct aa_ct_pt_list *pt_list = aa_ct_pt_list_create(&reg);

    struct aa_ct_state *pt1 = aa_ct_state_create(&reg, 0, 0, AA_CT_ST_X);
    /* struct aa_ct_state *pt2 = aa_ct_state_create(&reg, 0, 0, AA_CT_ST_X); */
    /* struct aa_ct_state *pt3 = aa_ct_state_create(&reg, 0, 0, AA_CT_ST_X); */

    pt1->X[0] = 1; pt1->X[4] = 0;
    /* pt2->X[1] = 1; pt2->X[4] = 1; */
    /* pt3->X[0] = 1; pt3->X[4] = 0; */

    aa_ct_pt_list_add(pt_list, pt1);
    /* aa_ct_pt_list_add(pt_list, pt2); */
    /* aa_ct_pt_list_add(pt_list, pt3); */

    struct aa_ct_state *lts = aa_ct_state_create(&reg, 0, 0,
                                                 AA_CT_ST_DX | AA_CT_ST_DDX);
    for (size_t j = 0; j < 6; j++) {
        lts->dX[j] = 1;
        lts->ddX[j] = 1;
    }

    struct aa_ct_seg_list *seg_list =
        aa_ct_tjX_pb_generate(&reg, pt_list, lts);

    aa_ct_seg_list_destroy(seg_list);
    aa_ct_pt_list_destroy(pt_list);
    aa_mem_region_destroy(&reg);
}

void
test_tjq(void)
{
    struct aa_mem_region reg;
    aa_mem_region_init(&reg, 1024);

    struct aa_ct_pt_list *pt_list = aa_ct_pt_list_create(&reg);

    size_t n_q = 4;
    struct aa_ct_state *st = aa_ct_state_create(&reg, n_q, 0, AA_CT_ST_Q);
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < n_q; j++)
            st->q[j] = (double) rand() / RAND_MAX;

        aa_ct_pt_list_add(pt_list, st);
    }

    struct aa_ct_state *lts = aa_ct_state_create(&reg, n_q, 0,
                                                 AA_CT_ST_DQ | AA_CT_ST_DDQ);

    for (size_t j = 0; j < n_q; j++) {
        lts->dq[j] = 1;
        lts->ddq[j] = 1;
    }

    struct aa_ct_seg_list *seg_list =
        aa_ct_tjq_pb_generate(&reg, pt_list, lts);

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
    test_tjq();
    test_tjX();
}
