/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016 Rice University
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

#include <amino.h>

#include <amino/ct/state.h>
#include <amino/ct/traj.h>
#include <amino/ct/ctrl.h>

int
aa_ct_ctrl_qp_ct(struct aa_ct_ctrl *ct, struct aa_ct_state *st,
                 struct aa_ct_state *rf, struct aa_ct_state *cmd)
{
    double k_p = *((double *) ct->cx);

    for (size_t i = 0; i < st->n_q; i++)
        cmd->dq[i] = rf->dq[i] - k_p * (st->q[i] - rf->q[i]);

    return 0;
}

struct aa_ct_ctrl *
aa_ct_ctrl_qp_create(struct aa_mem_region *reg, double k_p)
{
    struct aa_ct_ctrl *ct = AA_MEM_REGION_NEW(reg, struct aa_ct_ctrl);
    double *cx = AA_MEM_REGION_NEW(reg, double);
    *cx = k_p;

    ct->ctrl = aa_ct_ctrl_qp_ct;
    ct->cx = cx;

    return ct;
}
