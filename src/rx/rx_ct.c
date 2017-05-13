/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2017, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name of the Rice University nor the names of its
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
#include "amino/rx/rxtype.h"
#include "amino/rx/rxerr.h"
#include "amino/rx/rx_ct.h"
#include "amino/rx/scenegraph.h"

AA_API struct aa_ct_pt_list *
aa_rx_ct_pt_list( struct aa_mem_region *region,
                     size_t n_q, size_t n_path, double *path )
{
    struct aa_ct_pt_list *list = aa_ct_pt_list_create(region);

    /* TODO: memory ownership? */
    for( size_t i = 0; i < n_q*n_path; i+=n_q ) {
        struct aa_ct_state *s = AA_MEM_REGION_NEW(region, struct aa_ct_state);
        AA_MEM_ZERO(s,1);
        s->n_q = n_q;
        s->q = &(path[i]);
        aa_ct_pt_list_add( list, s );
    }

    return list;
}

static void set_inf( double *min, double *max )
{
    *min = -INFINITY;
    *max = INFINITY;
}



AA_API struct aa_ct_limit *
aa_rx_ct_limits( struct aa_mem_region *region, const struct aa_rx_sg *sg )
{
    struct aa_ct_limit *limit = AA_MEM_REGION_NEW(region, struct aa_ct_limit);
    size_t n_q = aa_rx_sg_config_count(sg);

    struct aa_ct_state *smin = limit->min = aa_ct_state_alloc(region, n_q, 0);
    struct aa_ct_state *smax = limit->max = aa_ct_state_alloc(region, n_q, 0);

    for( aa_rx_config_id i = 0; i < (aa_rx_config_id)n_q; i++ )
    {
        if( aa_rx_sg_get_limit_pos(sg, i, &(smin->q[i]), &(smax->q[i])) )
            set_inf( &smin->q[i], &smax->q[i] );

        if( aa_rx_sg_get_limit_vel(sg, i, &(smin->dq[i]), &(smax->dq[i])) )
            set_inf( &smin->dq[i], &smax->dq[i] );

        if( aa_rx_sg_get_limit_acc(sg, i, &(smin->ddq[i]), &(smax->ddq[i])) )
            set_inf( &smin->ddq[i], &smax->ddq[i] );

        if( aa_rx_sg_get_limit_eff(sg, i, &(smin->eff[i]), &(smax->eff[i])) )
            set_inf( &smin->eff[i], &smax->eff[i] );
    }

    return limit;
}
