/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
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
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_kin_internal.h"

#include "amino/getset.h"


AA_API struct aa_rx_ksol_opts*
aa_rx_ksol_opts_create()
{

    struct aa_rx_ksol_opts *opt = AA_NEW0(struct aa_rx_ksol_opts);

    /* Set sane defaults */
    opt->dt = 1;
    opt->tol_angle = .1*M_PI/180;
    opt->tol_trans = .1e-3;

    opt->tol_angle_svd = 10*opt->tol_angle;
    opt->tol_trans_svd = 10*opt->tol_trans;

    opt->tol_dq = .1*M_PI/180;

    opt->gain_angle = 1;
    opt->gain_trans = 1;
    opt->max_iterations = 1000;

    opt->s2min = 5e-3;
    opt->k_dls = 5e-5;

    return opt;
}

AA_API void
aa_rx_ksol_opts_destroy( struct aa_rx_ksol_opts *opts)
{
    if( opts->dq_dt_data ) free( opts->dq_dt_data );
    if( opts->q_ref_data ) free( opts->q_ref_data );
}


AA_DEF_SETTER( aa_rx_ksol_opts, double, dt )
AA_DEF_SETTER( aa_rx_ksol_opts, double, tol_angle )
AA_DEF_SETTER( aa_rx_ksol_opts, double, tol_trans )

AA_DEF_SETTER( aa_rx_ksol_opts, double, tol_angle_svd )
AA_DEF_SETTER( aa_rx_ksol_opts, double, tol_trans_svd )

AA_DEF_SETTER( aa_rx_ksol_opts, double, tol_dq )
AA_DEF_SETTER( aa_rx_ksol_opts, double, s2min )
AA_DEF_SETTER( aa_rx_ksol_opts, double, k_dls )
AA_DEF_SETTER( aa_rx_ksol_opts, double, gain_angle )
AA_DEF_SETTER( aa_rx_ksol_opts, double, gain_trans )
AA_DEF_SETTER( aa_rx_ksol_opts, size_t, max_iterations )



AA_API void
aa_rx_ksol_opts_take_config( struct aa_rx_ksol_opts *opts, size_t n_q,
                        double *q, enum aa_mem_refop refop )
{
    AA_MEM_DUPOP( refop, double, opts->q_ref,
                  opts->q_ref_data, q, n_q );
    opts->n_q_ref = n_q;
}


AA_API void
aa_rx_ksol_opts_take_gain_config( struct aa_rx_ksol_opts *opts, size_t n_q,
                             double *q, enum aa_mem_refop refop )
{
    AA_MEM_DUPOP( refop, double, opts->dq_dt,
                  opts->dq_dt_data, q, n_q );
    opts->n_dq_dt = n_q;
}

AA_API void
aa_rx_ksol_opts_take_seed( struct aa_rx_ksol_opts *opts, size_t n_q,
                           double *q, enum aa_mem_refop refop )
{
    AA_MEM_DUPOP( refop, double, opts->q_all_seed,
                  opts->q_all_seed_data, q, n_q );
    opts->n_all_seed = n_q;
}


AA_API void
aa_rx_ksol_opts_center_configs( struct aa_rx_ksol_opts *opts,
                                const struct aa_rx_sg_sub *ssg,
                                double gain )
{
    size_t n_qs = aa_rx_sg_sub_config_count(ssg);

    double *q_ref = AA_NEW_AR(double, n_qs);
    double *q_gain = AA_NEW_AR(double, n_qs);

    aa_rx_sg_sub_center_configs( ssg, n_qs, q_ref );
    for( size_t i = 0; i < n_qs; i ++ ) {
        double min=0 ,max=0;
        aa_rx_config_id config_id = aa_rx_sg_sub_config(ssg, i);
        int r = aa_rx_sg_get_limit_pos( ssg->scenegraph, config_id, &min, &max );
        if( 0 == r ) {
            q_gain[i] = gain * (2*M_PI) / (max - min); // scale gain based on range
        } else {
            q_gain[i] = 0;
        }
    }

    aa_rx_ksol_opts_take_config( opts, n_qs, q_ref, AA_MEM_STEAL );
    aa_rx_ksol_opts_take_gain_config( opts, n_qs, q_gain, AA_MEM_STEAL );
}

AA_API void
aa_rx_ksol_opts_center_seed( struct aa_rx_ksol_opts *opts, const struct aa_rx_sg_sub *ssg )
{
    size_t n_qs = aa_rx_sg_sub_config_count(ssg);
    size_t n_qa = aa_rx_sg_sub_all_config_count(ssg);
    double q_s[n_qs];
    double *q_a = AA_NEW0_AR(double, n_qa);

    aa_rx_sg_sub_center_configs( ssg, n_qs, q_s );
    aa_rx_sg_sub_config_set( ssg, n_qs, q_s,
                             n_qa, q_a );

    aa_rx_ksol_opts_take_seed( opts, n_qa, q_a, AA_MEM_STEAL );
}
