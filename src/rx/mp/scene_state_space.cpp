/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
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
#include "amino/rx/rxerr.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_ik.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_collision.h"


#include "amino/rx/ompl/scene_state_space.h"


namespace amino {

sgStateSpace::sgStateSpace( const struct aa_rx_sg_sub *sub_sg ) :
        scene_graph(sub_sg->scenegraph),
        sub_scene_graph(sub_sg),
        ompl::base::RealVectorStateSpace((unsigned)aa_rx_sg_sub_config_count(sub_sg)),
        allowed(aa_rx_cl_set_create(scene_graph)) {

        aa_mem_region_init(&reg, 4000);

        // TODO: get actual bounds
        size_t n_configs = config_count_subset();
        ompl::base::RealVectorBounds vb( (unsigned int)n_configs );
        for( unsigned i = 0; i < (unsigned)n_configs; i ++ ) {
            double min,max;
            aa_rx_config_id cid = aa_rx_sg_sub_config(sub_scene_graph, i);
            int r = aa_rx_sg_get_limit_pos(scene_graph, cid, &min, &max);
            if(r) {
                fprintf(stderr, "ERROR: no position limits for %s\n",
                        aa_rx_sg_config_name(scene_graph, cid));
                /* This seems as good as anything */
                min = -M_PI;
                max = M_PI;
            }

            vb.setLow(i,min);
            vb.setHigh(i,max);

        }
        setBounds(vb);
        // // allowed
        // size_t n_q = config_count_all();
        // double q[n_q];
        // AA_MEM_ZERO(q, n_q); // TODO: give good config
        // allow_config(q);

        // Load allowable configs from scenegraph
        aa_rx_sg_cl_set_copy(scene_graph, allowed);
    }

double * sgStateSpace::get_tf_abs( const ompl::base::State *state)
{
    size_t n_q = this->config_count_all();
    double q_all[n_q];
    AA_MEM_ZERO(q_all,n_q); // TODO: or center?
    return this->get_tf_abs(state, q_all);
}

double * sgStateSpace::get_tf_abs( const ompl::base::State *state_, const double *q_all )
{
    const StateType *state = state_->as<StateType>();
    size_t n_q = this->config_count_all();
    size_t n_s = this->config_count_subset();
    size_t n_f = this->frame_count();

    // Set configs
    double q[n_q];
    std::copy( q_all, q_all + n_q, q );
    this->insert_state(state, q);

    // Find TFs
    double TF_rel[7*n_f];
    double *TF_abs = AA_MEM_REGION_NEW_N(&this->reg, double, 7*n_f);
    aa_rx_sg_tf( this->scene_graph, n_q, q,
                 n_f,
                 TF_rel, 7,
                 TF_abs, 7 );
    return TF_abs;
}

} /* namespace amino */
