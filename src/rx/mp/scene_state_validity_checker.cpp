/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015-2016, Rice University
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
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_collision.h"


#include "amino/rx/ompl/scene_state_validity_checker.h"

namespace amino {

sgStateValidityChecker::sgStateValidityChecker(sgSpaceInformation *si,
                                               const double *q_initial ) :
    TypedStateValidityChecker(si),
    q_all(new double[getTypedStateSpace()->config_count_all()]) {
    size_t n_all = getTypedStateSpace()->config_count_all();
    std::copy( q_initial, q_initial + n_all, q_all );
}

sgStateValidityChecker::~sgStateValidityChecker() {
    delete [] q_all;
}

bool sgStateValidityChecker::isValid(const ompl::base::State *state_) const
{
    const sgSpaceInformation::StateType *state = state_as(state_);
    sgStateSpace *space = getTypedStateSpace();
    size_t n_q = space->config_count_all();
    size_t n_s = space->config_count_subset();
    size_t n_f = space->frame_count();

    // Set configs

    double q[n_q];
    std::copy( q_all, q_all + n_q, q );
    space->insert_state(state, q);
    //aa_dump_vec(stdout, q, n_q);

    // Find TFs
    double TF_rel[7*n_f];
    double TF_abs[7*n_f];
    aa_rx_sg_tf( space->scene_graph, n_q, q,
                 n_f,
                 TF_rel, 7,
                 TF_abs, 7 );

    // check collision
    struct aa_rx_cl *cl = aa_rx_cl_create( space->scene_graph );
    aa_rx_cl_allow_set( cl, space->allowed );
    int col = aa_rx_cl_check( cl, n_f, TF_abs, 7, NULL );
    aa_rx_cl_destroy(cl);

    bool valid = !col;
    return valid;
}

} /* namespace amino */
