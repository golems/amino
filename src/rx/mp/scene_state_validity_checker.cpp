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
#include "amino/rx/scene_ik.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_collision.h"


#include "amino/rx/ompl/scene_state_validity_checker.h"

namespace amino {

sgStateValidityChecker::sgStateValidityChecker(sgSpaceInformation *si)
    :
    TypedStateValidityChecker(si),
    q_all(new double[getTypedStateSpace()->config_count_all()]),
    cl(aa_rx_cl_create(getTypedStateSpace()->scene_graph)),
    collisions(NULL)
{
    this->allow();
}

sgStateValidityChecker::~sgStateValidityChecker()
{
    delete [] q_all;
    aa_rx_cl_destroy(this->cl);
}

bool sgStateValidityChecker::isValid(const ompl::base::State *state) const
{
    sgStateSpace *space = getTypedStateSpace();
    size_t n_f = space->frame_count();
    int is_collision;

    // check collision
    {
        std::lock_guard<std::mutex> lock(mutex);
        double *TF_abs = space->get_tf_abs(state, this->q_all);
        is_collision = aa_rx_cl_check( cl, n_f, TF_abs, 7, this->collisions );
        space->region_pop(TF_abs);
    }

    return !is_collision;
}


void sgStateValidityChecker::set_start( size_t n_q, double *q_initial)
{
    assert( n_q == getTypedStateSpace()->config_count_all() );
    std::copy( q_initial, q_initial + n_q, q_all );
    this->allow();
}

void sgStateValidityChecker::allow( )
{
    aa_rx_cl_allow_set( cl, getTypedStateSpace()->allowed );
}

} /* namespace amino */
