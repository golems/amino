/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
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

#ifndef AMINO_RX_OMPL_SCENE_WORKSPACE_GOAL_H
#define AMINO_RX_OMPL_SCENE_WORKSPACE_GOAL_H

/**
 * @file scene_workspace_goal.h
 * @brief OMPL Goal Sampler
 */

#include "amino/rx/rxerr.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_ik.h"


#include "amino/rx/ompl/scene_state_space.h"
#include <ompl/base/goals/GoalLazySamples.h>


namespace amino {

class sgWorkspaceGoal : public ompl::base::GoalLazySamples {
public:
    sgWorkspaceGoal (const sgSpaceInformation::Ptr &si,
                     size_t n_e, const aa_rx_frame_id *frames,
                     const double *E, size_t ldE );

    virtual ~sgWorkspaceGoal ();

    const sgSpaceInformation::Ptr &typed_si;
    struct aa_rx_ik_parm *ko;
    struct aa_rx_ik_cx *ik_cx;

    ompl::base::StateSamplerPtr state_sampler;
    sgSpaceInformation::StateType *seed;

    double distanceGoal (const ompl::base::State *st) const;

    void setStart(size_t n_all, double *q);

    /** number of goal frames */
    size_t n_e;

    /** IDs of goal frames */
    aa_rx_frame_id *frames;

    /** Poses of goal frames (quaternion-translation) */
    double *E;

    /** Weighting of orientation error in distance computation */
    double weight_orientation;

    /** Weighting of translation error in distance computation */
    double weight_translation;
};


}



#endif
