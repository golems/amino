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

#ifndef AMINO_RX_SCENE_OMPL_H
#define AMINO_RX_SCENE_OMPL_H

#include "amino/rx/rxerr.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_kin_internal.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_planning.h"

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include <ompl/base/ScopedState.h>
#include <ompl/base/TypedSpaceInformation.h>
#include <ompl/base/TypedStateValidityChecker.h>
#include <ompl/base/Planner.h>

/**
 * @file scene_ompl.h
 * @brief OMPL-specific motion planning
 */

namespace amino {}


/* Forward Declaration */
namespace ompl {
namespace base {
class GoalLazySamples;
}
}

struct aa_rx_mp {
    aa_rx_mp( const struct aa_rx_sg_sub *sub_sg ) :
        config_start(NULL),
        space_information(
            new amino::sgSpaceInformation(
                amino::sgSpaceInformation::SpacePtr(
                    new amino::sgStateSpace (sub_sg)))),
        problem_definition(new ompl::base::ProblemDefinition(space_information)),
        simplify(0)
        { }
    ~aa_rx_mp();

    void set_planner( ompl::base::Planner *p ) {
        this->planner.reset(p);
    }

    amino::sgSpaceInformation::Ptr space_information;
    ompl::base::ProblemDefinitionPtr problem_definition;

    ompl::base::PlannerPtr planner;

    double *config_start;

    unsigned simplify : 1;

    ompl::base::GoalLazySamples *lazy_samples;
};

#endif /*AMINO_RX_SCENE_OMPL_H*/
