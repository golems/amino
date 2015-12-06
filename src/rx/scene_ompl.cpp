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
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_kin_internal.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_planning.h"

#include "amino/rx/scene_ompl.h"

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/PathGeometric.h>

struct aa_rx_mp;

AA_API struct aa_rx_mp*
aa_rx_mp_create( const struct aa_rx_sg_sub *sub_sg )
{
    struct aa_rx_mp *mp = new aa_rx_mp(sub_sg);
    return mp;
}

AA_API void
aa_rx_mp_destroy( struct aa_rx_mp *mp )
{
    delete mp;
}

AA_API void
aa_rx_mp_set_start( struct aa_rx_mp *mp,
                    size_t n_all,
                    double *q_all )
{

    amino::sgSpaceInformation::Ptr &si = mp->space_information;
    amino::sgStateSpace *ss = si->getTypedStateSpace();

    /* Add Start State */
    amino::sgSpaceInformation::ScopedStateType state(si);
    ss->extract_state( q_all, state.get() );
    mp->problem_definition->addStartState(state);

    /* Configure State Validty Checker */
    ss->allow_config(q_all);
    si->setStateValidityChecker(
        ompl::base::StateValidityCheckerPtr(
            new amino::sgStateValidityChecker(si.get(), q_all)) );

    /* Setup Space */
    si->setup();
}

AA_API void
aa_rx_mp_set_goal( struct aa_rx_mp *mp,
                   size_t n_q,
                   double *q_subset)
{
    amino::sgStateSpace *ss = mp->space_information->getTypedStateSpace();
    amino::sgSpaceInformation::ScopedStateType state(mp->space_information);
    ss->copy_state( q_subset, state.get() );
    mp->problem_definition->setGoalState(state);
}


AA_API void
aa_rx_mp_set_wsgoal( struct aa_rx_mp *mp,
                     size_t n_e,
                     double *E, size_t ldE )
{
    // TODO: use an OMPL goal sampler
    // TODO: add interface to set IK options for motion planner

    amino::sgStateSpace *ss = mp->space_information->getTypedStateSpace();
    const struct aa_rx_sg_sub *ssg = ss->sub_scene_graph;
    const struct aa_rx_sg *sg = ss->scene_graph;
    size_t n_all = aa_rx_sg_config_count(sg);
    size_t n_s = aa_rx_sg_sub_config_count(ssg);
    double q0[n_all], qs[n_s];

    // Find IK Solution
    for( size_t i = 0; i < n_all; i ++ ) {
        double min=0 ,max=0;
        int r = aa_rx_sg_get_limit_pos( sg, i, &min, &max );
        assert(0 == r);
        q0[i] = (max + min) / 2;
    }


    struct aa_rx_ksol_opts *ko = aa_rx_ksol_opts_create();
    aa_rx_ksol_opts_center_configs( ko, ssg, .1 );
    aa_rx_ksol_opts_set_tol_dq( ko, .01 );

    if( 1 == n_e ) {
        int r = aa_rx_sg_chain_ksol_dls( ssg, ko,
                                         E, n_all, q0,
                                         n_s, qs );
    } else {
        assert(0);
    }


    // Set JS Goal
    aa_rx_mp_set_goal(mp, n_s, qs);
}


AA_API int
aa_rx_mp_plan( struct aa_rx_mp *mp,
               double timeout,
               size_t *n_path,
               double **p_path_all )
{
    amino::sgSpaceInformation::Ptr &si = mp->space_information;
    amino::sgStateSpace *ss = si->getTypedStateSpace();
    ompl::base::ProblemDefinitionPtr &pdef = mp->problem_definition;

    ompl::base::PlannerPtr planner(new ompl::geometric::SBL(si));
    planner->setProblemDefinition(pdef);
    planner->solve(timeout);
    if( pdef->hasSolution() ) {
        const ompl::base::PathPtr &path0 = pdef->getSolutionPath();
        ompl::geometric::PathGeometric &path = static_cast<ompl::geometric::PathGeometric&>(*path0);
        /* Allocate a simple array */
        *n_path = path.getStateCount();
        *p_path_all = (double*)calloc( *n_path * ss->config_count_all(),
                                       sizeof(double) );

        /* Fill array */
        std::vector< ompl::base::State *> &states = path.getStates();
        double *ptr = *p_path_all;
        for( auto itr = states.begin(); itr != states.end(); itr++, ptr += ss->config_count_all() )
        {
            //AA_MEM_CPY( ptr, q0, ss->config_count_all() );
            // TODO: Copy initial all-config state
            AA_MEM_ZERO( ptr, ss->config_count_all() );
            amino::sgSpaceInformation::StateType *state = amino::sgSpaceInformation::state_as(*itr);
            ss->insert_state( state, ptr );
            //aa_dump_vec( stdout, ptr, ss->config_count_all() );
        }
        return 0;
    } else {
        *n_path = 0;
        *p_path_all = NULL;
        return -1;
    }
}
