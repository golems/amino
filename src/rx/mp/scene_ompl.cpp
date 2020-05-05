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
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_ik_internal.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_planning.h"


#include "amino/rx/ompl/scene_state_space.h"
#include "amino/rx/ompl/scene_state_validity_checker.h"
#include "amino/rx/ompl/scene_workspace_goal.h"
#include "amino/rx/ompl/scene_ompl_internal.h"


#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/goals/GoalLazySamples.h>


struct aa_rx_mp;


aa_rx_mp::aa_rx_mp( const struct aa_rx_sg_sub *sub_sg ) :
    config_start(NULL),
    space_information(
        new amino::sgSpaceInformation(
            amino::sgSpaceInformation::SpacePtr(
                new amino::sgStateSpace (sub_sg)))),
    problem_definition(new ompl::base::ProblemDefinition(space_information)),
    simplify(0),
    validity_checker(new amino::sgStateValidityChecker(space_information.get())),
    lazy_samples(NULL),
    collisions(NULL)
{

    space_information->setStateValidityChecker( ompl::base::StateValidityCheckerPtr(validity_checker) );
    space_information->setup();
}


AA_API struct aa_rx_mp*
aa_rx_mp_create( const struct aa_rx_sg_sub *sub_sg )
{
    return new aa_rx_mp(sub_sg);
}

AA_API void
aa_rx_mp_destroy( struct aa_rx_mp *mp )
{
    aa_checked_free(mp->config_start);
    delete mp;
}


aa_rx_mp::~aa_rx_mp()
{
    if( this->collisions ) aa_rx_cl_set_destroy(this->collisions);
}

AA_API void
aa_rx_mp_set_start( struct aa_rx_mp *mp,
                    size_t n_all,
                    double *q_all )
{
    /* Assume the start state is valid */
    aa_rx_mp_allow_config(mp, n_all, q_all);

    // FIXME: Seems that there are problems if this is called repeatedly
    amino::sgSpaceInformation::Ptr &si = mp->space_information;
    amino::sgStateSpace *ss = si->getTypedStateSpace();

    /* Add Start State */
    amino::sgSpaceInformation::ScopedStateType state(si);
    ss->extract_state( q_all, state.get() );
    mp->problem_definition->addStartState(state);


    /* Copy full start */
    aa_checked_free(mp->config_start);
    mp->config_start = AA_MEM_DUP(double, q_all, n_all);
}

AA_API void
aa_rx_mp_allow_config( struct aa_rx_mp *mp,
                       size_t n_all,
                       double *q_all )
{
    amino::sgSpaceInformation::Ptr &si = mp->space_information;
    amino::sgStateSpace *ss = si->getTypedStateSpace();
    ss->allow_config(q_all);

    mp->validity_checker->set_start(n_all, q_all);
}


AA_API void
aa_rx_mp_allow_collision( struct aa_rx_mp *mp,
                          aa_rx_frame_id id0, aa_rx_frame_id id1, int allowed )
{
    amino::sgSpaceInformation::Ptr &si = mp->space_information;
    amino::sgStateSpace *ss = si->getTypedStateSpace();
    aa_rx_cl_set_set( ss->allowed, id0, id1, allowed );
    mp->validity_checker->allow();
}

AA_API int
aa_rx_mp_set_goal( struct aa_rx_mp *mp,
                   size_t n_q,
                   double *q_subset)
{

    amino::sgSpaceInformation::Ptr &si = mp->space_information;
    amino::sgStateSpace *ss = si->getTypedStateSpace();
    const struct aa_rx_sg_sub *ssg = ss->sub_scene_graph;
    size_t n_s = aa_rx_sg_sub_config_count(ssg);
    assert( n_q == n_s );
    amino::sgSpaceInformation::ScopedStateType state(mp->space_information);
    ss->copy_state( q_subset, state.get() );

    mp->validity_checker->allow();
    if( si->isValid( state.get() ) ) {
        mp->problem_definition->setGoalState(state);
        return AA_RX_OK;
    } else {
        return AA_RX_INVALID_STATE;
    }
}

static void
path_cleanup( struct aa_rx_mp *mp, ompl::geometric::PathGeometric &path )
{
    amino::sgSpaceInformation::Ptr &si = mp->space_information;

    if( mp->simplify ) {
        ompl::geometric::PathSimplifier ps(si);
        int n = (int)path.getStateCount();
        path.interpolate(n*10);

        for( int i = 0; i < 10; i ++ ) {
            ps.reduceVertices(path);
            ps.collapseCloseVertices(path);
            ps.shortcutPath(path);
        }

        ps.smoothBSpline(path, 3, path.length()/100.0);
    }
}

AA_API int
aa_rx_mp_plan( struct aa_rx_mp *mp,
               double timeout,
               size_t *n_path,
               double **p_path_all )
{

    amino::sgSpaceInformation::Ptr &si = mp->space_information;
    amino::sgStateSpace *ss = si->getTypedStateSpace();

    /* Configure State Validity Checker */
    mp->validity_checker->allow();

    /* clear set */
    if( mp->collisions ) {
        aa_rx_cl_set_clear(mp->collisions);
    }

    /* Setup Space */

    *n_path = 0;
    *p_path_all = NULL;

    ompl::base::ProblemDefinitionPtr &pdef = mp->problem_definition;

    ompl::base::PlannerPtr planner = (NULL == mp->planner.get()) ?
        ompl::base::PlannerPtr(new ompl::geometric::RRTConnect(si)) :
        mp->planner;

    planner->setProblemDefinition(pdef);
    try {
        if( mp->lazy_samples ) {
            fprintf(stderr, "Starting sampling thread\n");
            mp->lazy_samples->clear();
            mp->lazy_samples->setStart(ss->config_count_all(), mp->config_start);
            mp->lazy_samples->startSampling();
        }
        planner->solve(timeout);
        if( mp->lazy_samples ) {
            fprintf(stderr, "Stopping sampling thread\n");
            mp->lazy_samples->stopSampling();
        }
    } catch(...) {
        return AA_RX_NO_SOLUTION;
    }
    if( pdef->hasExactSolution() ) {
        const ompl::base::PathPtr &path_ptr = pdef->getSolutionPath();
        ompl::geometric::PathGeometric &path = static_cast<ompl::geometric::PathGeometric&>(*path_ptr);
        path_cleanup(mp, path);


        /* Allocate a simple array */
        *n_path = path.getStateCount();
        *p_path_all = (double*)calloc( *n_path * ss->config_count_all(),
                                       sizeof(double) );

        /* Fill array */
        std::vector< ompl::base::State *> &states = path.getStates();
        double *ptr = *p_path_all;
        for( auto itr = states.begin(); itr != states.end(); itr++, ptr += ss->config_count_all() )
        {
            AA_MEM_CPY( ptr, mp->config_start, ss->config_count_all() );
            amino::sgSpaceInformation::StateType *state = amino::sgSpaceInformation::state_as(*itr);
            ss->insert_state( state, ptr );
        }
        return AA_RX_OK;
    } else {
        return AA_RX_NO_SOLUTION | AA_RX_NO_MP;
    }
}

AA_API void
aa_rx_mp_set_simplify( struct aa_rx_mp *mp,
                       int simplify )
{
    mp->simplify = simplify ? 1 : 0;
}

AA_API void
aa_rx_mp_set_track_collisions( struct aa_rx_mp *mp, int track )
{
    /* Destroy existing collision set */
    if( mp->collisions ) {
        aa_rx_cl_set_destroy(mp->collisions);
        mp->collisions = NULL;
    }

    /* Create new collision set */
    if( track ) {
        mp->track_collisions = 1;
        amino::sgStateSpace *ss = mp->space_information->getTypedStateSpace();
        mp->collisions = aa_rx_cl_set_create( ss->get_scene_graph() );
    } else {
        mp->track_collisions = 0;

    }

    /* Setup state validity checker tracking */
    mp->validity_checker->collisions = mp->collisions;
}

AA_API const struct aa_rx_cl_set *
aa_rx_mp_get_collisions( const struct aa_rx_mp *mp )
{
    return mp->collisions;
}


AA_API struct aa_rx_cl_set*
aa_rx_mp_get_allowed( const struct aa_rx_mp* mp)
{
    return mp->space_information->getTypedStateSpace()->allowed;
}


ompl::base::SpaceInformationPtr
aa_rx_mp_get_space_information( const struct aa_rx_mp *mp)
{
    return mp->space_information;
}

void
aa_rx_mp_set_planner( struct aa_rx_mp *mp, ::ompl::base::Planner *planner)
{
    mp->set_planner(planner);
}
