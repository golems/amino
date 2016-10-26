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
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_collision.h"

#include "amino/rx/scene_planning.h"


#include "amino/rx/ompl/scene_workspace_goal.h"
#include "amino/rx/ompl/scene_ompl_internal.h"

namespace ob = ::ompl::base;

namespace amino {


static bool
sampler_fun( const ob::GoalLazySamples *arg, ob::State *state )
{
    const sgWorkspaceGoal *wsg = static_cast<const sgWorkspaceGoal*>(arg);

    amino::sgStateSpace *ss = wsg->typed_si->getTypedStateSpace();
    const struct aa_rx_sg_sub *ssg = ss->sub_scene_graph;
    const struct aa_rx_sg *sg = ss->scene_graph;


    size_t n_all = aa_rx_sg_config_count(sg);
    size_t n_s = aa_rx_sg_sub_config_count(ssg);
    double qs[n_s];

    int r = AA_RX_NO_IK;

    while( AA_RX_OK != r && wsg->isSampling() ) {
        /* Re-seed */
        wsg->state_sampler->sampleUniform(wsg->seed);
        double q[n_all];
        if( wsg->q_start ) {
            AA_MEM_CPY(q, wsg->q_start, n_all);
        } else {
            AA_MEM_ZERO(q,n_all);
        }
        aa_rx_sg_sub_config_set( ssg,
                                 n_s, wsg->seed->values,
                                 n_all, q );
        aa_rx_ksol_opts_take_seed( wsg->ko, n_all, q, AA_MEM_COPY );

        /* solve */
        r = aa_rx_ik_jac_solve( wsg->ik_cx,
                                wsg->n_e, wsg->E, 7,
                                n_s, qs );
    }

    if( AA_RX_OK == r ) {
        ss->copy_state( qs, wsg->typed_si->state_as(state) );
        return true;
    } else {
        return false;
    }

}

sgWorkspaceGoal::sgWorkspaceGoal (const sgSpaceInformation::Ptr &si,
                                  size_t n_e_,
                                  const aa_rx_frame_id *frame_arg,
                                  const double *E_arg, size_t ldE ) :
    typed_si(si),
    ob::GoalLazySamples(si, ob::GoalSamplingFn(sampler_fun), false),
    ko( aa_rx_ksol_opts_create() ),
    ik_cx( aa_rx_ik_jac_cx_create(si->getTypedStateSpace()->sub_scene_graph, ko) ),
    n_e(n_e_),
    state_sampler( si->allocStateSampler() ),
    seed(typed_si->allocTypedState()),
    q_start(NULL),
    weight_orientation(1),
    weight_translation(1)
{
    const struct aa_rx_sg_sub *ssg = si->getTypedStateSpace()->sub_scene_graph;
    const struct aa_rx_sg *sg = si->getTypedStateSpace()->scene_graph;

    /* Set frame id */
    this->frames = new aa_rx_frame_id[n_e];
    if( frame_arg ) {
        AA_MEM_CPY(this->frames, frame_arg, n_e);
    } else {
        /* Fill in last frame */
        size_t n_s = aa_rx_sg_sub_frame_count(ssg);
        aa_rx_frame_id id_last = aa_rx_sg_sub_frame(ssg, n_s-1);
        AA_MEM_SET(this->frames, id_last, n_e);
    }
    // TODO: multiple frames
    aa_rx_ksol_opts_set_frame(ko, this->frames[0]);

    /* Set goals */
    this->E = new double[n_e*7];
    aa_cla_dlacpy( '\0', 7, (int)n_e,
                   E_arg, (int)ldE,
                   this->E, 7 );

    /* These settings should be optional */
    aa_rx_ksol_opts_center_seed(ko, ssg);
    aa_rx_ksol_opts_center_configs(ko, ssg, .1);
    aa_rx_ksol_opts_set_tol_dq(ko, .01);
}


sgWorkspaceGoal::~sgWorkspaceGoal ()
{
    typed_si->freeState(this->seed);
    aa_rx_ksol_opts_destroy(this->ko);
    aa_rx_ik_jac_cx_destroy(this->ik_cx);
    delete [] this->E;
    delete[] this->frames;
    aa_checked_free( this->q_start );
}

void sgWorkspaceGoal::setStart(size_t n_all, double *q)
{
    this->q_start = q ? AA_MEM_DUP(double, q, n_all) : NULL;
}


// unsigned int sgWorkspaceGoal::maxSampleCount () const
// {
//     return UINT_MAX;
// }


double sgWorkspaceGoal::distanceGoal (const ompl::base::State *state) const
{
    amino::sgStateSpace *space = typed_si->getTypedStateSpace();
    double *TF_abs = space->get_tf_abs(state, this->q_start);

    double a = 0;
    for( size_t i = 0; i < n_e; i ++ ) {
        double *Eref = this->E + 7*i;
        double *Eact = TF_abs + this->frames[i]*7;

        double *qref = Eref + AA_TF_QUTR_Q;
        double *qact = Eact + AA_TF_QUTR_Q;
        double *vref = Eref + AA_TF_QUTR_T;
        double *vact = Eact + AA_TF_QUTR_T;

        a += weight_orientation * aa_tf_qangle_rel(qref, qact);
        a += weight_translation * sqrt( AA_TF_VDOT(vref, vact) );
    }

    space->region_pop(TF_abs);
    return a;
}

} /* namespace amino */




AA_API int
aa_rx_mp_set_wsgoal( struct aa_rx_mp *mp,
                     size_t n_e, const aa_rx_frame_id *frames,
                     const double *E, size_t ldE )
{
    // TODO: add interface to set IK options for motion planner
    amino::sgWorkspaceGoal *g = new amino::sgWorkspaceGoal(mp->space_information,
                                                           n_e, frames, E, ldE);
    mp->problem_definition->setGoal(ompl::base::GoalPtr(g));
    mp->lazy_samples = g;
    return 0;

    // amino::sgStateSpace *ss = mp->space_information->getTypedStateSpace();
    // const struct aa_rx_sg_sub *ssg = ss->sub_scene_graph;
    // const struct aa_rx_sg *sg = ss->scene_graph;
    // size_t n_all = aa_rx_sg_config_count(sg);
    // size_t n_s = aa_rx_sg_sub_config_count(ssg);
    // double qs[n_s];

    // struct aa_rx_ksol_opts *ko = NULL;
    // if( NULL == opts ) {
    //     ko = aa_rx_ksol_opts_create();
    //     aa_rx_ksol_opts_center_seed( ko, ssg );
    //     aa_rx_ksol_opts_center_configs( ko, ssg, .1 );
    //     aa_rx_ksol_opts_set_tol_dq( ko, .01 );
    //     opts = ko;
    // }

    // int r = ik_fun( ik_cx,
    //                 n_e, E, 7,
    //                 n_s, qs );

    //aa_dump_vec( stdout, opts->q_all_seed, n_all );
    //aa_dump_vec( stdout, qs, n_s );

    // Set JS Goal
    // if( AA_RX_OK == r ) {
    //     return aa_rx_mp_set_goal(mp, n_s, qs);
    // } else {
    //     return r;
    // }

    // TODO: check state validity

    // if( ko ) {
    //     aa_rx_ksol_opts_destroy(ko);
    // }

}
