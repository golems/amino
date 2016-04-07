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


#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorBounds.h>

#include <ompl/base/ScopedState.h>
#include <ompl/base/TypedSpaceInformation.h>
#include <ompl/base/TypedStateValidityChecker.h>

namespace amino {


class sgStateSpace : public ompl::base::RealVectorStateSpace {
public:
    sgStateSpace( const struct aa_rx_sg_sub *sub_sg ) :
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
                max = -M_PI;
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
    }

    virtual ~sgStateSpace() {
        aa_mem_region_destroy(&reg);
        aa_rx_cl_set_destroy(allowed);
    }

    const aa_rx_sg *get_scene_graph() const {
        return scene_graph;
    }

    size_t config_count_all() const {
        return aa_rx_sg_config_count(get_scene_graph());
    }
    size_t config_count_subset() const {
        return getDimension();
    }

    size_t frame_count() const {
        return aa_rx_sg_frame_count(get_scene_graph());
    }

    void allow_config( double *q ) {
        size_t n_f = frame_count();
        size_t n_q = config_count_all();
        double *TF_rel = AA_MEM_REGION_NEW_N(&reg, double, 7*n_f);
        double *TF_abs = AA_MEM_REGION_NEW_N(&reg, double, 7*n_f);
        aa_rx_sg_tf(scene_graph, n_q, q,
                    n_f,
                    TF_rel, 7,
                    TF_abs, 7 );

        struct aa_rx_cl *cl = aa_rx_cl_create(scene_graph);
        aa_rx_cl_check(cl, n_f, TF_abs, 7, allowed);
        aa_rx_cl_destroy(cl);
        aa_mem_region_pop(&reg, TF_rel);
    }

    void extract_state( const double *q_all, double *q_set ) const {
        aa_rx_sg_sub_config_get( sub_scene_graph,
                                 config_count_all(), q_all,
                                 config_count_subset(), q_set );
    }

    void extract_state( const double *q_all, StateType *state ) const {
        extract_state( q_all, state->values );
    }

    void insert_state( const double *q_set, double *q_all ) const {
        aa_rx_sg_sub_config_set( sub_scene_graph,
                                 config_count_subset(), q_set,
                                 config_count_all(), q_all );
    }

    void insert_state( const StateType *state, double *q_all ) const {
        insert_state( state->values, q_all );
    }

    void copy_state( const double *q_set, StateType *state ) {
        std::copy( q_set, q_set + config_count_subset(), state->values );
    }


    const aa_rx_sg *scene_graph;
    const aa_rx_sg_sub *sub_scene_graph;
    struct aa_rx_cl_set *allowed;
    struct aa_mem_region reg;
};

typedef ::ompl::base::TypedSpaceInformation<amino::sgStateSpace> sgSpaceInformation;


class sgStateValidityChecker : public ::ompl::base::TypedStateValidityChecker<sgStateSpace> {
public:
    sgStateValidityChecker(sgSpaceInformation *si,
                           const double *q_initial ) :
        TypedStateValidityChecker(si),
        q_all(new double[getTypedStateSpace()->config_count_all()]) {
        size_t n_all = getTypedStateSpace()->config_count_all();
        std::copy( q_initial, q_initial + n_all, q_all );
    }

    ~sgStateValidityChecker() {
        delete [] q_all;
    }

    virtual bool isValid(const ompl::base::State *state_) const
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
    double *q_all;
};


} /* namespace amino */

struct aa_rx_mp {
    aa_rx_mp( const struct aa_rx_sg_sub *sub_sg ) :
        space_information(
            new amino::sgSpaceInformation(
                amino::sgSpaceInformation::SpacePtr(
                    new amino::sgStateSpace (sub_sg)))),
        problem_definition(new ompl::base::ProblemDefinition(space_information)),
        simplify(0)
        { }
    ~aa_rx_mp();

    amino::sgSpaceInformation::Ptr space_information;
    ompl::base::ProblemDefinitionPtr problem_definition;

    unsigned simplify : 1;
};



#endif /*AMINO_RX_SCENE_OMPL_H*/
