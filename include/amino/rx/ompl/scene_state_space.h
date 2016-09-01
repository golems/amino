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

#ifndef AMINO_RX_OMPL_SCENE_STATE_SPACE_H
#define AMINO_RX_OMPL_SCENE_STATE_SPACE_H

/**
 * @file scene_state_space.h
 * @brief OMPL State Space
 */

#include "amino/rx/rxerr.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_kin_internal.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_planning.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/TypedSpaceInformation.h>


namespace amino {

/**
 * An OMPL state space for an amino scene graph.
 *
 * The OMPL state space variables correspond to the scene graph
 * configuration space.
 */
class sgStateSpace : public ompl::base::RealVectorStateSpace {
public:

    /**
     * Create a state space for the sub-scenegraph `sub_sg'.
     */
    sgStateSpace( const struct aa_rx_sg_sub *sub_sg ) ;

    /**
     * Destroy the state space.
     */
    virtual ~sgStateSpace() {
        aa_mem_region_destroy(&reg);
        aa_rx_cl_set_destroy(allowed);
    }

    /**
     * Return the scene graph for the state space.
     */
    const aa_rx_sg *get_scene_graph() const {
        return scene_graph;
    }

    /**
     * Return the number of configuration variables in the full scenegraph.
     */
    size_t config_count_all() const {
        return aa_rx_sg_config_count(get_scene_graph());
    }

    /**
     * Return the number of configuration variables in the sub-scenegraph.
     */
    size_t config_count_subset() const {
        return getDimension();
    }

    /**
     * Return the number of frames in the full scenegraph.
     */
    size_t frame_count() const {
        return aa_rx_sg_frame_count(get_scene_graph());
    }

    /**
     * Mark configuration q as allowed.
     */
    void allow_config( double *q ) {
        aa_rx_sg_get_collision(scene_graph, q, allowed);
    }

    /**
     * Retrieve the sub-scenegraph configuration `q_set' from the full
     * scenegraph array `q_all'.
     */
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

}



#endif
