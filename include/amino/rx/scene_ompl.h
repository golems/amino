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

namespace amino {

class rxSpace {
public:
    rxSpace( const aa_rx_sg *sg_, size_t n_configs,
             const char **config_names ) :
        sg(sg_),
        state_space(new ompl::base::RealVectorStateSpace(n_configs)),
        space_information(new ompl::base::SpaceInformation(state_space)),
        ids(new aa_rx_config_id[n_configs]),
        allowed(aa_rx_cl_set_create(sg)),
        cl(aa_rx_cl_create(sg)) {

        // Fill indices
        aa_rx_sg_config_indices( sg, n_configs, config_names, ids );

        // Get allowed collisions
        size_t n_q = dim_all();
        double q[n_q];
        AA_MEM_ZERO(q, n_q); // TODO: give good config
        allow_config(q);
    }

    ~rxSpace() {
        delete [] ids;
        aa_rx_cl_set_destroy(allowed);
        aa_rx_cl_destroy(cl);
    }

    void allow_config( double *q ) {
        size_t n_f = frame_count();
        double TF_rel[7*n_f];
        double TF_abs[7*n_f];
        aa_rx_sg_tf(sg, dim_all(), q,
                    n_f,
                    TF_rel, 7,
                    TF_abs, 7 );
        aa_rx_cl_check( cl, n_f, TF_abs, 7, allowed );
    }

    size_t dim_set() const {
        return state_space->getDimension();
    }
    size_t dim_all() const {
        return aa_rx_sg_config_count(sg);
    }

    size_t frame_count() const {
        return aa_rx_sg_frame_count(sg);
    }

    void state_get( const double *q_all, double *q_set ) const {
        aa_rx_sg_config_get( sg, dim_all(), dim_set(),
                             ids, q_all, q_set );
    }

    void state_get( const double *q_all, ompl::base::State *state_ ) const {
        ompl::base::RealVectorStateSpace::StateType *state
            = state_->as<ompl::base::RealVectorStateSpace::StateType>();
        state_get( q_all, state->values );
    }

    void state_set( const double *q_set, double *q_all ) const {
        aa_rx_sg_config_set( sg, dim_all(), dim_set(),
                             ids, q_set, q_all );
    }

    void state_set( const ompl::base::State *state_, double *q_all ) const {
        const ompl::base::RealVectorStateSpace::StateType *state
            = state_->as<ompl::base::RealVectorStateSpace::StateType>();
        state_set( state->values, q_all );
    }

    void fill_state( const double *q_set, const ompl::base::State *state_ ) {
        const ompl::base::RealVectorStateSpace::StateType *state
            = state_->as<ompl::base::RealVectorStateSpace::StateType>();
        std::copy( q_set, q_set + dim_set(), state->values );
    }

    const aa_rx_sg *sg;
    ompl::base::StateSpacePtr state_space;
    ompl::base::SpaceInformationPtr space_information;
    aa_rx_config_id *ids;
    struct aa_rx_cl *cl;
    struct aa_rx_cl_set *allowed;
};

class rxStateValidityChecker : public ompl::base::StateValidityChecker {
public:
    rxStateValidityChecker(const rxSpace *space_, const double *q_initial ) :
        ompl::base::StateValidityChecker(space_->space_information),
        space(space_) {

        q_all = new double[space->dim_all()];
        std::copy( q_initial, q_initial + space->dim_all(), q_all );

    }
    virtual bool isValid(const ompl::base::State *state) const
    {
        size_t n_q = space->dim_all();
        size_t n_s = space->dim_set();
        size_t n_f = space->frame_count();

        // Set configs

        double q[n_q];
        std::copy( q_all, q_all + n_q, q );
        space->state_set(state, q);
        //aa_dump_vec(stdout, q, n_q);

        // Find TFs
        double TF_rel[7*n_f];
        double TF_abs[7*n_f];
        aa_rx_sg_tf( space->sg, n_q, q,
                     n_f,
                     TF_rel, 7,
                     TF_abs, 7 );

        // check collision
        struct aa_rx_cl *cl = aa_rx_cl_create( space->sg );
        aa_rx_cl_allow_set( cl, space->allowed );
        int col = aa_rx_cl_check( cl, n_f, TF_abs, 7, NULL );
        aa_rx_cl_destroy(cl);

        bool valid = !col;
        return valid;
    }

    const rxSpace *space;
    double *q_all;
};
}

#endif /*AMINO_RX_SCENE_OMPL_H*/
