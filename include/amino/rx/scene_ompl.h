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
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>

namespace amino {

template<class SpaceType_>
class TypedSpaceInformation : public ::ompl::base::SpaceInformation {
public:

    /*--- Type Definitions ---*/

    /**
     * The actual type of the state space
     */
    typedef SpaceType_ SpaceType;

    /**
     * The actual type of states in the space.
     */
    typedef typename SpaceType::StateType StateType;

    /**
     * The actual type for a Scoped State.
     */
    typedef ::ompl::base::ScopedState<SpaceType> ScopedStateType;

    /**
     * Shared Pointer to the actual type of the space.
     */
    typedef boost::shared_ptr<SpaceType> SpacePtr;

    /**
     * Shared pointer to the typed space.
     */
    typedef boost::shared_ptr< TypedSpaceInformation<SpaceType> > Ptr;


    /*--- Constructor ---*/

    /**
     * Construct from shared pointer to the actual space.
     */
    TypedSpaceInformation( const SpacePtr &space ) :
        ::ompl::base::SpaceInformation(space)
        {}

    /*--- Space Accessors ---*/

    /**
     * Get space pointer of the proper type, const.
     */
    const SpaceType *getTypedStateSpace() const {
        const ::ompl::base::StateSpacePtr &ptr = getStateSpace();
        return ptr->as<SpaceType>();
    }

    /**
     * Get space pointer of the proper type.
     */
    SpaceType *getTypedStateSpace() {
        const ::ompl::base::StateSpacePtr &ptr = getStateSpace();
        return ptr->as<SpaceType>();
    }

    /*--- State Memory Management ---*/

    /**
     * Allocate a state of the proper type.
     */
    StateType * allocTypedState () const {
        ompl::base::State *s = this->allocState();
        return s->as<StateType>();
    }

    /**
     * Allocate memory for typed states in array
     */
    void allocTypedStates (std::vector<StateType *> &states) const {
        allocStates(states);
    }

    /**
     * Free a state of the proper type.
     */
    void freeTypedState ( StateType *state ) const {
        freeState(state);
    }

    /**
     * Free typed states in array
     */
    void freeTypedStates (std::vector<StateType *> &states) const {
        freeStates(states);
    }

    /**
     * Copy a state of the proper type.
     */
    void copyTypedState ( StateType *destination,
                          const StateType *source ) const {
        copyState(destination, source );
    }

    /**
     * Clone a state of the proper type.
     */
    StateType * cloneTypedState ( const StateType *source ) const {
        ompl::base::State *s = this->cloneState();
        return s->as<StateType>();
    }

    static StateType * state_as ( ompl::base::State *s ) {
        return s->as<StateType>();
    }

    static const StateType * state_as( const ompl::base::State *s ) {
        return s->as<StateType>();
    }

};

template <class SpaceType_>
class TypedStateValidityChecker : public ompl::base::StateValidityChecker {
public:

    typedef SpaceType_ SpaceType;
    typedef TypedSpaceInformation<SpaceType> SpaceInformationType;

    TypedStateValidityChecker(SpaceInformationType *si) :
        ompl::base::StateValidityChecker(si) { }

    TypedStateValidityChecker(const typename SpaceInformationType::Ptr si) :
        ompl::base::StateValidityChecker(si) { }

    SpaceInformationType *getTypedSpaceInformation() const {
        return static_cast<SpaceInformationType*>(si_);
    }

    SpaceType *getTypedStateSpace() const {
        return getTypedSpaceInformation()->getTypedStateSpace();
    }

    static typename SpaceType::StateType *
    state_as ( ompl::base::State *s ) {
        return SpaceInformationType::state_as(s);
    }

    static const typename SpaceType::StateType *
    state_as( const ompl::base::State *s ) {
        return SpaceInformationType::state_as(s);
    }
};



class sgStateSpace : public ompl::base::RealVectorStateSpace {
public:
    sgStateSpace( const aa_rx_sg *sg, size_t n_configs,
                  const char **config_names ) :
        ompl::base::RealVectorStateSpace((unsigned)n_configs),
        ids(new aa_rx_config_id[n_configs]),
        allowed(aa_rx_cl_set_create(sg)),
        scene_graph(sg) {

        // Fill indices
        aa_rx_sg_config_indices( sg, n_configs, config_names, ids );

        // TODO: get actual bounds
        setBounds(-M_PI, M_PI);

        // // allowed
        // size_t n_q = config_count_all();
        // double q[n_q];
        // AA_MEM_ZERO(q, n_q); // TODO: give good config
        // allow_config(q);
    }

    virtual ~sgStateSpace() {
        delete[] ids;
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

        double TF_rel[7*n_f];
        double TF_abs[7*n_f];
        aa_rx_sg_tf(scene_graph, n_q, q,
                    n_f,
                    TF_rel, 7,
                    TF_abs, 7 );

        struct aa_rx_cl *cl = aa_rx_cl_create(scene_graph);
        aa_rx_cl_check(cl, n_f, TF_abs, 7, allowed);
        aa_rx_cl_destroy(cl);
    }

    void extract_state( const double *q_all, double *q_set ) const {
        aa_rx_sg_config_get( scene_graph, config_count_all(), config_count_subset(),
                             ids, q_all, q_set );
    }

    void extract_state( const double *q_all, StateType *state ) const {
        extract_state( q_all, state->values );
    }

    void insert_state( const double *q_set, double *q_all ) const {
        aa_rx_sg_config_set( scene_graph, config_count_all(), config_count_subset(),
                             ids, q_set, q_all );
    }

    void insert_state( const StateType *state, double *q_all ) const {
        insert_state( state->values, q_all );
    }

    void copy_state( const double *q_set, StateType *state ) {
        std::copy( q_set, q_set + config_count_subset(), state->values );
    }


    const aa_rx_sg *scene_graph;
    aa_rx_config_id *ids;
    struct aa_rx_cl_set *allowed;
};

typedef TypedSpaceInformation<amino::sgStateSpace> sgSpaceInformation;


class sgStateValidityChecker : public TypedStateValidityChecker<sgStateSpace> {
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
    aa_rx_mp( const struct aa_rx_sg *sg,
              size_t n,
              const char **names ) :
        space_information(
            new amino::sgSpaceInformation(
                amino::sgSpaceInformation::SpacePtr(
                    new amino::sgStateSpace (sg, n, names)))),
        problem_definition(new ompl::base::ProblemDefinition(space_information))
        { }

    amino::sgSpaceInformation::Ptr space_information;
    ompl::base::ProblemDefinitionPtr problem_definition;
};



#endif /*AMINO_RX_SCENE_OMPL_H*/
