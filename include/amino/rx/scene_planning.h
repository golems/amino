/* -*- mode: C; c-basic-offset: 4; -*- */
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

#ifndef AMINO_RX_SCENE_PLANNING_H
#define AMINO_RX_SCENE_PLANNING_H

#include "scene_kin.h"

/**
 * @file scene_planning.h
 * @brief Motion Planning
 */

/*--- Motion Planning ---*/

/**
 * Opaque structure for a motion planning context
 */
struct aa_rx_mp;

/**
 * Opague structure for a motion planner
 */
struct aa_rx_mp_planner;

/**
 * Create a new motion plannet context for the given sub-scenegraph.
 */
AA_API struct aa_rx_mp*
aa_rx_mp_create( const struct aa_rx_sg_sub *sub_sg );

/**
 * Destroy a motion planning context.
 */
AA_API void
aa_rx_mp_destroy( struct aa_rx_mp *mp );

/**
 * Set the motion planning start configuration.
 *
 * \param mp The motion planning context
 * \param n_all Length of array q_all
 * \param q_all Array of start configurations for the entire scene graph.
 */
AA_API void
aa_rx_mp_set_start( struct aa_rx_mp *mp,
                    size_t n_all,
                    double *q_all );


/**
 * Indicate a valid configuration for the motion planning context.
 *
 * All collisions at the given configuration are assumed to be
 * allowed.
 *
 * \param mp The motion planning context
 * \param n_all Length of array q_all
 * \param q_all A valid configuration array (for the entire scene graph).
 */
AA_API void
aa_rx_mp_allow_config( struct aa_rx_mp *mp,
                       size_t n_all,
                       double *q_all );
/**
 * Indicate whether collisions between the two given frames are
 * allowed.
 *
 * By default, no collisions are allowed.
 */
AA_API void
aa_rx_mp_allow_collision( struct aa_rx_mp *mp,
                          aa_rx_frame_id id0, aa_rx_frame_id id1, int allowed );

/**
 * Set the goal as a joint-space configuration.
 */
AA_API int
aa_rx_mp_set_goal( struct aa_rx_mp *mp,
                   size_t n_q,
                   double *q_subset);

/**
 * Set the goal as workspace pose.
 */
AA_API int
aa_rx_mp_set_wsgoal( struct aa_rx_mp *mp,
                     size_t n_e, const aa_rx_frame_id *frames,
                     const double *E, size_t ldE );


/**
 * Set whether to simplify the planned path.
 */
AA_API void
aa_rx_mp_set_simplify( struct aa_rx_mp *mp,
                       int simplify );

/**
 * Execute the planner.
 *
 * \param mp The motion planning context
 *
 * \param timeout Maximum time to execute the planner
 *
 * \param n_path Number of waypoints in the path.
 *
 * \param p_path_all Output path data.  Contains configurations at
 * each waypoint for the entire scene graph.  Size is n_path times the
 * configuration space size of the entire scene graph.
 */

AA_API int
aa_rx_mp_plan( struct aa_rx_mp *mp,
               double timeout,
               size_t *n_path,
               double **p_path_all );

/**
 * Return a pointer to the allowed collision set for the motion
 * planning context.
 */
AA_API struct aa_rx_cl_set* aa_rx_mp_get_allowed( const struct aa_rx_mp* mp);


/*---- RRT -----*/

/**
 * Opaque structure for RRT planner attributes
 */
struct aa_rx_mp_rrt_attr;

/**
 * Create an RRT attribute struct
 */
AA_API struct aa_rx_mp_rrt_attr*
aa_rx_mp_rrt_attr_create(void);

/**
 * Destroy an RRT attribute struct
 */
AA_API void
aa_rx_mp_rrt_attr_destroy(struct aa_rx_mp_rrt_attr*);


/**
 * Whether the RRT should be bidirectional
 */
AA_API void
aa_rx_mp_rrt_attr_set_bidirectional( struct aa_rx_mp_rrt_attr* attrs,
                                     int is_bidirectional );

/**
 * Use the RRT motion planning algorithm
 *
 * @param mp   The motion planning context
 * @param attr Attributes for the planning algorithm (NULL uses defaults)
 */
AA_API void
aa_rx_mp_set_rrt( struct aa_rx_mp* mp,
                  const struct aa_rx_mp_rrt_attr *attr );


/*---- SBL -----*/

/**
 * Opaque structure for SBL planner attributes
 */
struct aa_rx_mp_sbl_attr;

/**
 * Create an SBL attribute struct
 */
AA_API struct aa_rx_mp_sbl_attr*
aa_rx_mp_sbl_attr_create(void);

/**
 * Destroy an SBL attribute struct
 */
AA_API void
aa_rx_mp_sbl_attr_destroy(struct aa_rx_mp_sbl_attr*);

/**
 * Use the SBL motion planning algorithm
 *
 * @param mp   The motion planning context
 * @param attr Attributes for the planning algorithm (NULL uses defaults)
 */
AA_API void
aa_rx_mp_set_sbl( struct aa_rx_mp* mp,
                  const struct aa_rx_mp_sbl_attr *attr );


/*---- KPIECE -----*/

/**
 * Opaque structure for KPIECE planner attributes
 */
struct aa_rx_mp_kpiece_attr;

/**
 * Create an KPIECE attribute struct
 */
AA_API struct aa_rx_mp_kpiece_attr*
aa_rx_mp_kpiece_attr_create(void);

/**
 * Destroy an KPIECE attribute struct
 */
AA_API void
aa_rx_mp_kpiece_attr_destroy(struct aa_rx_mp_kpiece_attr*);

/**
 * Use the KPIECE motion planning algorithm
 *
 * @param mp   The motion planning context
 * @param attr Attributes for the planning algorithm (NULL uses defaults)
 */
AA_API void
aa_rx_mp_set_kpiece( struct aa_rx_mp* mp,
                     const struct aa_rx_mp_kpiece_attr *attr );




#endif /*AMINO_RX_SCENE_PLANNING_H*/
