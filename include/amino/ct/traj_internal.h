/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
 * All rights reserved.
 *
 * Author(s): Zachary K. Kingston <zak@rice.edu>
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

#ifndef AMINO_TRAJ_INTERNAL_H
#define AMINO_TRAJ_INTERNAL_H

/**
 * @file traj_internal.h
 */

/**
 * Iterate over segment list from last remembered segment until either a value
 * is found or the end of the list is reached. For internal use by
 * aa_ct_traj_value().
 *
 * @param traj Trajectory with segments to iterate over
 * @param v    Value to be filled by segment
 * @param t    Time to evaluate trajectory at
 *
 * @return If value is found 1 is returned. Otherwise 0.
 *
 * @pre aa_ct_traj_generate() has been called.
 */
int aa_ct_traj_value_it(struct aa_ct_traj *traj, void *v, double t);

/**
 * Add a trajectory segment to a trajectory at the end of the current list.
 * Links the next and previous segments of the trajectory segment for traversal.
 * For internal use by aa_ct_traj_generate() to build a trajectory.
 *
 * @param traj The trajectory to add the segment to
 * @param seg  The segment to add
 *
 * @pre aa_ct_traj_init() has been called.
 */
void aa_ct_trajseg_add(struct aa_ct_traj *traj, struct aa_ct_trajseg *seg);

/**
 * Parabolic Blend Trajectory
 */

/**
 * Trajectory segment for a parabolic blend trajectory.
 */
struct aa_ct_pb_trajseg {
    AA_CT_TRAJSEG_FD;
    struct aa_ct_pb_trajpt *pt; ///< Reference to waypoint segment is based on
    double *dq;                 ///< Velocity vector
    double *ddq;                ///< Acceleration vector
    double dt;                  ///< Total segment time
    double b;                   ///< Blend time
};

/**
 * Calculate the maximum value of (a - b) / m elementwise for n_q elements.
 * Utility function for parabolic blend trajectory generation.
 *
 * @param a   The first vector
 * @param b   The second vector
 * @param m   The denominator vector
 * @param n_q Number of elements in the vectors
 *
 * @return The maximum value of (a - b) / m elementwise.
 */
double aa_ct_pb_trajseg_limit(double *a, double *b, double *m, size_t n_q);

/**
 * Evaluates a struct aa_ct_pb_trajseg at time t. Used within the trajectory
 * segment. Called from aa_ct_traj_value().
 *
 * @param traj Trajectory being evaluated
 * @param seg  Segment being evaluated. Of type struct aa_ct_pb_trajseg
 * @param v    Value to fill. Of type struct aa_ct_pb_trajval
 * @param t    Time to evaluate trajectory segment at
 *
 * @return 1 if time within segment, 0 otherwise. v is modified if true.
 */
int aa_ct_pb_trajseg_value(struct aa_ct_traj *traj, struct aa_ct_trajseg *seg,
                           void *v, double t);

/**
 * Creates and allocated a new struct aa_ct_pb_trajseg.
 *
 * @param traj Trajectory to use memory region and n_q from
 *
 * @return A newly allocated struct aa_ct_pb_trajseg.
 */
struct aa_ct_pb_trajseg *aa_ct_pb_trajseg_create(struct aa_ct_traj *traj);

/**
 * Updates values of a struct aa_ct_pb_trajseg based on the connecting waypoints
 * and new timing value seg->dt.
 *
 * @param seg    Segment to update
 * @param ddqmax Acceleration limits
 * @param n_q    Number of configurations
 */
void aa_ct_pb_trajseg_update(struct aa_ct_pb_trajseg *seg, double *ddqmax,
                             size_t n_q);

#endif /*AMINO_TRAJ_INTERNAL_H*/
