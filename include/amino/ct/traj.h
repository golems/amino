/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016 Rice University
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

#ifndef AMINO_CT_TRAJ_H
#define AMINO_CT_TRAJ_H

/**
 * @file traj.h
 */

#ifdef __cplusplus
extern "C" {
#endif

#define AA_CT_SEG_IN   1
#define AA_CT_SEG_OUT  0

/**
 * Waypoint. For use in aa_ct_pt_list.
 */
struct aa_ct_pt {
    struct aa_ct_state state;     ///< Description of state at waypoint
    struct aa_ct_pt *prev, *next; ///< Links to next and previous points
};

/**
 * Waypoint list.
 */
struct aa_ct_pt_list;

/**
 * Trajectory segment list.
 */
struct aa_ct_seg_list;


/**
 * Initialize and construct a point list from a memory region.
 *
 * @param reg Memory region to allocate from
 *
 * @return A freshly allocated struct aa_ct_pt_list.
 */
struct aa_ct_pt_list *aa_ct_pt_list_create(struct aa_mem_region *reg);

/**
 * Add a reference to a waypoint to a point list. The reference will be kept in
 * the list.
 *
 * @param list  List to add point to
 * @param state State to add to list
 */
void aa_ct_pt_list_add(struct aa_ct_pt_list *list, struct aa_ct_state *state);

/**
 * Destroys an allocated point list.
 *
 * @param list List to destroy
 */
void aa_ct_pt_list_destroy(struct aa_ct_pt_list *list);

/**
 * Return the initial state of the point list.
 */
const struct aa_ct_state *
aa_ct_pt_list_start_state(const struct aa_ct_pt_list *list);

/**
 * Return the final state of the point list.
 */
const struct aa_ct_state *
aa_ct_pt_list_final_state(const struct aa_ct_pt_list *list);


/**
 * Print out a list of points to a file.
 *
 * @param stream File to print to
 * @param list   Point list to print
 */
void aa_ct_pt_list_dump(FILE *stream, struct aa_ct_pt_list *list);

/**
 * Evaluates a segment list at a given time. Fills in the provided state struct
 * With the reference state at that time.
 *
 * @param list  Segment list to evaluate
 * @param state State structure to fill in
 * @param t     Time to evaluate segment list at
 *
 * @return 1 if time is within segment list, 0 if not.
 */
int aa_ct_seg_list_eval(struct aa_ct_seg_list *list, struct aa_ct_state *state,
                        double t);

/**
 * Plots a segment list with a given resolution. Pipes commands to gnuplot.
 *
 * @param list Segment list to plot
 * @param n_q  Number of configurations
 * @param dt   Timestep to plot
 */
void aa_ct_seg_list_plot(struct aa_ct_seg_list *list, size_t n_q, double dt);

/**
 * Destroys an allocated segment list.
 *
 * @param list List to destroy
 */
void aa_ct_seg_list_destroy(struct aa_ct_seg_list *list);

/**
 * Return segment list configuration count.
 */
size_t aa_ct_seg_list_n_q(const struct aa_ct_seg_list *list);

/**
 * Return duration (time) of segment list.
 */
double aa_ct_seg_list_duration(const struct aa_ct_seg_list *list);

/**
 * Generate a parabolic blend trajectory from a point list.
 *
 * @param reg Region to allocate from
 * @param list Point list to build segment list from
 * @param limits State structure with dq and ddq kinematic limits
 *
 * @return An allocated segment list describing a parabolic blend trajectory.
 */
struct aa_ct_seg_list *aa_ct_tjq_pb_generate(struct aa_mem_region *reg,
                                             struct aa_ct_pt_list *list,
                                             struct aa_ct_state *limits);


/**
 * Generate a lienar trajectory from a point list.
 *
 * @param reg Region to allocate from
 * @param list Point list to build segment list from
 * @param limits State structure with dq and ddq kinematic limits
 *
 * @return An allocated segment list describing a parabolic blend trajectory.
 */
struct aa_ct_seg_list *aa_ct_tjq_lin_generate(struct aa_mem_region *reg,
                                              struct aa_ct_pt_list *list,
                                              struct aa_ct_state *limits);

/**
 * Generate a parabolic blend trajectory from a point list.
 *
 * @param reg Region to allocate from
 * @param list Point list to build segment list from
 * @param limits State structure with dq and ddq kinematic limits
 *
 * @return An allocated segment list describing a parabolic blend trajectory.
 */
struct aa_ct_seg_list *aa_ct_tjX_pb_generate(struct aa_mem_region *reg,
                                             struct aa_ct_pt_list *list,
                                             struct aa_ct_state *limits);


/**
 * Check C0 (position) continuity of the trajectory.
 *
 * @param segs The segment list to check
 * @param dt   Time intervel between steps
 * @param tol  Maximum distance between two steps
 * @param eps  Maximum distance between final and inital position of subsequent segments
 *
 * @return 0 if trajectory is C0 continuous, non-zero otherwise
 */
int aa_ct_seg_list_check_c0( struct aa_ct_seg_list * segs, double dt,
                             double tol, double eps );

#ifdef __cplusplus
}
#endif

#endif
