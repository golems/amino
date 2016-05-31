#ifndef AMINO_TRAJ_INTERNAL_H
#define AMINO_TRAJ_INTERNAL_H

/**
 * @file traj_internal.h
 */

/**
 * Iterate over segment list from last remembered segment until either a value
 * is found or the end of the list is reached. For internal use by
 * aa_rx_traj_value().
 *
 * @param traj Trajectory with segments to iterate over
 * @param v    Value to be filled by segment
 * @param t    Time to evaluate trajectory at
 *
 * @return If value is found 1 is returned. Otherwise 0.
 *
 * @pre aa_rx_traj_generate() has been called.
 */
int aa_rx_traj_value_it(struct aa_rx_traj *traj, void *v, double t);

/**
 * Add a trajectory segment to a trajectory at the end of the current list.
 * Links the next and previous segments of the trajectory segment for traversal.
 * For internal use by aa_rx_traj_generate() to build a trajectory.
 *
 * @param traj The trajectory to add the segment to
 * @param seg  The segment to add
 *
 * @pre aa_rx_traj_init() has been called.
 */
void aa_rx_trajseg_add(struct aa_rx_traj *traj, struct aa_rx_trajseg *seg);

/**
 * Parabolic Blend Trajectory
 */

/**
 * Trajectory segment for a parabolic blend trajectory.
 */
struct aa_rx_pb_trajseg {
    AA_RX_TRAJSEG_FD;
    struct aa_rx_pb_trajpt *pt; ///< Reference to waypoint segment is based on
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
double aa_rx_pb_trajseg_limit(double *a, double *b, double *m, size_t n_q);

/**
 * Evaluates a struct aa_rx_pb_trajseg at time t. Used within the trajectory
 * segment. Called from aa_rx_traj_value().
 *
 * @param traj Trajectory being evaluated
 * @param seg  Segment being evaluated. Of type struct aa_rx_pb_trajseg
 * @param v    Value to fill. Of type struct aa_rx_pb_trajval
 * @param t    Time to evaluate trajectory segment at
 *
 * @return 1 if time within segment, 0 otherwise. v is modified if true.
 */
int aa_rx_pb_trajseg_value(struct aa_rx_traj *traj, struct aa_rx_trajseg *seg,
                           void *v, double t);

/**
 * Creates and allocated a new struct aa_rx_pb_trajseg.
 *
 * @param traj Trajectory to use memory region and n_q from
 *
 * @return A newly allocated struct aa_rx_pb_trajseg.
 */
struct aa_rx_pb_trajseg *aa_rx_pb_trajseg_create(struct aa_rx_traj *traj);

/**
 * Updates values of a struct aa_rx_pb_trajseg based on the connecting waypoints
 * and new timing value seg->dt.
 *
 * @param seg    Segment to update
 * @param ddqmax Acceleration limits
 * @param n_q    Number of configurations
 */
void aa_rx_pb_trajseg_update(struct aa_rx_pb_trajseg *seg, double *ddqmax,
                             size_t n_q);

#endif /*AMINO_TRAJ_INTERNAL_H*/
