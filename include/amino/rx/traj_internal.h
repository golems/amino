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

struct aa_rx_pb_trajseg;
struct aa_rx_pb_trajseg {
    AA_RX_TRAJSEG_FD;       ///< Basic aa_rx_trajseg 
    struct aa_rx_pb_trajpt *pt;
    double *dq, *ddq;
    double dt, b;
};


double aa_rx_pb_trajseg_limit(double *a, double *b, double *m, size_t n_q);

int aa_rx_pb_trajseg_value(struct aa_rx_traj *traj, struct aa_rx_trajseg *seg,
                           void *v, double t);

struct aa_rx_pb_trajseg *aa_rx_pb_trajseg_create(struct aa_rx_traj *traj);

void aa_rx_pb_trajseg_generate(struct aa_rx_pb_trajseg *seg, double *amax,
                               size_t n_q);

#endif /*AMINO_TRAJ_INTERNAL_H*/
