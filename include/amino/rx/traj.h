#ifndef AMINO_TRAJ_H
#define AMINO_TRAJ_H

/**
 * @file traj.h
 */

// Forward declarations
struct aa_rx_trajpt;
struct aa_rx_trajseg;
struct aa_rx_traj;

/**
 * Structure parameters for a struct aa_rx_trajpt
 * 
 * i          - Index of point in trajectory. Filled by aa_rx_trajpt_add().
 * prev, next - Linked list of adjacent points. Filled by aa_rx_trajpt_add().
 */
#define AA_RX_TRAJPT_FD                         \
    size_t i;                                   \
    struct aa_rx_trajpt *prev, *next;

/**
 * Structure parameters for a struct aa_rx_trajseg
 * 
 * value      - Function pointer to evaluation function.
 *              Called by aa_rx_traj_value()
 * i          - Index of point in trajectory. Filled by aa_rx_trajpt_add()
 * t_s, t_f   - Start and end times of trajectory 
 * prev, next - Linked list of adjacent points. Filled by aa_rx_trajpt_add()
 */
#define AA_RX_TRAJSEG_FD                                    \
    int (*value)(struct aa_rx_traj *traj,                   \
                 struct aa_rx_trajseg *seg,                 \
                 void *v, double t);                        \
    size_t i;                                               \
    double t_s, t_f;                                        \
    struct aa_rx_trajseg *prev, *next;                           

/**
 * Base struct for a trajectory point. Generic functions cast points down to
 * this type. Any trajectory point should begin its fields with AA_RX_TRAJPT_FD,
 * to have similar memory layout.
 */
struct aa_rx_trajpt {
    AA_RX_TRAJPT_FD;
};

/**
 * Base struct for a trajectory segment. Generic functions cast segments down to
 * this type. Any trajectory segment should begin its fields with
 * AA_RX_TRAJSEG_FD, to have similar memory layout.
 */
struct aa_rx_trajseg {
    AA_RX_TRAJSEG_FD;
};

/**
 * Continuous trajectory that can be executed Initialized with
 * aa_rx_traj_init(). Populated further with aa_rx_traj_generate(), which calls
 * the function pointer generate to create the trajectory.
 */
struct aa_rx_traj {
    size_t n_q;                     ///< Number of configurations

    aa_mem_region_t *reg;           ///< Memory region to use

    aa_mem_rlist_t *points;         ///< List of discrete points
    struct aa_rx_trajpt *tail_pt;   ///< Last point in points

    aa_mem_rlist_t *segments;       ///< List of continuous segments
    struct aa_rx_trajseg *tail_seg; ///< Last segment in segments

    struct aa_mem_cons *last_seg;   ///< Last segment evaluated

    double t_o;                     ///< Time offset of trajectory
    int (*generate)(struct aa_rx_traj *traj, void *cx); ///< Generation function
};

/**
 * Initialize a trajectory structure.
 *
 * @param traj       Trajectory structure to initialize
 * @param scenegraph Scenegraph trajectory is for
 * @param reg        Memory region to use for allocation
 * @param generate   Trajectory generation function
 *
 * @pre aa_mem_region_init() has been called for the memory region
 * @pre aa_rx_sg_init() has been called after all frames were added to the
 *      scenegraph.
 */
AA_API void aa_rx_traj_init(struct aa_rx_traj *traj, size_t n_q,
                            aa_mem_region_t *reg,
                            int (*generate)(struct aa_rx_traj *traj, void *cx));

/**
 * Generate an initialized trajectory.
 *
 * @param traj Trajectory to initialize. Calls generate function inside struct
 * @param cx   Context to pass to generation function
 *
 * @return 0 upon success and non-zero on failure.
 *
 * @pre aa_rx_traj_init() has been called.
 * @pre aa_rx_trajpt_add() has been called for all points added.
 */
AA_API int aa_rx_traj_generate(struct aa_rx_traj *traj, void *cx);

/**
 * Evaluates a trajectory at a given time.
 *
 * @param traj Trajectory to evaluate
 * @param v    Value to fill in. Assumes memory has been allocated if needed
 * @param t    Time to evaluate trajectory at
 *
 * @return 1 if value found, 0 if not.
 *
 * @pre aa_rx_traj_generate() has been called.
 * @pre v has been initialized with whatever memory is required.
 */
AA_API int aa_rx_traj_value(struct aa_rx_traj *traj, void *v, double t);

/**
 * Adds a point to a trajectory. Links the next and previous points of the
 * trajectory for traversal. Should be called before aa_rx_traj_generate().
 *
 * @param traj Trajectory to add point to
 * @param pt   Point to add to trajectory
 *
 * @pre aa_rx_traj_init() has been called.
 */
AA_API void aa_rx_trajpt_add(struct aa_rx_traj *traj, struct aa_rx_trajpt *pt);

/**
 * Parabolic Blend Trajectory
 */

struct aa_rx_pb_trajval {
    double *q;
    double *dq;
};

struct aa_rx_pb_trajpt {
    AA_RX_TRAJPT_FD;
    double *q;
};

struct aa_rx_pb_limits {
    double *dqmax;
    double *ddqmax;
};

/**
 * Trajectory generation function for a parabolic blend trajectory.
 * Requires points added to be of type aa_rx_pb_trajpt_t. When evaluated, value
 * must be of type aa_rx_pb_trajval_t.
 *
 * @param traj Trajectory to fill in. 
 * @param cx   Context. Should be of type struct aa_rx_pb_limits. 
 *
 * @return 0 on success, non-zero on failure.
 */
AA_API int aa_rx_pb_traj_generate(struct aa_rx_traj *traj, void *cx);

#endif /*AMINO_TRAJ_H*/
