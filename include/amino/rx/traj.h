#ifndef AMINO_TRAJ_H
#define AMINO_TRAJ_H

/**
 * @file traj.h
 */

/**
 * Structure parameters for an struct aa_rx_trajpt
 */
#define AA_RX_TRAJPT_FD                         \
    size_t i;                                   \
    struct aa_rx_trajpt *prev, *next; 

/**
 * Structure parameters for an struct aa_rx_trajseg
 */
#define AA_RX_TRAJSEG_FD                                    \
    int (*value)(struct aa_rx_traj *traj,                   \
                 struct aa_rx_trajseg *seg,                 \
                 void *v, double t);                        \
    size_t i;                                               \
    double t_s, t_f;                                        \
    struct aa_rx_trajseg *prev, *next;                           


struct aa_rx_trajpt;
struct aa_rx_trajseg;
struct aa_rx_traj;

struct aa_rx_trajpt {
    AA_RX_TRAJPT_FD;
};

struct aa_rx_trajseg {
    AA_RX_TRAJSEG_FD;
};

/**
 * Continuous trajectory that can be executed Initialized with
 * aa_rx_traj_init(). Populated further with aa_rx_traj_generate(), which calls
 * the function pointer generate to create the trajectory.
 */
struct aa_rx_traj {
    struct aa_rx_sg *scenegraph;
    size_t n_q;

    aa_mem_region_t *reg;           ///< Memory region to use.

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
AA_API void aa_rx_traj_init(struct aa_rx_traj *traj, struct aa_rx_sg *scenegraph,
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

/**
 * Trajectory generation function for a parabolic blend trajectory.
 * Requires points added to be of type aa_rx_pb_trajpt_t. When evaluated, value
 * must be of type aa_rx_pb_trajval_t.
 *
 * @param traj Trajectory to fill in. 
 * @param cx   Context. Used as acceleration limits. Should be of type double[]
 *             and of size equal to the number of configurations used in
 *             aa_rx_pb_trajpt_t.
 *
 * @return 0 on success, non-zero on failure.
 */
AA_API int aa_rx_pb_traj_generate(struct aa_rx_traj *traj, void *cx);

#endif /*AMINO_TRAJ_H*/
