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

#ifndef AMINO_SCENEGRAPH_H
#define AMINO_SCENEGRAPH_H

/**
 * @file scenegraph.h
 */

/**
 * Type for frame indices.
 */
typedef signed long aa_rx_frame_id;

/**
 * Type for configuration indices.
 */
typedef signed long aa_rx_config_id;

/**
 * Magic frame_id for the root/global/absolute frame.
 */
#define AA_RX_FRAME_ROOT ((aa_rx_frame_id)-1)

#define AA_RX_FRAME_NONE ((aa_rx_frame_id)-2)

/**
 * Magic config_id for no configuration variable.
 */
#define AA_RX_CONFIG_NONE ((aa_rx_config_id)-1)
#define AA_RX_CONFIG_MULTI ((aa_rx_config_id)-2)

/**
 * Enum of frame types
 */
enum aa_rx_frame_type {
    AA_RX_FRAME_FIXED,       /**< A fixed transform */
    AA_RX_FRAME_REVOLUTE,    /**< A rotating transform */
    AA_RX_FRAME_PRISMATIC,   /**< A prismatic (sliding) transform */
};

/**
 *  Construct a new, empty scene graph
 */
AA_API struct aa_rx_sg *
aa_rx_sg_create();

/**
 *  Destroy a scene graph
 */
AA_API void aa_rx_sg_destroy(struct aa_rx_sg *scene_graph);

/**
 * Setup the scenegraph internal indices.
 *
 * This function must be called before any frame_ids or config_ids can
 * be used with the scenegraph.
 */
AA_API int aa_rx_sg_init ( struct aa_rx_sg *scene_graph );

/**
 * Return the type of the given frame
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API enum aa_rx_frame_type
aa_rx_sg_frame_type (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id );

/**
 * Return the name of the given frame
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API const char *
aa_rx_sg_frame_name (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id );

/**
 * Return the config of the given frame
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API const char *
aa_rx_sg_config_name (
    const struct aa_rx_sg *scene_graph, aa_rx_config_id config_id );

/**
 * Return the parent id of the frame
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API aa_rx_frame_id
aa_rx_sg_frame_parent (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id );


/**
 * Return the number of frames in scene_graph.
 */
AA_API size_t
aa_rx_sg_frame_count (
    const struct aa_rx_sg *scene_graph );

/**
 * Return the number of configuration variables in scene_graph.
 */
AA_API size_t
aa_rx_sg_config_count (
    const struct aa_rx_sg *scene_graph );

/**
 * Return the config id of frame.
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API aa_rx_config_id
aa_rx_sg_frame_config (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame);

/**
 * Fill names with pointers to config names
 *
 */
AA_API size_t
aa_rx_sg_config_names (
    const struct aa_rx_sg *scene_graph, size_t n_names,
    const char **names );

/**
 *  Return the index of a configuration variable in the scene graph
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API aa_rx_config_id aa_rx_sg_config_id(
    const struct aa_rx_sg *scene_graph, const char *config_name);

/**
 *  Return the indices of a configuration variable in the scene graph
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API void
aa_rx_sg_config_indices(
    const struct aa_rx_sg *scene_graph, size_t n,
    const char **config_name, aa_rx_config_id *ids );


AA_API void
aa_rx_sg_config_get(
    const struct aa_rx_sg *scene_graph, size_t n_all, size_t n_subset,
    const aa_rx_config_id *ids,
    const double *config_all,
    double *config_subset );

AA_API void
aa_rx_sg_config_set(
    const struct aa_rx_sg *scene_graph, size_t n_all, size_t n_subset,
    const aa_rx_config_id *ids, const double *config_subset,
    double *config_all
    );

/**
 *  Return the index of a frame in the scene graph
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API aa_rx_frame_id aa_rx_sg_frame_id (
    const struct aa_rx_sg *scene_graph, const char *frame_name);


/* /\** */
/*  *  Return the index of a configuration variable for the given frame. */
/*  *\/ */
/* AA_API aa_rx_config_id aa_rx_sg_frame_config_id( */
/*     struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id); */

/**
 *  Add a fixed-transform frame to the scene graph
 *
 * Note that adding a new frame may changes the frame_ids of all
 * previously added frames.
 *
 * @param scene_graph The scene graph container
 * @param parent      The name of the parent frame
 * @param name        The name of the frame to be added
 * @param q           The unit quaternion frame rotation (xyzw)
 * @param v           The frame translation vector
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API void aa_rx_sg_add_frame_fixed
( struct aa_rx_sg *scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3] );

/**
 *  Add a prismatic-joint frame to the scene graph
 *
 * Note that adding a new frame may changes the frame_ids and
 * config_ids of all previously added frames.
 *
 * @param scene_graph The scene graph container
 * @param parent      The name of the parent frame
 * @param name        The name of the frame to be added
 * @param q           The unit quaternion frame initial rotation (xyzw)
 * @param v           The frame initial translation vector
 * @param axis        The axis of rotation.  A non-unit axis will
 *                    scale the translation accordingly.
 * @param offset      An offset to be added to the configuration value
 */
AA_API void aa_rx_sg_add_frame_prismatic
( struct aa_rx_sg *scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3],
  const char *config_name,
  const double axis[3], double offset );

/**
 *  Add a revolute-joint frame to the scene graph
 *
 * Note that adding a new frame may changes the frame_ids and
 * config_ids of all previously added frames.
 *
 * @param scene_graph The scene graph container
 * @param parent      The name of the parent frame
 * @param name        The name of the frame to be added
 * @param q           The unit quaternion frame initial rotation (xyzw)
 * @param v           The frame initial translation vector
 * @param axis        The axis of rotation.  A non-unit axis will
 *                    scale the rotation accordingly.
 * @param offset      An offset to be added to the configuration value
 */
AA_API void aa_rx_sg_add_frame_revolute
( struct aa_rx_sg *scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3],
  const char *config_name,
  const double axis[3], double offset );

/**
 *  Remove a frame
 */
AA_API void aa_rx_sg_rm_frame
( struct aa_rx_sg *scene_graph,
  const char *name );

/**
 * Set position limit values
 */
AA_API void
aa_rx_sg_set_limit_pos( struct aa_rx_sg *scenegraph,
                        const char *config_name,
                        double min, double max );

/**
 * Set velocity limit values
 */
AA_API void
aa_rx_sg_set_limit_vel( struct aa_rx_sg *scenegraph,
                        const char *config_name,
                        double min, double max );

/**
 * Set acceleration limit values
 */
AA_API void
aa_rx_sg_set_limit_acc( struct aa_rx_sg *scenegraph,
                        const char *config_name,
                        double min, double max );

/**
 * Set effort limit values
 */
AA_API void
aa_rx_sg_set_limit_eff( struct aa_rx_sg *scenegraph,
                        const char *config_name,
                        double min, double max );
/**
 * Get position limit values
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API int
aa_rx_sg_get_limit_pos( const struct aa_rx_sg *scenegraph,
                        aa_rx_config_id config_id,
                        double *min, double *max );

/**
 * Get velocity limit values
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API int
aa_rx_sg_get_limit_vel( const struct aa_rx_sg *scenegraph,
                        aa_rx_config_id config_id,
                        double *min, double *max );

/**
 * Get acceleration limit values
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API int
aa_rx_sg_get_limit_acc( const struct aa_rx_sg *scenegraph,
                        aa_rx_config_id config_id,
                        double *min, double *max );

/**
 * Get effort limit values
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API int
aa_rx_sg_get_limit_eff( const struct aa_rx_sg *scenegraph,
                        aa_rx_config_id config_id,
                        double *min, double *max );


/**
 * Return pointer to frame axis.
 *
 * Only valid for univariate joint frames.
 */
AA_API const double *aa_rx_sg_frame_axis
( const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame );

/**
 *  Compute transforms for the scene graph
 *
 * Transform entries are in {q_x, q_y, q_z, q_w, v_x, v_y, v_z}
 * format.
 *
 * @param scene_graph The scene graph container
 * @param n_q         Size of configuration vector q
 * @param q           Configuraiton vector
 * @param n_tf        Number of entries in the TF array
 * @param TF_rel      Relative transform matrix in quaternion-vector format
 * @param ld_rel      Leading dimensional of TF_rel, i.e., space between each entry
 * @param TF_abs      Absolute transform matrix in quaternion-vector format
 * @param ld_abs      Leading dimensional of TF_abs, i.e., space between each entry
 *
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API void aa_rx_sg_tf
( const struct aa_rx_sg *scene_graph,
  size_t n_q, const double *q,
  size_t n_tf,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs );

AA_API void aa_rx_sg_tf_update
( const struct aa_rx_sg *scene_graph,
  size_t n_q,
  const double *q0,
  const double *q,
  size_t n_tf,
  const double *TF_rel0, size_t ld_rel0,
  const double *TF_abs0, size_t ld_abs0,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs );



/**
 * Call function for every geometry object in the scene graph
 *
 * @param scene_graph The scene graph container
 * @param function    A function to call on every geometry object
 * @param context     The context argument to function
 *
 * @pre aa_rx_sg_init() has been called after all frames were added to
 * the scenegraph.
 */
AA_API void aa_rx_sg_map_geom (
    const struct aa_rx_sg *scene_graph,
    void (*function)(void *context, aa_rx_frame_id frame_id, struct aa_rx_geom *geom),
    void *context );

/**
  * Get transform between two given frames
  *
  * @param scene_graph The scene graph container
  * @param frame_from
  * @param frame_to
  * @param q           Current configs
  * @param tf_rel      Relative transform from the from frame to the to frame
  */


AA_API void aa_rx_sg_get_tf (
  const struct aa_rx_sg *scene_graph,
  const aa_rx_frame_id frame_from,
  const aa_rx_frame_id frame_to,
  const double * tf_abs,
  double * from_tf_to);

AA_API void aa_rx_sg_reparent (
			       const struct aa_rx_sg *scene_graph,
			       const aa_rx_frame_id frame,
			       const aa_rx_frame_id new_parent,
			       const double * q);

AA_API  struct aa_rx_sg *  aa_rx_sg_copy( const struct aa_rx_sg * orig);

#endif /*AMINO_SCENEGRAPH_H*/
