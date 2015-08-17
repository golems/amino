/**
 *  Opaque type for a scene_graph
 */
struct scene_graph;

/**
 *  Construct a new scene graph
 */
struct scene_graph *scene_graph_create();

/**
 *  Destroy a scene graph
 */
void scene_graph_destroy(struct *scene_graph scene_graph);

/**
 *  Return the index of a frame in the scene graph
 */
size_t scene_graph_index_frame(struct *scene_graph scene_graph, const char *frame_name);

/**
 *  Return the index of a configuration variable in the scene graph
 */
size_t scene_graph_config_index(struct *scene_graph scene_graph, const char *config_name);

/**
 *  Add a fixed-transform frame to the scene graph
 */
void scene_graph_add_frame_fixed
( struct *scene_graph scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3] );

/**
 *  Add a prismatic-joint frame to the scene graph
 */
void scene_graph_add_frame_prismatic
( struct *scene_graph scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3],
  const char *config_name,
  const double axis[3], double offset );

/**
 *  Add a revolute-joint frame to the scene graph
 */
void scene_graph_add_frame_revolute
( struct *scene_graph scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3],
  const char *config_name,
  const double axis[3], double offset );

/**
 *  Remove a frame
 */
void scene_graph_rm_frame
( struct *scene_graph scene_graph,
  const char *name );

/**
 *  Compute transforms for the scene graph
 */
void scene_graph_tf
( struct *scene_graph scene_graph,
  size_t n_q, double *q,
  size_t n_tf,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs );
