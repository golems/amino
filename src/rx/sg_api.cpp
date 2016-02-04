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


#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"


AA_API struct aa_rx_sg *aa_rx_sg_create()
{
    aa_rx_sg *sg = new aa_rx_sg;
    sg->sg = new amino::SceneGraph;
    return sg;
}

AA_API void aa_rx_sg_destroy(struct aa_rx_sg *scene_graph)
{
    delete scene_graph->sg;
    delete scene_graph;
}

AA_API int aa_rx_sg_init ( struct aa_rx_sg *scene_graph )
{
    return scene_graph->sg->index();
}

AA_API size_t aa_rx_sg_config_count(
    const struct aa_rx_sg *scene_graph )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );
    return scene_graph->sg->config_size;
}

AA_API size_t aa_rx_sg_frame_count(
    const struct aa_rx_sg *scene_graph )
{
    return scene_graph->sg->frames.size();
}

AA_API enum aa_rx_frame_type aa_rx_sg_frame_type (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );
    return scene_graph->sg->frames[frame_id]->type;
}

AA_API aa_rx_config_id aa_rx_sg_config_id(
    const struct aa_rx_sg *scene_graph, const char *config_name)
{
    aa_rx_sg_ensure_clean_frames( scene_graph );
    return scene_graph->sg->config_map[config_name];
}

AA_API aa_rx_frame_id aa_rx_sg_frame_id (
    const struct aa_rx_sg *scene_graph, const char *frame_name)
{
    if( '\0' == *frame_name ) return AA_RX_FRAME_ROOT;

    auto itr = scene_graph->sg->frame_map.find(frame_name);
    if( itr != scene_graph->sg->frame_map.end() ) {
        return itr->second->frame_id;
    }
    return AA_RX_FRAME_NONE;
}

AA_API const char *
aa_rx_sg_frame_name (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id )
{
    return scene_graph->sg->frames[frame_id]->name.c_str();
}

AA_API const char *
aa_rx_sg_config_name (
    const struct aa_rx_sg *scene_graph, aa_rx_config_id id )
{
    return scene_graph->sg->config_rmap[id];
}

AA_API aa_rx_frame_id
aa_rx_sg_frame_parent (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );
    return scene_graph->sg->frames[frame_id]->parent_id;
}


AA_API void aa_rx_sg_add_frame_fixed
( struct aa_rx_sg *scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3] )
{
    scene_graph->sg->add( new amino::SceneFrameFixed(parent, name, q, v) );
}


AA_API void aa_rx_sg_add_frame_prismatic
( struct aa_rx_sg *scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3],
  const char *config_name,
  const double axis[3], double offset )
{
    scene_graph->sg->add( new amino::SceneFramePrismatic( parent, name, q, v,
                                                          config_name, offset, axis) );
}

AA_API void aa_rx_sg_add_frame_revolute
( struct aa_rx_sg *scene_graph,
  const char *parent, const char *name,
  const double q[4], const double v[3],
  const char *config_name,
  const double axis[3], double offset )
{
    scene_graph->sg->add( new amino::SceneFrameRevolute( parent, name, q, v,
                                                         config_name, offset, axis) );
}

AA_API void aa_rx_sg_rm_frame
( struct aa_rx_sg *scene_graph,
  const char *name )
{

    amino::SceneGraph *sg = scene_graph->sg;
    sg->dirty_indices = 1;

    auto itr = sg->frame_map.find(name);
    if( sg->frame_map.end() != itr ) {
        amino::SceneFrame *f = itr->second;
        delete f;
    }

    sg->frame_map.erase(name);
}

AA_API void aa_rx_sg_tf
( const struct aa_rx_sg *scene_graph,
  size_t n_q, const double *q,
  size_t n_tf,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );

    amino::SceneGraph *sg = scene_graph->sg;
    size_t i_frame = 0;
    for( size_t i_rel = 0, i_abs = 0;
         i_frame < n_tf && i_frame < sg->frames.size();
         i_frame++, i_rel += ld_rel, i_abs += ld_abs )
    {
        amino::SceneFrame *f = sg->frames[(aa_rx_frame_id)i_frame];
        double *E_rel = TF_rel + i_rel;
        double *E_abs = TF_abs + i_abs;
        // compute relative
        f->tf_rel( q, E_rel );
        // chain to global
        if( f->in_global() ) {
            // TODO: can we somehow get rid of this branch?
            //       maybe a separate type for global frames
            AA_MEM_CPY(E_abs, E_rel, 7);
        } else {
            assert( f->parent_id < (ssize_t)i_frame );
            double *E_abs_parent = TF_abs + (ld_abs * f->parent_id);;
            aa_tf_qutr_mul(E_abs_parent, E_rel, E_abs);
        }
    }
}



AA_API void aa_rx_sg_tf_update
( const struct aa_rx_sg *scene_graph,
  size_t n_q,
  const double *q0,
  const double *q,
  size_t n_tf,
  const double *TF_rel0, size_t ld_rel0,
  const double *TF_abs0, size_t ld_abs0,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );

    amino::SceneGraph *sg = scene_graph->sg;

    bool updated[sg->frames.size()];

    size_t i_frame = 0;
    for( size_t i_rel0 = 0, i_abs0 = 0,
             i_rel = 0, i_abs = 0;
         i_frame < n_tf && i_frame < sg->frames.size();
         i_frame++, i_rel += ld_rel, i_abs += ld_abs,
             i_rel0 += ld_rel0, i_abs0 += ld_abs0
        )
    {
        amino::SceneFrame *f = sg->frames[(aa_rx_frame_id)i_frame];
        enum aa_rx_frame_type type = f->type;
        bool update_abs = 0;
        bool in_global =  f->in_global();

        const double *E_rel0 = TF_rel0 + i_rel0;
        const double *E_abs0 = TF_abs0 + i_abs0;
        double *E_rel  = TF_rel + i_rel;
        double *E_abs  = TF_abs + i_abs;

        switch( type ) {
        case AA_RX_FRAME_FIXED:
            f->tf_rel(q, E_rel);
            update_abs = !in_global && updated[f->parent_id];
            break;
        case AA_RX_FRAME_REVOLUTE:
        case AA_RX_FRAME_PRISMATIC: {
            amino::SceneFrameJoint *fj = static_cast<amino::SceneFrameJoint*>(f);
            if( aa_feq(q0[fj->config_index], q[fj->config_index], 0 )) {
                AA_MEM_CPY(E_rel, E_rel0, 7);
                update_abs = !in_global && updated[f->parent_id];
            } else {
                f->tf_rel(q, E_rel);
                update_abs = 1;
            }
            break; }
        }

        if( update_abs ) {
            // chain to global
            if( in_global ) {
                // TODO: can we somehow get rid of this branch?
                //       maybe a separate type for global frames
                AA_MEM_CPY(E_abs, E_rel, 7);
            } else {
                assert( f->parent_id < (ssize_t)i_frame );
                double *E_abs_parent = TF_abs + (ld_abs * f->parent_id);;
                aa_tf_qutr_mul(E_abs_parent, E_rel, E_abs);
            }
            updated[i_frame] = 1;
        } else {
            AA_MEM_CPY(E_abs, E_abs0, 7);
            updated[i_frame] = 0;
        }
    }
}


AA_API void aa_rx_sg_map_geom (
    const struct aa_rx_sg *scene_graph,
    void (*function)(void *context, aa_rx_frame_id frame_id, struct aa_rx_geom *geom),
    void *context )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );
    amino::SceneGraph *sg = scene_graph->sg;
    for( auto itr = sg->frames.begin(); itr != sg->frames.end(); itr++ ) {
        amino::SceneFrame *f = *itr;
        for( auto g = f->geometry.begin(); g != f->geometry.end(); g++ ) {
            function(context, f->frame_id, *g);
        }
    }
}



AA_API void
aa_rx_sg_config_indices(
    const struct aa_rx_sg *scene_graph, size_t n,
    const char **config_name, aa_rx_config_id *ids )
{
    for( size_t i = 0; i < n; i ++ ) {
        ids[i] = aa_rx_sg_config_id( scene_graph, config_name[i] );
    }
}



AA_API void
aa_rx_sg_config_get(
    const struct aa_rx_sg *scene_graph, size_t n_all, size_t n_subset,
    const aa_rx_config_id *ids,
    const double *config_all,
    double *config_subset )
{
    (void) scene_graph;
    for( size_t i = 0; i < n_subset; i ++ ) {
        config_subset[i] = config_all[ ids[i] ];
    }
}

AA_API void
aa_rx_sg_config_set(
    const struct aa_rx_sg *scene_graph, size_t n_all, size_t n_subset,
    const aa_rx_config_id *ids, const double *config_subset,
    double *config_all
    )
{
    (void) scene_graph;
    for( size_t i = 0; i < n_subset; i ++ ) {
        config_all[ ids[i] ] = config_subset[i];
    }
}

AA_API const double *
aa_rx_sg_frame_axis ( const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame )
{
    amino::SceneGraph *sg = scene_graph->sg;
    amino::SceneFrameJoint *f = static_cast<amino::SceneFrameJoint*> (sg->frames[frame]);
    return f->axis;
}

AA_API aa_rx_config_id
aa_rx_sg_frame_config (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame)
{
    amino::SceneGraph *sg = scene_graph->sg;
    amino::SceneFrame *f = sg->frames[frame];
    switch( f->type ) {
    case AA_RX_FRAME_FIXED:
        return AA_RX_CONFIG_NONE;
    case AA_RX_FRAME_REVOLUTE:
    case AA_RX_FRAME_PRISMATIC: {
        amino::SceneFrameJoint *fj = static_cast<amino::SceneFrameJoint*>(f);
        return (aa_rx_config_id)(fj->config_index);
    }
    }
    assert(0);
}

static struct aa_rx_config_limits *
get_limits( struct aa_rx_sg *scenegraph,
            const char *config_name )
{
    amino::SceneGraph *sg = scenegraph->sg;
    struct aa_rx_config_limits *l = sg->limits_map[config_name];
    if( NULL == l ) {
        l = AA_NEW0(struct aa_rx_config_limits);
        sg->limits_map[config_name] = l;
    }

    return l;
}

#define DEF_SET_LIMIT(value)                                            \
    AA_API void                                                         \
    aa_rx_sg_set_limit_ ## value( struct aa_rx_sg *scenegraph,          \
                                  const char *config_name,              \
                                  double min, double max )              \
    {                                                                   \
        struct aa_rx_config_limits *l =                                 \
            get_limits(scenegraph,config_name);                         \
        l->has_##value = 1;                                             \
        l->value##_min = min;                                           \
        l->value##_max = max;                                           \
    }                                                                   \

DEF_SET_LIMIT(pos)
DEF_SET_LIMIT(vel)
DEF_SET_LIMIT(acc)
DEF_SET_LIMIT(eff)

#define DEF_GET_LIMIT(value)                                            \
    AA_API int                                                          \
    aa_rx_sg_get_limit_ ## value( const struct aa_rx_sg *scenegraph,    \
                                  aa_rx_config_id config_id,            \
                                  double *min, double *max )            \
    {                                                                   \
        amino::SceneGraph *sg = scenegraph->sg;                         \
        struct aa_rx_config_limits *l = sg->limits[config_id];          \
        if( l->has_##value ) {                                          \
            *min = l->value##_min;                                      \
            *max = l->value##_max;                                      \
            return 0;                                                   \
        } else {                                                        \
            return -1;                                                  \
        }                                                               \
    }

DEF_GET_LIMIT(pos)
DEF_GET_LIMIT(vel)
DEF_GET_LIMIT(acc)
DEF_GET_LIMIT(eff)


/* Inertial */

AA_API void
aa_rx_sg_frame_set_inertial( struct aa_rx_sg *scenegraph,
                             const char *frame,
                             double mass,
                             const double inertia[9] )
{
    amino::SceneGraph *sg = scenegraph->sg;
    struct amino::SceneFrame *f = sg->frame_map[frame];
    if( NULL == f->inertial ) {
        f->inertial = AA_NEW(struct aa_rx_inertial);
    }
    f->inertial->mass = mass;
    AA_MEM_CPY( f->inertial->inertia, inertia, 9 );
}

AA_API double
aa_rx_sg_frame_get_mass( struct aa_rx_sg *scenegraph,
                         aa_rx_frame_id frame )
{
    amino::SceneGraph *sg = scenegraph->sg;
    struct amino::SceneFrame *f = sg->frames[frame];
    if( f->inertial ) {
        return f->inertial->mass;
    } else {
        return nan("");
    }
}

AA_API const double*
aa_rx_sg_frame_get_inertia( struct aa_rx_sg *scenegraph,
                            aa_rx_frame_id frame )
{
    amino::SceneGraph *sg = scenegraph->sg;
    struct amino::SceneFrame *f = sg->frames[frame];
    if( f->inertial ) {
        return f->inertial->inertia;
    } else {
        return NULL;
    }
}

AA_API void
aa_rx_sg_set_destructor(
    const struct aa_rx_sg *scene_graph,
    void (*destructor)(void*),
    void *context )
{
    scene_graph->sg->destructor = destructor;
    scene_graph->sg->destructor_context = context;
}
