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

AA_API void aa_rx_sg_index ( struct aa_rx_sg *scene_graph )
{
    scene_graph->sg->index();
}

AA_API aa_rx_config_id aa_rx_sg_config_count(
    const struct aa_rx_sg *scene_graph )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );
    return scene_graph->sg->config_size;
}

AA_API aa_rx_config_id aa_rx_sg_frame_count(
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
    aa_rx_sg_ensure_clean_frames( scene_graph );
    return scene_graph->sg->frame_map[frame_name]->frame_id;
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

AA_API void aa_rx_sg_tf
( const struct aa_rx_sg *scene_graph,
  aa_rx_config_id n_q, double *q,
  aa_rx_frame_id n_tf,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );

    amino::SceneGraph *sg = scene_graph->sg;
    aa_rx_frame_id i_frame = 0;
    for( size_t i_rel = 0, i_abs = 0;
         i_frame < n_tf && (size_t)i_frame < sg->frames.size();
         i_frame++, i_rel += ld_rel, i_abs += ld_abs )
    {
        amino::SceneFrame *f = sg->frames[i_frame];
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
