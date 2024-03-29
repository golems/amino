/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@mines.edu>
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
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_fk.h"
#include "amino/mat_internal.h"


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
    if( scene_graph ) {
        aa_rx_sg_ensure_clean_frames( scene_graph );
        return scene_graph->sg->config_size;
    } else {
        return 0;
    }
}

AA_API size_t aa_rx_sg_frame_count(
    const struct aa_rx_sg *scene_graph )
{
    return scene_graph ? scene_graph->sg->frames.size() : 0;
}

AA_API enum aa_rx_frame_type aa_rx_sg_frame_type (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );
    return scene_graph->sg->frames[(size_t)frame_id]->type;
}

AA_API aa_rx_config_id aa_rx_sg_config_id(
    const struct aa_rx_sg *scene_graph, const char *config_name)
{
    aa_rx_sg_ensure_clean_frames( scene_graph );
    if(scene_graph->sg->config_map.count(config_name)){
        return scene_graph->sg->config_map[config_name];
    }
    return AA_RX_CONFIG_NONE;
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
    aa_rx_sg_ensure_clean_frames( scene_graph );


    switch(frame_id) {
        case AA_RX_FRAME_ROOT: return "";
        case AA_RX_FRAME_NONE: return "?";
        default:
            if (frame_id >= (aa_rx_frame_id)aa_rx_sg_frame_count(scene_graph) ||
                frame_id < 0) {
                return "?";
            }
            return scene_graph->sg->frames[(size_t)frame_id]->name.c_str();
    }
}

AA_API const char *
aa_rx_sg_config_name (
    const struct aa_rx_sg *scene_graph, aa_rx_config_id id )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );

    switch(id) {
        case AA_RX_CONFIG_NONE: return "NONE";
        case AA_RX_CONFIG_MULTI: return "MULTI";
        default:
            if (id < 0 ||
                id >= (aa_rx_config_id)aa_rx_sg_frame_count(scene_graph)) {
                return "NONE";
            }
            return scene_graph->sg->config_rmap[(size_t)id];
    }
}

AA_API aa_rx_frame_id
aa_rx_sg_frame_parent (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame_id )
{
    aa_rx_sg_ensure_clean_frames( scene_graph );

    switch(frame_id) {
        case AA_RX_FRAME_NONE:
        case AA_RX_FRAME_ROOT:
            return AA_RX_FRAME_NONE;
        default:
            return scene_graph->sg->frames[(size_t)frame_id]->parent_id;
    }
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

// AA_API void
// aa_rx_sg_tf_alloc( const struct aa_rx_sg *scene_graph,
//                    struct aa_mem_region *reg,
//                    double **TF_rel, size_t *ld_rel,
//                    double **TF_abs, size_t *ld_abs)
// {

//     size_t ld_tf = 14;
//     size_t n_f = aa_rx_sg_frame_count(scene_graph);
//     double *TF = AA_MEM_REGION_NEW_N(reg, double, ld_tf*n_f);
//     *TF_rel = TF, *TF_abs = TF+ld_tf/2;
//     *ld_rel = ld_tf;
//     *ld_abs = ld_tf;
// }


// AA_API void
// aa_rx_sg_tf_pop( const struct aa_rx_sg *scene_graph,
//                  struct aa_mem_region *reg,
//                  double *tf_rel, double *tf_abs )
// {
//     (void)scene_graph;
//     (void)tf_abs;
//     aa_mem_region_pop(reg,tf_rel);
// }


AA_API void aa_rx_sg_tf
( const struct aa_rx_sg *scene_graph,
  size_t n_q, const double *q,
  size_t n_tf,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs )
{
    if( NULL == scene_graph ) return;

    aa_rx_sg_ensure_clean_frames( scene_graph );
    assert( n_q == scene_graph->sg->config_size );

    amino::SceneGraph *sg = scene_graph->sg;


    double *E_rel = TF_rel, *E_abs = TF_abs;
    for( size_t i_frame = 0;
         i_frame < n_tf && i_frame < sg->frames.size();
         i_frame++,
             E_rel += ld_rel, E_abs += ld_abs )
    {
        amino::SceneFrame *f = sg->frames[i_frame];
        // compute relative
        f->tf_rel( q, E_rel );
        // chain to global
        if( f->in_global() ) {
            // TODO: can we somehow get rid of this branch?
            //       maybe a separate type for global frames
            AA_MEM_CPY(E_abs, E_rel, 7);
        } else {
            assert( f->parent_id < (ssize_t)i_frame );
            double *E_abs_parent = TF_abs + (ld_abs * (size_t)f->parent_id);
            aa_tf_qutr_mul(E_abs_parent, E_rel, E_abs);
        }
    }

}


AA_API void
aa_rx_sg_fill_tf_abs( const struct aa_rx_sg *scene_graph,
                      const struct aa_dvec *q,
                      struct aa_dmat *TF_abs )
{
    size_t n_f = aa_rx_sg_frame_count(scene_graph);
    struct aa_mem_region *reg = aa_mem_region_local_get();
    struct aa_dmat *TF_rel = aa_dmat_alloc(reg, 7, n_f);
    struct aa_dvec *qp;
    if( 1 != q->inc ) {
        qp = aa_dvec_alloc(reg,q->len);
        aa_dvec_copy(q,qp);
    } else {
        qp = (struct aa_dvec*)q;
    }

    aa_rx_sg_tf( scene_graph,
                 qp->len, qp->data,
                 n_f,
                 TF_rel->data, TF_rel->ld,
                 TF_abs->data, TF_abs->ld );

    aa_mem_region_pop(reg,TF_rel);
}

AA_API struct aa_dmat *
aa_rx_sg_get_tf_abs ( const struct aa_rx_sg *scene_graph,
                  struct aa_mem_region *reg,
                  const struct aa_dvec *q )
{
    size_t n_f = aa_rx_sg_frame_count(scene_graph);
    struct aa_dmat *TF_abs = aa_dmat_alloc(reg, 7, n_f);
    aa_rx_sg_fill_tf_abs(scene_graph,q,TF_abs);
    return TF_abs;
}

AA_API void aa_rx_sg_tfmat
( const struct aa_rx_sg *scene_graph,
  size_t n_q, const double *q,
  size_t n_tf,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs )
{
    if( NULL == scene_graph ) return;

    aa_rx_sg_ensure_clean_frames( scene_graph );
    assert( n_q == scene_graph->sg->config_size );

    amino::SceneGraph *sg = scene_graph->sg;
    size_t i_frame = 0;
    for( size_t i_rel = 0, i_abs = 0;
         i_frame < n_tf && i_frame < sg->frames.size();
         i_frame++, i_rel += ld_rel, i_abs += ld_abs )
    {
        amino::SceneFrame *f = sg->frames[i_frame];
        double *E_rel = TF_rel + i_rel;
        double *E_abs = TF_abs + i_abs;
        // compute relative
        f->tfmat_rel( q, E_rel );
        // chain to global
        if( f->in_global() ) {
            // TODO: can we somehow get rid of this branch?
            //       maybe a separate type for global frames
            AA_MEM_CPY(E_abs, E_rel, 12);
        } else {
            assert( f->parent_id < (ssize_t)i_frame );
            double *E_abs_parent = TF_abs + (ld_abs * (size_t)f->parent_id);
            aa_tf_tfmat_mul(E_abs_parent, E_rel, E_abs);
        }
    }
}

AA_API void aa_rx_sg_duqu
( const struct aa_rx_sg *scene_graph,
  size_t n_q, const double *q,
  size_t n_tf,
  double *TF_rel, size_t ld_rel,
  double *TF_abs, size_t ld_abs )
{

    if( NULL == scene_graph ) return;

    aa_rx_sg_ensure_clean_frames( scene_graph );
    assert( n_q == scene_graph->sg->config_size );

    amino::SceneGraph *sg = scene_graph->sg;
    size_t i_frame = 0;
    for( size_t i_rel = 0, i_abs = 0;
         i_frame < n_tf && i_frame < sg->frames.size();
         i_frame++, i_rel += ld_rel, i_abs += ld_abs )
    {
        amino::SceneFrame *f = sg->frames[i_frame];
        double *E_rel = TF_rel + i_rel;
        double *E_abs = TF_abs + i_abs;
        // compute relative
        f->duqu_rel( q, E_rel );
        // chain to global
        if( f->in_global() ) {
            // TODO: can we somehow get rid of this branch?
            //       maybe a separate type for global frames
            AA_MEM_CPY(E_abs, E_rel, 8);
        } else {
            assert( f->parent_id < (ssize_t)i_frame );
            double *E_abs_parent = TF_abs + (ld_abs * (size_t)f->parent_id);
            aa_tf_duqu_mul(E_abs_parent, E_rel, E_abs);
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
    assert( n_q == scene_graph->sg->config_size );

    bool updated[sg->frames.size()];

    const double *E_rel0 = TF_rel0;
    const double *E_abs0 = TF_abs0;
    double *E_rel  = TF_rel;
    double *E_abs  = TF_abs;
    for( size_t i_frame = 0;
         i_frame < n_tf && i_frame < sg->frames.size();
         i_frame++,
             E_rel0 += ld_rel0,
             E_abs0 += ld_abs0,
             E_rel += ld_rel,
             E_abs += ld_abs
        )
    {
        amino::SceneFrame *f = sg->frames[i_frame];
        enum aa_rx_frame_type type = f->type;
        bool update_abs = 0;
        bool in_global =  f->in_global();

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
                double *E_abs_parent = TF_abs + (ld_abs * (size_t)f->parent_id);
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
    if( scene_graph ) {
        aa_rx_sg_ensure_clean_frames( scene_graph );
        amino::SceneGraph *sg = scene_graph->sg;
        for( auto itr = sg->frames.begin(); itr != sg->frames.end(); itr++ ) {
            amino::SceneFrame *f = *itr;
            for( auto g = f->geometry.begin(); g != f->geometry.end(); g++ ) {
                function(context, f->frame_id, *g);
            }
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
    amino::SceneFrameJoint *f = static_cast<amino::SceneFrameJoint*> (sg->frames[(size_t)frame]);
    return f->axis;
}

AA_API aa_rx_config_id
aa_rx_sg_frame_config (
    const struct aa_rx_sg *scene_graph, aa_rx_frame_id frame)
{
    amino::SceneGraph *sg = scene_graph->sg;
    amino::SceneFrame *f = sg->frames[(size_t)frame];
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
        struct aa_rx_config_limits *l = sg->limits[(size_t)config_id];  \
        if( l && l->has_##value ) {                                     \
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


AA_API double
aa_rx_sg_config_center( const struct aa_rx_sg *sg, aa_rx_config_id config_id )
{
    double min=0 ,max=0;
    int r = aa_rx_sg_get_limit_pos( sg, config_id, &min, &max );
    return ( 0 == r )
        ? ((max + min) / 2)
        : 0;
}

AA_API void
aa_rx_sg_center_configs( const struct aa_rx_sg *sg,
                             size_t n, double *q )
{
    size_t n_qs = aa_rx_sg_config_count(sg);
    size_t n_min = AA_MIN(n,n_qs);
    for( size_t i = 0; i < n_min; i ++ ) {
        q[i] = aa_rx_sg_config_center(sg,  (aa_rx_config_id)i);
    }
}

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
    aa_rx_sg_ensure_clean_frames( scenegraph );

    amino::SceneGraph *sg = scenegraph->sg;
    struct amino::SceneFrame *f = sg->frames[(size_t)frame];
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
    aa_rx_sg_ensure_clean_frames( scenegraph );

    amino::SceneGraph *sg = scenegraph->sg;
    struct amino::SceneFrame *f = sg->frames[(size_t)frame];
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

AA_API void aa_rx_sg_rel_tf (
  const struct aa_rx_sg *scene_graph,
  const aa_rx_frame_id frame_from,
  const aa_rx_frame_id frame_to,
  const double * tf_abs,
  size_t ld_abs,
  double *from_tf_to)
{
    aa_rx_sg_ensure_clean_frames( scene_graph );

    if (frame_from == AA_RX_FRAME_ROOT){
        if (frame_to == AA_RX_FRAME_ROOT){
            AA_MEM_CPY( from_tf_to,
                        aa_tf_qutr_ident,
                        7 );
        } else {
            AA_MEM_CPY( from_tf_to,
                        &tf_abs[(size_t)frame_to * ld_abs],
                        7 );
        }
    } else {
        if (frame_to == AA_RX_FRAME_ROOT){
            aa_tf_qutr_conj( &tf_abs[(size_t)frame_from * ld_abs],
                             from_tf_to );
        } else {
            aa_tf_qutr_cmul( &tf_abs[(size_t)frame_from * ld_abs],
                             &tf_abs[(size_t)frame_to * ld_abs],
                             from_tf_to );
        }
    }
}

AA_API void aa_rx_sg_reparent_id (const struct aa_rx_sg *scene_graph,
                                  aa_rx_frame_id new_parent,
                                  aa_rx_frame_id frame,
                                  const double * E1)
{
    aa_rx_sg_reparent_name( scene_graph,
                            aa_rx_sg_frame_name(scene_graph, new_parent),
                            aa_rx_sg_frame_name(scene_graph, frame),
                            E1 );
}

AA_API void aa_rx_sg_reparent_name (const struct aa_rx_sg *scene_graph,
                                    const char *new_parent,
                                    const char *frame,
                                    const double * E1)
{
    amino::SceneFrame *f = scene_graph->sg->frame_map[frame];

    f->parent = ( (NULL == new_parent || '\0' == new_parent[0])
                  ? ""
                  : new_parent );

    AA_MEM_CPY(f->E, E1, 7);

    scene_graph->sg->dirty_indices = 1;
}

struct sg_copy_geom_cx{
    const aa_rx_sg *sg_old;
    aa_rx_sg *sg_new;
};

static void sg_copy_geom(void *context, aa_rx_frame_id frame_id, struct aa_rx_geom *geom0)
{
    sg_copy_geom_cx * cx = (sg_copy_geom_cx* ) context;
    struct aa_rx_geom *geom1 = aa_rx_geom_copy(geom0);
    aa_rx_geom_attach( cx->sg_new,
                       aa_rx_sg_frame_name(cx->sg_old, frame_id),
                       geom1);
};

AA_API  struct aa_rx_sg *  aa_rx_sg_copy( const struct aa_rx_sg * orig){
    aa_rx_sg_ensure_clean_frames( orig );

    aa_rx_sg * dest = aa_rx_sg_create();

    // add frames
    for (size_t i=0; i<orig->sg->frames.size(); i++){
        amino::SceneFrame * f = orig->sg->frames[i];
        amino::SceneFrame * f_new = NULL;
        switch (f->type){
        case AA_RX_FRAME_FIXED: {
            f_new = new amino::SceneFrameFixed(f->parent.c_str(), f->name.c_str(), f->E + AA_TF_QUTR_Q, f->E + AA_TF_QUTR_V);
            } break;
        case AA_RX_FRAME_REVOLUTE: {
            amino::SceneFrameRevolute * fr = (amino::SceneFrameRevolute*) f;
            f_new = new amino::SceneFrameRevolute(fr->parent.c_str(), fr->name.c_str(),
                fr->E + AA_TF_QUTR_Q, fr->E + AA_TF_QUTR_V,
                fr->config_name.c_str(), fr->offset, fr->axis);
            } break;
        case AA_RX_FRAME_PRISMATIC: {
            amino::SceneFramePrismatic * fp = (amino::SceneFramePrismatic*) f;
            f_new = new amino::SceneFramePrismatic(fp->parent.c_str(), fp->name.c_str(),
                fp->E + AA_TF_QUTR_Q, fp->E + AA_TF_QUTR_V,
                fp->config_name.c_str(), fp->offset, fp->axis);
            } break;
        }
        assert(f_new);
        dest->sg->add(f_new);
    }

    // set limits
    for( auto &pair : orig->sg->limits_map ) {
        if ( pair.second ) {
            aa_rx_config_limits * nl = new aa_rx_config_limits();
            *nl = *pair.second;
            dest->sg->limits_map[pair.first] = nl;
        }else{
            dest->sg->limits_map[pair.first] = NULL;
        }
    }

    // set geometries
    sg_copy_geom_cx cx;
    cx.sg_old = orig;
    cx.sg_new = dest;
    aa_rx_sg_map_geom(orig, sg_copy_geom, &cx);

    // set allowed collision
    for (auto &pair : orig->sg->allowed){
        aa_rx_sg_allow_collision_name(dest, pair.first, pair.second, 1);
    }

    // initialize sg
    dest->sg->index();

    return dest;

}

AA_API void
aa_rx_sg_allow_collision( struct aa_rx_sg *scene_graph,
                          aa_rx_frame_id id0, aa_rx_frame_id id1, int allowed )
{
    aa_rx_sg_allow_collision_name(scene_graph,
                                  aa_rx_sg_frame_name(scene_graph, id0),
                                  aa_rx_sg_frame_name(scene_graph, id1), allowed);
}

AA_API void
aa_rx_sg_allow_collision_name( struct aa_rx_sg *scene_graph,
                               const char* frame0, const char* frame1, const int allowed )
{
    bool order = strcmp(frame0, frame1) <0;
    const char *string0, *string1;
    if (strcmp(frame0, frame1)<0){
        string0 = scene_graph->sg->frame_map[frame0]->name.c_str();
        string1 = scene_graph->sg->frame_map[frame1]->name.c_str();
    } else {
        string0 = scene_graph->sg->frame_map[frame1]->name.c_str();
        string1 = scene_graph->sg->frame_map[frame0]->name.c_str();
    }
    std::pair<const char*, const char*> p(string0, string1);
    if (allowed){
        scene_graph->sg->allowed.insert(p);
    } else {
        scene_graph->sg->allowed.erase(p);
    }
    scene_graph->sg->dirty_collision = 1;
}

AA_API double *
aa_rx_sg_alloc_tf ( const struct aa_rx_sg *sg, struct aa_mem_region *region )
{
    return AA_MEM_REGION_NEW_N( region, double, 7*aa_rx_sg_frame_count(sg) );
}

AA_API double *
aa_rx_sg_alloc_config ( const struct aa_rx_sg *sg, struct aa_mem_region *region )
{
    return AA_MEM_REGION_NEW_N( region, double, aa_rx_sg_config_count(sg) );
}



struct aa_rx_fk *
aa_rx_fk_alloc(const struct aa_rx_sg *scene_graph, struct aa_mem_region *reg)
{
    struct aa_rx_fk *fk = AA_MEM_REGION_NEW(reg,struct aa_rx_fk);
    fk->sg = scene_graph;
    fk->TF_abs = AA_MEM_REGION_NEW_N( reg,double,
                                      AA_RX_FK_LD*aa_rx_sg_frame_count(scene_graph));
    fk->in_heap = 0;
    return fk;
}

/**
 * Heap allocate a forward kinematics struct.
 */
struct aa_rx_fk *
aa_rx_fk_malloc(const struct aa_rx_sg *scene_graph)
{
    struct aa_rx_fk *fk = AA_NEW(struct aa_rx_fk);
    fk->sg = scene_graph;
    fk->TF_abs = AA_NEW_AR(double, AA_RX_FK_LD*aa_rx_sg_frame_count(scene_graph));
    fk->in_heap = 1;
    return fk;
}

AA_API void
aa_rx_fk_cpy(struct aa_rx_fk *dst, const struct aa_rx_fk *src)
{
    assert( src->sg == dst->sg );
    AA_MEM_CPY( dst->TF_abs, src->TF_abs,
                AA_RX_FK_LD * aa_rx_sg_frame_count(src->sg) );
}

AA_API double *
aa_rx_fk_ref(const struct aa_rx_fk *fk, aa_rx_frame_id id)
{
    return AA_RX_FK_REF(fk, (size_t)(id));
}

AA_API void
aa_rx_fk_get_abs_qutr(const struct aa_rx_fk *fk, aa_rx_frame_id id, double E[7] )
{
    if( id < 0 || (size_t)id > aa_rx_fk_cnt(fk) ) {
        fprintf(stderr, "ERROR: Invalid frame id %d\n", id);
        abort();
        exit(EXIT_FAILURE);
    }
    AA_MEM_CPY(E, AA_RX_FK_REF(fk, (size_t)(id)), AA_RX_TF_LEN);
}


AA_API void
aa_rx_fk_get_rel_qutr(const struct aa_rx_fk *fk,
                      aa_rx_frame_id parent, aa_rx_frame_id child,
                      double E[7])
{
    const double *gEp = (AA_RX_FRAME_ROOT == parent) ?
        aa_tf_qutr_ident : AA_RX_FK_REF(fk, (size_t)(parent));

    const double *gEc = (AA_RX_FRAME_ROOT == child) ?
        aa_tf_qutr_ident : AA_RX_FK_REF(fk, (size_t)(child));

    aa_tf_qutr_cmul(gEp, gEc, E);
}

/**
 * Destroy a malloc'ed struct aa_rx_fk.
 */
AA_API void
aa_rx_fk_destroy(struct aa_rx_fk * fk)
{
    if( fk->in_heap ) {
        free(fk->TF_abs);
        free(fk);
    } else {
        fprintf(stderr, "ERROR: attempting to free non-heap allocated aa_rx_fk.");
        abort();
        exit(EXIT_FAILURE);
    }
}

AA_API void
aa_rx_fk_set_rel(struct aa_rx_fk *fk, aa_rx_frame_id id, const double E_rel[AA_RX_TF_LEN])
{
    aa_rx_frame_id parent = aa_rx_sg_frame_parent(fk->sg,id);
    double *E_abs = aa_rx_fk_ref(fk, id);
    if( AA_RX_FRAME_ROOT ==  parent ) {
        // TODO: can we somehow get rid of this branch?
        //       maybe a separate type for global frames
        AA_MEM_CPY(E_abs, E_rel, AA_RX_TF_LEN);
    } else {
        double *E_abs_parent = aa_rx_fk_ref(fk, parent);
        aa_tf_qutr_mul(E_abs_parent, E_rel, E_abs);
    }
}

/**
 * Compute the forward kinematics.
 */
AA_API void
aa_rx_fk_all( struct aa_rx_fk *fk,
              const struct aa_dvec *q )

{
    const struct aa_rx_sg *scenegraph = fk->sg;
    aa_rx_sg_ensure_clean_frames(scenegraph);
    aa_dvec_check_size( aa_rx_sg_config_count(scenegraph), q );

    amino::SceneGraph *sg = scenegraph->sg;

    double E_rel[AA_RX_TF_LEN], *E_abs = fk->TF_abs;
    for( size_t i_frame = 0;
         i_frame < sg->frames.size();
         i_frame++, E_abs += AA_RX_FK_LD )
    {
        amino::SceneFrame *f = sg->frames[i_frame];
        f->tf_rel( q->data, E_rel );
        aa_rx_fk_set_rel(fk,i_frame,E_rel);
    }
}


/** Pointer to FK data */
AA_API double *
aa_rx_fk_data( const struct aa_rx_fk *fk )
{
    return fk->TF_abs;
}

/** Leading dimension of FK data */
AA_API size_t
aa_rx_fk_ld( const struct aa_rx_fk *fk )
{
    return AA_RX_FK_LD;
}

AA_API size_t
aa_rx_fk_cnt( const struct aa_rx_fk *fk )
{
    return aa_rx_sg_frame_count(fk->sg);
}
