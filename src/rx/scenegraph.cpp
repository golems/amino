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
#include "amino/rx/rxerr.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_geom.h"
#include "sg_convenience.h"

#include <list>
#include <set>


namespace amino {

SceneFrame::SceneFrame(
    enum aa_rx_frame_type type_,
    const char *_parent,
    const char *_name,
    const double q[4], const double v[3]
    ) :
    type(type_),
    name(_name),
    parent(_parent),
    inertial(NULL)
{
    AA_MEM_CPY(E+AA_TF_QUTR_Q, q ? q : aa_tf_quat_ident, 4);
    AA_MEM_CPY(E+AA_TF_QUTR_V, v ? v : aa_tf_vec_ident, 3);
}

SceneFrame::~SceneFrame( )
{
    /* Delete Geometry */
    for( struct aa_rx_geom *g : geometry ) {
        aa_rx_geom_destroy(g);
    }

    /* Delete Inertial */
    aa_checked_free(inertial);
}


int SceneFrame::in_global()
{
    return 0 == parent.size();
}

SceneFrameFixed::SceneFrameFixed(
    const char *_parent,
    const char *_name,
    const double q[4], const double v[3]
    ) :
    SceneFrame( AA_RX_FRAME_FIXED, _parent, _name, q, v )
{ }


SceneFrameFixed::~SceneFrameFixed()
{ }

SceneFrameJoint::SceneFrameJoint(
    enum aa_rx_frame_type type_,
    const char *_parent,
    const char *_name,
    const double q[4], const double v[3],
    const char *_config_name,
    double _offset, const double _axis[3]
    ) :
    SceneFrame( type_, _parent, _name, q, v ),
    config_name(_config_name ? _config_name : _name),
    offset(_offset)
{
    AA_MEM_CPY(this->axis, _axis, 3);
}


SceneFrameJoint::~SceneFrameJoint()
{ }


SceneFrameRevolute::SceneFrameRevolute(
    const char *_parent,
    const char *_name,
    const double q[4], const double v[3],
    const char *_config_name,
    double _offset, const double _axis[3]
    ) :
    SceneFrameJoint( AA_RX_FRAME_REVOLUTE,
                     _parent, _name, q, v,
                     _config_name, _offset, _axis)
{ }


SceneFrameRevolute::~SceneFrameRevolute()
{ }

SceneFramePrismatic::SceneFramePrismatic(
    const char *_parent,
    const char *_name,
    const double q[4], const double v[3],
    const char *_config_name,
    double _offset, const double _axis[3]
    ) :
    SceneFrameJoint( AA_RX_FRAME_PRISMATIC,
                     _parent, _name, q, v,
                     _config_name, _offset, _axis)
{ }


SceneFramePrismatic::~SceneFramePrismatic()
{ }

void SceneFrameFixed::tf_rel( const double *q, double _E[7] )
{
    (void)q;
    AA_MEM_CPY(_E, this->E, 7);
}

void SceneFramePrismatic::tf_rel( const double *q, double _E[7] )
{
    (void)q;
    double x[3];
    double qo = q[this->config_index] + this->offset;

    for( size_t i = 0; i < 3; i ++ ) {
        x[i] = qo*this->axis[i];
    }
    // TODO: remove redundant operations
    aa_tf_qv_chain( E + AA_TF_QUTR_Q, E+AA_TF_QUTR_V,
                    aa_tf_quat_ident, x,
                    _E + AA_TF_QUTR_Q, _E+AA_TF_QUTR_V );
}

void SceneFrameRevolute::tf_rel( const double *q, double _E[7] )
{
    (void)q;
    double h[4];
    double a[3];
    double qo = q[this->config_index] + this->offset;
    for( size_t i = 0; i < 3; i ++ ) {
        a[i] = qo * axis[i];
    }
    // TODO: call qln directly
    aa_tf_rotvec2quat(a, h);
    // TODO: remove redundant operation
    aa_tf_qv_chain( E + AA_TF_QUTR_Q, E+AA_TF_QUTR_V,
                    h, aa_tf_vec_ident,
                    _E + AA_TF_QUTR_Q, _E+AA_TF_QUTR_V );
}

// aa_rx_frame_type SceneFrameFixed::type()
// {
//     return AA_RX_FRAME_FIXED;
// }

// aa_rx_frame_type SceneFrameRevolute::type()
// {
//     return AA_RX_FRAME_REVOLUTE;
// }

// aa_rx_frame_type SceneFramePrismatic::type()
// {
//     return AA_RX_FRAME_PRISMATIC;
// }

SceneGraph::SceneGraph()
    : dirty_indices(0),
      destructor(NULL)
{}

SceneGraph::~SceneGraph()
{
    /* User destructor(s) */
    if( destructor ) {
        destructor(destructor_context);
    }

    /* Delete Frames */
    for( auto &pair : frame_map ) delete pair.second;

    /* Delete Limits */
    for( auto &pair : limits_map ) free(pair.second);
}

static void sort_frame_helper( std::list<SceneFrame*> &list,
                               std::set<std::string> &visited,
                               std::map<std::string,SceneFrame*> &map,
                               std::string &name )
{
    if( 0 == name.size() ||  // root frame
        visited.find(name) != visited.end()  // already visited
        ) { return; }
    // mark visited
    visited.insert(name);
    SceneFrame *f = map[name];
    // visit parent
    sort_frame_helper(list,visited,map,f->parent);
    // append
    list.push_back(f);
}

int SceneGraph::index()
{
    if( ! dirty_indices ) return 0;

    // Check parents
    for( auto itr = frame_map.begin(); itr != frame_map.end(); itr++ ) {
        SceneFrame *f = itr->second;
        if( 0 != f->parent.size() ) {
            if( frame_map.end() == frame_map.find(f->parent) ) {
                return AA_RX_INVALID_FRAME;
            }
        }
    }

    // Sort frames
    std::list<SceneFrame*> list;
    std::set<std::string> visited;
    for( auto itr = frame_map.begin(); itr != frame_map.end(); itr++ ) {
        SceneFrame *f = itr->second;
        // invalidate indices
        f->frame_id = f->parent_id = (size_t)-1;
        // Recursive sort
        sort_frame_helper( list, visited, frame_map, f->name );
    }

    // Index names and configs
    frames.resize( frame_map.size() );
    {
        std::set<std::string> config_set;
        config_map.clear();
        config_rmap.clear();
        size_t i_frame=0;
        config_size = 0;
        for( auto itr = list.begin();
             itr != list.end();
             itr++, i_frame++ )
        {
            SceneFrame *f = *itr;
            frames[i_frame] = f;
            f->frame_id = i_frame;
            if( f->in_global() ) {
                f->parent_id = AA_RX_FRAME_ROOT;
            } else {
                f->parent_id = frame_map[f->parent]->frame_id;
            }
            assert( f->parent_id < f->frame_id );
            switch( f->type ) {
            case AA_RX_FRAME_FIXED:
                break;
            case AA_RX_FRAME_REVOLUTE:
            case AA_RX_FRAME_PRISMATIC: {
                SceneFrameJoint *fj = static_cast<SceneFrameJoint*>(f);
                const std::string &config_name = fj->config_name;
                if( config_set.end() == config_set.find(config_name) ) {
                    // not in set
                    config_set.insert( config_name );
                    config_rmap.push_back(config_name.c_str());
                    limits.push_back( limits_map[config_name] );
                    config_map[config_name] = config_size;
                    config_size++;
                }
                fj->config_index = config_map[config_name];
                break;
            }
            }
        }
    }

    dirty_indices = 0;
    return 0;
}


void SceneGraph::add(SceneFrame *f)
{
    dirty_indices = 1;

    /* delete if already exists */
    auto itr = frame_map.find(f->name);
    if( frame_map.end() != itr ) {
        amino::SceneFrame *old_f = itr->second;
        delete old_f;
    }

    frame_map[f->name] = f;
}

} /* amino */


void
aa_rx_sg_add_geom( aa_rx_sg *scene_graph, const char *frame,
                   struct aa_rx_geom* geom )
{
    aa_rx_sg_dirty_geom( scene_graph );
    aa_rx_scene_frame *f = aa_rx_sg_find(scene_graph, frame);
    f->geometry.push_back(geom);
}

AA_API void
aa_rx_sg_dirty_geom( struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    sg->dirty_gl = 1;
    sg->dirty_collision = 1;
}

AA_API void
aa_rx_sg_ensure_clean_frames( const struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    if( sg->dirty_indices ) {
        fprintf(stderr, "ERROR: scene graph indices are dirty.  Must call aa_rx_sg_init()\n");
        abort();
        exit(EXIT_FAILURE);
    }
}

AA_API void
aa_rx_sg_ensure_clean_gl( const struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    if( sg->dirty_gl ) {
        fprintf(stderr, "ERROR: scene graph GL data is dirty.  Must call aa_rx_sg_gl_init()\n");
        abort();
        exit(EXIT_FAILURE);
    }
}

AA_API void
aa_rx_sg_ensure_clean_collision( const struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    if( sg->dirty_collision ) {
        fprintf(stderr, "ERROR: scene graph collision data is dirty.  Must call aa_rx_sg_cl_init()\n");
        abort();
        exit(EXIT_FAILURE);
    }
}

AA_API void
aa_rx_sg_clean_gl( struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    sg->dirty_gl = 0;
}


AA_API void
aa_rx_sg_clean_collision( struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    sg->dirty_collision = 0;
}

AA_API int
aa_rx_sg_is_clean_gl( struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    return sg->dirty_gl ? 0 : 1;
}


AA_API int
aa_rx_sg_is_clean_collision( struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    return sg->dirty_collision ? 0 : 1;
}

AA_API int
aa_rx_sg_is_clean( struct aa_rx_sg *scene_graph )
{
    amino::SceneGraph *sg = scene_graph->sg;
    return sg->dirty_indices ? 0 : 1;
}
