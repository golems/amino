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
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"

namespace amino {

SceneFrame::SceneFrame(
    const char *_name,
    const char *_parent,
    double q[4], double v[3]
    ) :
    name(strdup(_name)),
    parent(strdup(_parent))
{
    AA_MEM_CPY(E+AA_TF_QUTR_Q, q, 4);
    AA_MEM_CPY(E+AA_TF_QUTR_V, v, 3);
}

SceneFrame::~SceneFrame( )
{
    free(name);
    free(parent);
}


SceneFrameJoint::SceneFrameJoint(
    const char *_name,
    const char *_parent,
    double q[4], double v[3],
    const char *_config_name,
    double _offset, double _axis[3]
    ) :
    SceneFrame( _name, _parent, q, v ),
    config_name(strdup(_config_name)),
    offset(_offset)
{
    AA_MEM_CPY(this->axis, _axis, 3);
}


SceneFrameJoint::~SceneFrameJoint()
{
    free(config_name);
}


SceneFrameRevolute::SceneFrameRevolute(
    const char *_name,
    const char *_parent,
    double q[4], double v[3],
    const char *_config_name,
    double _offset, double _axis[3]
    ) :
    SceneFrameJoint( _name, _parent, q, v,
                     _config_name, _offset, _axis)
{ }


SceneFrameRevolute::~SceneFrameRevolute()
{ }

SceneFramePrismatic::SceneFramePrismatic(
    const char *_name,
    const char *_parent,
    double q[4], double v[3],
    const char *_config_name,
    double _offset, double _axis[3]
    ) :
    SceneFrameJoint( _name, _parent, q, v,
                     _config_name, _offset, _axis)
{ }


SceneFramePrismatic::~SceneFramePrismatic()
{ }

void SceneFrame::tf_rel( const double *q, double _E[7] )
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

SceneGraph::SceneGraph() {}

SceneGraph::~SceneGraph()
{
    for( auto itr = frames.begin(); itr != frames.end(); itr++ )
    {
        free( *itr );
    }
}


} /* amino */
