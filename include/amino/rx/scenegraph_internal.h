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

#ifndef AMINO_SCENEGRAPH_INTERNAL_H
#define AMINO_SCENEGRAPH_INTERNAL_H

#include <vector>

struct aa_rx_sg { };
struct aa_rx_sg_frame { };


#ifdef __cplusplus

namespace amino {


struct SceneFrame : public aa_rx_sg_frame {
    SceneFrame( const char *name,
                const char *parent,
                double q[4], double v[3] );
    virtual ~SceneFrame();

    virtual void tf_rel( const double *q, double E[7] );

    char *name;
    char *parent;
    size_t parent_index;
    double E[7];
    void *data;
};

struct SceneFrameJoint : public SceneFrame {
    SceneFrameJoint( const char *name,
                     const char *parent,
                     double q[4], double v[3],
                     const char *config_name,
                     double offset, double axis[3] );
    virtual ~SceneFrameJoint();

    char *config_name;
    size_t config_index;
    double offset;
    double axis[3];
};

struct SceneFramePrismatic : public SceneFrameJoint {
    SceneFramePrismatic( const char *name,
                         const char *parent,
                         double q[4], double v[3],
                         const char *config_name,
                         double offset, double axis[3] );
    virtual ~SceneFramePrismatic();
    virtual void tf_rel( const double *q, double E[7] );
};


struct SceneFrameRevolute : public SceneFrameJoint {
    SceneFrameRevolute( const char *name,
                        const char *parent,
                        double q[4], double v[3],
                        const char *config_name,
                        double offset, double axis[3] );
    virtual ~SceneFrameRevolute();
    virtual void tf_rel( const double *q, double E[7] );
};

struct SceneGraph : public aa_rx_sg {
    SceneGraph();
    ~SceneGraph();

    std::vector<SceneFrame*> frames;
};

}
#endif

#endif /*AMINO_SCENEGRAPH_H*/
