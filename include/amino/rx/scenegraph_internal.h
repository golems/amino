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

#ifdef __cplusplus

#include <vector>
#include <string>
#include <map>




namespace amino {


struct SceneFrame  {
    SceneFrame( const char *parent,
                const char *name,
                const double q[4], const double v[3] );
    virtual ~SceneFrame();

    virtual void tf_rel( const double *q, double E[7] ) = 0;
    virtual aa_rx_frame_type type() = 0;
    int in_global();

    std::string name;
    std::string parent;
    aa_rx_frame_id frame_id;
    aa_rx_frame_id parent_id;
    size_t config_index;
    double E[7];
    void *data;
};


struct SceneFrameFixed : public SceneFrame {
    SceneFrameFixed( const char *parent,
                     const char *name,
                     const double q[4], const double v[3] );
    virtual ~SceneFrameFixed();
    virtual void tf_rel( const double *q, double E[7] );
    virtual aa_rx_frame_type type();
};

struct SceneFrameJoint : public SceneFrame {
    SceneFrameJoint( const char *parent,
                     const char *name,
                     const double q[4], const double v[3],
                     const char *config_name,
                     double offset, const double axis[3] );
    virtual ~SceneFrameJoint();

    std::string config_name;
    size_t config_index;
    double offset;
    double axis[3];
};

struct SceneFramePrismatic : public SceneFrameJoint {
    SceneFramePrismatic( const char *parent,
                         const char *name,
                         const double q[4], const double v[3],
                         const char *config_name,
                         double offset, const double axis[3] );
    virtual ~SceneFramePrismatic();
    virtual void tf_rel( const double *q, double E[7] );
    virtual aa_rx_frame_type type();
};


struct SceneFrameRevolute : public SceneFrameJoint {
    SceneFrameRevolute( const char *parent,
                        const char *name,
                        const double q[4], const double v[3],
                        const char *config_name,
                        double offset, const double axis[3] );
    virtual ~SceneFrameRevolute();
    virtual void tf_rel( const double *q, double E[7] );
    virtual aa_rx_frame_type type();
};

struct SceneGraph  {
    SceneGraph();
    ~SceneGraph();

    void index();
    void add(SceneFrame *f);

    /** Array of frames */
    std::vector<SceneFrame*> frames;

    /** Map from frame name to frame */
    std::map<std::string,SceneFrame*> frame_map;

    /** Map from configuration name to configuration index */
    std::map<std::string,size_t> config_map;

    /** Number of configuration variables */
    aa_rx_config_id config_size;

    /** Are the indices invalid? */
    int dirty_indices;
};

}


struct aa_rx_sg {
    amino::SceneGraph *sg;
};



#endif

#endif /*AMINO_SCENEGRAPH_H*/
