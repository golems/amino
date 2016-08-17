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


struct aa_rx_config_limits {
    double pos_min;
    double pos_max;

    double vel_min;
    double vel_max;

    double acc_min;
    double acc_max;

    double eff_min;
    double eff_max;

    unsigned has_pos : 1;
    unsigned has_vel : 1;
    unsigned has_acc : 1;
    unsigned has_eff : 1;
};

struct aa_rx_inertial {
    double inertia[9];
    double mass;
};

#ifdef __cplusplus

#include <vector>
#include <string>
#include <map>
#include <set>



namespace amino {


struct SceneFrame  {
    SceneFrame( enum aa_rx_frame_type type_,
                const char *parent,
                const char *name,
                const double q[4], const double v[3] );
    virtual ~SceneFrame() ;

    virtual void tf_rel( const double *q, double E[7] ) = 0;
    //virtual aa_rx_frame_type type() = 0;
    int in_global();

    enum aa_rx_frame_type type;
    struct aa_rx_inertial *inertial;

    /* Kinematic values */
    std::string name;
    std::string parent;
    aa_rx_frame_id frame_id;
    aa_rx_frame_id parent_id;
    double E[7];


    /* Geometry */
    std::vector<struct aa_rx_geom*> geometry;
};


struct SceneFrameFixed : public SceneFrame {
    SceneFrameFixed( const char *parent,
                     const char *name,
                     const double q[4], const double v[3] );
    virtual ~SceneFrameFixed();
    virtual void tf_rel( const double *q, double E[7] );
    //virtual aa_rx_frame_type type();
};

struct SceneFrameJoint : public SceneFrame {
    SceneFrameJoint( enum aa_rx_frame_type type,
                     const char *parent,
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
    //virtual aa_rx_frame_type type();
};


struct SceneFrameRevolute : public SceneFrameJoint {
    SceneFrameRevolute( const char *parent,
                        const char *name,
                        const double q[4], const double v[3],
                        const char *config_name,
                        double offset, const double axis[3] );
    virtual ~SceneFrameRevolute();
    virtual void tf_rel( const double *q, double E[7] );
    //virtual aa_rx_frame_type type();
};


struct SceneGraph  {
    SceneGraph();
    ~SceneGraph();

    int index();
    void add(SceneFrame *f);

    /** Map from frame name to frame */
    std::map<std::string,SceneFrame*> frame_map;

    /** Map from configuration name to configuration limits */
    std::map<std::string,struct aa_rx_config_limits*> limits_map;

    /** Array of frames */
    std::vector<SceneFrame*> frames;

    /** Array of configuration limits */
    std::vector<struct aa_rx_config_limits*> limits;

    /** Map from configuration name to configuration index */
    std::map<std::string,aa_rx_config_id> config_map;

    /** Map from configuration index to configuration name */
    std::vector<const char *> config_rmap;

    /** Number of configuration variables */
    size_t config_size;

    /** Set of allowable collision frames by name */
    std::set<std::pair<const char*,const char*> > allowed;

    /** List of allowable collisions by indices */
    std::vector<size_t> allowed_indices1;
    std::vector<size_t> allowed_indices2;

    void (*destructor)(void *);
    void *destructor_context;

    /** Are the indices invalid? */
    unsigned dirty_indices : 1;
    unsigned dirty_collision : 1;
    unsigned dirty_gl : 1;
};

}


struct aa_rx_sg {
    amino::SceneGraph *sg;
};

#endif /* __cplusplus */

AA_API void
aa_rx_sg_add_geom( struct aa_rx_sg *scene_graph, const char *frame,
                   struct aa_rx_geom *geom );


AA_API void
aa_rx_sg_dirty_geom( struct aa_rx_sg *scene_graph );

AA_API void
aa_rx_sg_ensure_clean_frames( const struct aa_rx_sg *scene_graph );

AA_API void
aa_rx_sg_clean_gl( struct aa_rx_sg *scene_graph );

AA_API void
aa_rx_sg_clean_collision( struct aa_rx_sg *scene_graph );

AA_API int
aa_rx_sg_is_clean( struct aa_rx_sg *scene_graph );

AA_API int
aa_rx_sg_is_clean_gl( struct aa_rx_sg *scene_graph );

AA_API int
aa_rx_sg_is_clean_collision( struct aa_rx_sg *scene_graph );

AA_API void
aa_rx_sg_ensure_clean_gl( const struct aa_rx_sg *scene_graph );

AA_API void
aa_rx_sg_ensure_clean_collision( const struct aa_rx_sg *scene_graph );

AA_API void
aa_rx_sg_set_destructor(
    const struct aa_rx_sg *scene_graph,
    void (*destructor)(void*),
    void *context );



#endif /*AMINO_SCENEGRAPH_H*/
