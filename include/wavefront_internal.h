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


#ifndef AMINO_WAVEFRONT_H
#define AMINO_WAVEFRONT_H

#include "amino/rx/wavefront.h"


/**
 * A face (triangle) in a wavefront obj file
 */
struct aa_rx_wf_obj_face {
    int32_t v[3];    ///< Vertex indices
    int32_t n[3];    ///< Normal indices
    int32_t t[3];    ///< Texture indices
    int32_t material; ///< Material index
};

struct wf_extra {
    struct aa_rx_wf_obj *wf_obj;
    struct aa_rx_wf_obj_face face;
};


#define WF_FACE(thing) yyextra->face.thing
#define SET_WF_FACE(thing) WF_FACE(thing) = atol(yytext)-1
#define SKIP_WF_FACE(thing) WF_FACE(thing) = AA_RX_WF_OBJ_FACE_NONE

/**
 * Create an object for a wavefront obj file.
 */
AA_API struct aa_rx_wf_obj *
aa_rx_wf_obj_create();

AA_API void
aa_rx_wf_obj_set_filename( struct aa_rx_wf_obj *obj, const char *filename );

/**
 * Add a new object name
 */
AA_API void
aa_rx_wf_obj_push_object( struct aa_rx_wf_obj *obj,
                          const char *object );

/**
 * Add a new mtl file
 */
AA_API int
aa_rx_wf_obj_push_mtl( struct aa_rx_wf_obj *obj,
                       const char *mtl_file );

/**
 * Add a new vertex element
 */
AA_API void
aa_rx_wf_obj_push_vertex( struct aa_rx_wf_obj *obj, double f );

/**
 * Add a new normal element
 */
AA_API void
aa_rx_wf_obj_push_normal( struct aa_rx_wf_obj *obj, double f );

/**
 * Add a new face
 */
AA_API void
aa_rx_wf_obj_push_face( struct aa_rx_wf_obj *obj,
                        struct aa_rx_wf_obj_face *face );

/**
 * Set the active material by name
 */
AA_API void
aa_rx_wf_obj_use_material( struct aa_rx_wf_obj *obj,
                           const char *material );




struct aa_rx_wf_material {
    char *name;
    double specular_weight; /* Ns */
    double specular[3];     /* Ks */
    double ambient[3];      /* Ka */
    double emission[3];     /* Ke */
    double diffuse[3];      /* Kd */
    double alpha;           /* d */
    double ior;             /* Ni */

    unsigned has_specular_weight : 1;
    unsigned has_specular : 1;
    unsigned has_ambient : 1;
    unsigned has_emission : 1;
    unsigned has_diffuse : 1;
    unsigned has_alpha : 1;
    unsigned has_ior : 1;
};

AA_VECTOR_DEF( struct aa_rx_wf_material *, mvec_type )

struct aa_rx_wf_mtl {
    struct aa_rx_wf_material *current;
    char *filename;
    mvec_type materials;
};


#define WF_MTL_CURRENT (yyextra->mtl->current)

struct wf_mtl_extra {
    struct aa_rx_wf_mtl *mtl;
    double *v;
};


AA_API struct aa_rx_wf_mtl *
aa_rx_wf_mtl_create();

AA_API void
aa_rx_wf_mtl_push( struct aa_rx_wf_mtl *mtl, const char *name );


AA_API double
aa_rx_wf_parse_float( const char *str );

#endif //AMINO_WAVEFRONT_H
