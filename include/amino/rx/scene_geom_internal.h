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

#ifndef AMINO_RX_SCENE_GEOM_INTERNAL_H
#define AMINO_RX_SCENE_GEOM_INTERNAL_H

/**
 * Opaque structure for geometry options.
 */
struct aa_rx_geom_opt
{
    double color[4]; ///< RGBA
    double specular[3];
    unsigned no_shadow : 1;
    unsigned visual : 1;
    unsigned collision : 1;
};

/* Forward declaration */
struct aa_gl_buffers;

struct aa_rx_geom {
    struct aa_rx_geom_opt opt;
    enum aa_rx_geom_shape type;
    struct aa_rx_cl_geom *cl_geom;

    /* Set to 1 one create.
     * Increment on Copy
     * Decrement on Destroy
     * Free when it reaches 0
     */
    size_t refcount;
    struct aa_gl_buffers *gl_buffers;
};

struct aa_rx_geom_box {
    struct aa_rx_geom base;
    struct aa_rx_shape_box shape;
};

struct aa_rx_geom_sphere {
    struct aa_rx_geom base;
    struct aa_rx_shape_sphere shape;
};

struct aa_rx_geom_cylinder {
    struct aa_rx_geom base;
    struct aa_rx_shape_cylinder shape;
};

struct aa_rx_geom_cone {
    struct aa_rx_geom base;
    struct aa_rx_shape_cone shape;
};

struct aa_rx_geom_grid {
    struct aa_rx_geom base;
    struct aa_rx_shape_grid shape;
};

struct aa_rx_geom_mesh {
    struct aa_rx_geom base;
    struct aa_rx_mesh *shape;
};



struct aa_rx_mesh {
    /**
     * 3 x n_vertices array of mesh vertices
     */
    const float *vertices;
    float *vertices_data;

    /**
     * 3 x n_vertices array of vertex normals
     */
    const float *normals;
    float *normals_data;

    /**
     * Number of vertices
     */
    size_t n_vertices;

    /**
     * 3 x n_indices array of face vertices
     */
    const unsigned *indices;
    unsigned *indices_data;

    const float *uv;
    float *uv_data;

    size_t n_indices;

    const uint8_t *rgba;
    uint8_t *rgba_data;
    size_t width_rgba;
    size_t height_rgba;

    size_t refcount;
};

#ifdef __cplusplus

#endif /*__cplusplus */

#endif /*AMINO_RX_SCENE_GEOM_INTERNAL_H*/
