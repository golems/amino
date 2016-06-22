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

#ifndef AMINO_RX_SCENE_GEOM_H
#define AMINO_RX_SCENE_GEOM_H

/**
 * @file scene_geom.h
 */

/*-----------*/
/*- Options -*/
/*-----------*/

/**
 * Opaque structure for geometry options.
 *
 * All options are set through accessor functions so that future
 * options can be added while preserving API and ABI compatability.
 */
struct aa_rx_geom_opt;

/**
 * Create a geometry option struct.
 */
AA_API struct aa_rx_geom_opt*
aa_rx_geom_opt_create();

/**
 * Destroy a geometry option struct.
 */
AA_API void
aa_rx_geom_opt_destroy(struct aa_rx_geom_opt*);

/**
 * Set no-shadow option
 */
AA_API void
aa_rx_geom_opt_set_no_shadow (
    struct aa_rx_geom_opt *opt,
    int no_shadow );

/**
 * Get no-shadow option
 */
AA_API int
aa_rx_geom_opt_get_no_shadow ( struct aa_rx_geom_opt *opt );

/**
 * Get visual option.
 */
AA_API int
aa_rx_geom_opt_get_visual ( struct aa_rx_geom_opt *opt );

/**
 * Get collision option.
 */
AA_API int
aa_rx_geom_opt_get_collision ( struct aa_rx_geom_opt *opt );

/**
 * Get red color value.
 */
AA_API double
aa_rx_geom_opt_get_color_red ( struct aa_rx_geom_opt *opt );

/**
 * Get blue color value.
 */
AA_API double
aa_rx_geom_opt_get_color_blue ( struct aa_rx_geom_opt *opt );


/**
 * Get green color value.
 */
AA_API double
aa_rx_geom_opt_get_color_green ( struct aa_rx_geom_opt *opt );

/**
 * Get alpha value.
 */
AA_API double
aa_rx_geom_opt_get_alpha ( struct aa_rx_geom_opt *opt );

/**
 * Get red specular value.
 */
AA_API double
aa_rx_geom_opt_get_specular_red ( struct aa_rx_geom_opt *opt );

/**
 * Get blue specular value.
 */
AA_API double
aa_rx_geom_opt_get_specular_blue ( struct aa_rx_geom_opt *opt );

/**
 * Get green specular value.
 */
AA_API double
aa_rx_geom_opt_get_specular_green ( struct aa_rx_geom_opt *opt );

/**
 * Set color option
 */
AA_API void
aa_rx_geom_opt_set_color3 (
    struct aa_rx_geom_opt *opt,
    double red, double blue, double green );

/**
 * Set alpha (transparency) option
 */
AA_API void
aa_rx_geom_opt_set_alpha (
    struct aa_rx_geom_opt *opt,
    double alpha );

/**
 * Set visual flag
 *
 * Does this geometry represent a visual object?
 */
AA_API void
aa_rx_geom_opt_set_visual (
    struct aa_rx_geom_opt *opt,
    int visual );

/**
 * Set collision flag
 *
 * Does this geometry represent a collision object?
 */
AA_API void
aa_rx_geom_opt_set_collision (
    struct aa_rx_geom_opt *opt,
    int collision );

/**
 * Set specular reflection.
 */
AA_API void
aa_rx_geom_opt_set_specular3 (
    struct aa_rx_geom_opt *opt,
    double red, double green, double blue );


/**
 * Set mesh scaling
 */
AA_API void
aa_rx_geom_opt_set_scale (
    struct aa_rx_geom_opt *opt,
    double scale );

/**
 * Get mesh scaling
 */
AA_API double
aa_rx_geom_opt_get_scale (
    const struct aa_rx_geom_opt *opt );

/*----------*/
/*- Shapes -*/
/*----------*/


/**
 * Copy a geometry struct
 */
AA_API struct aa_rx_geom *
aa_rx_geom_copy( struct aa_rx_geom *src );

/**
 * Destroy a geometry struct
 */
AA_API void
aa_rx_geom_destroy( struct aa_rx_geom *geom );

/**
 * Enumeration of geometry shape types
 */
enum aa_rx_geom_shape {
    AA_RX_NOSHAPE,    ///< Invalid shape
    AA_RX_MESH,       ///< A triangular mesh shape
    AA_RX_BOX,        ///< A box (cube) shape
    AA_RX_SPHERE,     ///< A sphere (ball) shape
    AA_RX_CYLINDER,   ///< A cylinder shape
    AA_RX_CONE,       ///< A cone shape
    AA_RX_GRID        ///< A grid-lines shape
};

/**
 * Return a string for the shape type
 */
AA_API const char *aa_rx_geom_shape_str( enum aa_rx_geom_shape shape );

/**
 * Shape for a box
 */
struct aa_rx_shape_box {
    double dimension[3];
};

/**
 * Shape for a sphere
 */
struct aa_rx_shape_sphere {
    double radius;
};

/**
 * Shape for a cylinder
 */
struct aa_rx_shape_cylinder {
    double height;
    double radius;
};

/**
 * Shape for a cone
 */
struct aa_rx_shape_cone {
    double height;
    double start_radius;
    double end_radius;
};


/**
 * Shape for a grid
 */
struct aa_rx_shape_grid {
    double dimension[2];
    double delta[2];
    double width;
};

/**
 * Extract the shape type from a geometry object.
 *
 * @param[in]  g            the geometry object
 * @param[out] shape_type   the shape of the geometry
 *
 * @return poiner to the shape
 *
 */
AA_API void *
aa_rx_geom_shape ( const struct aa_rx_geom *g,
                   enum aa_rx_geom_shape *shape_type );

/**
 * Create a box
 */
AA_API struct aa_rx_geom *
aa_rx_geom_box (
    struct aa_rx_geom_opt *opt,
    const double dimension[3] );

/**
 * Create a sphere
 */
AA_API struct aa_rx_geom *
aa_rx_geom_sphere (
    struct aa_rx_geom_opt *opt,
    double radius );

/**
 * Create a cylinder
 */
AA_API struct aa_rx_geom *
aa_rx_geom_cylinder (
    struct aa_rx_geom_opt *opt,
    double height,
    double radius );

/**
 * Create a cone
 */
AA_API struct aa_rx_geom *
aa_rx_geom_cone (
    struct aa_rx_geom_opt *opt,
    double height,
    double start_radius,
    double end_radius );

/**
 * Create a grid
 */
AA_API struct aa_rx_geom *
aa_rx_geom_grid (
    struct aa_rx_geom_opt *opt,
    const double dimension[2],
    const double delta[2],
    double width );

/**
 * Opaque type for a mesh objects
 */
struct aa_rx_mesh;

/**
 * Create a mesh
 */
AA_API struct aa_rx_mesh* aa_rx_mesh_create();

/**
 * Destroy the mesh
 */
AA_API void aa_rx_mesh_destroy( struct aa_rx_mesh * mesh );

/**
 * Set the mesh vertices
 */
AA_API void aa_rx_mesh_set_vertices (
    struct aa_rx_mesh *mesh, size_t n,
    const float *vectors, int copy );

/**
 * Set the mesh normals
 */
AA_API void aa_rx_mesh_set_normals (
    struct aa_rx_mesh *mesh, size_t n,
    const float *normals, int copy );

/**
 * Set the mesh indices
 */
AA_API void aa_rx_mesh_set_indices (
    struct aa_rx_mesh *mesh, size_t n,
    const unsigned *indices, int copy );

/**
 * Set the mesh colors and alpha
 */
AA_API void aa_rx_mesh_set_rgba (
    struct aa_rx_mesh *mesh,
    size_t width, size_t height,
    const uint8_t *rgba, int copy );

/**
 * Set the mesh colors and uv values
 */
AA_API void aa_rx_mesh_set_uv (
    struct aa_rx_mesh *mesh,
    size_t n, const float *uv, int copy );

/**
 * Set the mesh texture parameters
 */
AA_API void aa_rx_mesh_set_texture (
    struct aa_rx_mesh *mesh,
    const struct aa_rx_geom_opt *opt );

/**
 * Attach a mesh to frame.
 */
AA_API struct aa_rx_geom *
aa_rx_geom_mesh (
    struct aa_rx_geom_opt *opt,
    struct aa_rx_mesh *mesh );

/**
 * Attach geometry to the scene graph
 */
AA_API void
aa_rx_geom_attach (
    struct aa_rx_sg *sg,
    const char *frame,
    struct aa_rx_geom *geom );


AA_API unsigned
aa_rx_geom_refcount ( const struct aa_rx_geom *g );

AA_API unsigned
aa_rx_mesh_refcount ( struct aa_rx_mesh *sg );

#endif /*AMINO_RX_SCENE_GEOM_H*/
