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
 *   AND ON ANY HEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef AMINO_RX_SCENE_GEOM_H
#define AMINO_RX_SCENE_GEOM_H

/*-----------*/
/*- Options -*/
/*-----------*/

/**
 * Opaque structure for geometry options.
 */
struct aa_rx_geom_opt;

/**
 * Create a geometry option struct.
 */
struct aa_rx_geom_opt*
aa_rx_geom_opt_create();

/**
 * Destroy a geometry option struct.
 */
struct aa_rx_geom_opt*
aa_rx_geom_opt_destroy();

/**
 * Set no-shadow option
 */
void
aa_rx_geom_opt_set_no_shadow (
    struct aa_rx_geom_opt *opt,
    int no_shadow );

/**
 * Set color option
 */
void
aa_rx_geom_opt_set_color (
    struct aa_rx_geom_opt *opt,
    double red, double blue, double green );

/**
 * Set alpha (transparency) option
 */
void
aa_rx_geom_opt_set_alpha (
    struct aa_rx_geom_opt *opt,
    double red, double blue, double green );

/**
 * Set visual flag
 */
void
aa_rx_geom_opt_set_visual (
    struct aa_rx_geom_opt *opt,
    int visual );

/**
 * Set collision flag
 */
void
aa_rx_geom_opt_set_collision (
    struct aa_rx_geom_opt *opt,
    int collision );

/*----------*/
/*- Shapes -*/
/*----------*/

enum aa_rx_geom_shape {
    AA_RX_NOSHAPE,
    AA_RX_MESH,
    AA_RX_BOX,
    AA_RX_SPHERE,
    AA_RX_CYLINDER,
    AA_RX_CONE
};

/**
 * Attach a box to frame.
 */
void aa_rx_geom_attach_box (
    struct aa_rx_sg *sg,
    const char *frame,
    struct aa_rx_geom_opt *opt,
    const double dimension[3] );

/**
 * Attach a sphere to frame.
 */
void aa_rx_geom_attach_sphere (
    struct aa_rx_sg *sg,
    const char *frame,
    struct aa_rx_geom_opt *opt,
    double radius );

/**
 * Attach a cylinder to frame.
 */
void aa_rx_geom_attach_cylinder (
    struct aa_rx_sg *sg,
    const char *frame,
    struct aa_rx_geom_opt *opt,
    double height,
    double radius );

/**
 * Attach a cone to frame.
 */
void aa_rx_geom_attach_cone (
    struct aa_rx_sg *sg,
    const char *frame,
    struct aa_rx_geom_opt *opt,
    double height,
    double start_radius,
    double end_radius );

/**
 * Opaque type for a mesh objects
 */
struct aa_rx_mesh;

void aa_rx_mesh_fill_vertex_vectors (
    struct aa_rx_mesh *mesh,
    size_t n, float *vectors);

void aa_rx_mesh_fill_vertex_vectors_dbl (
    struct aa_rx_mesh *mesh,
    size_t n, double *vectors );

void aa_rx_mesh_fill_vertex_indices (
    struct aa_rx_mesh *mesh,
    size_t n, size_t *indices);


void aa_rx_mesh_fill_normal_vectors (
    struct aa_rx_mesh *mesh,
    size_t n, float *vectors);

void aa_rx_mesh_fill_normal_vectors_dbl (
    struct aa_rx_mesh *mesh,
    size_t n, double *vectors );

void aa_rx_mesh_fill_normal_indices (
    struct aa_rx_mesh *mesh,
    size_t n, size_t *indices);


void aa_rx_mesh_fill_uv_vectors (
    struct aa_rx_mesh *mesh,
    size_t n, float *vectors);

void aa_rx_mesh_fill_uv_vectors_dbl (
    struct aa_rx_mesh *mesh,
    size_t n, double *vectors );

void aa_rx_mesh_fill_uv_indices (
    struct aa_rx_mesh *mesh,
    size_t n, size_t *indices);

/**
 * Attach a mesh to frame.
 */
void aa_rx_geom_attach_mesh (
    struct aa_rx_sg *sg,
    const char *frame,
    struct aa_rx_geom_opt *opt,
    struct aa_rx_mesh *mesh );

#endif /*AMINO_RX_SCENE_GEOM_H*/
