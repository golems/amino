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

#ifndef AMINO_RX_SCENE_GL_H
#define AMINO_RX_SCENE_GL_H

/**
 * @file scene_gl.h
 */

/* DESIGN
 * ======
 *
 * - Per-frame GL arrays
 *   - vertices
 *   - colors
 *   - indices
 * - Initially create per-frame VBOs
 * - Initially compile shaders
 *
 *
 */


/** Forward declaration */
struct aa_rx_sg;
struct aa_gl_globals;


AA_API void
aa_rx_sg_gl_init( struct aa_rx_sg *sg );

/**
 * Render the scene graph to the current GL context.
 */
AA_API void
aa_rx_sg_render(
    const struct aa_rx_sg *sg,
    const struct aa_gl_globals *globals,
    size_t n_TF, double *TF_abs, size_t ld_TF);

/**
 * Convert a quaternion-translation to an OpenGL matrix
 */
AA_API void
aa_gl_qutr2glmat( const double E[AA_RESTRICT 7],
                  GLfloat M[AA_RESTRICT 16] );

/**
 * Convert a condensed transformation matrix to an OpenGL matrix
 */
AA_API void
aa_gl_tfmat2glmat( const double T[AA_RESTRICT 12],
                   GLfloat M[AA_RESTRICT 16] );


AA_API void
aa_gl_mat_perspective( double fovy,
                       double aspect,
                       double znear, double zfar,
                       GLfloat M[16] );

/**
 * Compile a shader from a text string.
 */
AA_API GLuint aa_gl_create_shader(
    GLenum shader_type, const char* source );


/**
 * Create a GLSL program and attach shaders.
 */
AA_API GLuint aa_gl_create_program(GLuint vert_shader, GLuint frag_shader);


/**
 * Initialize GL engine
 */
AA_API void aa_gl_init();


/**
 * Container for globals rendering info (Forward declaration)
 */

AA_API struct aa_gl_globals *
aa_gl_globals_create();

AA_API void
aa_gl_globals_destroy( struct aa_gl_globals *globals );

AA_API void
aa_gl_globals_set_camera(
    struct aa_gl_globals *globals,
    const double world_E_camera[7]);

AA_API void
aa_gl_globals_set_camera_home(
    struct aa_gl_globals *globals,
    const double world_E_camera_home[7]);

AA_API void
aa_gl_globals_home_camera(
    struct aa_gl_globals *globals );

AA_API void
aa_gl_globals_set_light_position(
    struct aa_gl_globals *globals,
    const double world_v_light[3]);

AA_API void
aa_gl_globals_set_aspect(
    struct aa_gl_globals *globals,
    double aspect );

AA_API void
aa_gl_globals_set_perspective(
    struct aa_gl_globals *globals,
    double fovy,
    double aspect,
    double znear,
    double zfar );

AA_API void
aa_gl_globals_set_light_color(
    struct aa_gl_globals *globals,
    const double color[3] );

AA_API void
aa_gl_globals_set_light_power(
    struct aa_gl_globals *globals,
    double power );

AA_API void
aa_gl_globals_set_ambient(
    struct aa_gl_globals *globals,
    const double ambient[3] );

AA_API void
aa_gl_globals_set_show_visual(
    struct aa_gl_globals *globals,
    int show_visual );

AA_API void
aa_gl_globals_set_show_collision (
    struct aa_gl_globals *globals,
    int show_collision );

struct aa_sg_gl_buffers;

AA_API void aa_geom_gl_buffers_init (
    struct aa_rx_geom *geom
    );

#endif /*AMINO_RX_SCENE_GL_H*/
