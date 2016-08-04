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
 * @brief OpenGL Support
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
//struct aa_rx_sg;

/**
 * Opaque type for global GL values
 */
struct aa_gl_globals;


/**
 * Initialize OpenGL objects in scene graph
 *
 * This function is not threadsafe.  Do not attempt to access the same
 * GL context from multiple threads.
 *
 * @see aa_rx_win_sg_gl_init()
 */
AA_API void
aa_rx_sg_gl_init( struct aa_rx_sg *sg );

/**
 * Render the scene graph to the current GL context.
 *
 * This function is not threadsafe.  Do not attempt to access the same
 * GL context from multiple threads.
 *
 * @param scengreaph   the scene graph to render
 * @param globals      the opengl global values
 * @param n_TF         size of the TF array
 * @param TF_abs       absolute transforms of frames in the scene graph
 *                     (quaternion-translation format)
 * @param ld_TF        leading dimension of TF_abs (typically 7)
 *
 * @pre aa_rx_sg_init() was called after all frames added and
 * aa_rx_sg_gl_init() was called after all geometry attached.
 */
AA_API void
aa_rx_sg_render(
    const struct aa_rx_sg *scenegraph,
    const struct aa_gl_globals *globals,
    size_t n_TF, const double *TF_abs, size_t ld_TF);

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

/**
 * Create a OpenGL matrix for a perspective transform
 */
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

/**
 * Destroy a aa_gl_globals struct.
 *
 */
AA_API void
aa_gl_globals_destroy( struct aa_gl_globals *globals );

/**
 * Set the camera transform
 */
AA_API void
aa_gl_globals_set_camera(
    struct aa_gl_globals *globals,
    const double world_E_camera[7]);

/**
 * Set the camera "home" transform
 */
AA_API void
aa_gl_globals_set_camera_home(
    struct aa_gl_globals *globals,
    const double world_E_camera_home[7]);

/**
 * Set the camera transform to its "home" value
 */
AA_API void
aa_gl_globals_home_camera(
    struct aa_gl_globals *globals );

/**
 * Set the position of the light
 */
AA_API void
aa_gl_globals_set_light_position(
    struct aa_gl_globals *globals,
    const double world_v_light[3]);

/**
 * Set the camera aspect ratio.
 */
AA_API void
aa_gl_globals_set_aspect(
    struct aa_gl_globals *globals,
    double aspect );

/**
 * Set the camera perspective matrix
 */
AA_API void
aa_gl_globals_set_perspective(
    struct aa_gl_globals *globals,
    double fovy,
    double aspect,
    double znear,
    double zfar );

/**
 * Set the color of the light
 */
AA_API void
aa_gl_globals_set_light_color(
    struct aa_gl_globals *globals,
    const double color[3] );

/**
 * Set the power (intensity) of the light
 */
AA_API void
aa_gl_globals_set_light_power(
    struct aa_gl_globals *globals,
    double power );

/**
 * Set the light ambient color
 */
AA_API void
aa_gl_globals_set_ambient(
    struct aa_gl_globals *globals,
    const double ambient[3] );

/**
 * Set flag to enable render of visual geometry
 */
AA_API void
aa_gl_globals_set_show_visual(
    struct aa_gl_globals *globals,
    int show_visual );

/**
 * Set flag to enable render of collision geometry
 */
AA_API void
aa_gl_globals_set_show_collision (
    struct aa_gl_globals *globals,
    int show_collision );

/**
 * Set the display mask value of all frames to false.
 */
AA_API void
aa_gl_globals_unmask_all( struct aa_gl_globals *globals );

/**
 * Return the display mask value of the i'th frame.
 *
 */
AA_API int
aa_gl_globals_is_masked( const struct aa_gl_globals *globals, size_t i );

/**
 * Set the display mask value of the i'th frame to `value'.
 *
 * Frames with a true display mask are hidden.
 */
AA_API void
aa_gl_globals_mask( struct aa_gl_globals *globals, size_t i, int value );


/**
 * Opaque struct for opengl buffers
 */
struct aa_sg_gl_buffers;

/**
 * Initialize OpenGL buffers for geometry object.
 */
AA_API void aa_geom_gl_buffers_init (
    struct aa_rx_geom *geom
    );

struct aa_gl_buffers;

/**
 * Destroy OpenGL buffers.
 *
 * This function should only be called from the thread that owns the
 * GL context for buffers.
 */
AA_API void
aa_gl_buffers_destroy( struct aa_gl_buffers *buffers );

/**
 * Schedule destruction of OpenGL buffers.
 *
 * This function can be called from any thread.
 */
AA_API void
aa_gl_buffers_schedule_destroy( struct aa_gl_buffers *buffers );

/**
 * Destroy previously schedule OpenGL buffers.
 *
 * This function should only be called from the thread that owns the
 * GL context for buffers, i.e., the rendering thread..
 */
AA_API void
aa_gl_buffers_cleanup( void );

#endif /*AMINO_RX_SCENE_GL_H*/
