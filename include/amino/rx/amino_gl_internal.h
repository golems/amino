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

#ifndef AMINO_RX_GL_INTERNAL_H
#define AMINO_RX_GL_INTERNAL_H


struct aa_gl_globals {

    /* GL Data */
    double world_E_cam[7];
    double world_E_camhome[7];

    double fovy;
    double aspect;
    double znear;
    double zfar;

    GLfloat cam_M_world[16];
    GLfloat perspective[16];

    union {
        struct {
            GLfloat world_v_cam[3];
            GLfloat ambient[3];
            union {
                GLfloat light_position[3];
                GLfloat world_v_light[3];
            };
            GLfloat light_color[3];
        };
        GLfloat vec3_data[12];
    };

    GLfloat light_power;

    /* UI data */
    int mouse[2];
    double scroll_ratio;
    double angle_ratio;
    double fps;


    /* Flags */
    unsigned show_visual : 1;
    unsigned show_collision : 1;
};

struct aa_gl_buffers {
    GLuint values;
    GLint  values_size;

    GLuint colors;
    GLint  colors_size;

    GLuint indices;
    GLint  indices_size;

    GLuint normals;
    GLint  normals_size;

    GLuint tex2d;
    GLuint uv;

    GLsizei count;

    GLfloat specular[3];

    unsigned has_indices : 1;
    unsigned has_colors : 1;
    unsigned has_values : 1;
    unsigned has_normals : 1;
    unsigned has_tex2d : 1;
    unsigned has_uv : 1;

    GLenum mode;

    struct aa_gl_buffers *next;
};

/**
 * Draw a transform frame frame
 */
AA_API void aa_gl_draw_tf (
    const double *world_E_model,
    const struct aa_gl_buffers *buffers);

#endif /*AMINO_RX_GL_INTERNAL_H*/
