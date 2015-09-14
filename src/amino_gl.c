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

#include "amino.h"

#define GL_GLEXT_PROTOTYPES

#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL.h>

#include "amino/rx/rxtype.h"
#include "amino/rx/rxtype_internal.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/amino_gl.h"
#include "amino/rx/amino_gl_internal.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"


#define CHECK_GL_STATUS(TYPE,handle,pname) {                            \
        GLint status;                                                   \
        glGet ## TYPE ## iv(handle, pname, &status);                    \
        if (GL_FALSE == status) {                                       \
            GLint logLength;                                            \
            glGet ## TYPE ## iv(handle, GL_INFO_LOG_LENGTH, &logLength); \
            char infoLog[logLength+2];                                  \
            glGet ## TYPE ## InfoLog(handle, logLength, NULL, infoLog); \
            fprintf(stderr, "%d: %d, %s\n", __LINE__, pname, infoLog);  \
        }                                                               \
    }

void
aa_gl_qutr2glmat( const double E[AA_RESTRICT 7],
                  GLfloat M[AA_RESTRICT 16] )
{
    double T[12];
    aa_tf_qutr2tfmat(E,T);
    aa_gl_tfmat2glmat(T,M);
}


void
aa_gl_tfmat2glmat( const double T[AA_RESTRICT 12],
                   GLfloat M[AA_RESTRICT 16] )
{
    for( size_t j = 0; j < 4; j++ ) {
        for( size_t i = 0; i < 3; i++ ) {
            M[j*4 + i] = (GLfloat)T[j*3 + i];
        }
    }

    for( size_t j = 0; j < 3; j++ ) {
        M[j*4 + 3] = 0;
    }

    M[3*4 + 3] = 1;
}

AA_API void
aa_gl_mat_perspective( double fovy,
                       double aspect,
                       double znear, double zfar,
                       GLfloat M[16] )
{
    AA_MEM_SET(M,0,16);
    double t = tan(fovy/2);
    AA_MATREF(M,4,0,0) = (GLfloat)(1.0 / (aspect*t));
    AA_MATREF(M,4,1,1) = (GLfloat)(1.0/t);
    AA_MATREF(M,4,2,2) = (GLfloat)(- (zfar+znear) / (zfar-znear));
    AA_MATREF(M,4,3,2) = (GLfloat)(- 1);
    AA_MATREF(M,4,2,3) = (GLfloat)(- (2*zfar*znear) / (zfar-znear));

}


AA_API GLuint aa_gl_create_shader(
    GLenum shader_type, const char* source)
{
    GLuint shader = glCreateShader(shader_type);
    glShaderSource(shader, 1, (const GLchar **)&source, NULL);
    glCompileShader(shader);

    CHECK_GL_STATUS(Shader, shader, GL_COMPILE_STATUS);

    return shader;
}

AA_API GLuint aa_gl_create_program(GLuint vert_shader, GLuint frag_shader)
{
    GLuint program = glCreateProgram();
    glAttachShader(program, vert_shader);
    glAttachShader(program, frag_shader);
    glLinkProgram(program);
    CHECK_GL_STATUS(Program, program, GL_LINK_STATUS);
    return program;
}

static const char aa_gl_vertex_shader[] =
    "#version 130\n"
    ""
    "in vec4 position;"
    //"in vec4 color;"
    "in vec3 normal;"
    "in vec2 UV;"
    ""
    "uniform mat4 matrix_model;"   // parent: world, child: model
    "uniform mat4 matrix_camera;"  // camera perspective * pose
    "uniform vec3 light_world;"    // position of light in world
    "uniform vec3 camera_world;"    // position of camera in world
    ""
    "smooth out vec4 vColor;"
    "out vec2 vUV;"

    "out vec3 eye_world;"
    "out vec3 light_dir_world;"
    "out vec3 normal_world;"
    ""
    "void main() {"
    "  vec4 position_world = matrix_model * position;"
    "  gl_Position = matrix_camera * position_world;"

    /* "  eye_camera = vec3(0,0,0) - position_camera.xyz;" // vector from vertex to camera origin */
    /* "  vec3 light_pos_camera = (matrix_camera * vec4(light_world,1)).xyz;" */
    /* "  light_dir_camera = eye_camera + light_pos_camera;" // vector from light to vertex */
    /* "  normal_camera = (matrix_camera * matrix_model * vec4(normal,0)).xyz;" */

    "  eye_world = camera_world - position_world.xyz;"         // tail: vertex, tip: camera
    "  light_dir_world = light_world - position_world.xyz;" // tail: vertex, tip: light
    "  normal_world = mat3(matrix_model) * normal;"

    //"  vColor = color;"
    "  vUV = UV;"
    "}";

static const char aa_gl_fragment_shader[] =
    "#version 130\n"
    //"smooth in vec4 vColor;"
    ""

    "in vec3 normal_world;"
    "in vec3 eye_world;"
    "in vec3 light_dir_world;"
    "in vec2 vUV;"

    "uniform vec3 ambient;"
    "uniform vec3 light_color;"
    "uniform float light_power;"
    "uniform vec3 specular;"
    "uniform sampler2D texture_sampler;"
    ""
    "void main() {"
    ""
    ""
    "  vec4 rgba = texture(texture_sampler, vUV);"
    "  vec3 diffuse = rgba.rgb;"
    "  float alpha = rgba.a;"
    ""
    "  float dist = length( light_dir_world );"
    "  vec3 n = normal_world;" // already a unit vector
    "  vec3 l = light_dir_world / dist;" // normalize
    "  float ct = clamp(dot(n,l), 0.1, 1);"
    ""
    "  vec3 E = normalize(eye_world);"
    "  vec3 R = reflect(-l,n);"
    "  float ca = clamp(dot(E,R), 0.1, 1);"
    ""
    ""
    "  vec3 color ="
    // Ambient : simulates indirect lighting
    "    ambient * alpha"
    // Diffuse : "color" of the object
    "    + diffuse * light_color * light_power * ct / (dist*dist)"
    // Specular : reflective highlight, like a mirror
    "    + specular * light_color * light_power * pow(ca,5) / (dist*dist)"
    "  ;"
    "" // apply color and alpha
    "  gl_FragColor = vec4(color,alpha);"
    "}";


static GLuint aa_gl_id_program;
static GLint aa_gl_id_position;
//static GLint aa_gl_id_color;
static GLint aa_gl_id_normal;
static GLint aa_gl_id_matrix_model;
static GLint aa_gl_id_matrix_camera;
static GLint aa_gl_id_camera_world;;
static GLint aa_gl_id_light_position;
static GLint aa_gl_id_ambient;
static GLint aa_gl_id_light_color;
static GLint aa_gl_id_light_power;
static GLint aa_gl_id_specular;
static GLint aa_gl_id_uv;
static GLint aa_gl_id_texture;

static int aa_gl_initialized = 0;
AA_API void aa_gl_init()
{
    if( aa_gl_initialized ) return;

    aa_gl_initialized = 1;

    GLuint vertexShader = aa_gl_create_shader(GL_VERTEX_SHADER, aa_gl_vertex_shader);
    GLuint fragmentShader = aa_gl_create_shader(GL_FRAGMENT_SHADER, aa_gl_fragment_shader);

    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glEnable (GL_DEPTH_TEST);

    aa_gl_id_program = aa_gl_create_program(vertexShader, fragmentShader);

    aa_gl_id_position = glGetAttribLocation(aa_gl_id_program, "position");
    aa_gl_id_normal = glGetAttribLocation(aa_gl_id_program, "normal");
    //aa_gl_id_color = glGetAttribLocation(aa_gl_id_program, "color");
    aa_gl_id_uv = glGetAttribLocation(aa_gl_id_program, "UV");

    aa_gl_id_light_position = glGetUniformLocation(aa_gl_id_program, "light_world");
    aa_gl_id_ambient = glGetUniformLocation(aa_gl_id_program, "ambient");
    aa_gl_id_light_color = glGetUniformLocation(aa_gl_id_program, "light_color");
    aa_gl_id_light_power = glGetUniformLocation(aa_gl_id_program, "light_power");
    aa_gl_id_specular = glGetUniformLocation(aa_gl_id_program, "specular");
    aa_gl_id_texture = glGetUniformLocation(aa_gl_id_program, "texture_sampler");

    /* printf("ids: %d\n", */
    /*        aa_gl_id_specular ); */

    aa_gl_id_camera_world = glGetUniformLocation(aa_gl_id_program, "camera_world");
    aa_gl_id_matrix_model = glGetUniformLocation(aa_gl_id_program, "matrix_model");
    aa_gl_id_matrix_camera = glGetUniformLocation(aa_gl_id_program, "matrix_camera");
}


static void check_error( const char *name ){
    for (GLenum err = glGetError(); err != GL_NO_ERROR; err = glGetError()) {
        fprintf(stderr, "error %s: %d: %s\n",  name,  (int)err, gluErrorString(err));
        abort();
    }
}

AA_API void aa_gl_draw_tf (
    const double *world_E_model,
    const struct aa_gl_buffers *buffers )
{
    // Uniforms
    glUniform3fv(aa_gl_id_specular, 1, buffers->specular);
    check_error("unform specular");

    // matrices
    GLfloat M_model[16];
    aa_gl_qutr2glmat( world_E_model, M_model);

    glUniformMatrix4fv(aa_gl_id_matrix_model, 1, GL_FALSE, M_model);
    check_error("uniform mat model");

    // positions
    glBindBuffer(GL_ARRAY_BUFFER, buffers->values);
    check_error("glBindBuffer");

    glEnableVertexAttribArray((GLuint)aa_gl_id_position);
    check_error("glEnableVert");

    glVertexAttribPointer((GLuint)aa_gl_id_position, buffers->values_size, GL_FLOAT, GL_FALSE, 0, 0);
    check_error("glVertAttribPointer position");

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // colors
    /* glBindBuffer(GL_ARRAY_BUFFER, buffers->colors); */
    /* check_error("glBindBuffer"); */

    /* glEnableVertexAttribArray((GLuint)aa_gl_id_color); */
    /* check_error("glEnabeVert"); */

    /* glVertexAttribPointer((GLuint)aa_gl_id_color, buffers->colors_size, GL_FLOAT, GL_FALSE, 0, 0); */
    /* check_error("glVerteAttribPointer"); */

    // normals
    if( 0 <= aa_gl_id_normal && buffers->normals_size ) {
        glBindBuffer(GL_ARRAY_BUFFER, buffers->normals);
        check_error("glBindBuffer normal");

        glEnableVertexAttribArray((GLuint)aa_gl_id_normal);
        check_error("glEnabeVert normal");

        glVertexAttribPointer((GLuint)aa_gl_id_normal, buffers->normals_size, GL_FLOAT, GL_FALSE, 0, 0);
        check_error("glVerteAttribPointer normal");
    }


    if( buffers->has_tex2d ) {
        glBindTexture(GL_TEXTURE_2D, buffers->tex2d);
    }


    if( buffers->has_uv ) {
        glBindBuffer(GL_ARRAY_BUFFER, buffers->uv);
        check_error("glBindBuffer uv");

        glEnableVertexAttribArray((GLuint)aa_gl_id_uv);
        check_error("glEnabeVert uv");

        glVertexAttribPointer((GLuint)aa_gl_id_uv, 2, GL_FLOAT, GL_FALSE, 0, 0);
        check_error("glVerteAttribPointer uv");
    }
    // Light position
    //glUniform3f(aa_gl_id_light_position,
                //(GLfloat)v_light[0], (GLfloat)v_light[1], (GLfloat)v_light[2] );
    //aa_dump_mat( stdout, v_light, 1, 3 );



    if( buffers->has_indices ) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffers->indices);
        glDrawElements(
            buffers->mode,     // mode
            buffers->count,    // count
            GL_UNSIGNED_INT,   // type
            (void*)0           // element array buffer offset
            );
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    } else {
        glDrawArrays(buffers->mode, 0, buffers->count);
    }
    check_error("glDraw");


    glDisableVertexAttribArray((GLuint)aa_gl_id_position);
    /* glDisableVertexAttribArray((GLuint)aa_gl_id_color); */

    if( 0 <= aa_gl_id_normal ) {
        glDisableVertexAttribArray((GLuint)aa_gl_id_normal);
    }
    if( buffers->has_uv ) {
        glDisableVertexAttribArray((GLuint)aa_gl_id_uv);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);

}


static void aa_gl_buffers_destroy( struct aa_gl_buffers *bufs ) {
    if( bufs->has_indices ) glDeleteProgram(bufs->indices);
    if( bufs->has_colors ) glDeleteProgram(bufs->colors);
    if( bufs->has_values ) glDeleteProgram(bufs->values);
}

static void quad_tr( unsigned *indices,
                     unsigned pp,
                     unsigned pm,
                     unsigned mp,
                     unsigned mm )
{
    size_t j = 0;
    indices[j++] = pp;
    indices[j++] = pm;
    indices[j++] = mp;

    indices[j++] = mm;
    indices[j++] = pm;
    indices[j++] = mp;
}

/* AA_API void aa_gl_normals( struct aa_gl_buffers *buffers, */
/*                            size_t n_values, size_t n_indices, */
/*                            GLfloat *values, size_t ld_values, */
/*                            unsigned *indices, size_t ld_indices ) */
/* { */
/*     size_t size = sizeof(GLfloat) * 3 * n_values; */
/*     GLfloat *normals = (GLfloat*)aa_mem_region_local_alloc( size ); */
/*     AA_MEM_ZERO(normals,size); */
/*     for( size_t i = 0; i < n_indices; i ++ ) { */
/*         unsigned *index = &indices[ld_indices*i]; */
/*         printf("%u, %u, %u\n", */
/*                index[0], index[1], index[2] ); */
/*         GLfloat *v[3] = {values + ld_values*index[0], */
/*                          values + ld_values*index[1], */
/*                          values + ld_values*index[2] }; */
/*         GLfloat *n[3] = {normals + 3*index[0], */
/*                          normals + 3*index[1], */
/*                          normals + 3*index[2] }; */
/*         GLfloat e[2][3], tri_norm[3]; */
/*         for( int k = 0; k < 2; k ++ ) { */
/*             for( int j = 0; j < 3; j ++ ) { */
/*                 e[k][j] = v[1+k][j] - v[0][j]; */
/*             } */
/*         } */
/*         printf("v0: "); aa_dump_matf( stdout, v[0], 1, 3 ); */
/*         printf("v1: "); aa_dump_matf( stdout, v[1], 1, 3 ); */
/*         printf("v2: "); aa_dump_matf( stdout, v[2], 1, 3 ); */
/*         printf("e0: "); aa_dump_matf( stdout, e[0], 1, 3 ); */
/*         printf("e1: "); aa_dump_matf( stdout, e[1], 1, 3 ); */

/*         aa_tf_crossf(e[0],e[1],tri_norm); */

/*         printf("tn0: "); aa_dump_matf( stdout, tri_norm, 1, 3 ); */
/*         aa_tf_vnormalizef(tri_norm); */



/*         printf("tnn: "); aa_dump_matf( stdout, tri_norm, 1, 3 ); */

/*         aa_dump_matf( stdout, tri_norm, 1, 3 ); */
/*         for( int k = 0; k < 3; k ++ ) { */
/*             for( int j = 0; j < 3; j ++ ) { */
/*                 n[k][j] += tri_norm[j]; */
/*             } */
/*         } */
/*     } */

/*     aa_dump_matf( stdout, normals, 3, n_values ); */

/*     for( size_t i = 0; i < n_values; i ++ ) { */
/*         aa_tf_vnormalizef( normals+3*i ); */
/*     } */


    /* glGenBuffers(1, &buffers->values); */
    /* glBindBuffer(GL_ARRAY_BUFFER, buffers->normals); */
    /* glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)size, normals, GL_STATIC_DRAW); */
    /* glBindBuffer(GL_ARRAY_BUFFER, 0); */
    /* buffers->has_normals = 1; */

/*     printf("values:\n"); aa_dump_matf( stdout, values, 3, n_values ); */
/*     printf("--\n"); */
/*     aa_dump_matf( stdout, normals, 3, n_values ); */

/*     aa_mem_region_local_pop(normals); */
/* } */

static void bind_mesh (
    struct aa_rx_geom *geom,
    struct aa_rx_mesh *mesh,
    size_t size
    )
{
    assert(geom->gl_buffers);
    assert(sizeof(float) == sizeof(GLfloat));

    struct aa_gl_buffers *bufs = geom->gl_buffers;
    size_t n_vert = mesh->n_vertices;

    bufs->values_size = 3;
    if( mesh->indices ) {
        bufs->indices_size = (GLint)size;
        bufs->count = (GLsizei)(size*mesh->n_indices);
    } else {
        bufs->count = (GLsizei)(size*mesh->n_vertices);
    }



    glGenBuffers(1, &bufs->values);
    glBindBuffer(GL_ARRAY_BUFFER, bufs->values);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)((size_t)bufs->values_size*sizeof(float)*n_vert),
                 mesh->vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    bufs->has_values = 1;

    // textures
    if( mesh->uv ) {
        glGenBuffers(1, &bufs->uv);
        glBindBuffer(GL_ARRAY_BUFFER, bufs->uv);
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(2*n_vert*sizeof(GLfloat)), mesh->uv, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        bufs->has_uv = 1;
    }


    if( mesh->rgba ) {
        glGenTextures(1, &bufs->tex2d);
        glBindTexture(GL_TEXTURE_2D, bufs->tex2d);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, (GLsizei)mesh->width_rgba, (GLsizei)mesh->height_rgba,
                     0, GL_RGBA, GL_UNSIGNED_BYTE, mesh->rgba);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        bufs->has_tex2d = 1;
    }

    if( mesh->indices ) {
        bufs->indices_size = 3;
        glGenBuffers(1, &bufs->indices);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, bufs->indices);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)((size_t)bufs->indices_size*sizeof(unsigned)*mesh->n_indices),
                     mesh->indices, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        bufs->has_indices = 1;
    }

    if( mesh->normals ) {
        bufs->normals_size = 3;
        glGenBuffers(1, &bufs->normals);
        glBindBuffer(GL_ARRAY_BUFFER, bufs->normals);
        glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)((size_t)bufs->normals_size*sizeof(float)*n_vert),
                     mesh->normals, GL_STATIC_DRAW);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        bufs->has_normals = 1;
    }
}


static void tri_mesh (
    struct aa_rx_geom *geom,
    struct aa_rx_mesh *mesh
    )
{
    geom->gl_buffers = AA_NEW0(struct aa_gl_buffers);
    geom->gl_buffers->mode = GL_TRIANGLES;
    bind_mesh( geom, mesh, 3 );
}

static void quad_mesh (
    struct aa_rx_geom *geom,
    struct aa_rx_mesh *mesh
    )
{
    geom->gl_buffers = AA_NEW0(struct aa_gl_buffers);
    geom->gl_buffers->mode = GL_QUADS;
    bind_mesh( geom, mesh, 4 );
}

static void line_mesh (
    struct aa_rx_geom *geom,
    struct aa_rx_mesh *mesh
    )
{
    geom->gl_buffers = AA_NEW0(struct aa_gl_buffers);
    geom->gl_buffers->mode = GL_LINES;
    bind_mesh( geom, mesh, 2 );
}

AA_API void aa_geom_gl_buffers_init_mesh(
    struct aa_rx_geom_mesh *geom
    )
{
    tri_mesh( &geom->base, geom->shape );
}


AA_API void aa_geom_gl_buffers_init_grid (
    struct aa_rx_geom_grid *geom
    )
{
    double *dmaxd = geom->shape.dimension;
    double *delta = geom->shape.delta;

    size_t n_x = (size_t)(dmaxd[0]/delta[0]) + 1;
    size_t n_y = (size_t)(dmaxd[1]/delta[1]) + 1;
    GLfloat dmax[2] = { (GLfloat)((double)(n_x-1)*delta[0]),
                        (GLfloat)((double)(n_y-1)*delta[1])};
    size_t n_vert = 8*(n_x+n_y-1);
    double w = geom->shape.width / 2;

    size_t vsize = 3 * n_vert * sizeof(GLfloat);
    GLfloat *values = (GLfloat*)aa_mem_region_local_alloc(vsize);;

    static const double a[2] = {1,-1};

    // x
    size_t c = 0;
    for( size_t k = 0; k < 2; k ++ ) { // +/- x
        for( size_t i = k; i < n_x; i ++ ) { // step from 0
            double x_nom = a[k]*(double)i*delta[0];
            GLfloat x0 = (GLfloat)(x_nom + w);
            GLfloat x1 = (GLfloat)(x_nom - w);
            GLfloat y0 = -dmax[1];
            GLfloat y1 = dmax[1];

            values[c++] = x0;
            values[c++] = y0;
            values[c++] = 0;

            values[c++] = x0;
            values[c++] = y1;
            values[c++] = 0;

            values[c++] = x1;
            values[c++] = y1;
            values[c++] = 0;

            values[c++] = x1;
            values[c++] = y0;
            values[c++] = 0;

        }
    }

    // y
    for( size_t k = 0; k < 2; k ++ ) { // +/- x
        for( size_t i = k; i < n_y; i ++ ) { // step from 0
            double y_nom = a[k]*(double)i*delta[1];
            GLfloat y0 = (GLfloat)(y_nom + w);
            GLfloat y1 = (GLfloat)(y_nom - w);
            GLfloat x0 = -dmax[0];
            GLfloat x1 = dmax[0];

            values[c++] = x0;
            values[c++] = y0;
            values[c++] = 0;

            values[c++] = x0;
            values[c++] = y1;
            values[c++] = 0;

            values[c++] = x1;
            values[c++] = y1;
            values[c++] = 0;

            values[c++] = x1;
            values[c++] = y0;
            values[c++] = 0;

        }
    }

    assert( c == 3*n_vert );

    struct aa_rx_mesh *mesh = aa_rx_mesh_create();

    aa_rx_mesh_set_vertices( mesh, n_vert, values, 0 );
    aa_rx_mesh_set_texture(mesh, &geom->base.opt);


    quad_mesh( &geom->base, mesh );

    aa_mem_region_local_pop(values);
    aa_rx_mesh_destroy(mesh);
}

AA_API void aa_geom_gl_buffers_init_box (
    struct aa_rx_geom_box *geom
    )
{
    //static const size_t n_vert = 4*2;
    //static const size_t n_indices = 6*2;

    /* GLfloat values[3*n_vert]; // 3 values, 4 corners, two squares */
    /* GLfloat colors[6*n_vert]; // same as vertices */
    /* unsigned indices[n_indices*3]; // 6 sides, 2 triangles, 3 vertices */
    /* double *d = geom->shape.dimension; */


    GLfloat values[6*4*3];
    GLfloat normals[6*4*3];
    GLfloat d[3] = { (GLfloat)geom->shape.dimension[0] / 2,
                     (GLfloat)geom->shape.dimension[1] / 2,
                     (GLfloat)geom->shape.dimension[2] / 2};
    GLfloat a[2] = {1,-1};
    size_t ii[4][2] = {{0,0}, {0,1}, {1,1}, {1,0}};

    // Z
    size_t n = 0;
    for( size_t k = 0; k < 2; k ++ ) {
        for( size_t ell = 0; ell<4; ell++ ) {
            size_t i = ii[ell][0];
            size_t j = ii[ell][1];
            values[n*3 + 0] = a[i]*d[0];
            values[n*3 + 1] = a[j]*d[1];
            values[n*3 + 2] = a[k]*d[2];
            normals[n*3+0] = 0;
            normals[n*3+1] = 0;
            normals[n*3+2] = a[k];
            n++;
        }
    }
    // X
    for( size_t k = 0; k < 2; k ++ ) {
        for( size_t ell = 0; ell<4; ell++ ) {
            size_t i = ii[ell][0];
            size_t j = ii[ell][1];
            values[n*3 + 0] = a[k]*d[0];
            values[n*3 + 1] = a[i]*d[1];
            values[n*3 + 2] = a[j]*d[2];
            normals[n*3+0] = a[k];
            normals[n*3+1] = 0;
            normals[n*3+2] = 0;
            n++;
        }
    }
    // Y
    for( size_t k = 0; k < 2; k ++ ) {
        for( size_t ell = 0; ell<4; ell++ ) {
            size_t i = ii[ell][0];
            size_t j = ii[ell][1];
            values[n*3 + 0] = a[i]*d[0];
            values[n*3 + 1] = a[k]*d[1];
            values[n*3 + 2] = a[j]*d[2];
            normals[n*3+0] = 0;
            normals[n*3+1] = a[k];
            normals[n*3+2] = 0;
            n++;
        }
    }
    //fprintf(stderr, "n: %lu\n", n );

    //aa_dump_matf( stdout, values, 3, 6*4 );



    /* unsigned n_indices = 6*4; */
    /* unsigned indices[n_indices]; */
    /* for( unsigned i = 0; i < n_indices; i++ ) { */
    /*     indices[i] = i; */
    /* } */

    /* aa_rx_mesh_set_indices( mesh, n_indices, indices, 0 ); */

    struct aa_rx_mesh *mesh = aa_rx_mesh_create();
    aa_rx_mesh_set_vertices( mesh, 6*4, values, 0 );
    aa_rx_mesh_set_texture(mesh, &geom->base.opt);
    aa_rx_mesh_set_normals( mesh, 6*4, normals, 0 );
    quad_mesh( &geom->base, mesh );
    aa_rx_mesh_destroy(mesh);


    /* // fill vertices */
    /* for( size_t x = 0, j=0; x < 2; x ++ ) { */
    /*     for( size_t y = 0; y < 2; y ++ ) { */
    /*         for( size_t z = 0; z < 2; z ++ ) { */
    /*             values[j++] = (GLfloat)(a[x]*d[0]); */
    /*             values[j++] = (GLfloat)(a[y]*d[1]); */
    /*             values[j++] = (GLfloat)(a[z]*d[2]); */
    /*         } */
    /*     } */
    /* } */
    /* for( size_t i = 0; i < 8; i ++ ){ */
    /*     AA_MEM_CPY( normals+3*i, values+3*i, 3 ); */
    /*     aa_tf_vnormalizef( normals+3*i ); */
    /* } */

    /* /\*    xyz */
    /*  *    === */
    /*  * 0: +++ */
    /*  * 1: ++- */
    /*  * 2: +-+ */
    /*  * 3: +-- */
    /*  * */
    /*  * 4: -++ */
    /*  * 5: -+- */
    /*  * 6: --+ */
    /*  * 7: --- */
    /*  *\/ */

    /* { */
    /*     size_t j = 0; */
    /*     // +x */
    /*     quad_tr(indices+j, 0,1,2,3 ); */
    /*     j+=6; */
    /*     // -x */
    /*     quad_tr(indices+j, 4,5,6,7 ); */
    /*     j+=6; */
    /*     // +y */
    /*     quad_tr(indices+j, 0,1,4,5 ); */
    /*     j+=6; */
    /*     // -y */
    /*     quad_tr(indices+j, 2,3,6,7 ); */
    /*     j+=6; */
    /*     // +z */
    /*     quad_tr(indices+j, 0,2,4,6 ); */
    /*     j+=6; */
    /*     // -z */
    /*     quad_tr(indices+j, 1,3,5,7 ); */
    /*     j+=6; */
    /* } */

    /* for( size_t i = 0; i < sizeof(colors)/(4*sizeof(*colors)); i ++ ) { */
    /*     for( size_t j = 0; j < 4; j ++ ) { */
    /*         colors[4*i + j ] = (GLfloat)geom->base.opt.color[j]; */
    /*     } */
    /* } */

    /* struct aa_rx_mesh vmesh = {0}; */
    /* struct aa_rx_mesh *mesh = &vmesh; */
    /* aa_rx_mesh_set_vertices( mesh, n_vert, values, 0 ); */
    /* aa_rx_mesh_set_normals( mesh, n_vert, normals, 0 ); */
    /* aa_rx_mesh_set_indices( mesh, n_indices, indices, 0 ); */
    /* tri_mesh( geom, mesh ); */
}


AA_API void aa_geom_gl_buffers_init_cylinder (
    struct aa_rx_geom_cylinder *geom
    )
{
    double r = geom->shape.radius;
    GLfloat h[2] = {0, (GLfloat)geom->shape.height};

    unsigned p = 36;

    unsigned n_values = p*2*2+2;
    unsigned n_indices = p*2*2;
    GLfloat values[n_values*3];
    GLfloat normals[n_values*3];
    unsigned indices[n_indices*3];

    unsigned a = 0;
    unsigned b = 0;

    // sides
    for( unsigned i = 0; i < p; i ++ ) {

        { // values
            double theta = i * (2*M_PI/p);
            double s = sin(theta);
            double c = cos(theta);
            double x = c*r;
            double y = s*r;

            for( unsigned j = 0; j < 2; j ++ ) {
                values[a]    = (GLfloat)x;
                normals[a++] = (GLfloat)c;
                values[a]    = (GLfloat)y;
                normals[a++] = (GLfloat)s;
                values[a] =   h[j];
                normals[a++] = 0;
            }
        }
        { // indices
            unsigned k0 = 2*i;
            unsigned k1 = k0 + 1;
            unsigned k2 = (k1 + 1) % (2*p);
            unsigned k3 = (k2 + 1) % (2*p);

            indices[b++] = k0;
            indices[b++] = k1;
            indices[b++] = k2;

            indices[b++] = k3;
            indices[b++] = k1;
            indices[b++] = k2;

        }
    }

    // ends
    AA_MEM_CPY( values+a, values, a );
    unsigned k_end = a/3;
    unsigned an = a;
    a += a;

    assert( a/3 == 4*p );
    assert(a/3 + 2 == n_values );

    unsigned centers[2] = {a/3, a/3+1};
    GLfloat sg[2] = {-1,1};

    for( unsigned i = 0; i < p; i ++ ) {
        unsigned k0 = k_end + (2*i + 0);
        unsigned k1 = k_end + (2*i + 1);
        unsigned k2 = k_end + ((2*i + 2) % (2*p));
        unsigned k3 = k_end + ((2*i + 3) % (2*p));
        unsigned kk[2][2] = { {k0, k2}, {k1, k3} };
        for( unsigned j = 0; j < 2; j ++ ) {
            indices[b++] = centers[j];
            indices[b++] = kk[j][0];
            indices[b++] = kk[j][1];

            normals[an++] = 0;
            normals[an++] = 0;
            normals[an++] = sg[j];
        }
    }

    for( unsigned j = 0; j < 2; j ++ ) {

        values[a++]    =  0;
        values[a++]    =  0;
        values[a++]    =  h[j];

        normals[an++] = 0;
        normals[an++] = 0;
        normals[an++] = sg[j];
    }

    assert( a/3  == n_values );
    assert( an/3  == n_values );
    assert( b/3  == n_indices );

    struct aa_rx_mesh *mesh = aa_rx_mesh_create();

    aa_rx_mesh_set_vertices( mesh, a/3, values, 0 );
    aa_rx_mesh_set_normals( mesh, an/3, normals, 0 );
    aa_rx_mesh_set_indices( mesh, b/3, indices, 0 );
    aa_rx_mesh_set_texture(mesh, &geom->base.opt);
    tri_mesh( &geom->base, mesh );
    aa_rx_mesh_destroy(mesh);
}



AA_API void aa_geom_gl_buffers_init (
    struct aa_rx_geom *geom
    )
{
    if( geom->gl_buffers ) aa_gl_buffers_destroy( geom->gl_buffers );
    //geom->gl_buffers = AA_NEW0(struct aa_gl_buffers);

    switch( geom->type ) {
    case AA_RX_BOX:
        aa_geom_gl_buffers_init_box((struct aa_rx_geom_box*)geom);
        break;
    case AA_RX_GRID:
        aa_geom_gl_buffers_init_grid((struct aa_rx_geom_grid*)geom);
        break;
    case AA_RX_MESH:
        aa_geom_gl_buffers_init_mesh((struct aa_rx_geom_mesh*)geom);
        break;
    case AA_RX_CYLINDER:
        aa_geom_gl_buffers_init_cylinder((struct aa_rx_geom_cylinder*)geom);
        break;
    default:
        break;
        //abort();
    }

    // specular
    if( geom->gl_buffers ) {
        for( int i = 0; i < 3; i ++ ) {
            geom->gl_buffers->specular[i] = (GLfloat) geom->opt.specular[i];
        }
    }
}



struct aa_gl_globals *
aa_gl_globals_create()
{
    struct aa_gl_globals *G = AA_NEW0(struct aa_gl_globals);

    aa_gl_globals_set_camera( G, aa_tf_qutr_ident );

    double light_position[3] = {0,0,1};
    aa_gl_globals_set_light_position( G, light_position );

    aa_gl_globals_set_perspective( G,
                                   M_PI_2, 16.0 / 9.0 ,
                                   .1, 100 );

    double light_color[3] = {1,1,1};
    aa_gl_globals_set_light_color( G, light_color );

    double ambient[3] = {0.5,0.5,0.5};
    aa_gl_globals_set_ambient( G, ambient );

    aa_gl_globals_set_light_power( G, 50 );

    G->scroll_ratio = .05;
    G->angle_ratio = .05;

    G->show_visual = 1;

    return G;
}

void
aa_gl_globals_destroy( struct aa_gl_globals *globals )
{
    free(globals);
}

void
aa_gl_globals_set_camera(
    struct aa_gl_globals *globals,
    const double world_E_camera[7])
{

    for( size_t i = 0; i < 3; i ++ ) {
        globals->world_v_cam[i] = (GLfloat) world_E_camera[AA_TF_QUTR_T + i] ;
    }

    double world_T_camera[12], camera_T_world[12];
    // Copy camera pose
    AA_MEM_CPY(globals->world_E_cam, world_E_camera, 7);
    // Convert inverse camera pose
    aa_tf_qutr2tfmat(world_E_camera,world_T_camera);
    aa_tf_tfmat_inv2(world_T_camera, camera_T_world);
    aa_gl_tfmat2glmat( camera_T_world, globals->cam_M_world );
}

void
aa_gl_globals_set_camera_home(
    struct aa_gl_globals *globals,
    const double world_E_camera_home[7])
{

    AA_MEM_CPY(globals->world_E_camhome, world_E_camera_home, 7);
}

void
aa_gl_globals_home_camera(
    struct aa_gl_globals *globals )
{
    aa_gl_globals_set_camera( globals, globals->world_E_camhome );
}

void
aa_gl_globals_set_light_position(
    struct aa_gl_globals *globals,
    const double world_v_light[3])
{
    for( size_t i = 0; i < 3; i ++ ) {
        globals->world_v_light[i] = (GLfloat)world_v_light[i];
    }
}

void
aa_gl_globals_set_perspective(
    struct aa_gl_globals *globals,
    double fovy,
    double aspect,
    double znear,
    double zfar )
{
    globals->fovy = fovy;
    globals->znear = znear;
    globals->zfar = zfar;
    aa_gl_globals_set_aspect( globals, aspect );
}

void
aa_gl_globals_set_aspect(
    struct aa_gl_globals *globals,
    double aspect )
{
    for( size_t i = 0; i < 4; i ++ ) {
        globals->perspective[i*4 + i] = 1;
    }
    globals->aspect = aspect;

    aa_gl_mat_perspective( globals->fovy, globals->aspect,
                           globals->znear, globals->zfar,
                           globals->perspective);
}

void
aa_gl_globals_set_light_color(
    struct aa_gl_globals *globals,
    double color[3] )
{
    for( size_t i = 0; i < 3; i ++ ) {
        globals->light_color[i] = (GLfloat)color[i];
    }
}

void
aa_gl_globals_set_light_power(
    struct aa_gl_globals *globals,
    double power )
{
    globals->light_power = (GLfloat)power;
}

void
aa_gl_globals_set_ambient(
    struct aa_gl_globals *globals,
    double ambient[3] )
{

    for( size_t i = 0; i < 3; i ++ ) {
        globals->ambient[i] = (GLfloat)ambient[i];
    }
}

void
aa_gl_globals_set_show_visual(
    struct aa_gl_globals *globals,
    int show_visual )
{
    globals->show_visual = show_visual ? 1 : 0;
}

void
aa_gl_globals_set_show_collision (
    struct aa_gl_globals *globals,
    int show_collision )
{
    globals->show_collision = show_collision ? 1 : 0;
}


struct sg_render_cx {
    const struct aa_rx_sg *sg;
    double *TF;
    size_t ld_tf;
    const struct aa_gl_globals *globals;
};

void render_helper( void *cx_, aa_rx_frame_id frame_id, struct aa_rx_geom *geom )
{
    struct sg_render_cx *cx = (struct sg_render_cx*)cx_;
    double *E = cx->TF + ((size_t)frame_id*cx->ld_tf);
    if( geom->gl_buffers &&
        ((cx->globals->show_visual && geom->opt.visual) ||
         (cx->globals->show_collision && geom->opt.collision)) )
    {
        //printf("rendering %s\n", aa_rx_geom_shape_str(geom->type));
        aa_gl_draw_tf( E, geom->gl_buffers);
    }
}

AA_API void
aa_rx_sg_render(
    const struct aa_rx_sg *sg,
    const struct aa_gl_globals *globals,
    size_t n_TF, double *TF_abs, size_t ld_tf)
{

    glUseProgram(aa_gl_id_program);
    check_error("glUseProgram");

    // uniforms
    glUniform3fv(aa_gl_id_light_position, 1, globals->world_v_light);
    check_error("unform light position");

    glUniform3fv(aa_gl_id_ambient, 1, globals->ambient);
    check_error("unform ambient");

    glUniform3fv(aa_gl_id_light_color, 1, globals->light_color);
    check_error("unform light color");

    glUniform1f(aa_gl_id_light_power, globals->light_power);
    check_error("unform light power");


    GLfloat M_c[16];
    cblas_sgemm( CblasColMajor, CblasNoTrans, CblasNoTrans,
                 4, 4, 4,
                 1.0, globals->perspective, 4,
                 globals->cam_M_world, 4,
                 0, M_c, 4 );

    glUniformMatrix4fv(aa_gl_id_matrix_camera, 1, GL_FALSE, M_c);
    check_error("uniform mat camera");

    glUniform3fv(aa_gl_id_camera_world, 1, globals->world_v_cam);
    check_error("uniform pos camera");


    // TODO: optimize globals handling
    struct sg_render_cx cx = {.sg = sg,
                              .TF = TF_abs,
                              .ld_tf = ld_tf,
                              .globals = globals };
    aa_rx_sg_map_geom( sg, &render_helper, &cx );

    glUseProgram(0);
}

void gl_init_helper( void *cx, aa_rx_frame_id frame_id, struct aa_rx_geom *geom ) {
    (void)cx;
    (void)frame_id;
    aa_geom_gl_buffers_init( geom );
}

AA_API void
aa_rx_sg_gl_init( struct aa_rx_sg *sg )
{
    aa_rx_sg_map_geom( sg, &gl_init_helper, NULL );
}
