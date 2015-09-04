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
#define GL_GLEXT_PROTOTYPES

#include <error.h>
#include <stdio.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "amino.h"
#include "amino/rx/amino_gl.h"
#include "amino/rx/amino_sdl.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"

const int SCREEN_WIDTH = 1000;
const int SCREEN_HEIGHT = 1000;

#include <SDL.h>


struct aa_rx_geom_box geom;
struct aa_rx_geom_grid grid;


void Init(void)
{
    memset(&geom,0,sizeof(geom));
    memset(&grid,0,sizeof(grid));

    geom.base.opt.color[0] = 1;
    geom.base.opt.color[1] = 0;
    geom.base.opt.color[2] = 0;
    geom.base.opt.color[3] = 1;
    geom.base.type = AA_RX_BOX;
    geom.base.gl_buffers = NULL;
    geom.shape.dimension[0] = 0.1;
    geom.shape.dimension[1] = 0.1;
    geom.shape.dimension[2] = 0.1;

    double spec[3] = {.3, .3, .3 };
    aa_rx_geom_opt_set_specular( &geom.base.opt, spec );
    aa_geom_gl_buffers_init( &geom.base );

    grid.base.opt.color[0] = .5;
    grid.base.opt.color[1] = .5;
    grid.base.opt.color[2] = 1;
    grid.base.opt.color[3] = 1;
    grid.base.type = AA_RX_GRID;
    grid.base.gl_buffers = NULL;

    grid.shape.dimension[0] = 1;
    grid.shape.dimension[1] = 1;

    grid.shape.delta[0] = .05;
    grid.shape.delta[1] = .05;

    aa_rx_geom_opt_set_specular( &grid.base.opt, spec );
    aa_geom_gl_buffers_init( &grid.base );


}

void check_error( const char *name ){
    for (GLenum err = glGetError(); err != GL_NO_ERROR; err = glGetError()) {
        fprintf(stderr, "error %s: %d: %s\n",  name,  (int)err, gluErrorString(err));
    }
}


void display( const struct aa_gl_globals *globals,
             const double world_E_model[7] )
{

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    check_error("glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    check_error("glClear");


    //world_E_camera[AA_TF_QUTR_TZ] = 1.5;
    //world_E_camera[AA_TF_QUTR_TX] = .5;
    //world_E_camera[AA_TF_QUTR_TY] = .5;


    //aa_tf_yangle2quat(M_PI/4, world_E_model );

    /* { */
    /*     double eye[3] = {0,0,2}; */
    /*     double target[3] = {0,0,0}; */
    /*     double up[3] = {0,1,0}; */
    /*     aa_tf_qutr_mzlook(eye, target, up, world_E_camera ); */
    /*     } */
    //aa_gl_draw_tf( E, &buffers );
    /* GLfloat P[16] = {0}; */
    /* for( size_t i = 0; i < 4; i ++ ) { */
    /*     P[i*4 + i] = 1; */
    /* } */
    /* aa_gl_mat_perspective(M_PI_2, ((double)SCREEN_WIDTH)/SCREEN_HEIGHT, */
    /*                       0.1, 100, */
    /*                       P ); */

    aa_gl_draw_tf( globals, world_E_model,
                   geom.base.gl_buffers );


    aa_gl_draw_tf( globals, world_E_model,
                   grid.base.gl_buffers );

}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;
    SDL_Window* window = NULL;


    if( SDL_Init( SDL_INIT_VIDEO ) < 0 ) {
        printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
        abort();
    }


    //SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);

    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 2);

    window = SDL_CreateWindow( "SDL Test",
                               SDL_WINDOWPOS_UNDEFINED,
                               SDL_WINDOWPOS_UNDEFINED,
                               SCREEN_WIDTH,
                               SCREEN_HEIGHT,
                               SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL );
    if( window == NULL ) {
        printf( "Window could not be created! SDL_Error: %s\n", SDL_GetError() );
        abort();
    }

    SDL_GLContext gContext = SDL_GL_CreateContext( window );
    if( gContext == NULL )
    {
        printf( "OpenGL context could not be created! SDL Error: %s\n", SDL_GetError() );
        abort();
    }

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));

    Init();

    struct aa_gl_globals *globals = aa_gl_globals_create();
    // global camera
    {
        double world_E_camera_home[7] = AA_TF_QUTR_IDENT_INITIALIZER;
        double eye[3] = {1,1,0.5};
        double target[3] = {0,0,0};
        double up[3] = {0,0,1};
        aa_tf_qutr_mzlook( eye, target, up, world_E_camera_home );
        aa_gl_globals_set_camera_home( globals, world_E_camera_home );
        aa_gl_globals_home_camera( globals );

    }

    // global lighting
    {
        double v_light[3] = {.5,1,5};
        double ambient[3] = {.1,.1,.1};
        aa_gl_globals_set_light_position( globals, v_light );
        aa_gl_globals_set_perspective( globals,
                                       M_PI_2,
                                       ((double)SCREEN_WIDTH)/SCREEN_HEIGHT,
                                       .1,
                                   100 );
        aa_gl_globals_set_ambient(globals, ambient);
    }
    double world_E_model[7] = AA_TF_QUTR_IDENT_INITIALIZER;


    int quit,update=1;
    struct timespec delta = aa_tm_sec2timespec( 1.0 / 120 );

    for(;;) {
        //printf("update: %d\n", update );
        if( update ) {
            display( globals, world_E_model );
            SDL_GL_SwapWindow(window);
        } else {
            struct timespec now = aa_tm_add( now, delta );
            /* Required for mouse wheel to work? */
            clock_nanosleep( AA_CLOCK, 0, &delta, NULL );
        }
        aa_sdl_scroll(globals, &update, &quit);
        if( quit ) break;
    }

    SDL_Delay( 1000 );

    SDL_GL_DeleteContext(gContext);
    SDL_DestroyWindow( window );

    SDL_Quit();
    return 0;
}
