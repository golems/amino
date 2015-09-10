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
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/amino_gl.h"
#include "amino/rx/amino_sdl.h"
#include "amino/rx/scene_geom.h"

const int SCREEN_WIDTH = 1000;
const int SCREEN_HEIGHT = 1000;

#include <SDL.h>

struct aa_rx_sg *generate_scenegraph(struct aa_rx_sg *sg);


//struct aa_rx_geom_box geom;
//struct aa_rx_geom_grid grid;
struct aa_rx_sg *scenegraph;


void Init(void)
{
    scenegraph = generate_scenegraph(NULL);

    /* // box */
    /* { */
    /*     aa_rx_sg_add_frame_fixed( scenegraph, */
    /*                               "", "box", */
    /*                               aa_tf_quat_ident, aa_tf_vec_ident ); */

    /*     struct aa_rx_geom_opt *box_opt = aa_rx_geom_opt_create(); */
    /*     aa_rx_geom_opt_set_color( box_opt, 1, 0, 0); */
    /*     aa_rx_geom_opt_set_specular( box_opt, .3, .3, .3); */

    /*     double d[3] = {.1, .1, .1}; */
    /*     aa_rx_geom_attach( scenegraph, "box", aa_rx_geom_box(box_opt, d) ); */

    /*     aa_rx_geom_opt_destroy(box_opt); */
    /* } */

    /* // grid */
    /* { */
    /*     aa_rx_sg_add_frame_fixed( scenegraph, */
    /*                               "", "grid", */
    /*                               aa_tf_quat_ident, aa_tf_vec_ident ); */
    /*     struct aa_rx_geom_opt *grid_opt = aa_rx_geom_opt_create(); */
    /*     aa_rx_geom_opt_set_color( grid_opt, .5, .5, .5); */

    /*     double dim[2] = {1,1}; */
    /*     double delta[2] = {.05, .05}; */
    /*     aa_rx_geom_attach( scenegraph, "grid", aa_rx_geom_grid(grid_opt, dim, delta) ); */

    /*     aa_rx_geom_opt_destroy(grid_opt); */
    /* } */
    aa_rx_sg_index(scenegraph);
    aa_rx_sg_gl_init(scenegraph);

}

void check_error( const char *name ){
    for (GLenum err = glGetError(); err != GL_NO_ERROR; err = glGetError()) {
        fprintf(stderr, "error %s: %d: %s\n",  name,  (int)err, gluErrorString(err));
    }
}


void display( const struct aa_gl_globals *globals )
{

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    check_error("glClearColor");

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    check_error("glClear");

    aa_rx_frame_id n = aa_rx_sg_frame_count(scenegraph);
    aa_rx_frame_id m = aa_rx_sg_config_count(scenegraph);
    double q[m];
    AA_MEM_ZERO(q,m);
    double TF_rel[7*n];
    double TF_abs[7*n];
    aa_rx_sg_tf(scenegraph, m, q,
                n,
                TF_rel, 7,
                TF_abs, 7 );
    aa_rx_sg_render( scenegraph, globals,
                     (size_t)n, TF_abs, 7 );
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
    SDL_SetHint(SDL_HINT_VIDEO_MINIMIZE_ON_FOCUS_LOSS, "0");

    window = SDL_CreateWindow( "SDL Test",
                               SDL_WINDOWPOS_UNDEFINED,
                               SDL_WINDOWPOS_UNDEFINED,
                               SCREEN_WIDTH,
                               SCREEN_HEIGHT,
                               SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE );
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

    int quit,update=1;
    struct timespec delta = aa_tm_sec2timespec( 1.0 / 120 );

    for(;;) {
        //printf("update: %d\n", update );
        if( update ) {
            display( globals );
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
