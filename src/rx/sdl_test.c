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
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"

const int SCREEN_WIDTH = 1000;
const int SCREEN_HEIGHT = 1000;

#include <SDL.h>


struct aa_rx_geom_box geom;


void Init(void)
{
    geom.base.opt.color[0] = 1;
    geom.base.opt.color[1] = 0;
    geom.base.opt.color[2] = 0;
    geom.base.opt.color[3] = 1;
    geom.base.type = AA_RX_BOX;
    geom.base.gl_buffers = NULL;
    geom.shape.dimension[0] = 0.1;
    geom.shape.dimension[1] = 0.1;
    geom.shape.dimension[2] = 0.1;

    aa_geom_gl_buffers_init( &geom.base );

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


}

void scroll( double x, double y,
             double *R_cam,
             double *E_pre, double *E_post )
{
    AA_MEM_CPY(E_pre, aa_tf_qutr_ident, 7);
    AA_MEM_CPY(E_post, aa_tf_qutr_ident, 7);

    double q1[4], q2[4];
    aa_tf_axang2quat2( R_cam, x, q2 );
    aa_tf_axang2quat2( R_cam+3, y, q1 );
    aa_tf_qmul(q2,q1, E_pre);
    /* aa_dump_mat(stdout, R_cam, 3, 3); */
    /* aa_tf_xangle2quat(  x, q2 ); */
    /* aa_tf_yangle2quat( y, q1 ); */
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

    printf("version: %s\n", glGetString(GL_VERSION));

    Init();

    double world_E_camera_home[7] = AA_TF_QUTR_IDENT_INITIALIZER;
    {
        double eye[3] = {1,1,0.5};
        double target[3] = {0,0,0};
        double up[3] = {0,0,1};
        aa_tf_qutr_mzlook( eye, target, up, world_E_camera_home );
    }
    double world_E_camera[7];
    AA_MEM_CPY( world_E_camera, world_E_camera_home, 7 );
    double scroll_ratio = .05;
    double angle_ratio = .05;
    int mouse[2];

    double world_E_model[7] = AA_TF_QUTR_IDENT_INITIALIZER;
    world_E_model[AA_TF_QUTR_TZ] = .4;

    struct aa_gl_globals *globals = aa_gl_globals_create();
    double v_light[3] = {0,5,5};
    double ambient[3] = {.1,.1,.1};
    aa_gl_globals_set_light_position( globals, v_light );
    aa_gl_globals_set_perspective( globals,
                                   M_PI_2,
                                   ((double)SCREEN_WIDTH)/SCREEN_HEIGHT,
                                   .1,
                                   100 );

    aa_gl_globals_set_ambient(globals, ambient);


    int quit = 0;
    while( !quit ) {
        //Handle events on queue
        SDL_Event e;
        while( SDL_PollEvent( &e ) != 0 ) {
            const Uint8 *state = SDL_GetKeyboardState(NULL);
            int shift =  state[SDL_SCANCODE_LSHIFT] || state[SDL_SCANCODE_RSHIFT];
            int ctrl =  state[SDL_SCANCODE_LCTRL] || state[SDL_SCANCODE_RCTRL] ||
                state[SDL_SCANCODE_CAPSLOCK] ; /* keyboards are wrong, this is CTRL.
                                                * And SDL doesn't respect ctrl:swapcaps */
            int alt =  state[SDL_SCANCODE_LALT] || state[SDL_SCANCODE_RALT];

            double cam_E_camp[7] = AA_TF_QUTR_IDENT_INITIALIZER;
            double world_E_cam0[7] = AA_TF_QUTR_IDENT_INITIALIZER;
            double R_cam[9];
            aa_tf_quat2rotmat(world_E_camera, R_cam);
            int update_tf = 0;
            //User requests quit
            switch (e.type) {
            case SDL_QUIT:
                quit = 1;
                break;
            case SDL_KEYDOWN: {
                aa_dump_mat(stdout, R_cam, 3, 3);
                double sign = 1;
                switch( e.key.keysym.sym ) {

                case SDLK_UP: world_E_model[AA_TF_QUTR_TZ] += .1; break;
                case SDLK_DOWN: world_E_model[AA_TF_QUTR_TZ] -= .1; break;

                case SDLK_KP_2: sign = -1;
                case SDLK_KP_8:
                    if( ctrl ) {
                        cam_E_camp[AA_TF_QUTR_TY] += sign*scroll_ratio;
                    } else {
                        scroll( sign*angle_ratio, 0,
                                R_cam, world_E_cam0, cam_E_camp );
                    }
                    update_tf = 1;
                    break;
                case SDLK_KP_4: sign = -1;
                case SDLK_KP_6:
                    if( ctrl ) {
                        cam_E_camp[AA_TF_QUTR_TX] += sign*scroll_ratio;
                    } else {
                        scroll( 0, sign*angle_ratio,
                                R_cam, world_E_cam0, cam_E_camp );
                    }
                    update_tf = 1;
                    break;

                case SDLK_KP_MINUS: sign = -1;
                case SDLK_KP_PLUS:
                    cam_E_camp[AA_TF_QUTR_TZ] -= sign*scroll_ratio;
                    update_tf = 1;
                    break;
                case SDLK_HOME:
                    AA_MEM_CPY(world_E_camera, world_E_camera_home, 7);
                    update_tf = 1;
                    break;
                default:
                    printf("other key\n");
                    break;
                }
                break;
            }
            case SDL_MOUSEBUTTONDOWN:
                SDL_GetMouseState(&mouse[0], &mouse[1]);
                break;
            case SDL_MOUSEMOTION: {
                if( SDL_BUTTON(e.motion.state) & SDL_BUTTON_LEFT ) {
                    scroll( -.1*angle_ratio*(e.motion.y-mouse[1]),
                            -.1*angle_ratio*(e.motion.x-mouse[0]),
                            R_cam, world_E_cam0, cam_E_camp );
                    mouse[0] = e.motion.x;
                    mouse[1] = e.motion.y;
                    update_tf = 1;
                } else if( SDL_BUTTON(e.motion.state) & SDL_BUTTON_MIDDLE ) {
                    cam_E_camp[AA_TF_QUTR_TX] = -.1*scroll_ratio*(e.motion.x-mouse[0]),
                    cam_E_camp[AA_TF_QUTR_TY] = .1*scroll_ratio*(e.motion.y-mouse[1]),
                    mouse[0] = e.motion.x;
                    mouse[1] = e.motion.y;
                    update_tf = 1;
                }

                break;
            }
            case SDL_MOUSEWHEEL: {
                    if( ctrl && shift ) {
                        aa_tf_zangle2quat( angle_ratio * e.wheel.y, cam_E_camp );
                    } else if( alt && shift ) {
                        scroll( angle_ratio*e.wheel.y, 0,
                                R_cam, world_E_cam0, cam_E_camp );
                    } else if( alt && ctrl ) {
                        scroll( 0, angle_ratio*e.wheel.y,
                                R_cam, world_E_cam0, cam_E_camp );
                    } else if( ctrl ) {
                        cam_E_camp[AA_TF_QUTR_TX] = scroll_ratio * e.wheel.y;
                    } else if( shift ) {
                        cam_E_camp[AA_TF_QUTR_TY] = scroll_ratio * e.wheel.y;
                    } else if (!ctrl && !shift && !alt ) {
                        cam_E_camp[AA_TF_QUTR_TZ] = -scroll_ratio * e.wheel.y;
                    }

                    update_tf = 1;
                }
                break;

            } // end event switch

            if( update_tf ) {
                double Etmp[2][7];
                aa_tf_qutr_mul( world_E_cam0, world_E_camera, Etmp[0] );
                aa_tf_qutr_mul( Etmp[0], cam_E_camp, Etmp[1] );
                AA_MEM_CPY( world_E_camera, Etmp[1], 7 );
                printf("camera: " );aa_dump_vec( stdout, world_E_camera, 7 );
            }
        }

        //SDL_UpdateWindowSurface( window );
        //SDL_UpdateWindowSurface( window );

        aa_gl_globals_set_camera( globals, world_E_camera );
        display( globals, world_E_model );
        SDL_GL_SwapWindow(window);
    }

    SDL_Delay( 1000 );

    SDL_GL_DeleteContext(gContext);
    SDL_DestroyWindow( window );

    SDL_Quit();
    return 0;
}
