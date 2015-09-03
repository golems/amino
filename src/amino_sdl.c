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

#include "amino/rx/amino_gl.h"
#include "amino/rx/amino_gl_internal.h"
#include "amino/rx/amino_sdl.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"


static void scroll( double x, double y,
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

void aa_sdl_scroll( struct aa_gl_globals * globals,
                    int *update, int *quit )
{
    SDL_Event e;
    *quit = 0;
    *update = 0;
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
        aa_tf_quat2rotmat(globals->world_E_cam, R_cam);
        int update_tf = 0;
        //User requests quit
        switch (e.type) {
        case SDL_WINDOWEVENT:
            switch( e.window.event ) {
            case SDL_WINDOWEVENT_EXPOSED:
                *update = 1;
                break;
            }
            break;
        case SDL_QUIT:
            *quit = 1;
            break;
        case SDL_KEYDOWN: {
            //aa_dump_mat(stdout, R_cam, 3, 3);
            double sign = 1;
            switch( e.key.keysym.sym ) {

                //case SDLK_UP: world_E_model[AA_TF_QUTR_TZ] += .1; break;
                //case SDLK_DOWN: world_E_model[AA_TF_QUTR_TZ] -= .1; break;

            case SDLK_KP_2: sign = -1;
            case SDLK_KP_8:
                if( ctrl ) {
                    cam_E_camp[AA_TF_QUTR_TY] += sign*globals->scroll_ratio;
                } else {
                    scroll( sign*globals->angle_ratio, 0,
                            R_cam, world_E_cam0, cam_E_camp );
                }
                update_tf = 1;
                break;
            case SDLK_KP_4: sign = -1;
            case SDLK_KP_6:
                if( ctrl ) {
                    cam_E_camp[AA_TF_QUTR_TX] += sign*globals->scroll_ratio;
                } else {
                    scroll( 0, sign*globals->angle_ratio,
                            R_cam, world_E_cam0, cam_E_camp );
                }
                update_tf = 1;
                break;

            case SDLK_KP_MINUS: sign = -1;
            case SDLK_KP_PLUS:
                cam_E_camp[AA_TF_QUTR_TZ] -= sign*globals->scroll_ratio;
                update_tf = 1;
                break;
            case SDLK_HOME:
                aa_gl_globals_home_camera( globals );
                update_tf = 1;
                break;
            default:
                break;
            }
            break;
        }
        case SDL_MOUSEBUTTONDOWN:
            SDL_GetMouseState(&globals->mouse[0], &globals->mouse[1]);
            break;
        case SDL_MOUSEMOTION: {
            double dx = (e.motion.x-globals->mouse[0]);
            double dy = (e.motion.y-globals->mouse[1]);
            if( SDL_BUTTON(SDL_BUTTON_LEFT) & e.motion.state ) {
                scroll( -.1*globals->angle_ratio*dy, -.1*globals->angle_ratio*dx,
                        R_cam, world_E_cam0, cam_E_camp );
                update_tf = 1;
            } else if( e.motion.state & SDL_BUTTON(SDL_BUTTON_RIGHT) ) {
                cam_E_camp[AA_TF_QUTR_TX] = -.1*globals->scroll_ratio*dx;
                cam_E_camp[AA_TF_QUTR_TY] = .1*globals->scroll_ratio*dy;
                update_tf = 1;
            }
            globals->mouse[0] = e.motion.x;
            globals->mouse[1] = e.motion.y;

            break;
        }
        case SDL_MOUSEWHEEL: {
            uint32_t b = SDL_GetMouseState(NULL, NULL);
            if( (ctrl && shift) ||
                b & SDL_BUTTON(SDL_BUTTON_LEFT) ) {
                aa_tf_zangle2quat( globals->angle_ratio * e.wheel.y, cam_E_camp );
            } else if( alt && shift ) {
                scroll( globals->angle_ratio*e.wheel.y, 0,
                        R_cam, world_E_cam0, cam_E_camp );
            } else if( alt && ctrl ) {
                scroll( 0, globals->angle_ratio*e.wheel.y,
                        R_cam, world_E_cam0, cam_E_camp );
            } else if( ctrl ) {
                cam_E_camp[AA_TF_QUTR_TX] = globals->scroll_ratio * e.wheel.y;
            } else if( shift ) {
                cam_E_camp[AA_TF_QUTR_TY] = globals->scroll_ratio * e.wheel.y;
            } else if (!ctrl && !shift && !alt ) {
                cam_E_camp[AA_TF_QUTR_TZ] = -globals->scroll_ratio * e.wheel.y;
            }

            update_tf = 1;
        }
            break;

        } // end event switch

        if( update_tf ) {
            double Etmp[2][7];
            aa_tf_qutr_mul( world_E_cam0, globals->world_E_cam, Etmp[0] );
            aa_tf_qutr_mul( Etmp[0], cam_E_camp, Etmp[1] );
            aa_gl_globals_set_camera( globals, Etmp[1] );
            //printf("camera: " );
            //aa_dump_vec( stdout, world_E_camera, 7 );
            *update = 1;
        }
    }

    //SDL_UpdateWindowSurface( window );
    //SDL_UpdateWindowSurface( window );

}
