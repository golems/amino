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

#include "config.h"

#include "amino.h"

#define GL_GLEXT_PROTOTYPES

#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL.h>

#include <pthread.h>

#ifdef HAVE_SPNAV_H
#include <spnav.h>
#endif /*HAVE_SPNAV_H*/


#include "amino/rx/rxtype.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_gl_internal.h"
#include "amino/rx/scene_sdl.h"


struct aa_sdl_display_params {
    struct timespec time_now;
    struct timespec time_initial;
    struct timespec time_last;

    int update;
    int quit;

    int first;
};


const struct timespec *
aa_sdl_display_params_get_time_now( struct aa_sdl_display_params *params )
{
    return &params->time_now;
}

const struct timespec *
aa_sdl_display_params_get_time_initial( struct aa_sdl_display_params *params )
{
    return &params->time_initial;
}

AA_API const struct timespec *
aa_sdl_display_params_get_time_last( struct aa_sdl_display_params *params )
{
    return &params->time_last;
}

int
aa_sdl_display_params_get_update( struct aa_sdl_display_params *params )
{
    return params->update;
}

void
aa_sdl_display_params_set_quit( struct aa_sdl_display_params *params )
{
    params->quit = 1;
}

void
aa_sdl_display_params_set_update( struct aa_sdl_display_params *params )
{
    params->update = 1;
}

int
aa_sdl_display_params_is_first( struct aa_sdl_display_params *params )
{
    return params->first;
}

#ifdef HAVE_SPNAV_H

static void aa_spnav_init() {
    int r = spnav_open();
    if( -1 == r ) {
        fprintf(stderr, "Error, could not open spacenav\n");
    }
}

static void aa_spnav_poll(double dv[3], double omega[3]) {
    const double m = 350;
    spnav_event spnevent = {0};

    // get the latest event
    while( spnav_poll_event(&spnevent) ) {}

    dv[0] = (double)spnevent.motion.x /  m;
    dv[1] = (double)spnevent.motion.y /  m;
    dv[2] = -(double)spnevent.motion.z /  m;
    omega[0] = (double)spnevent.motion.rx / m;
    omega[1] = (double)spnevent.motion.ry / m;
    omega[2] = -(double)spnevent.motion.rz / m;

    // deadzone
    /* for( size_t i = 0; i < 3; i ++ ) { */
    /*     dv[i] = aa_fdeadzone( dv[i], -0.1, 0.1, 0 ); */
    /*     omega[i] = aa_fdeadzone( omega[i], -0.1, 0.1, 0 ); */
    /* } */

}

static void aa_spnav_scroll(struct aa_gl_globals * globals, int *update )
{
    double dx[6];
    aa_spnav_poll(dx+AA_TF_DX_V,
                  dx+AA_TF_DX_W);

    int sp_update = 0;
    for( size_t i = 0; i < 6; i ++ ) {
        if( !aa_feq(0.0,dx[i],0) ) sp_update = 1;
    }

    if( sp_update ) {

        for( size_t i = 0; i < 3; i ++ ) {
            dx[ i + AA_TF_DX_V ] *= 1.25;
            dx[ i + AA_TF_DX_W ] *= .4;
        }

        //double cam1[7];
        //aa_tf_qutr_svel(globals->world_E_cam, dx, 1.0/globals->fps, cam1);

        double S0[8], twist[8], etwist[8], S1[8], E1[7];
        /* TODO: avoid the memcpy()s */
        aa_tf_qutr2duqu( globals->world_E_cam, S0 );
        aa_tf_duqu_vel2twist( S0, dx, twist );
        for( size_t i = 0; i < 8; i ++ ) twist[i] *= (1.0/globals->fps);
        aa_tf_duqu_exp( twist, etwist);
        aa_tf_duqu_mul( S0, etwist, S1 );
        aa_tf_duqu2qutr(S1, E1);

        aa_gl_globals_set_camera( globals, E1 );
        *update = 1;
    }
    /* printf("%f, %f, %f, %f, %f, %f\n", */
    /*         dv[0], dv[1], dv[2], */
    /*         omega[0], omega[1], omega[2] ); */

}

#endif /*HAVE_SPNAV_H*/

pthread_once_t sdl_once = PTHREAD_ONCE_INIT;

static void sdl_init_once( void )
{
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

    SDL_GL_SetSwapInterval(1);

    SDL_SetHint(SDL_HINT_VIDEO_MINIMIZE_ON_FOCUS_LOSS, "0");

#ifdef HAVE_SPNAV_H
    aa_spnav_init();
#endif
}


AA_API void aa_sdl_init( void )
{
    if( pthread_once(&sdl_once, sdl_init_once ) ) {
        perror("pthread_once");
        abort();
    }
}


static void scroll( double x, double y,
                    const double *R_cam, const double *v_cam,
                    double *E_pre, double *E_post )
{
    AA_MEM_CPY(E_pre, aa_tf_qutr_ident, 7);
    AA_MEM_CPY(E_post, aa_tf_qutr_ident, 7);
    (void)R_cam;
    (void)v_cam;

    double q1[4], q2[4];
    const double *a_x = R_cam;
    const double *a_y = aa_tf_vec_z;
    aa_tf_axang2quat2( a_y, y, q1 );
    aa_tf_axang2quat2( a_x, x, q2 );
    aa_tf_qmul(q1,q2, E_pre);
}

void aa_sdl_scroll_event( struct aa_gl_globals * globals,
                          int *update, int *quit,
                          const SDL_Event *e)
{
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
    switch (e->type) {
    case SDL_WINDOWEVENT:
        switch( e->window.event ) {
        case SDL_WINDOWEVENT_EXPOSED:
            *update = 1;
            break;
        case SDL_WINDOWEVENT_RESIZED:
        case SDL_WINDOWEVENT_SIZE_CHANGED: {
            int w = e->window.data1;
            int h = e->window.data2;
            glViewport(0,0,w,h);
            aa_gl_globals_set_aspect(globals, ((double)w)/((double)h));
            break;
        }
        }
        break;
    case SDL_QUIT:
        printf("quitting\n");
        *quit = 1;
        break;
    case SDL_KEYDOWN: {
        //aa_dump_mat(stdout, R_cam, 3, 3);
        double sign = 1;
        switch( e->key.keysym.sym ) {

            //case SDLK_UP: world_E_model[AA_TF_QUTR_TZ] += .1; break;
            //case SDLK_DOWN: world_E_model[AA_TF_QUTR_TZ] -= .1; break;

        case SDLK_F11: {
            SDL_Window *w = SDL_GetWindowFromID(e->key.windowID);
            uint32_t f = SDL_GetWindowFlags(w);
            if( (f & SDL_WINDOW_FULLSCREEN) ||
                (f & SDL_WINDOW_FULLSCREEN_DESKTOP ) )
            {
                SDL_SetWindowFullscreen( w, 0 );
            } else {
                SDL_SetWindowFullscreen( w, SDL_WINDOW_FULLSCREEN_DESKTOP);
            }
        }
            break;
        case SDLK_KP_2: sign = -1;
        case SDLK_KP_8:
            if( ctrl ) {
                cam_E_camp[AA_TF_QUTR_TY] += sign*globals->scroll_ratio;
            } else {
                scroll( sign*globals->angle_ratio, 0,
                        R_cam, globals->world_E_cam+4,
                        world_E_cam0, cam_E_camp );
            }
            update_tf = 1;
            break;
        case SDLK_KP_4: sign = -1;
        case SDLK_KP_6:
            if( ctrl ) {
                cam_E_camp[AA_TF_QUTR_TX] += sign*globals->scroll_ratio;
            } else {
                scroll( 0, sign*globals->angle_ratio,
                        R_cam, globals->world_E_cam+4,
                        world_E_cam0, cam_E_camp );
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
        double dx = (e->motion.x-globals->mouse[0]);
        double dy = (e->motion.y-globals->mouse[1]);
        if( SDL_BUTTON(SDL_BUTTON_LEFT) & e->motion.state ) {
            scroll( -.1*globals->angle_ratio*dy, -.1*globals->angle_ratio*dx,
                    R_cam, globals->world_E_cam+4,
                    world_E_cam0, cam_E_camp );
            update_tf = 1;
        } else if( e->motion.state & SDL_BUTTON(SDL_BUTTON_RIGHT) ) {
            cam_E_camp[AA_TF_QUTR_TX] = -.1*globals->scroll_ratio*dx;
            cam_E_camp[AA_TF_QUTR_TY] = .1*globals->scroll_ratio*dy;
            update_tf = 1;
        }
        globals->mouse[0] = e->motion.x;
        globals->mouse[1] = e->motion.y;

        break;
    }
    case SDL_MOUSEWHEEL: {
        uint32_t b = SDL_GetMouseState(NULL, NULL);
        if( (ctrl && shift) ||
            b & SDL_BUTTON(SDL_BUTTON_LEFT) ) {
            aa_tf_zangle2quat( globals->angle_ratio * e->wheel.y, cam_E_camp );
        } else if( alt && shift ) {
            scroll( globals->angle_ratio*e->wheel.y, 0,
                    R_cam, globals->world_E_cam+4,
                    world_E_cam0, cam_E_camp );
        } else if( alt && ctrl ) {
            scroll( 0, globals->angle_ratio*e->wheel.y,
                    R_cam, globals->world_E_cam+4,
                    world_E_cam0, cam_E_camp );
        } else if( ctrl ) {
            cam_E_camp[AA_TF_QUTR_TX] = globals->scroll_ratio * e->wheel.y;
        } else if( shift ) {
            cam_E_camp[AA_TF_QUTR_TY] = globals->scroll_ratio * e->wheel.y;
        } else if (!ctrl && !shift && !alt ) {
            cam_E_camp[AA_TF_QUTR_TZ] = -globals->scroll_ratio * e->wheel.y;
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
    //SDL_UpdateWindowSurface( window );
    //SDL_UpdateWindowSurface( window );

}

void aa_sdl_scroll( struct aa_gl_globals * globals,
                    int *update, int *quit )
{
    SDL_Event e;
    while( SDL_PollEvent( &e ) != 0 ) {
        aa_sdl_scroll_event( globals, update, quit, &e );
    }
}

AA_API void aa_sdl_display_loop(
    SDL_Window* window,
    struct aa_gl_globals * globals,
    aa_sdl_display_fun display,
    void *context )
{
    struct timespec delta = aa_tm_sec2timespec( 1.0 / globals->fps );
    struct timespec next;

    struct aa_sdl_display_params params;
    params.update = 1;
    params.quit = 0;
    clock_gettime( CLOCK_MONOTONIC, &params.time_initial );
    params.time_now = params.time_initial;
    params.time_last = params.time_initial;
    params.first = 1;

    do {
        // update screen
        display(context, &params);
        params.first = 0;
        if( params.update ) {
            aa_gl_lock();
            SDL_GL_SwapWindow(window);
            aa_gl_unlock();
            params.update = 0;
        }
        params.time_last = params.time_now;
        next = aa_tm_add(params.time_now, delta);

        // wait for SDL events
        int timeout = 1;
        while( !params.quit && timeout > 0 ) {
            clock_gettime( CLOCK_MONOTONIC, &params.time_now );
            struct timespec ts_timeout = aa_tm_sub(next, params.time_now);
            timeout = (int) aa_tm_timespec2msec(ts_timeout);

            SDL_Event e;
            aa_gl_lock();
            int r = SDL_WaitEventTimeout( &e, timeout < 0 ? 0 : timeout );
            aa_gl_unlock();
            if( r ) {
                aa_sdl_scroll_event(globals, &params.update, &params.quit, &e);
            }

        }

        // poll spnav
#ifdef HAVE_SPNAV_H
        aa_spnav_scroll(globals, &params.update);
#endif

        // update times
        clock_gettime( CLOCK_MONOTONIC, &params.time_now );
    } while ( !params.quit );
}


AA_API void aa_sdl_gl_window(
    const char* title,
    int x_pos,
    int y_pos,
    int width,
    int height,
    Uint32 flags,
    SDL_Window **pwindow,
    SDL_GLContext *p_glcontext )
{
    aa_sdl_init();

    *pwindow = SDL_CreateWindow( title,
                                 x_pos, y_pos,
                                 width, height,
                                 flags | SDL_WINDOW_OPENGL );
    if( *pwindow == NULL ) {
        printf( "Window could not be created! SDL_Error: %s\n", SDL_GetError() );
        abort();
    }

    *p_glcontext = SDL_GL_CreateContext( *pwindow );
    if( p_glcontext == NULL )
    {
        fprintf( stderr, "OpenGL context could not be created! SDL Error: %s\n", SDL_GetError() );
        abort();
    }

    aa_gl_init();
    //printf("OpenGL Version: %s\n", glGetString(GL_VERSION));

}
