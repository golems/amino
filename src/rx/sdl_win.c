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

#ifdef HAVE_SPNAV_H
#include <spnav.h>
#endif /*HAVE_SPNAV_H*/


#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_gl_internal.h"
#include "amino/rx/scene_sdl.h"
#include "amino/rx/scene_win.h"

#include <pthread.h>

static int default_display( void *cx_, struct aa_sdl_display_params *params );

struct aa_rx_win {
    struct aa_gl_globals *gl_globals;
    const struct aa_rx_sg *sg;
    size_t n_config;
    double *q;

    double *plan_q;
    double *plan_q_data;
    size_t plan_n_points;
    struct timespec plan_t0;

    aa_sdl_display_fun display;
    void *display_cx;

    SDL_Window* window;
    SDL_GLContext gl_cx;

    pthread_t thread;
    pthread_mutex_t mutex;

    unsigned stop : 1;
    unsigned stop_on_quit : 1;

    unsigned updated : 1;
    unsigned running : 1;
};

AA_API struct aa_rx_win *
aa_rx_win_create(
    const char* title,
    int x_pos,
    int y_pos,
    int width,
    int height,
    Uint32 flags )
{
    struct aa_rx_win *win = AA_NEW0( struct aa_rx_win );
    pthread_mutex_init( & win->mutex, NULL );

    aa_sdl_gl_window( title,
                      x_pos,
                      y_pos,
                      width,
                      height,
                      flags,
                      &win->window, &win->gl_cx);

    win->gl_globals = aa_gl_globals_create();
    aa_gl_globals_set_aspect( win->gl_globals, (double)width / (double)height );

    win->display = default_display;
    win->display_cx = win;

    win->stop_on_quit = 1;

    return win;
}

AA_API struct aa_rx_win *
aa_rx_win_default_create(
    const char* title, int width, int height )
{
    return aa_rx_win_create(
        title,
        SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
        width, height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE );
}



AA_API void
aa_rx_win_destroy( struct aa_rx_win *  win)
{
    aa_gl_globals_destroy( win->gl_globals );

    aa_checked_free( win->q );
    aa_checked_free( win->plan_q_data );

    pthread_mutex_destroy( &win->mutex );

    SDL_GL_DeleteContext(win->gl_cx);
    SDL_DestroyWindow(win->window);

    free(win);
}

AA_API struct aa_gl_globals *
aa_rx_win_gl_globals( struct aa_rx_win * win)
{
    return win->gl_globals;
}


AA_API void
aa_rx_win_set_sg( struct aa_rx_win * win,
                  const struct aa_rx_sg *sg )
{
    pthread_mutex_lock( &win->mutex );

    aa_checked_free( win->q );
    win->sg = sg;
    win->n_config = aa_rx_sg_config_count(sg);
    win->q = AA_NEW0_AR( double, win->n_config );
    win->updated = 1;

    pthread_mutex_unlock( &win->mutex );
}

AA_API void
aa_rx_win_set_config( struct aa_rx_win * win,
                      size_t n,
                      const double *q )
{
    pthread_mutex_lock( &win->mutex );

    AA_MEM_CPY( win->q, q, AA_MIN(n, win->n_config) );
    win->updated = 1;

    pthread_mutex_unlock( &win->mutex );
}


static int default_display( void *cx_, struct aa_sdl_display_params *params )
{
    /* We hold the mutex now */
    struct aa_rx_win *cx = (struct aa_rx_win*) cx_;
    int updated = aa_sdl_display_params_get_update(params);

    if( updated || cx->updated ) {
        aa_sdl_display_params_set_update(params);

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        const struct aa_rx_sg *scenegraph = cx->sg;
        if( scenegraph ) {
            const struct aa_gl_globals *globals = cx->gl_globals;
            size_t n = aa_rx_sg_frame_count(scenegraph);
            size_t m = aa_rx_sg_config_count(scenegraph);

            double TF_rel[7*n];
            double TF_abs[7*n];
            aa_rx_sg_tf(scenegraph, m, cx->q,
                        n,
                        TF_rel, 7,
                        TF_abs, 7 );
            aa_rx_sg_render( scenegraph, globals,
                             (size_t)n, TF_abs, 7 );

        }
    }

    return updated;
}

static int win_display( void *cx_, struct aa_sdl_display_params *params )
{
    struct aa_rx_win *cx = (struct aa_rx_win*) cx_;
    pthread_mutex_lock( &cx->mutex );
    int r = cx->display(cx->display_cx, params );
    pthread_mutex_unlock( &cx->mutex );
    return r;
}


AA_API void
aa_rx_win_set_display( struct aa_rx_win * win,
                       aa_sdl_display_fun display,
                       void *context )
{
    win->display = display;
    win->display_cx = context;
}


static int plan_display( void *cx_, struct aa_sdl_display_params *params )
{
    struct aa_rx_win *cx = (struct aa_rx_win*) cx_;
    const struct aa_gl_globals *globals = cx->gl_globals;
    const struct aa_rx_sg *scenegraph = cx->sg;

    const struct timespec *now = aa_sdl_display_params_get_time_now(params);
    //const struct timespec *last = aa_sdl_display_params_get_time_last(params);


    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if( aa_sdl_display_params_is_first(params) ) {
        memcpy( &cx->plan_t0,
                aa_sdl_display_params_get_time_initial(params),
                sizeof(cx->plan_t0) );

        return 0;
    }

    size_t m = aa_rx_sg_config_count(scenegraph);
    size_t n = aa_rx_sg_frame_count(scenegraph);

    double t = aa_tm_timespec2sec( aa_tm_sub( *now, cx->plan_t0 ) );
    double dt = .8;
    //double t_max = (g_n_path-1) * dt;


    size_t i0 = (size_t) (t / dt);
    size_t i1 = i0 + 1;
    double *q0, *q1;
    if( cx->plan_n_points == i1 ) {
        i1 = i0;
    } else if( cx->plan_n_points < i1 ) {
        t = 0;
        i1 = 0;
        i0 = 0;
        memcpy( &cx->plan_t0, now, sizeof(cx->plan_t0) );
    }

    assert( i0 < cx->plan_n_points );
    assert( i1 < cx->plan_n_points );
    q0 = AA_MATCOL(cx->plan_q,m,i0);
    q1 = AA_MATCOL(cx->plan_q,m,i1);


    double q[m];
    if( q0 == q1 ) {
        AA_MEM_CPY( q, q0, m );
    } else {
        aa_la_linterp( m,
                       (double)i0*dt, q0,
                       (double)i1*dt, q1,
                       t, q );
    }
    //double qs[ g_space->config_count_set() ];
    //g_space->state_get( q, qs );


    double *TF_rel = AA_MEM_REGION_LOCAL_NEW_N( double, 7*n );
    double *TF_abs = AA_MEM_REGION_LOCAL_NEW_N( double, 7*n );
    aa_rx_sg_tf(scenegraph, m, q,
                n,
                TF_rel, 7,
                TF_abs, 7 );
    aa_rx_sg_render( scenegraph, globals,
                     (size_t)n, TF_abs, 7 );


    aa_mem_region_local_pop( TF_rel );

    aa_sdl_display_params_set_update(params);

    return 0;
}


AA_API void
aa_rx_win_set_display_plan( struct aa_rx_win * win,
                            size_t n_plan_elements,
                            const double *plan,
                            enum aa_mem_refop refop )
{
    AA_MEM_DUPOP( refop, double,
                  win->plan_q, win->plan_q_data,
                  (double*)plan, n_plan_elements );
    win->plan_n_points = n_plan_elements / aa_rx_sg_config_count(win->sg) ;
    aa_rx_win_set_display( win, plan_display, win );
}


AA_API void aa_rx_win_display_loop( struct aa_rx_win * win )
{

    SDL_GL_MakeCurrent(win->window, win->gl_cx);

    pthread_mutex_lock( &win->mutex );
    win->running = 1;

    pthread_mutex_unlock( &win->mutex );

    int stop = 0;
    int stop_on_quit = 0;

    do {
        aa_sdl_display_loop( win->window, win->gl_globals,
                             win_display, win );
        pthread_mutex_lock( &win->mutex );
        stop = win->stop;
        stop_on_quit = win->stop_on_quit;
        pthread_mutex_unlock( &win->mutex );
    } while( ! stop && !stop_on_quit );


    pthread_mutex_lock( &win->mutex );
    win->running = 0;
    pthread_mutex_unlock( &win->mutex );

}

static void* win_thread_fun( void *arg )
{
    struct aa_rx_win * win = (struct aa_rx_win*)arg;
    aa_rx_win_display_loop(win);

    return NULL;
}

AA_API void
aa_rx_win_start( struct aa_rx_win * win )
{
    pthread_create( &win->thread, NULL, win_thread_fun, win );
}

AA_API void
aa_rx_win_stop( struct aa_rx_win * win )
{
    pthread_mutex_lock( &win->mutex );
    win->stop = 1;
    pthread_mutex_unlock( &win->mutex );

}

AA_API void
aa_rx_win_stop_on_quit( struct aa_rx_win * win, int value )
{
    pthread_mutex_lock( &win->mutex );
    win->stop_on_quit = value ? 1 : 0;
    pthread_mutex_unlock( &win->mutex );

}


AA_API void
aa_rx_win_join( struct aa_rx_win * win )
{
    pthread_join( win->thread, NULL );
}

AA_API void
aa_rx_win_sg_gl_init( struct aa_rx_win * win,
                      struct aa_rx_sg *sg )
{
    pthread_mutex_lock( &win->mutex );
    SDL_GL_MakeCurrent(win->window, win->gl_cx);
    aa_rx_sg_gl_init(sg);
    pthread_mutex_unlock( &win->mutex );
}


/* AA_API void */
/* aa_rx_win_sg_render( */
/*     struct aa_rx_win * win, */
/*     const struct aa_rx_sg *scenegraph, */
/*     const struct aa_gl_globals *globals, */
/*     size_t n_TF, double *TF_abs, size_t ld_TF ) */
/* { */
/*     pthread_mutex_lock( &win->mutex ); */
/*     SDL_GL_MakeCurrent(win->window, win->gl_cx); */
/*     aa_rx_sg_render( scenegraph, globals, */
/*                      n_TF, TF_abs, ld_TF ); */
/*     pthread_mutex_unlock( &win->mutex ); */
/* } */
