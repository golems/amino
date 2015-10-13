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

struct aa_rx_win {
    struct aa_gl_globals *gl_globals;
    const struct aa_rx_sg *sg;
    size_t n_config;
    double *q;

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


struct win_display_cx {
    struct aa_rx_win *win;
    aa_sdl_display_fun display;
    void *context;
};

static int win_display( void *cx_, struct aa_sdl_display_params *params )
{
    struct win_display_cx *cx = (struct win_display_cx*)cx_;
    pthread_mutex_lock( &cx->win->mutex );
    int r = cx->display(cx->context, params );
    pthread_mutex_unlock( &cx->win->mutex );
    return r;
}

AA_API void aa_win_display_loop(
    struct aa_rx_win * win,
    aa_sdl_display_fun display,
    void *context )
{

    SDL_GL_MakeCurrent(win->window, win->gl_cx);

    pthread_mutex_lock( &win->mutex );
    win->running = 1;
    pthread_mutex_unlock( &win->mutex );

    struct win_display_cx cx = {.win = win,
                                .display = display,
                                .context = context };
    if( NULL == display ) {
        cx.display = default_display;
        cx.context = win;
    }

    int stop = 0;
    int stop_on_quit = 0;

    do {
        aa_sdl_display_loop( win->window, win->gl_globals,
                             win_display, &cx );

        pthread_mutex_lock( &win->mutex );
        stop = win->stop;
        stop_on_quit = win->stop_on_quit;
        pthread_mutex_unlock( &win->mutex );
    } while( ! stop && !stop_on_quit );


    pthread_mutex_lock( &win->mutex );
    win->running = 0;
    pthread_mutex_unlock( &win->mutex );

}



struct win_thread_cx {
    struct aa_rx_win *win;
    aa_sdl_display_fun display;
    void *display_cx;
};

static void* win_thread_fun( void *arg )
{
    struct win_thread_cx *cx =  (struct win_thread_cx*)arg;

    aa_win_display_loop( cx->win, cx->display, cx->display_cx );

    free(cx );

    return NULL;
}

AA_API void
aa_rx_win_default_start( struct aa_rx_win * win )
{
    aa_rx_win_start( win, NULL, NULL );
}

AA_API void
aa_rx_win_start( struct aa_rx_win * win,
                 aa_sdl_display_fun display,
                 void *context )
{

    struct win_thread_cx *thread_cx = AA_NEW( struct win_thread_cx );
    thread_cx->win = win;
    thread_cx->display = display;
    thread_cx->display_cx = context;
    pthread_create( &win->thread, NULL, win_thread_fun, thread_cx );
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


AA_API void
aa_rx_win_sg_render(
    struct aa_rx_win * win,
    const struct aa_rx_sg *scenegraph,
    const struct aa_gl_globals *globals,
    size_t n_TF, double *TF_abs, size_t ld_TF )
{
    pthread_mutex_lock( &win->mutex );
    SDL_GL_MakeCurrent(win->window, win->gl_cx);
    aa_rx_sg_render( scenegraph, globals,
                     n_TF, TF_abs, ld_TF );
    pthread_mutex_unlock( &win->mutex );
}
