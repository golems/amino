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

#include <pthread.h>

struct aa_rx_sdl_win {
    struct aa_gl_globals *gl_globals;
    const struct aa_rx_sg *sg;
    size_t n_config;
    double *q;

    SDL_Window* window;
    SDL_GLContext gl_cx;

    pthread_t thread;
    pthread_mutex_t mutex;

    unsigned stop : 1;
    unsigned updated : 1;
    unsigned running : 1;
};

AA_API struct aa_rx_sdl_win *
aa_rx_sdl_win_create(
    const char* title,
    int x_pos,
    int y_pos,
    int width,
    int height,
    Uint32 flags )
{
    struct aa_rx_sdl_win *win = AA_NEW0( struct aa_rx_sdl_win );
    pthread_mutex_init( & win->mutex, NULL );

    aa_sdl_gl_window( title,
                      x_pos,
                      y_pos,
                      width,
                      height,
                      flags,
                      &win->window, &win->gl_cx);

    win->gl_globals = aa_gl_globals_create();

    return win;
}


AA_API void
aa_rx_sdl_win_destroy( struct aa_rx_sdl_win *  win)
{
    aa_gl_globals_destroy( win->gl_globals );
    aa_checked_free( win->q );
    pthread_mutex_destroy( &win->mutex );

    SDL_GL_DeleteContext(win->gl_cx);
    SDL_DestroyWindow(win->window);

    free(win);
}

AA_API struct aa_gl_globals *
aa_rx_sdl_win_gl_globals( struct aa_rx_sdl_win * win)
{
    return win->gl_globals;
}


AA_API void
aa_rx_sdl_win_set_sg( struct aa_rx_sdl_win * win,
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
aa_rx_sdl_win_set_config( struct aa_rx_sdl_win * win,
                              size_t n,
                              const double *q )
{
    pthread_mutex_lock( &win->mutex );

    AA_MEM_CPY( win->q, q, AA_MIN(n, win->n_config) );
    win->updated = 1;

    pthread_mutex_unlock( &win->mutex );
}


AA_API void aa_sdl_win_display_loop(
    struct aa_rx_sdl_win * win,
    aa_sdl_display_fun display,
    void *context )
{

    SDL_GL_MakeCurrent(win->window, win->gl_cx);

    pthread_mutex_lock( &win->mutex );
    win->running = 1;
    pthread_mutex_unlock( &win->mutex );

    aa_sdl_display_loop( win->window, win->gl_globals,
                         display, context );
}



static int default_display( void *cx_, int updated, const struct timespec *now )
{
    (void) now;
    struct aa_rx_sdl_win *cx = (struct aa_rx_sdl_win*) cx_;
    pthread_mutex_lock( &cx->mutex );

    if( updated || cx->updated ) {
        updated = 1;

        const struct aa_gl_globals *globals = cx->gl_globals;
        const struct aa_rx_sg *scenegraph = cx->sg;

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


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

    pthread_mutex_unlock( &cx->mutex );
    return updated;
}


struct win_thread_cx {
    struct aa_rx_sdl_win *win;
    aa_sdl_display_fun display;
    void *display_cx;
};

static void* win_thread_fun( void *arg )
{
    struct win_thread_cx *cx =  (struct win_thread_cx*)arg;

    aa_sdl_win_display_loop( cx->win, cx->display, cx->display_cx );

    free(cx );

    return NULL;
}


AA_API void
aa_rx_sdl_win_start( struct aa_rx_sdl_win * win,
                     aa_sdl_display_fun display,
                     void *context )
{

    struct win_thread_cx *thread_cx = AA_NEW( struct win_thread_cx );
    thread_cx->win = win;
    if( display ) {
        thread_cx->display = display;
        thread_cx->display_cx = context;
    } else {
        thread_cx->display = default_display;
        thread_cx->display_cx = win;
    }
    pthread_create( &win->thread, NULL, win_thread_fun, thread_cx );
}

AA_API void
aa_rx_sdl_win_stop( struct aa_rx_sdl_win * win )
{
    pthread_mutex_lock( &win->mutex );

    win->stop = 1;

    pthread_mutex_unlock( &win->mutex );

}

AA_API void
aa_rx_sdl_win_join( struct aa_rx_sdl_win * win )
{
    pthread_join( win->thread, NULL );
}
