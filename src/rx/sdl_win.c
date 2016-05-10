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
#include "amino/rx/mp_seq.h"

#include <pthread.h>


static int default_display( struct aa_rx_win *win,
                            void *cx_, struct aa_sdl_display_params *params );


struct aa_rx_win {
    struct aa_gl_globals *gl_globals;

    aa_sdl_win_display_fun display;
    void *display_cx;

    SDL_Window* window;
    SDL_GLContext gl_cx;

    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_mutexattr_t mattr;

    unsigned stop : 1;
    unsigned stop_on_quit : 1;

    unsigned updated : 1;
    unsigned running : 1;

    unsigned paused : 1;
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


    if( pthread_mutexattr_init(&win->mattr) ) {
        perror("pthread_mutexattr_init");
        abort();
    }
    if( pthread_mutexattr_settype(&win->mattr, PTHREAD_MUTEX_RECURSIVE) ) {
        perror("pthread_mutexattr_settype");
        abort();
    }

    if( pthread_mutex_init( & win->mutex, &win->mattr) ) {
        perror("pthread_mutex_init");
        abort();
    }


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
    win->display_cx = NULL;

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

    /* TODO: free display_cx */

    pthread_mutex_destroy( &win->mutex );
    pthread_mutexattr_destroy( &win->mattr );

    SDL_GL_DeleteContext(win->gl_cx);
    SDL_DestroyWindow(win->window);

    free(win);
}

AA_API struct aa_gl_globals *
aa_rx_win_gl_globals( struct aa_rx_win * win)
{
    return win->gl_globals;
}



AA_API const struct aa_rx_sg *
aa_rx_win_get_sg( struct aa_rx_win * win )
{
    (void)win;
    abort();
    //return win->sg;
}

AA_API void
aa_rx_win_display_sg_tf( struct aa_rx_win *win, struct aa_sdl_display_params *params,
                   const struct aa_rx_sg *scenegraph,
                   size_t n_tf, const double *tf_abs, size_t ld_tf )
{
    aa_sdl_display_params_set_update(params);
    const struct aa_gl_globals *globals = win->gl_globals;
    aa_rx_sg_render( scenegraph, globals, n_tf, tf_abs, ld_tf );
}


AA_API void aa_rx_win_display_sg_config(
    struct aa_rx_win *win, struct aa_sdl_display_params *params,
    const struct aa_rx_sg *scenegraph,
    size_t n_q, const double *q )
{
    size_t n = aa_rx_sg_frame_count(scenegraph);
    struct aa_mem_region *reg = aa_mem_region_local_get();
    double *TF_rel = AA_MEM_REGION_NEW_N( reg, double, 2 * 7 * n );
    double *TF_abs = TF_rel + 7*n;
    aa_rx_sg_tf(scenegraph, n_q, q,
                n,
                TF_rel, 7,
                TF_abs, 7 );
    aa_rx_win_display_sg_tf( win, params, scenegraph,
                             n, TF_abs, 7 );

    aa_mem_region_pop(reg,TF_rel);
}



struct default_display_cx {
    const struct aa_rx_sg *sg;
    size_t n_config;
    double *q;
    struct aa_rx_win *win;
};

static int default_display( struct aa_rx_win *win, void *cx_, struct aa_sdl_display_params *params )
{
    /* We hold the mutex now */
    struct default_display_cx *cx = (struct default_display_cx*) cx_;
    int updated = aa_sdl_display_params_get_update(params);

    if( (updated || win->updated) && cx && cx->sg ) {
        aa_rx_win_display_sg_config( win, params,
                                     cx->sg, aa_rx_sg_frame_count(cx->sg), cx->q );
    }

    return updated;
}

AA_API void
aa_rx_win_set_sg( struct aa_rx_win * win,
                  const struct aa_rx_sg *sg )
{
    aa_rx_win_lock(win);

    struct default_display_cx *cx = AA_NEW(struct default_display_cx);
    cx->sg = sg;
    cx->n_config = aa_rx_sg_config_count(sg);
    /* TODO: fill q with something reasonable */
    cx->q = AA_NEW0_AR( double, cx->n_config );

    win->display_cx = cx;
    win->display = default_display;

    win->updated = 1;
    aa_gl_globals_unmask_all( win->gl_globals );

    aa_rx_win_unlock(win);
}

AA_API void
aa_rx_win_set_config( struct aa_rx_win * win,
                      size_t n,
                      const double *q )
{
    aa_rx_win_lock(win);
    if( win->display == default_display ) {
        struct default_display_cx *cx = (struct default_display_cx *) win->display_cx;
        AA_MEM_CPY( cx->q, q, AA_MIN(n, cx->n_config) );
        win->updated = 1;
    } else {
        fprintf(stderr, "Invalid context to set config\n");
    }


    aa_rx_win_unlock(win);
}


static int win_display( void *cx_, struct aa_sdl_display_params *params )
{
    struct aa_rx_win *win = (struct aa_rx_win*) cx_;
    int r = 0;
    aa_rx_win_lock(win);
    if( !win->paused ) {
        r = win->display(win, win->display_cx, params );
    }
    aa_rx_win_unlock(win);
    return r;
}


AA_API void
aa_rx_win_set_display( struct aa_rx_win * win,
                       aa_sdl_win_display_fun display,
                       void *context )
{
    /* TODO: need a context destructor */
    win->display = display;
    win->display_cx = context;
}

struct mp_seq_display_cx {

    double *plan_q;
    double *plan_q_data;
    size_t plan_n_points;
    struct timespec plan_t0;

    struct aa_rx_mp_seq *mp_seq;
};

static int plan_display_1( struct aa_rx_win *win,
                           struct mp_seq_display_cx *cx,
                           struct aa_sdl_display_params *params,
                           const struct aa_rx_sg *scenegraph,
                           size_t n_points, const double *path,
                           double dt, size_t offset )
{

    const struct timespec *now = aa_sdl_display_params_get_time_now(params);
    double t = aa_tm_timespec2sec( aa_tm_sub( *now, cx->plan_t0 ) );
    size_t i0 = (size_t) (t / dt);
    assert( i0 >= offset );
    i0 -= offset;
    size_t i1 = i0 + 1;
    const double *q0, *q1;
    size_t m = aa_rx_sg_config_count(scenegraph);

    if( n_points == i1 ) {
        i1 = i0;
    }

    /* printf("disp1:\n" ); */
    /* printf("i0: %lu\n", i0 ); */
    /* printf("i1: %lu\n", i1 ); */

    assert( i0 < n_points );
    assert( i1 < n_points );
    q0 = AA_MATCOL(path,m,i0);
    q1 = AA_MATCOL(path,m,i1);


    double q[m];
    if( q0 == q1 ) {
        AA_MEM_CPY( q, q0, m );
    } else {
        aa_la_linterp( m,
                       (double)i0*dt, q0,
                       (double)i1*dt, q1,
                       t-dt*(double)offset, q );
    }
    //double qs[ g_space->config_count_set() ];
    //g_space->state_get( q, qs );

    aa_rx_win_display_sg_config( win, params,
                                 scenegraph, m, q );

    return 0;

}


/* static int plan_display( void *cx_, struct aa_sdl_display_params *params ) */
/* { */
/*     struct aa_rx_win *cx = (struct aa_rx_win*) cx_; */
/*     const struct aa_rx_sg *scenegraph = cx->sg; */

/*     if( aa_sdl_display_params_is_first(params) ) { */
/*         memcpy( &cx->plan_t0, */
/*                 aa_sdl_display_params_get_time_initial(params), */
/*                 sizeof(cx->plan_t0) ); */

/*         return 0; */
/*     } */

/*     //double t_max = (g_n_path-1) * dt; */

/*     double dt = .8; */

/*     const struct timespec *now = aa_sdl_display_params_get_time_now(params); */
/*     double t = aa_tm_timespec2sec( aa_tm_sub( *now, cx->plan_t0 ) ); */
/*     size_t i0 = (size_t) (t / dt); */
/*     size_t i1 = i0 + 1; */

/*     if( cx->plan_n_points < i1 ) { */
/*         memcpy( &cx->plan_t0, now, sizeof(cx->plan_t0) ); */
/*     } */

/*     return plan_display_1(cx, params, scenegraph, */
/*                           cx->plan_n_points, cx->plan_q, dt, 0 ); */
/* } */


static int mp_seq_display( struct aa_rx_win *win, void *cx_, struct aa_sdl_display_params *params )
{
    struct mp_seq_display_cx *cx = (struct mp_seq_display_cx *)cx_;

    if( aa_sdl_display_params_is_first(params) ) {
        memcpy( &cx->plan_t0,
                aa_sdl_display_params_get_time_initial(params),
                sizeof(cx->plan_t0) );

        return 0;
    }

    //double t_max = (g_n_path-1) * dt;

    double dt = .8;
    const struct timespec *now = aa_sdl_display_params_get_time_now(params);
    double t = aa_tm_timespec2sec( aa_tm_sub( *now, cx->plan_t0 ) );
    size_t i0 = (size_t) (t / dt);
    size_t i1 = i0 + 1;

    const struct aa_rx_sg *sg;
    size_t n_path = 0;
    const double *q_all_path;
    size_t offset = 0;
    size_t seq_cnt = aa_rx_mp_seq_count(cx->mp_seq);
    size_t all_cnt = aa_rx_mp_seq_point_count(cx->mp_seq);
    /* printf("--\n" ); */
    /* printf("t: %f\n", t ); */
    /* printf("i0: %lu\n", i0 ); */
    /* printf("i1: %lu\n", i1 ); */
    /* printf("seqs: %lu\n", seq_cnt ); */
    /* printf("points: %lu\n", all_cnt ); */
    for( size_t i_mp = 0; i_mp < seq_cnt;  i_mp++ )
    {
        //printf("  i_mp: %lu\n", i_mp );
        aa_rx_mp_seq_elt(cx->mp_seq, i_mp,
                         &sg, &n_path, &q_all_path);
        if( offset + n_path > i0 ) break;
        offset += n_path;
    }
    //printf("offset: %lu\n", offset );

    // check last segment
    if( all_cnt < i1 ) {
        //printf("reset\n");
        memcpy( &cx->plan_t0, now, sizeof(cx->plan_t0) );
        aa_rx_mp_seq_elt(cx->mp_seq, 0, &sg, &n_path, &q_all_path);
        offset = 0;
    }


    return plan_display_1(win, cx, params, sg,
                          n_path, q_all_path, dt, offset );
}


/* static int mp_seq_display( void *cx_, struct aa_sdl_display_params *params ) */
/* { */
/*     struct aa_rx_win *cx = (struct aa_rx_win*) cx_; */
/*     const struct aa_rx_sg *scenegraph = cx->sg; */

/*     const struct timespec *now = aa_sdl_display_params_get_time_now(params); */
/*     //const struct timespec *last = aa_sdl_display_params_get_time_last(params); */


/*     if( aa_sdl_display_params_is_first(params) ) { */
/*         memcpy( &cx->plan_t0, */
/*                 aa_sdl_display_params_get_time_initial(params), */
/*                 sizeof(cx->plan_t0) ); */

/*         return 0; */
/*     } */

/*     size_t m = aa_rx_sg_config_count(scenegraph); */

/*     double t = aa_tm_timespec2sec( aa_tm_sub( *now, cx->plan_t0 ) ); */
/*     double dt = .8; */
/*     //double t_max = (g_n_path-1) * dt; */


/*     size_t i0 = (size_t) (t / dt); */
/*     size_t i1 = i0 + 1; */
/*     double *q0, *q1; */
/*     if( cx->plan_n_points == i1 ) { */
/*         i1 = i0; */
/*     } else if( cx->plan_n_points < i1 ) { */
/*         t = 0; */
/*         i1 = 0; */
/*         i0 = 0; */
/*         memcpy( &cx->plan_t0, now, sizeof(cx->plan_t0) ); */
/*     } */

/*     assert( i0 < cx->plan_n_points ); */
/*     assert( i1 < cx->plan_n_points ); */
/*     q0 = AA_MATCOL(cx->plan_q,m,i0); */
/*     q1 = AA_MATCOL(cx->plan_q,m,i1); */


/*     double q[m]; */
/*     if( q0 == q1 ) { */
/*         AA_MEM_CPY( q, q0, m ); */
/*     } else { */
/*         aa_la_linterp( m, */
/*                        (double)i0*dt, q0, */
/*                        (double)i1*dt, q1, */
/*                        t, q ); */
/*     } */
/*     //double qs[ g_space->config_count_set() ]; */
/*     //g_space->state_get( q, qs ); */


/*     aa_rx_win_display_sg_config( cx, params, */
/*                                  scenegraph, m, q ); */

/*     return 0; */
/* } */


AA_API void
aa_rx_win_set_display_seq( struct aa_rx_win * win, struct aa_rx_mp_seq *mp_seq)
{
    struct mp_seq_display_cx *cx = AA_NEW0(struct mp_seq_display_cx);
    cx->mp_seq = mp_seq;
    aa_rx_win_set_display( win, mp_seq_display, cx );
}

AA_API void
aa_rx_win_set_display_plan( struct aa_rx_win * win,
                            struct aa_rx_sg *sg,
                            size_t n_plan_elements,
                            const double *plan )
{
    /* AA_MEM_DUPOP( refop, double, */
    /*               win->plan_q, win->plan_q_data, */
    /*               (double*)plan, n_plan_elements ); */
    /* win->plan_n_points = n_plan_elements / aa_rx_sg_config_count(win->sg) ; */
    /* aa_rx_win_set_display( win, plan_display, win ); */


    /* TODO: check if we leak this object */
    struct aa_rx_mp_seq *mp_seq = aa_rx_mp_seq_create();
    aa_rx_mp_seq_append_all(mp_seq, sg, n_plan_elements, plan );
    aa_rx_win_set_display_seq( win, mp_seq );
}


AA_API void aa_rx_win_display_loop( struct aa_rx_win * win )
{


    aa_rx_win_lock(win);

    SDL_GL_MakeCurrent(win->window, win->gl_cx);

    win->running = 1;

    aa_rx_win_unlock(win);

    int stop = 0;
    int stop_on_quit = 0;

    do {
        aa_sdl_display_loop( win->window, win->gl_globals,
                             win_display, win );
        aa_rx_win_lock(win);
        stop = win->stop;
        stop_on_quit = win->stop_on_quit;
        aa_rx_win_unlock(win);
    } while( ! stop && !stop_on_quit );


    aa_rx_win_lock(win);
    win->running = 0;
    aa_rx_win_unlock(win);

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
    aa_rx_win_lock(win);
    win->stop = 1;
    aa_rx_win_unlock(win);

}

AA_API void
aa_rx_win_stop_on_quit( struct aa_rx_win * win, int value )
{
    aa_rx_win_lock(win);
    win->stop_on_quit = value ? 1 : 0;
    aa_rx_win_unlock(win);

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
    aa_rx_win_lock(win);
    SDL_GL_MakeCurrent(win->window, win->gl_cx);
    aa_rx_sg_gl_init(sg);
    aa_rx_win_unlock(win);
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

AA_API void
aa_rx_win_pause( struct aa_rx_win * win, int paused )
{
    aa_rx_win_lock(win);
    win->paused = paused ? 1 : 0;
    aa_rx_win_unlock(win);
}

AA_API void
aa_rx_win_get_tf_cam( const struct aa_rx_win * win, double *E )
{
    AA_MEM_CPY(E, win->gl_globals->world_E_cam, 7 );
}

AA_API void
aa_rx_win_lock( struct aa_rx_win * win )
{
    if( pthread_mutex_lock( &win->mutex ) ) {
        perror("Mutex Lock");
        abort();
    }
}

AA_API void
aa_rx_win_unlock( struct aa_rx_win * win )
{
    if( pthread_mutex_unlock( &win->mutex ) ) {
        perror("Mutex Unlock");
        abort();
    }
}
