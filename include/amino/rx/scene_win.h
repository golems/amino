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

#ifndef AMINO_RX_SCENE_WIN_H
#define AMINO_RX_SCENE_WIN_H

/**
 * @file scene_win.h
 */

/**
 * Create a new SDL / OpenGL window.
 */
AA_API struct aa_rx_win *
aa_rx_win_create(
    const char* title,
    int x_pos,
    int y_pos,
    int width,
    int height,
    Uint32 flags );

/**
 * Create a new SDL / OpenGL window with default parameters.
 */
AA_API struct aa_rx_win *
aa_rx_win_default_create(
    const char* title, int width, int height );


/**
 * Destroy SDL / OpenGL window.
 */
AA_API void
aa_rx_win_destroy( struct aa_rx_win *  win);

/**
 * Return a pointer to the window's GL globals struct.
 *
 * Note that the window's rendering thread may asynchronously access
 * the GL globals.  If you need to modify any parameters, synchronize
 * access by locking the window.
 *
 * \sa aa_rx_win_lock
 * \sa aa_rx_win_unlock
 */
AA_API struct aa_gl_globals *
aa_rx_win_gl_globals( struct aa_rx_win * win);


/**
 * Set a scenegraph for the window.
 *
 * The scenegraph is used by the default rendering function.  Custom
 * rendering functions may not need to set the window scene graph
 * first.
 */
AA_API void
aa_rx_win_set_sg( struct aa_rx_win * win,
                  const struct aa_rx_sg *sg );

AA_API const struct aa_rx_sg *
aa_rx_win_get_sg( struct aa_rx_win * win );

/**
 * Render a scenegraph at the given configuration in the window.
 */
AA_API void
aa_rx_win_display_sg_config( struct aa_rx_win *win, struct aa_sdl_display_params *params,
                             const struct aa_rx_sg *scenegraph,
                             size_t n_q, const double *q );


AA_API void
aa_rx_win_display_sg_tf( struct aa_rx_win *win, struct aa_sdl_display_params *params,
                         const struct aa_rx_sg *scenegraph,
                         size_t n_tf, const double *tf_abs, size_t ld_tf );

/**
 * Set the configuration vector for the window.
 *
 * This configuration is used by the default rendering function.
 * Custom rendering functions may not need to set the configuration with this function.
 */
AA_API void
aa_rx_win_set_config( struct aa_rx_win * win,
                      size_t n,
                      const double *q );


/**
 * Set the window's display function
 *
 * @param win The window
 *
 * @param display The display function
 *
 * @param context The context argument for the display function
 *
 * @param destructor A cleanup function that is called on context when
 *                   either another display function is set for the
 *                   window or the window is destroyed.
 */
AA_API void
aa_rx_win_set_display( struct aa_rx_win * win,
                       aa_sdl_win_display_fun display,
                       void *context,
                       void (*destructor)(void*) );


AA_API void
aa_rx_win_set_display_plan( struct aa_rx_win * win,
                            struct aa_rx_sg *sg,
                            size_t n_plan_elements,
                            const double *plan );

struct aa_rx_mp_seq;

AA_API void
aa_rx_win_set_display_seq( struct aa_rx_win * win, struct aa_rx_mp_seq *mp_seq);

/**
 * Synchronous display using current thread
 *
 * This function is thread-safe.  The window will be locked while
 * display() is called.
 *
 */
AA_API void aa_rx_win_display_loop( struct aa_rx_win * window );



/**
 * Start a thread to asynchronously render the window.
 */
AA_API void
aa_rx_win_start( struct aa_rx_win * win );


/*
 * Asynchronous display using new thread and default rendering
 * function.
 *
 * This function renders the previously set scenegraph and and
 * configuration vector.  The configuration vector may be updated
 * while the asynchronous thread is running.
 *
 * @see aa_rx_win_set_sg()
 * @see aa_rx_win_set_config()
 */
//AA_API void
//aa_rx_win_default_start( struct aa_rx_win * win );

/**
 * Instruct the rendering thread to stop.
 *
 * The rendering thread will gracefully terminate, possibly after it
 * finishes displaying another frame.
 */
AA_API void
aa_rx_win_stop( struct aa_rx_win * win );

/**
 * Instruct the rendering thread to stop when the user closes the window.
 */
AA_API void
aa_rx_win_stop_on_quit( struct aa_rx_win * win, int value );

/**
 * Join the asynchronous display thread.
 */
AA_API void
aa_rx_win_join( struct aa_rx_win * win );

/**
 * Pause rendering.
 */
AA_API void
aa_rx_win_pause( struct aa_rx_win * win, int paused );

/**
 * Lock the window.
 *
 * The rendering thread also takes this lock before calling the
 * display function.
 */
AA_API void
aa_rx_win_lock( struct aa_rx_win * win );

/**
 * Unlock the window
 */
AA_API void
aa_rx_win_unlock( struct aa_rx_win * win );

/**
 * Initialize scenegraph GL values for the given window.
 *
 * This function is threadsafe.
 */
AA_API void
aa_rx_win_sg_gl_init( struct aa_rx_win * win,
                      struct aa_rx_sg *sg );


/**
 * Return the current camera pose.
 */
AA_API void
aa_rx_win_get_tf_cam( struct aa_rx_win * win, double *E );

/**
 * Set the camera pose.
 */
AA_API void
aa_rx_win_set_tf_cam( struct aa_rx_win * win, const double *E );


#endif /*AMINO_RX_SCENE_WIN_H*/
