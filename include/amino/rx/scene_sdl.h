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

#ifndef AMINO_RX_SCENE_SDL_H
#define AMINO_RX_SCENE_SDL_H

/**
 * @file scene_sdl.h
 */


AA_API void aa_sdl_init( void );

AA_API void aa_sdl_scroll(
    struct aa_gl_globals * globals,
    int *update, int *quit );


/**
 * @param context A pointer to local context
 * @param updated Whether other parts of these scene are updated
 * @param now The present time
 * @return Whether any update has occurred
 */
typedef int (*aa_sdl_display_fun)(
    void *context,
    int updated,
    const struct timespec *now);

AA_API void aa_sdl_display_loop(
    SDL_Window* window,
    struct aa_gl_globals * globals,
    aa_sdl_display_fun display,
    void *context );



AA_API void aa_sdl_gl_window(
    const char* title,
    int x_pos,
    int y_pos,
    int width,
    int height,
    Uint32 flags,
    SDL_Window **pwindow,
    SDL_GLContext *p_glcontext );


/* Simple Asynchronous Window */

struct aa_rx_sdl_win;

AA_API struct aa_rx_sdl_win *
aa_rx_sdl_win_create(
    const char* title,
    int x_pos,
    int y_pos,
    int width,
    int height,
    Uint32 flags );

AA_API void
aa_rx_sdl_win_destroy( struct aa_rx_sdl_win *  sdl_win);

AA_API struct aa_gl_globals *
aa_rx_sdl_win_gl_globals( struct aa_rx_sdl_win * sdl_win);


AA_API void
aa_rx_sdl_win_set_sg( struct aa_rx_sdl_win * sdl_win,
                      const struct aa_rx_sg *sg );

AA_API void
aa_rx_sdl_win_set_config( struct aa_rx_sdl_win * sdl_win,
                          size_t n,
                          const double *q );

/**
 * Synchronous display using current thread
 */
AA_API void aa_sdl_win_display_loop(
    struct aa_rx_sdl_win * window,
    aa_sdl_display_fun display,
    void *context );


/**
 * Asynchronous display using new thread
 */
AA_API void
aa_rx_sdl_win_start( struct aa_rx_sdl_win * sdl_win,
                     aa_sdl_display_fun display,
                     void *context );

AA_API void
aa_rx_sdl_win_stop( struct aa_rx_sdl_win * sdl_win );

AA_API void
aa_rx_sdl_win_join( struct aa_rx_sdl_win * sdl_handler );

#endif /*AMINO_RX_SCENE_SDL_H*/
