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


#include <pthread.h>

#ifdef HAVE_SPNAV_H
#include <spnav.h>
#endif /*HAVE_SPNAV_H*/


#include "amino/rx/rxtype.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_sdl.h"
#include "amino/rx/scene_gl_internal.h"
#include "amino/rx/scene_sdl_internal.h"

int s_handle_easter( void *cx, struct aa_sdl_display_params *params )
{
    (void) cx;
    (void) params;
    printf("Happy Easter!\n");
    return 0;
}

int s_handle_fullscreen( void *cx, struct aa_sdl_display_params *params )
{
    (void)cx;
    const SDL_Event *e = aa_sdl_display_params_get_event(params);

    SDL_Window *w = SDL_GetWindowFromID(e->key.windowID);
    uint32_t f = SDL_GetWindowFlags(w);
    if( (f & SDL_WINDOW_FULLSCREEN) ||
        (f & SDL_WINDOW_FULLSCREEN_DESKTOP ) )
    {
        SDL_SetWindowFullscreen( w, 0 );
    } else {
        SDL_SetWindowFullscreen( w, SDL_WINDOW_FULLSCREEN_DESKTOP);
    }

    return 0;
}

int s_handle_quit( void *cx, struct aa_sdl_display_params *params )
{
    (void)cx;
    aa_sdl_display_params_set_quit(params);
    return 0;
}

int s_handle_expose ( void *cx, struct aa_sdl_display_params *params )
{
    (void) cx;
    aa_sdl_display_params_set_update(params);
    return 0;
}


AA_API void
aa_sdl_ui_setup()
{
    aa_sdl_bind_key(SDLK_BACKQUOTE, s_handle_easter, NULL );
    aa_sdl_bind_key(SDLK_F11, s_handle_fullscreen, NULL );

    aa_sdl_bind_event(SDL_QUIT, s_handle_quit, NULL );

}
