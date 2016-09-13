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

#include "amino/amino_gl.h"
#include <SDL.h>

#include <pthread.h>

#ifdef HAVE_SPNAV_H
#include <spnav.h>
#endif /*HAVE_SPNAV_H*/


#include "amino/rx/rxtype.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_gl_internal.h"
#include "amino/rx/scene_sdl.h"
#include "amino/rx/scene_sdl_internal.h"

static pthread_mutex_t s_mutex = PTHREAD_MUTEX_INITIALIZER;

void s_lock()
{
    if( pthread_mutex_lock( &s_mutex ) ) {
        perror("Mutex Lock");
        abort();
    }
}

void s_unlock()
{
    if( pthread_mutex_unlock( &s_mutex ) ) {
        perror("Mutex Unlock");
        abort();
    }
}


struct handler {
    union {
        SDL_EventType event_type;
        SDL_Keycode key;
    };

    aa_sdl_handler_function handler;
    void *cx;
};

AA_VECTOR_DEF( struct handler*, handler_vector );

static handler_vector s_event_vector;
static handler_vector s_key_vector;
static int s_vector_init = 0;

static void s_init()
{
    if( ! s_vector_init ) {
        s_vector_init = 1;
        handler_vector_init(&s_event_vector, 16);
        handler_vector_init(&s_key_vector, 16);
    }
}

AA_API void
aa_sdl_bind_event( SDL_EventType event_type,
                   aa_sdl_handler_function handler,
                   void *cx )
{
    s_lock();
    s_init();

    struct handler *h = AA_NEW(struct handler);
    h->cx = cx;
    h->event_type = event_type;
    h->handler = handler;

    handler_vector_push(&s_event_vector, h);


    s_unlock();
}

AA_API void
aa_sdl_bind_key( SDL_Keycode key,
                 aa_sdl_handler_function handler,
                 void *cx )
{
    s_lock();
    s_init();

    struct handler *h = AA_NEW(struct handler);
    h->cx = cx;
    h->key = key;
    h->handler = handler;

    handler_vector_push(&s_key_vector, h);


    s_unlock();

}

AA_API int
aa_sdl_handle_event( const SDL_Event *event,
                     struct aa_sdl_display_params *params)
{
    s_lock();

    struct handler *r = NULL;

    if( SDL_KEYDOWN == event->type ) {
        /* search for key */
        SDL_Keycode key = event->key.keysym.sym;
        for( size_t i = 0; !r && i < s_key_vector.size; i ++ ) {
            struct handler *h = s_key_vector.data[i];
            if( key == h->key ) r = h;
        }
    } else {
        /* search for event */
        for( size_t i = 0; !r && i < s_event_vector.size; i ++ ) {
            struct handler *h = s_event_vector.data[i];
            if( event->type == h->event_type ) r = h;
        }
    }

    s_unlock();

    return (r
            ? r->handler(r->cx, params)
            : -1);
}
