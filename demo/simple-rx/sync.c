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
 *   AND ON ANY HEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <error.h>
#include <stdio.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <SDL.h>



#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_win.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_sdl.h"

#include <pthread.h>

#include "scene.c.h"



static const int SCREEN_WIDTH = 1000;
static const int SCREEN_HEIGHT = 1000;

static void* fun( void *arg )
{
    fprintf(stderr, "+ worker thread starting\n");
    struct aa_rx_win * win = (struct aa_rx_win * ) arg;

    struct aa_rx_sg *sg0 = aa_rx_dl_sg__scenegraph (NULL);

    for( size_t i = 0; i < 102400; i ++ ) {
        struct aa_rx_sg *sg1 = aa_rx_dl_sg__scenegraph(NULL);
        aa_rx_sg_init(sg1); /* initialize scene graph internal structures */
        aa_rx_win_sg_gl_init(win, sg1); /* Initialize scene graph GL-rendering objects */

        aa_rx_win_set_sg(win, sg1);

        aa_rx_sg_destroy(sg0);

        sg0 = sg1;
    }
    fprintf(stderr, "- worker thread finished\n");
}

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    struct aa_rx_win * win =
        aa_rx_win_default_create ( "Sync Test", SCREEN_WIDTH, SCREEN_HEIGHT );

    struct aa_rx_sg *sg0 = aa_rx_dl_sg__scenegraph (NULL);

    aa_rx_sg_init(sg0); /* initialize scene graph internal structures */
    aa_rx_win_sg_gl_init(win, sg0); /* Initialize scene graph GL-rendering objects */
    aa_rx_win_set_sg(win, sg0); /* Set the scenegraph for the window */

    // start display
    aa_rx_win_start(win);

    int n_thread = 10;
    pthread_t thread[n_thread];
    for( size_t i = 0; i < n_thread; i ++ ) {
        pthread_create( thread + i, NULL, fun, win );
    }
    for( size_t i = 0; i < n_thread; i ++ ) {
        pthread_join(thread[i], NULL );
    }


    // Cleanup
    aa_rx_win_join(win);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
