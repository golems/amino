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

#include <stdio.h>
#include <math.h>

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_win.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_plugin.h"

#include "baxter-demo.h"

/* from ./baxter-model.c.h */
struct aa_rx_sg * aa_rx_dl_sg__baxter(struct aa_rx_sg *sg);

struct aa_rx_win *
baxter_demo_setup_window ( struct aa_rx_sg *sg  )
{

    struct aa_rx_win * win =
        aa_rx_win_default_create ( "Baxter Demo", SCREEN_WIDTH, SCREEN_HEIGHT );

    printf("OpenGL Version: %s\n", glGetString(GL_VERSION));

    // setup scene graph
    aa_rx_sg_init(sg); /* initialize scene graph internal structures */
    aa_rx_win_sg_gl_init(win, sg); /* Initialize scene graph GL-rendering objects */
    aa_rx_win_set_sg(win, sg); /* Set the scenegraph for the window */

    // result
    return win;
}

struct aa_rx_sg *
baxter_demo_load_baxter( struct aa_rx_sg *sg)
{
    return aa_rx_dl_sg__baxter(sg);
}
