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

#include <dlfcn.h>

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_plugin.h"


/* TODO: register destructors to close files when destroying meshes
 * and scene graphs*/

static void *
rx_dlopen( const char *filename )
{
    void *handle = dlopen(filename, RTLD_LOCAL | RTLD_LAZY);
    return handle;
}

/* AA_API struct aa_rx_mesh * */
/* aa_rx_dl_mesh( const char *filename, const char *name ) */
/* { */
/*     void *handle = rx_dlopen(filename); */
/* } */

AA_API struct aa_rx_sg *
aa_rx_dl_sg( const char *filename, const char *name,
             struct aa_rx_sg *sg)
{
    void *handle = rx_dlopen(filename);
    if( NULL == handle ) {
        fprintf(stderr, "ERROR: plugin '%s' not found\n", filename );
        return NULL;
    }

    size_t n = strlen(name);
    char buf[32+n];
    snprintf(buf, sizeof(buf), "aa_rx_dl_sg__%s", name);

    aa_rx_dl_sg_fun fun = (aa_rx_dl_sg_fun)dlsym(handle, buf);
    if( NULL == fun ) {
        fprintf(stderr, "ERROR: symbol '%s' not found in plugin '%s'\n",
                buf, filename);
        return NULL;
    }

    return fun(sg);
}

/* AA_API int */
/* aa_rx_dl_close( const char *filename ) */
/* { */
/* } */
