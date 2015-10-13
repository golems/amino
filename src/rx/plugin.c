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
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"
#include "amino/rx/scene_plugin.h"


/* TODO: register destructors to close files when destroying meshes
 * and scene graphs*/

static void *
rx_dlopen( const char *filename, const char *sym, void **handle )
{
    *handle = dlopen(filename, RTLD_LOCAL | RTLD_LAZY);
    if( NULL == *handle ) {
        fprintf(stderr, "ERROR: plugin '%s' not found\n", filename );
        return NULL;
    }

    void *result = dlsym(*handle, sym);
    if( NULL == result ) {
        dlclose(handle);
        fprintf(stderr, "ERROR: symbol '%s' not found in plugin '%s'\n",
                sym, filename);
        return NULL;
    }
    return result;
}

/* We register this destructor function with scenegraphs and meshes
* loaded from the plugin.  When the object is destroyed, the object's
* destructor calls the registered plugin_destructor to dlclose() the
* plugin.  Multiple objects loaded from the same plugin will increment
* the plugin's refcount (handled internally in dlopen()/dlclose()),
* and dlclose() will correspondingly decrement the plugin's refcout.
*/
static void plugin_destructor (void *handle)
{
    int r = dlclose(handle);
    if(r) {
        perror("ERROR (dlclose)");
    }
}

AA_API struct aa_rx_mesh *
aa_rx_dl_mesh( const char *filename, const char *name )
{
    size_t n = strlen(name);
    char buf[32+n];
    snprintf(buf, sizeof(buf), "aa_rx_dl_mesh__%s", name);

    void *handle;
    aa_rx_dl_mesh_fun fun = (aa_rx_dl_mesh_fun) rx_dlopen(filename, buf, &handle);
    if(fun) {
        struct aa_rx_mesh *mesh = fun();
        mesh->destructor = plugin_destructor;
        mesh->destructor_context = handle;
        return mesh;
    } else {
        return NULL;
    }
}

AA_API struct aa_rx_sg *
aa_rx_dl_sg( const char *filename, const char *name,
             struct aa_rx_sg *sg)
{
    size_t n = strlen(name);
    char buf[32+n];
    snprintf(buf, sizeof(buf), "aa_rx_dl_sg__%s", name);

    void *handle;
    aa_rx_dl_sg_fun fun = (aa_rx_dl_sg_fun)rx_dlopen(filename, buf, &handle);

    if(fun){
        sg = fun(sg);
        aa_rx_sg_set_destructor(sg, plugin_destructor, handle);
        return sg;
    } else {
        return NULL;
    }
}
