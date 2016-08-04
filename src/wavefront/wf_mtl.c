/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
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

#include "amino.h"
#include "wavefront_internal.h"

#include "amino/getset.h"


AA_API struct aa_rx_wf_mtl *
aa_rx_wf_mtl_create()
{
    struct aa_rx_wf_mtl *mtl = AA_NEW0(struct aa_rx_wf_mtl);
    mvec_type_init(&mtl->materials, 4);

    return mtl;
}

AA_API void
aa_rx_wf_mtl_push( struct aa_rx_wf_mtl *mtl, const char *name )
{

    mtl->current = AA_NEW0(struct aa_rx_wf_material);
    mtl->current->name = strdup(name);
    mvec_type_push(&mtl->materials, mtl->current);
}


AA_API void
aa_rx_wf_mtl_destroy( struct aa_rx_wf_mtl * mtl) {
    for( size_t i = 0; i < mtl->materials.size; i ++ ) {
        free( mtl->materials.data[i]->name );
        free( mtl->materials.data[i] );
    }
    free( mtl->materials.data );
    aa_checked_free(mtl->filename);
    free(mtl);
}

AA_API size_t
aa_rx_wf_mtl_material_count( const struct aa_rx_wf_mtl * mtl)
{
    return mtl->materials.size;
}

AA_API struct aa_rx_wf_material *
aa_rx_wf_mtl_get_material( const struct aa_rx_wf_mtl * mtl, size_t i)
{
    if( i < mtl->materials.size ) {
        return mtl->materials.data[i];
    } else {
        return NULL;
    }
}

#define DEF_HAS(THING)                                                  \
    AA_API int                                                          \
    aa_rx_wf_material_has_ ## THING                                     \
    ( const struct aa_rx_wf_material * material )                       \
    {                                                                   \
        return material->has_ ## THING;                                 \
    }                                                                   \

#define DEF_FIELD(TYPE,THING)                                   \
    DEF_HAS(THING)                                              \
    AA_DEF_GETTER( aa_rx_wf_material, TYPE, THING );

AA_DEF_GETTER( aa_rx_wf_material, const char*, name );
DEF_FIELD(const double *, specular)
DEF_FIELD(const double *, ambient)
DEF_FIELD(const double *, emission)
DEF_FIELD(const double *, diffuse)
DEF_FIELD(double, specular_weight)
DEF_FIELD(double, alpha)
DEF_FIELD(double, ior)
