/* -*- mode: C++; c-basic-offset: 4; -*- */
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
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_collision.h"

// TODO: packed storage for the triangular bit vector

struct aa_rx_cl_set {
    size_t n;
    std::vector<bool> *v;
};

AA_API struct aa_rx_cl_set*
aa_rx_cl_set_create( const struct aa_rx_sg *sg )
{
    struct aa_rx_cl_set *set = AA_NEW(struct aa_rx_cl_set);

    set->n = aa_rx_sg_frame_count(sg);
    set->v = new std::vector<bool>(set->n*set->n);

    return set;
}

AA_API void
aa_rx_cl_set_destroy(struct aa_rx_cl_set *cl_set)
{
    delete cl_set->v;
    free(cl_set);
}

static inline size_t cl_set_i(
    const struct aa_rx_cl_set *set,
    aa_rx_frame_id i,
    aa_rx_frame_id j )
{
    assert( i >= 0 );
    assert( j >= 0 );

    size_t r = (i < j) ?
        (set->n*j + i) :
        (set->n*i + j);

    assert( r < set->v->size() );
    return r;
}

#define CLSET_REF(set,i,j) ( (*(set)->v)[ cl_set_i(set,i,j) ] )

AA_API void
aa_rx_cl_set_set( struct aa_rx_cl_set *cl_set,
                  aa_rx_frame_id i,
                  aa_rx_frame_id j,
                  int is_colliding )
{
    CLSET_REF(cl_set, i, j) = is_colliding ? 1 : 0;
}

AA_API void
aa_rx_cl_set_fill( struct aa_rx_cl_set *dst,
                   const struct aa_rx_cl_set *src )
{
    dst->v->assign( src->v->begin(),
                    src->v->end() );
}

AA_API int
aa_rx_cl_set_get( const struct aa_rx_cl_set *cl_set,
                  aa_rx_frame_id i,
                  aa_rx_frame_id j )
{
    return (*cl_set->v)[ cl_set_i(cl_set,i,j) ];
}

AA_API void
aa_rx_sg_cl_set_copy(const struct aa_rx_sg* sg, struct aa_rx_cl_set * set){
    for (size_t i=0; i<sg->sg->allowed_indices1.size(); i++){
        aa_rx_cl_set_set(set, sg->sg->allowed_indices1[i], sg->sg->allowed_indices2[i], 1);
    }
}

AA_API void
aa_rx_cl_set_merge(struct aa_rx_cl_set* into, const struct aa_rx_cl_set* from){
    size_t n_f = into->n;
    assert(n_f == from->n);

    for (size_t i = 0; i<n_f; i++){
        for (size_t j=0; j<i; j++){
            if (aa_rx_cl_set_get(from, i, j)) {
                aa_rx_cl_set_set(into, i, j, 1);
            }
        }
    }
}
