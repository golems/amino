/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "amino.h"

void aa_region_init( aa_region_t *region, size_t size ) {
    //region->size = size;
    region->fill = 0;
    region->total = 0;
    region->node.buf = malloc( size );
    region->node.size = size;
    region->node.next = NULL;
}

static void aa_region_destroy_node( struct aa_region_node *p ) {
    while( p ) {
        struct aa_region_node *n = p->next;
        free(p->buf);
        free(p);
        p = n;
    }
}

void aa_region_destroy( aa_region_t *region ) {
    free( region->node.buf );
    aa_region_destroy_node( region->node.next );
}

// FIXME: maybe we should use mmap instead of malloc()
void *aa_region_alloc( aa_region_t *region, size_t size ) {
    // unsigned arithmetic, check for overflow too
    if(region->fill > region->node.size || size > region->node.size - region->fill ) {
        // slow path, malloc another buffer
        struct aa_region_node *n = AA_NEW( struct aa_region_node );
        n->buf = region->node.buf;
        n->next = region->node.next;
        n->size = region->node.size;
        region->node.next = n;

        region->node.size = AA_MAX( 2*region->total, size );
        region->node.buf = malloc(region->node.size);
        region->total += region->fill;
        region->fill = 0;

    }
    // fast path, pointer increment
    void* p = (char*)region->node.buf + region->fill;
    size_t align = size%16;
    region->fill += size + ( align ? 16 - align : 0 );
    return p;
}

void aa_region_release( aa_region_t *region ) {
    if( region->node.next ) {
        // compress buffers
        free( region->node.buf );
        aa_region_destroy_node( region->node.next );
        region->node.size = region->total + region->fill;
        region->node.buf = malloc(region->node.size);
        region->node.next = NULL;
    }
    region->total = 0;
    region->fill = 0;
}

void aa_pool_init( aa_pool_t *pool, size_t size, size_t count ) {
    pool->size = AA_MAX(size,(size_t)16);
    pool->top = NULL;
    aa_region_init( &pool->region, pool->size * count );
}

void aa_pool_destroy( aa_pool_t *pool ) {
    aa_region_destroy( &pool->region );
}

void *aa_pool_alloc( aa_pool_t *pool ) {
    if( pool->top ) {
        // allocate from freed pool
        void *p = pool->top;
        pool->top = *((void**)p);
        return p;
    } else {
        // allocate from region
        return aa_region_alloc( &pool->region, pool->size );
    }
}

void aa_pool_free( aa_pool_t *pool, void *ptr ) {
    *((void**)ptr) = pool->top;
    pool->top = ptr;
}

void aa_pool_release( aa_pool_t *pool ) {
    pool->top = NULL;
    aa_region_release( &pool->region );
}
