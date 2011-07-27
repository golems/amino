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

AA_API aa_flexbuf_t *aa_flexbuf_alloc(size_t n) {
    aa_flexbuf_t *fb = (aa_flexbuf_t*)aa_malloc0( sizeof(aa_flexbuf_t)+n );
    fb->n = n;
    return fb;
}
AA_API void aa_flexbuf_free(aa_flexbuf_t *p) {
    aa_free_if_valid(p);
}


void aa_region_init( aa_region_t *region, size_t size ) {
    region->fill = 0;
    region->total = 0;
    region->node.fbuf = aa_flexbuf_alloc(size);
    region->node.next = NULL;
}

static void aa_region_destroy_node( struct aa_region_node *p ) {
    while( p ) {
        struct aa_region_node *n = p->next;
        aa_flexbuf_free(p->fbuf);
        free(p);
        p = n;
    }
}

void aa_region_destroy( aa_region_t *region ) {
    aa_flexbuf_free( region->node.fbuf );
    // free rest of buffers
    aa_region_destroy_node( region->node.next );
}


static int aa_region_pop_in_node( uint8_t *ptr, struct aa_region_node *node ) {
    return ptr >= node->fbuf->d  && ptr < node->fbuf->d + node->fbuf->n;
}

AA_API void aa_region_pop( aa_region_t *reg, void *ptr ) {
    uint8_t *ptr8 = (uint8_t*)ptr;
    if( aa_region_pop_in_node( ptr8, &reg->node ) ){
        // simple case, in the top node
        assert(ptr8 >= reg->node.fbuf->d);
        reg->fill = (size_t)(ptr8 - reg->node.fbuf->d);
    } else {
        // ugly case, iterate through nodes
        // eh, fuck it, you'll just have to release the buffer later
    }
}

// FIXME: maybe we should use mmap instead of malloc()
void *aa_region_alloc( aa_region_t *region, size_t size ) {
    // unsigned arithmetic, check for overflow too
    if(region->fill > region->node.fbuf->n ||
       size > region->node.fbuf->n - region->fill ) {
        // slow path, malloc another buffer
        struct aa_region_node *n = AA_NEW( struct aa_region_node );
        n->fbuf = region->node.fbuf;
        n->next = region->node.next;
        region->node.next = n;

        region->node.fbuf = aa_flexbuf_alloc( AA_MAX(2*region->total, size) );
        region->total += region->fill;
        region->fill = 0;

    }
    // fast path, pointer increment
    void* p = (char*)region->node.fbuf->d + region->fill;
    size_t align = size%16;
    region->fill += size + ( align ? 16 - align : 0 );
    return p;
}

void aa_region_release( aa_region_t *region ) {
    if( region->node.next ) {
        // compress buffers
        aa_flexbuf_free( region->node.fbuf );
        aa_region_destroy_node( region->node.next );
        region->node.fbuf = aa_flexbuf_alloc( region->total + region->fill);
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
