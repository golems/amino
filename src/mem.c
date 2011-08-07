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
#include <stdarg.h>

#define AA_REGION_ALIGN 16

AA_API aa_flexbuf_t *aa_flexbuf_alloc(size_t n) {
    aa_flexbuf_t *fb = (aa_flexbuf_t*)aa_malloc0( sizeof(aa_flexbuf_t) + n - sizeof(fb->dalign) );
    fb->n = n;
    return fb;
}
AA_API void aa_flexbuf_free(aa_flexbuf_t *p) {
    aa_free_if_valid(p);
}

static struct aa_region_node  *aa_region_node_alloc(size_t n, struct aa_region_node *next) {
    // FIXME: maybe we should use mmap instead of malloc()
    struct aa_region_node *p =
        (struct aa_region_node*)aa_malloc0(sizeof(struct aa_region_node) + n - sizeof( p->dalign));
    p->end = p->d + n;
    p->next = next;
    return p;
}

/// free node p and return p->next
static struct aa_region_node  *aa_region_node_free(struct aa_region_node *p)
{
    struct aa_region_node *n = p->next;
    free(p);
    return n;
}

/// set reg->fill to nfill aligned upward to AA_REGION_ALIGN
static inline void aa_region_align(aa_region_t *reg, uint8_t *nhead) {
    // AA_REGION_ALIGN must be a power of 2
    const uintptr_t align = AA_REGION_ALIGN - 1;
    reg->head = (uint8_t*) (((uintptr_t)nhead + align) & ~align);
}


void aa_region_init( aa_region_t *region, size_t size ) {
    region->node = aa_region_node_alloc(size, NULL);
    region->head = region->node->d;
}

void aa_region_destroy( aa_region_t *region ) {
    struct aa_region_node *p = region->node;
    while (p) { p = aa_region_node_free(p); }
    region->node = NULL;
}

void *aa_region_ptr( aa_region_t *region ) {
    return region->head;
}

size_t aa_region_freesize( aa_region_t *region ) {
    return (region->head >= region->node->end) ?
        0 : (size_t)(region->node->end - region->head);
}

static inline int aa_region_pop_in_node( uint8_t *ptr, struct aa_region_node *node ) {
    return ptr >= node->d  && ptr < node->end;
}


AA_API void aa_region_pop( aa_region_t *reg, void *ptr ) {
    uint8_t *ptr8 = (uint8_t*)ptr;
    if( aa_region_pop_in_node( ptr8, reg->node ) ){
        // fast path, in the top node
        aa_region_align( reg, ptr8 );
    } else {
        // ugly case, iterate through nodes
        ptrdiff_t n = reg->node->end - reg->node->d;
        struct aa_region_node *p = aa_region_node_free( reg->node );
        // free old nodes
        while( p && !aa_region_pop_in_node(ptr8, p) ) {
            n += (p->end - p->d);
            p = aa_region_node_free( p );
        }
        assert( !p || (ptr8 >= p->d && ptr8 <= p->end));
        // now realloc a replacement node
        // how much of the last chunk is now unused
        ptrdiff_t popsize = p ? (p->end  - ptr8) : 0;
        assert( ( !p || ptr8 > p->d ) ?
                // popped in middle of chunk, keep it
                (!p || popsize < p->end - p->d ) :
                // poppsed start of chunk, realloc it
                ( (ptr8 == p->d) &&
                  (popsize == p->end - p->d) ) );
        reg->node = aa_region_node_alloc( (size_t)(n+popsize),
                                          (!p || ptr8 > p->d) ? p : aa_region_node_free(p) ) ;
        reg->head = reg->node->d;
    }
}

static void aa_region_grow( aa_region_t *reg, size_t size ) {
    size_t nsize = AA_MAX(2*(size_t)(reg->node->end - reg->node->d), size);
    // this growth strategy creates worst-case O(n) memory waste
    reg->node = aa_region_node_alloc( nsize,
                                      (reg->head == reg->node->d) ?  /* free unused chunk */
                                      aa_region_node_free(reg->node) : reg->node );
    reg->head = reg->node->d;
    assert( aa_region_freesize(reg) >= size );
    assert( ! ((uintptr_t)reg->head%16) );
}

void *aa_region_tmpalloc( aa_region_t *region, size_t size ) {
    if( region->head + size > region->node->end ) {
        // slow path
        aa_region_grow(region, size );
    }
    // fast path
    return region->head;
}

void *aa_region_alloc( aa_region_t *region, size_t size ) {
    uint8_t *nhead = region->head + size;
    if( nhead > region->node->end ) {
        // slow path
        aa_region_grow(region, size);
        nhead = region->head + size;
    }
    // fast path
    void *p = region->head;
    aa_region_align( region, nhead );
    return p;
}


AA_API size_t aa_region_chunk_count( aa_region_t *region ) {
    size_t i = 1;
    for( struct aa_region_node *p = region->node; p->next; p = p->next )
        i++;
    return i;
}


AA_API size_t aa_region_topsize( aa_region_t *region ) {
    return (size_t) (region->node->end - region->node->d);
}

void aa_region_release( aa_region_t *region ) {
    if( region->node->next ) {
        // compress buffers
        struct aa_region_node *p = region->node;
        size_t n = 0;
        while (p) { n+= (size_t)(p->end - p->d); p = aa_region_node_free(p); }
        region->node = aa_region_node_alloc( n, NULL);
    }
    region->head = region->node->d;
}


char* aa_region_printf(aa_region_t *reg, const char *fmt, ... ) {
    int r;
    do {
        va_list arg;
        va_start(arg, fmt);
        r = vsnprintf((char*)aa_region_ptr(reg), aa_region_freesize(reg),
                      fmt, arg);
        va_end(arg);
    }while( (r >= (int)aa_region_freesize(reg) ) &&
            aa_region_tmpalloc( reg, (size_t)r) );
    return (char*)aa_region_alloc(reg,(size_t)r);
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
