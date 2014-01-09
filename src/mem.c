/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
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
#include <stdarg.h>





AA_API aa_flexbuf_t *aa_flexbuf_alloc(size_t n) {
    aa_flexbuf_t *fb = (aa_flexbuf_t*)aa_malloc0( sizeof(aa_flexbuf_t) + n - sizeof(fb->dalign) );
    fb->n = n;
    return fb;
}
AA_API void aa_flexbuf_free(aa_flexbuf_t *p) {
    aa_free_if_valid(p);
}



static struct aa_mem_region_node  *aa_mem_region_node_alloc(size_t n,
                                                    struct aa_mem_region_node *next) {
    // FIXME: maybe we should use mmap instead of malloc()
    struct aa_mem_region_node *p =
        (struct aa_mem_region_node*)malloc(sizeof(struct aa_mem_region_node) + n + AA_MEMREG_ALIGN);
    p->end = p->d + n + AA_MEMREG_ALIGN;
    p->next = next;
    p->next = next;
    return p;
}

/// free node p and return p->next
static struct aa_mem_region_node  *aa_mem_region_node_free(struct aa_mem_region_node *p)
{
    struct aa_mem_region_node *n = p->next;
    free(p);
    return n;
}

static inline uint8_t *aa_mem_region_alignptr( uint8_t *ptr ) {
    // AA_MEMREG_ALIGN must be a power of 2
    return (uint8_t*)AA_ALIGN2( (uintptr_t)ptr,
                                (uintptr_t)(AA_MEMREG_ALIGN) );
}

static inline void aa_mem_region_align(aa_mem_region_t *reg, uint8_t *nhead) {
    reg->head = aa_mem_region_alignptr(nhead);
}


void aa_mem_region_init( aa_mem_region_t *region, size_t size ) {
    region->node = aa_mem_region_node_alloc(size, NULL);
    aa_mem_region_align( region, region->node->d );
}

void aa_mem_region_destroy( aa_mem_region_t *region ) {
    struct aa_mem_region_node *p = region->node;
    while (p) { p = aa_mem_region_node_free(p); }
    region->node = NULL;
}

void *aa_mem_region_ptr( aa_mem_region_t *region ) {
    return region->head;
}

size_t aa_mem_region_freesize( aa_mem_region_t *region ) {
    return (region->head >= region->node->end) ?
        0 : (size_t)(region->node->end - region->head);
}

static inline int aa_mem_region_pop_in_node( uint8_t *ptr, struct aa_mem_region_node *node ) {
    return ptr >= node->d  && ptr < node->end;
}


AA_API void aa_mem_region_pop( aa_mem_region_t *reg, void *ptr ) {
    uint8_t *ptr8 = (uint8_t*)ptr;
    if( aa_mem_region_pop_in_node( ptr8, reg->node ) ){
        // fast path, in the top node
        aa_mem_region_align( reg, ptr8 );
    } else {
        // ugly case, iterate through nodes
        ptrdiff_t n = reg->node->end - reg->node->d;
        struct aa_mem_region_node *p = aa_mem_region_node_free( reg->node );
        // free old nodes
        while( p && !aa_mem_region_pop_in_node(ptr8, p) ) {
            n += (p->end - p->d);
            p = aa_mem_region_node_free( p );
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
        reg->node =
            aa_mem_region_node_alloc( (size_t)(n+popsize),
                                  (p && aa_mem_region_alignptr(p->d) == ptr8) ?
                                  aa_mem_region_node_free(p) : p );
        reg->head = aa_mem_region_alignptr(reg->node->d);
    }
}

static void aa_mem_region_grow( aa_mem_region_t *reg, size_t size ) {
    size_t nsize = AA_MAX(2*(size_t)(reg->node->end - reg->node->d),
                          size + AA_MEMREG_ALIGN);
    // this growth strategy creates worst-case O(n) memory waste
    reg->node =
        aa_mem_region_node_alloc( nsize,
                              /* free unused chunk */
                              (aa_mem_region_alignptr(reg->node->d) == reg->head) ?
                              aa_mem_region_node_free(reg->node) : reg->node );
    aa_mem_region_align(reg, reg->node->d);
    assert( aa_mem_region_freesize(reg) >= size );
    assert( ! ((uintptr_t)reg->head%16) );
}

void *aa_mem_region_tmpalloc( aa_mem_region_t *region, size_t size ) {
    if( region->head + size > region->node->end ) {
        // slow path
        aa_mem_region_grow(region, size );
    }
    // fast path
    return region->head;
}

AA_API void *aa_mem_region_tmprealloc( aa_mem_region_t *region, size_t size )
{
    void *old_ptr = region->head;
    size_t old_size = aa_mem_region_topsize(region);
    void *new_ptr = aa_mem_region_tmpalloc(region, size);

    if( old_ptr != new_ptr ) {
        memcpy( new_ptr, old_ptr, old_size );
    }

    return new_ptr;
}

void *aa_mem_region_alloc( aa_mem_region_t *region, size_t size ) {
    uint8_t *nhead = region->head + size;
    if( nhead > region->node->end ) {
        // slow path
        aa_mem_region_grow(region, size);
        nhead = region->head + size;
    }
    // fast path
    void *p = region->head;
    aa_mem_region_align( region, nhead );
    return p;
}

AA_API void *aa_mem_region_dup( aa_mem_region_t *region, void *p, size_t size ) {
    void *n = aa_mem_region_alloc(region, size);
    memcpy( n, p, size );
    return n;
}


AA_API size_t aa_mem_region_chunk_count( aa_mem_region_t *region ) {
    size_t i = 1;
    for( struct aa_mem_region_node *p = region->node; p->next; p = p->next )
        i++;
    return i;
}


AA_API size_t aa_mem_region_topsize( aa_mem_region_t *region ) {
    return (size_t) (region->node->end - region->node->d);
}

void aa_mem_region_release( aa_mem_region_t *region ) {
    if( region->node->next ) {
        // compress buffers
        struct aa_mem_region_node *p = region->node;
        size_t n = 0;
        while (p) { n+= (size_t)(p->end - p->d); p = aa_mem_region_node_free(p); }
        region->node = aa_mem_region_node_alloc( n, NULL);
    }
    region->head = region->node->d;
}

char* aa_mem_region_vprintf(aa_mem_region_t *reg, const char *fmt, va_list ap ) {
    int r;
    do {
        va_list ap1;
        va_copy(ap1, ap);
        // r is one less than needed buffer size, (doesn't count '\0')
        r = vsnprintf((char*)aa_mem_region_ptr(reg), aa_mem_region_freesize(reg),
                      fmt, ap1);
        va_end(ap1);
    }while( (r >= (int)aa_mem_region_freesize(reg) ) &&
            aa_mem_region_tmpalloc( reg, (size_t)r + 1) );
    return (char*)aa_mem_region_alloc(reg,(size_t)r + 1);
}

char* aa_mem_region_printf(aa_mem_region_t *reg, const char *fmt, ... ) {
    va_list arg;
    va_start(arg, fmt);
    char *s = aa_mem_region_vprintf(reg, fmt, arg);
    va_end(arg);
    return s;
}

/******************************/
/* THREAD-LOCAL MEMORY REGION */
/******************************/

// Declare the thread-local variable

#ifdef HAVE_THREADS_H
// First, try the C11 standard
static _Thread_local aa_mem_region_t *aa_mem_region_local = NULL;
#elseif defined TLS
// Next, try some Autoconf magic
static TLS aa_mem_region_t *aa_mem_region_local = NULL;
#else
// Otherwise, pray this works...
static __thread aa_mem_region_t *aa_mem_region_local = NULL;
#endif

aa_mem_region_t *aa_mem_region_local_get(void) {
    if( NULL == aa_mem_region_local ) {
        aa_mem_region_local_init( 4096 * 64 );
    }
    return aa_mem_region_local;
}

void aa_mem_region_local_init(size_t size) {
    aa_mem_region_local = (aa_mem_region_t*)malloc(sizeof(aa_mem_region_t));
    aa_mem_region_init( aa_mem_region_local, size );
}

void aa_mem_region_local_destroy(void) {
    if( aa_mem_region_local ) {
        aa_mem_region_destroy( aa_mem_region_local );
    }
    aa_mem_region_local = NULL;
}

void *aa_mem_region_local_alloc(size_t size) {
    return aa_mem_region_alloc( aa_mem_region_local_get(), size );
}

void *aa_mem_region_local_tmpalloc(size_t size) {
    return aa_mem_region_tmpalloc( aa_mem_region_local_get(), size );
}

void aa_mem_region_local_pop(void *ptr) {
    aa_mem_region_pop( aa_mem_region_local_get(), ptr );
}

void aa_mem_region_local_release( void ) {
    aa_mem_region_release( aa_mem_region_local_get() );
}

/****************/
/* MEMORY POOLS */
/****************/

void aa_mem_pool_init( aa_mem_pool_t *pool, size_t size, size_t count ) {
    pool->size = AA_MAX(size,(size_t)16);
    pool->top = NULL;
    aa_mem_region_init( &pool->region, pool->size * count );
}

void aa_mem_pool_destroy( aa_mem_pool_t *pool ) {
    aa_mem_region_destroy( &pool->region );
}

void *aa_mem_pool_alloc( aa_mem_pool_t *pool ) {
    if( pool->top ) {
        // allocate from freed pool
        void *p = pool->top;
        pool->top = *((void**)p);
        return p;
    } else {
        // allocate from region
        return aa_mem_region_alloc( &pool->region, pool->size );
    }
}

void aa_mem_pool_free( aa_mem_pool_t *pool, void *ptr ) {
    *((void**)ptr) = pool->top;
    pool->top = ptr;
}

void aa_mem_pool_release( aa_mem_pool_t *pool ) {
    pool->top = NULL;
    aa_mem_region_release( &pool->region );
}


/*********/
/* Lists */
/*********/

struct aa_mem_rlist *aa_mem_rlist_alloc( struct aa_mem_region *reg ) {
    aa_mem_rlist_t *list = AA_MEM_REGION_NEW( reg, aa_mem_rlist_t);
    list->reg = reg;
    list->head = NULL;
    list->tailp = &list->head;
    return list;
}

/*-- Cons --*/
aa_mem_cons_t *aa_mem_cons_cpy( aa_mem_region_t *reg, void *p, size_t n, aa_mem_cons_t *next  ) {
    aa_mem_cons_t *node = (aa_mem_cons_t*) aa_mem_region_alloc( reg, sizeof(aa_mem_cons_t) + n );
    node->data = node+1;
    node->next = next;
    memcpy( node->data, p, n );
    return node;
}

aa_mem_cons_t *aa_mem_cons_ptr( aa_mem_region_t *reg, void *p, aa_mem_cons_t *next  ) {
    aa_mem_cons_t *node = AA_MEM_REGION_NEW( reg, aa_mem_cons_t );
    node->data = p;
    node->next = next;
    return node;
}

/*-- Push --*/
void aa_mem_rlist_push_cons( struct aa_mem_rlist *list, aa_mem_cons_t *node ) {
    list->head = node;
    if( &list->head == list->tailp )
        list->tailp = &node->next;
}

void aa_mem_rlist_push_cpy( struct aa_mem_rlist *list, void *p, size_t n ) {
    struct aa_mem_cons *node = aa_mem_cons_cpy(list->reg, p, n, list->head);
    aa_mem_rlist_push_cons(list,node);
}

void aa_mem_rlist_push_ptr( struct aa_mem_rlist *list, void *p ) {
    struct aa_mem_cons *node = aa_mem_cons_ptr(list->reg, p, list->head);
    aa_mem_rlist_push_cons(list,node);
}

/*-- Enqueue --*/
void aa_mem_rlist_enqueue_cons( struct aa_mem_rlist *list, aa_mem_cons_t *node ) {
    *list->tailp = node;
    list->tailp = &node->next;
}

void aa_mem_rlist_enqueue_cpy( struct aa_mem_rlist *list, void *p, size_t n ) {
    struct aa_mem_cons *node = aa_mem_cons_cpy(list->reg, p, n, NULL);
    aa_mem_rlist_enqueue_cons(list,node);
}

void aa_mem_rlist_enqueue_ptr( struct aa_mem_rlist *list, void *p ) {
    struct aa_mem_cons *node = aa_mem_cons_ptr(list->reg, p, NULL);
    aa_mem_rlist_enqueue_cons(list,node);
}

/*-- Pop --*/
void *aa_mem_rlist_pop( struct aa_mem_rlist *list ) {
    if( NULL == list->head ) return NULL;

    struct aa_mem_cons *node = list->head;
    list->head = node->next;

    if( NULL == list->head ) list->tailp = &list->head;
    return node->data;

}
