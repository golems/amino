/* -*- mode: C; c-basic-offset: 4  -*- */
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

#ifndef AA_MEM_H
#define AA_MEM_H

/**
 * \file amino/aa_mem.h
 */

/**************/
/* Allocation */
/**************/


/*----------- Local Allocation ------------------*/
#define AA_ALLOC_STACK_MAX (4096-64)

/** Allocate a local memory block.
 * This macro will stack-allocate small memory blocks and heap-allocate large ones.
 */
#define AA_ALLOCAL(size) ({ size_t aa_private_size = size; \
        aa_private_size > AA_ALLOC_STACK_MAX ? \
        malloc(aa_private_size) : alloca(aa_private_size);   })

/** Free the results of AA_ALLOCAL
 */
static inline void aa_frlocal( void *ptr, size_t size ) {
    if( size > AA_ALLOC_STACK_MAX) free(ptr);
}

/*----------- Region Allocation ------------------*/
struct aa_region_node {
    void *buf;
    struct aa_region_node *next;
};

typedef struct {
    size_t target;
    size_t size;
    size_t fill;
    size_t total;
    struct aa_region_node node;
} aa_region_t;

void *aa_region_alloc( aa_region_t *region, size_t size );
void aa_region_release( aa_region_t *region );

/**********/
/* Arrays */
/**********/

/// copy n double floats from src to dst
static inline void aa_fcpy( double *dst, const double *src, size_t n ) {
    memcpy( dst, src, sizeof( dst[0] ) * n );
}

/// set n double floats to val
static inline void aa_fset( double *dst, double val, size_t n ) {
    for( size_t i = 0; i < n; i ++ )
        dst[i] = val;
}

#endif //AA_MEM_H
