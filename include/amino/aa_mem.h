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

#ifndef AA_MEM_H
#define AA_MEM_H

/**
 * \file amino/aa_mem.h
 */

/**************/
/* Allocation */
/**************/

/*----------- General Allocation ------------------*/

/* FIXME: we should do something reasonable when malloc() fails, but
   since Linux lies about whether it can provide memory, there may be
   no point. */

/** Malloc and zero initialize size bytes. */
AA_CDECL static inline void *aa_malloc0( size_t size ) {
    void *p = malloc(size);
    if(p) memset(p,0,size);
    return p;
}

/** Malloc an array[n] of objects of type. */
#define AA_NEW_AR(type,n) ( (type*) malloc(sizeof(type)*(n)) )

/** Malloc and zero initialize an array[n] of objects of type. */
#define AA_NEW0_AR(type,n) ( (type*) aa_malloc0(sizeof(type)*(n)) )

/** Malloc an object of type. */
#define AA_NEW(type) AA_NEW_AR(type,1)

/** Malloc and zero initialize an object of type. */
#define AA_NEW0(type) AA_NEW0_AR(type,1)

/*----------- Local Allocation ------------------*/
#ifndef AA_ALLOC_STACK_MAX
#define AA_ALLOC_STACK_MAX (4096-64)
#endif //AA_ALLOC_STACK_MAX

/** Allocate a local memory block.
 *
 * This macro will stack-allocate small memory blocks and
 * heap-allocate large ones.

 * This is necessary because GCC does not verify that calls to
 * alloca() or VLAs can actually fit on the stack.  Exceeding the
 * stack bounds will usually cause a segfault.
 *
 * You can change limit for stack allocations by redefining
 * AA_ALLOC_STACK_MAX before including this header or amino.h
 */
#define AA_ALLOCAL(size) ({ size_t aa_$_allocal_size = (size);          \
            aa_$_allocal_size > AA_ALLOC_STACK_MAX ?                    \
                malloc(aa_$_allocal_size) : alloca(aa_$_allocal_size);   })

/** Free the results of AA_ALLOCAL.
 *
 * This function should be called once for every call to AA_ALLOCAL in
 * case the previously requested memory was put in the heap.
 */
AA_CDECL static inline void aa_frlocal( void *ptr, size_t size ) {
    if( size > AA_ALLOC_STACK_MAX) free(ptr);
}

/*----------- Region Allocation ------------------*/

/** Data Structure for Region-Based memory allocation.
 *
 *
 * Memory regions provide fast allocation of individual objects and
 * fast deallocation of all objects in the region.  It is not possible
 * to deallocate only a single object from the region.
 *
 * This provides two benefits over malloc/free.  First, as long as the
 * requested allocation can be fulfilled from the memory available in
 * the region, allocation and deallocation require only a pointer
 * increment or decrement.  Second, there is no need to store any
 * metadata for each individual allocation resulting in some space
 * savings, expecially for small objects.
 *
 * If the request cannot be satisfied from the current region, this
 * implemention will malloc() another buffer and add it to the region.
 * When objects are deallocated, all buffers are merged into one,
 * possibly requiring several calls to free().
 */
typedef struct {
    size_t size;   ///< size of the first buffer
    size_t fill;   ///< previously allocated bytes in first buffer
    size_t total;  ///< allocated bytes in all buffers after the first
    struct aa_region_node {
        void *buf;                   ///< the memory buffer to allocate from
        struct aa_region_node *next; ///< pointer to next buffer node list
    } node;        ///< linked list of buffers
} aa_region_t;

/** Initialize memory region with block of size bytes. */
AA_CDECL void aa_region_init( aa_region_t *region, size_t size );

/** Destroy memory region freeing all block buffers.
 */
AA_CDECL void aa_region_destroy( aa_region_t *region );

/** Allocate size bytes from the region.
 */
AA_CDECL void *aa_region_alloc( aa_region_t *region, size_t size );

/** Deallocates all allocated chunks from the region.
 */
AA_CDECL void aa_region_release( aa_region_t *region );

/*----------- Pooled Allocation ------------------*/

/** Data Structure for Object pools.
 *
 * Memory pools provide fast allocation and deallocation of fixed-size
 * objects.
 *
 * This implimentation uses a memory region to allocation from, which
 * allows fast deallocation of all objects in the pool.
 */
typedef struct {
    size_t size; ///< size of each element
    void *top;   ///< top of list of free elements
    aa_region_t region; ///< memory region to allocate from
} aa_pool_t;

/// untested
AA_CDECL void aa_pool_init( aa_pool_t *pool, size_t size, size_t count );
/// untested
AA_CDECL void aa_pool_destroy( aa_pool_t *pool );
/// untested
AA_CDECL void *aa_pool_alloc( aa_pool_t *pool );
/// untested
AA_CDECL void aa_pool_free( aa_pool_t *pool, void *ptr );
/// untested
AA_CDECL void aa_pool_release( aa_pool_t *pool );


/**********/
/* Arrays */
/**********/

/// copy n double floats from src to dst
AA_CDECL static inline void aa_fcpy( double *dst, const double *src, size_t n ) {
    memcpy( dst, src, sizeof( dst[0] ) * n );
}

/// set n double floats to val
AA_CDECL static inline void aa_fset( double *dst, double val, size_t n ) {
    for( size_t i = 0; i < n; i ++ )
        dst[i] = val;
}

#endif //AA_MEM_H
