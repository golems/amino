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

#ifndef AA_MEM_H
#define AA_MEM_H

/**
 * \file amino/mem.h
 */

/**************/
/* Allocation */
/**************/

/*----------- General Allocation ------------------*/

/* FIXME: we should do something reasonable when malloc() fails, but
   since Linux lies about whether it can provide memory, there may be
   no point. */

/** Malloc and zero initialize size bytes. */
static inline void *aa_malloc0( size_t size ) {
    void *p = malloc(size);
    if(p) memset(p,0,size);
    return p;
}

/** Frees ptr unless NULL == ptr */
static inline void aa_free_if_valid( void *ptr ) {
    if( NULL != ptr ) free(ptr);
}

/** Malloc an array[n] of objects of type. */
#define AA_NEW_AR(type,n) ( (type*) malloc(sizeof(type)*(n)) )

/** Malloc and zero initialize an array[n] of objects of type. */
#define AA_NEW0_AR(type,n) ( (type*) aa_malloc0(sizeof(type)*(n)) )

/** Malloc an object of type. */
#define AA_NEW(type) AA_NEW_AR(type,1)

/** Malloc and zero initialize an object of type. */
#define AA_NEW0(type) AA_NEW0_AR(type,1)

/** Returns val aligned to some multiple of align.  align must be
 * a power of 2.  Evaluates arguments multiple times. */
#define AA_ALIGN2( val, align ) (((val) + (align) - 1) & ~(align-1))


/*----------- Local Allocation ------------------*/
#ifndef AA_ALLOC_STACK_MAX
/// maximum size of objects to stack allocate
#define AA_ALLOC_STACK_MAX (4096-64)
#endif //AA_ALLOC_STACK_MAX

/** Allocate a local memory block.
 *
 * This macro will stack-allocate small memory blocks and
 * heap-allocate large ones.
 *
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

/** Allocate a local array of type with n elements.
 *
 * Uses AA_ALLOCAL.
 */
#define AA_NEW_LOCAL(type, n) ( (type*) AA_ALLOCAL( sizeof(type)*(n) ) )

/** Free the results of AA_ALLOCAL.
 *
 * This function should be called once for every call to AA_ALLOCAL in
 * case the previously requested memory was put in the heap.
 */
static inline void aa_frlocal( void *ptr, size_t size ) {
    if( size > AA_ALLOC_STACK_MAX) free(ptr);
}

/** Frees the result of AA_NEW_LOCAL. */
#define AA_DEL_LOCAL(ptr, type, n) aa_frlocal( ptr, sizeof(type)*(n) )

/// A buffer struct
typedef struct {
    size_t n;    ///< size of buffer
    union {
        uint64_t dalign;   ///< dummy element for alignment
        uint8_t d[1];      ///< data
    };
} aa_flexbuf_t;

/// allocate buffer of n bytes
AA_API aa_flexbuf_t *aa_flexbuf_alloc(size_t n);
/// free buffer
AA_API void aa_flexbuf_free(aa_flexbuf_t *p);

/*----------- Region Allocation ------------------*/

/// Alignment of each pointer allocated out of a memory region
#define AA_MEMREG_ALIGN 16

/** A single block of memory to be parceled out by the region allocator.
 *
 * Library users don't need to handle this directly.
 */
struct aa_memreg_node {
    //size_t n;                    ///< size of this chunk
    uint8_t *end;                ///< pointer to end of this chunk
    struct aa_memreg_node *next; ///< pointer to next chunk node
    uint8_t d[];           ///< data array
};

/** Data Structure for Region-Based memory allocation.
 *
 * Memory regions provide fast allocation of individual objects and
 * fast deallocation of all objects in the region and of all objects
 * allocated after some given object.  It is not possible to
 * deallocate only a single arbitrary object from the region.
 *
 * This provides two benefits over malloc/free.  First, as long as the
 * requested allocation can be fulfilled from the memory available in
 * the region, allocation and deallocation require only a pointer
 * increment or decrement.  Second, there is no need to store any
 * metadata for each individual allocation resulting in some space
 * savings, expecially for small objects.
 *
 * This implementation pre-allocates chunks of memory and fulfills
 * individual requests from those chunks.  If the request cannot be
 * satisfied from the currently available chunk, this implemention
 * will malloc() another chunk and add it to the region in a linked
 * list.  When objects are deallocated via a release or a pop, chunks
 * are merged into one, possibly requiring several calls to free()
 * followed by a single malloc of a new, larger chunk.
 */
typedef struct {
    uint8_t *head;                ///< pointer to first free element of top chunk
    struct aa_memreg_node *node;  ///< linked list of chunks
} aa_memreg_t;

/** Initialize memory region with an initial chunk of size bytes. */
AA_API void aa_memreg_init( aa_memreg_t *region, size_t size );

/** Destroy memory region freeing all chunks.
 */
AA_API void aa_memreg_destroy( aa_memreg_t *region );


/** Number of free contiguous bytes in region.  This is the number of
 * bytes which may be allocated without creating another chunk.
 */
AA_API size_t aa_memreg_freesize( aa_memreg_t *region );

/** Temporary allocation.  The next call to aa_memreg_alloc or
 * aa_memreg_tmpalloc will return the same pointer if sufficient space
 * is available in the top chunk for that subsequent call.
 */
AA_API void *aa_memreg_tmpalloc( aa_memreg_t *region, size_t size );

/** Allocate size bytes from the region.  The head pointer of the top
 * chunk will be returned if it has sufficient space for the
 * allocation.
 */
AA_API void *aa_memreg_alloc( aa_memreg_t *region, size_t size );

/** Deallocates all allocated objects from the region.  If the region
 * contains multiple chunks, they are merged into one.
 */
AA_API void aa_memreg_release( aa_memreg_t *region );

/** printf's into a buffer allocated from region
 */
AA_API char *aa_memreg_printf( aa_memreg_t *region, const char *fmt, ... );

/** printf's into a buffer allocated from region
 */
AA_API char* aa_memreg_vprintf(aa_memreg_t *reg, const char *fmt, va_list ap );

/** Deallocates ptr and all blocks allocated after ptr was allocated.
 */
AA_API void aa_memreg_pop( aa_memreg_t *region, void *ptr );


/** Pointer to start of free space in region.
 */
AA_API void *aa_memreg_ptr( aa_memreg_t *region );
/** Number of chunks in the region.
 */
AA_API size_t aa_memreg_chunk_count( aa_memreg_t *region );
/** Size of top chunk in region.
 */
AA_API size_t aa_memreg_topsize( aa_memreg_t *region );


/** Initialize the thread-local memory region.
 *
 *  \sa aa_memreg_init
 */
AA_API void aa_memreg_local_init( size_t size );

/** Destroy the thread-local memory region.
 *
 *  \sa aa_memreg_destroy
 */
AA_API void aa_memreg_local_destroy( void );

/** Return pointer to a thread-local memory region.
 */
AA_API aa_memreg_t *aa_memreg_local_get( void );

/** Allocate from thread-local memory region*
 *
 *  \sa aa_memreg_alloc
 */
AA_API void *aa_memreg_local_alloc( size_t size );

/** Temporary allocate from thread-local memory region*
 *
 *  \sa aa_memreg_tmpalloc
 */
AA_API void *aa_memreg_local_tmpalloc( size_t size );

/** Pop ptr from thread-local memory region.
 *
 *  \sa aa_memreg_pop
 */
AA_API void aa_memreg_local_pop( void *ptr );

/** Release all objects allocated from thread-local memory region
 *
 *  \sa aa_memreg_release
 */
AA_API void aa_memreg_local_release( void );


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
    aa_memreg_t region; ///< memory region to allocate from
} aa_pool_t;

/// untested
AA_API void aa_pool_init( aa_pool_t *pool, size_t size, size_t count );
/// untested
AA_API void aa_pool_destroy( aa_pool_t *pool );
/// untested
AA_API void *aa_pool_alloc( aa_pool_t *pool );
/// untested
AA_API void aa_pool_free( aa_pool_t *pool, void *ptr );
/// untested
AA_API void aa_pool_release( aa_pool_t *pool );

/*----------- Circular Buffers ------------------*/
/** Circular buffers use a fixed-size array that acts as if connected
 * end-to end
 */
typedef struct {
    void *buf;      ///< memory
    size_t n;       ///< size of the circbuf
    size_t start;   ///< start of filled data
    size_t end;     ///< end of filled data
} aa_circbuf_t;

/// untested
AA_API void aa_circbuf_create( aa_circbuf_t *cb, size_t n );
/// untested
AA_API void aa_circbuf_destroy( aa_circbuf_t *cb, size_t n );
/// untested
AA_API void aa_circbuf_realloc( aa_circbuf_t *cb, size_t n );
/// untested
AA_API size_t aa_circbuf_space( aa_circbuf_t *cb );
/// untested
AA_API size_t aa_circbuf_used( aa_circbuf_t *cb );
/// untested
AA_API void aa_circbuf_put( aa_circbuf_t *cb, void *buf, size_t n );
/// untested
AA_API int aa_circbuf_read( aa_circbuf_t *cb, int fd, size_t n );
/// untested
AA_API int aa_circbuf_write( aa_circbuf_t *cb, int fd, size_t n );

/**********/
/* Arrays */
/**********/

/// make a floating point array literal
#define AA_FAR(...) ((double[]){__VA_ARGS__})

/// copy n double floats from src to dst
static inline void aa_fcpy( double *dst, const double *src, size_t n ) {
    memcpy( dst, src, sizeof( dst[0] ) * n );
}

/// set n double floats to val
static inline void aa_fset( double *dst, double val, size_t n ) {
    for( size_t i = 0; i < n; i ++ )
        dst[i] = val;
}

/// set n bytes of p to zero
static inline void aa_zero( void *p, size_t n ) {
    memset(p,0,n);
}

/// zero array p of length n
static inline void aa_fzero( double *p, size_t n ) {
    aa_zero(p,n*sizeof(double));
}

/// zeros var, must know sizeof(var)
#define AA_ZERO_AR( var ) aa_zero( var, sizeof(var) )

/// sets each element of var to val, must know sizeof(var)
#define AA_SET_AR( var, val )                                   \
    for( size_t aa_$_set_ar_i = 0;                              \
         aa_$_set_ar_i < sizeof(var)/sizeof(var[0]);            \
         aa_$_set_ar_i ++ ) var[aa_$_set_ar_i] = val;

#endif //AA_MEM_H
