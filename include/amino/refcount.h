/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
 * All rights reserved.
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

#ifndef AA_REFCOUNT_H
#define AA_REFCOUNT_H

/***********************************/
/* Synchronized Reference Counting */
/***********************************/

#ifndef __cplusplus

#if  ( (__STDC_VERSION__ >= 201112L) && ! defined(__STD_NO_ATOMICS__) ) || defined (_STDATOMIC_H)

/* C11 atomics version */
#define AA_ATOMIC _Atomic
#define aa_mem_ref_inc aa_mem_ref_inc_atomic
#define aa_mem_ref_dec aa_mem_ref_dec_atomic

/**
 * Atomically increment the reference count and return the previous count.
 *
 * This version uses C11 atomics
 */
AA_API unsigned
aa_mem_ref_inc_atomic( AA_ATOMIC unsigned *count );

/**
 * Atomically decrement the reference count and return the previous count.
 *
 * This version uses C11 atomics
 */
AA_API unsigned
aa_mem_ref_dec_atomic( AA_ATOMIC unsigned *count );

#else

#warning "No C11 atomics.  Include stdatomic.h or refcounts will be slow."

/* Mutex fallback version */
#define AA_ATOMIC
#define aa_mem_ref_inc aa_mem_ref_inc_mutex
#define aa_mem_ref_dec aa_mem_ref_dec_mutex

#endif /* __STDC_VERSION__ >= 201112L */

#else
//TODO: better c++ atomics
#define AA_ATOMIC
#define aa_mem_ref_inc aa_mem_ref_inc_mutex
#define aa_mem_ref_dec aa_mem_ref_dec_mutex
#endif


/**
 * Atomically increment the reference count and return the previous count.
 *
 * This fallback version uses a global mutex instead of C11 atomics.
 */
AA_API unsigned
aa_mem_ref_inc_mutex( unsigned *count );

/**
 * Atomically decrement the reference count and return the previous count.
 *
 * This fallback version uses a global mutex instead of C11 atomics.
 */
AA_API unsigned
aa_mem_ref_dec_mutex( unsigned *count );

#endif /* AA_REFCOUNT_H */
