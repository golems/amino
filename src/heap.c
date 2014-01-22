/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include "amino.h"

//#define HEAP_PARENT(i) ((i)/2)
#define HEAP_LEFT(i) (2*(i)+1)
#define HEAP_RIGHT(i) (HEAP_LEFT(i)+1)

typedef int (*aa_compar_fun)(const void *, const void *);


size_t aa_aheap_check(uint8_t *base, size_t nmemb, size_t size, size_t i, aa_compar_fun compar )
{
    for( size_t j = HEAP_LEFT(i); j < HEAP_LEFT(i)+2; j ++ ) {
        if( j < nmemb ) {
            if( 0 > compar(&base[i*size], &base[j*size]) ) return i;
            else  {
                size_t k = aa_aheap_check( base, nmemb, size, j, compar );
                if(k) return k;
            }
        }
    }
    return 0;
}


void aa_aheap_heapify(uint8_t *base, size_t nmemb, size_t size,  size_t i, aa_compar_fun compar )
{
    // max heapify
    for(;;) {
        size_t i_l = HEAP_LEFT(i);
        size_t i_r = HEAP_RIGHT(i);
        size_t i_top = i;
        if( i_l < nmemb &&
            0 > compar(&base[i*size], &base[i_l*size]) )
        {
            i_top = i_l;
        } else {
            i_top = i;
        }
        if( i_r < nmemb &&
            0 > compar(&base[i_top*size], &base[i_r*size]) )
        {
            i_top = i_r;
        }

        if( i_top != i ) {
            aa_memswap( &base[i*size], &base[i_top*size], size );

            //assert( 0 < compar(&base[i*size], &base[i_l*size]) );
            //assert( i_r >= nmemb || 0 < compar(&base[i*size], &base[i_r*size]) );

            i = i_top;
        } else {
            break;
        }
    }
}


void aa_aheap_build(uint8_t *base, size_t nmemb,  size_t size, aa_compar_fun compar )
{
    size_t i = nmemb / 2 + 1;
    do {
        i--;
        aa_aheap_heapify( base, nmemb, size, i, compar );
    } while( i > 0 );
}

void aa_aheap_sort(void *_base, size_t nmemb,  size_t size,  aa_compar_fun compar )
{
    if( 1 >= nmemb ) return;

    uint8_t *base = (uint8_t*)_base;

    aa_aheap_build( base, nmemb, size, compar );

    for(size_t i = nmemb-1; i; i--) {
        aa_memswap( base, &base[i*size], size );
        if( i ) aa_aheap_heapify( base, i, size, 0, compar );
        else break;
    }
}
