/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
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
#include "amino/mem.h"

/* If possible, use spiffy C11 atomics */

#ifdef HAVE_STDATOMIC_H

#include <stdatomic.h>

unsigned aa_mem_ref_inc_atomic( _Atomic unsigned *count )
{
    return atomic_fetch_add(count, 1);
}

unsigned aa_mem_ref_dec_atomic( _Atomic unsigned *count )
{
    return atomic_fetch_sub(count, 1);
}

#endif /* HAVE_STDATOMIC_H */


/* Otherwise, we don't be have nice atomics.  "Do the right thing" and
 * use a mutex. */

#include <pthread.h>

/* Yeah, every refcount in the universe will go through this mutex.
 * If you don't like it, get proper C11 suport already. */
static pthread_mutex_t refcount_mutex = PTHREAD_MUTEX_INITIALIZER;

unsigned aa_mem_ref_inc_mutex( unsigned *count )
{
    pthread_mutex_lock( &refcount_mutex );
    unsigned old = *count;
    (*count)++;
    pthread_mutex_unlock( &refcount_mutex );
    return old;
}

unsigned aa_mem_ref_dec_mutex( unsigned *count )
{
    pthread_mutex_lock( &refcount_mutex );
    unsigned old = *count;
    (*count)--;
    pthread_mutex_unlock( &refcount_mutex );
    return old;
}
