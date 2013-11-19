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


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "amino.h"
#include "org_golems_amino_Lib.h"



JNIEXPORT jlong JNICALL
Java_org_golems_amino_Lib_mem_1region_1create
(JNIEnv *env, jclass clazz, jlong size)
{
    (void)env; (void)clazz;
    aa_mem_region_t *ptr = AA_NEW(struct aa_mem_region);
    aa_mem_region_init(ptr, (size_t)size);
    return (intptr_t)ptr;
}

JNIEXPORT void JNICALL
Java_org_golems_amino_Lib_mem_1region_1destroy
(JNIEnv *env, jclass clazz, jlong handle)
{
    (void)env; (void)clazz;
    aa_mem_region_t *ptr = (struct aa_mem_region*)handle;
    aa_mem_region_destroy(ptr);
    free(ptr);
}

JNIEXPORT void JNICALL
Java_org_golems_amino_Lib_mem_1region_1release
(JNIEnv *env, jclass clazz, jlong handle)
{
    (void)env; (void)clazz;
    aa_mem_region_t *ptr = (struct aa_mem_region*)handle;
    aa_mem_region_release(ptr);

}
