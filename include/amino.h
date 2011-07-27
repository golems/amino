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

#ifndef AMINO_H
#define AMINO_H
/** \file amino.h */
/** \file amino.h
 *
 * \mainpage
 *
 * Amino is package of utilites for robotics software.  In includes
 * basic mathematical and linear algebra routines, memory management,
 * and time-handling.  Design goals are easy integration, efficiency,
 * and simplicity.
 *
 * \author Neil T. Dantam
 */

// include everything we'll typically need
#ifdef __cplusplus
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <ctime>
#include <stdint.h>
#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <stack>
#include <string>
#else
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <errno.h>
#endif //__cplusplus

#include <cblas.h>
#include <time.h>

// for C symbols
#ifdef __cplusplus
#define AA_API extern "C"
#define AA_EXTERN extern "C"
#define AA_RESTRICT
#else
/// calling and name mangling convention for functions
#define AA_API
/// name mangling convention external symbols
#define AA_EXTERN extern
#define AA_RESTRICT restrict
#endif //__cplusplus

/// deprecated thing
#define AA_DEPRECATED  __attribute__ ((deprecated))

/// (int) 1e9
#define AA_IBILLION 1000000000
/// (int) 1e6
#define AA_IMILLION 1000000

// include our own headers
#include "amino/mem.h"
#include "amino/math.h"
#include "amino/tf.h"
#include "amino/lapack.h"
#include "amino/time.h"
#include "amino/debug.h"
#include "amino/plot.h"
#include "amino/kin.h"
#include "amino/type.h"
#include "amino/endconv.h"
#include "amino/unit.h"
#include "amino/validate.h"
#include "amino/io.h"


#endif //AMINO_H
