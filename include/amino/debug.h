/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
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

#ifndef AMINO_DEBUG_H
#define AMINO_DEBUG_H

/**
 * \file amino/debug.h
 */

/// bail out if test is false
AA_API void
aa_hard_assert(int test, const char fmt[], ...);

/// don't use
AA_API void
aa_verbf( int min_level, const char fmt[], ...) AA_DEPRECATED;

/// print a vec to file
AA_API void
aa_dump_vec( FILE *file, const double *v, size_t n );

/// print a matrix to file
AA_API void
aa_dump_mat( FILE *file, const double *A, size_t m, size_t n );

/** Print a matrix.
 *
 * \param file FILE pointer to print to
 * \param fmt format specifier for matrix element, ie. "%f" or "%d"
 * \param A pointer to matrix, column major
 * \param m matrix rows
 * \param n matrix cols
 */
#define AA_DUMP_MAT( file, fmt, A, m, n)                                \
    {                                                                   \
        for( size_t aa_debug_$_i = 0; aa_debug_$_i < m;                 \
             aa_debug_$_i ++ ) {                                        \
            for( size_t aa_debug_$_j = 0; aa_debug_$_j < n-1;           \
                 aa_debug_$_j ++ ) {                                    \
                fprintf(file, fmt"\t",                                  \
                        AA_MATREF(A, m, aa_debug_$_i,aa_debug_$_j));    \
            }                                                           \
            fprintf(file, fmt"\n",                                      \
                    AA_MATREF(A, m, aa_debug_$_i, n-1) );               \
        }                                                               \
    }                                                                   \

/// save time, printf fmt
AA_API void
aa_tick(const char fmt[], ...);

/// print and return elapsed time since aa_tick()
AA_API struct timespec
aa_tock(void);


/// don't use
AA_EXTERN const char *aa_verbf_prefix AA_DEPRECATED;
/// don't use
AA_EXTERN int aa_opt_verbosity AA_DEPRECATED;

#endif //AA_MATH_H
