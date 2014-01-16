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

#ifndef AMINO_IO_H
#define AMINO_IO_H

/** Reads from fd, reallocating buffer if necessary.
 */
AA_API ssize_t aa_read_realloc(int fd, void **buf, size_t off, size_t *max);

#define AA_IO_ISCOMMENT(x) ('#' == (x))

/** Read line from file, storing in memory region. */
char *aa_io_getline( FILE *fin, struct aa_mem_region *reg );

/** Skip leading blank characters */
char *aa_io_skipblank( const char *str );

/** Parse vector from a line */
size_t aa_io_parsevector( const char *str, size_t n, double *X, size_t incx, char **endptr );

/** Read a vector from a line in the file */
size_t aa_io_getvector( FILE *fin, struct aa_mem_region *reg, size_t n, double *X, size_t incx );

/** Read a fixed size matrix from file.
 *  Returns number of items read.
 */
size_t aa_io_fread_matrix_fix( FILE *fin, size_t m, size_t n,
                               double *A, size_t lda );

/** Read a fixed size matrix from file.
 *
 * Each line of the file becomes a column of the matrix.
 *
 *  On success, returns number of matrix columns. On error, returns
 *  the negative of the erring line.
 *
 *
 * @param n, number of items per line
 * @param A, pointer to matrix buffer
 * @param elts, pointer to number of elements in A
 */
ssize_t aa_io_fread_matrix_heap( FILE *fin, size_t n,
                                 double **A, size_t *elts );

#endif //AMINO_IO_H
