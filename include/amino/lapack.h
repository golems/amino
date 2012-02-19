/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2012, Georgia Tech Research Corporation
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


/* FILE:  lapack.h
 * BRIEF: C prototypes to various fortran lapack routines.
 *
 * Since there is no official c binding to lapack as there as with
 * BLAS, the only reasonable way to interface with lapack from C is to
 * call the fortran methods directly.
 *
 * Authors:
 *   Neil T. Dantam
 */

#ifndef LAPACK_H_
#define LAPACK_H_


/// type for operations
#define AA_TYPE_DOUBLE
#include "lapack_impl.h"


/// type for operations
#define AA_TYPE_FLOAT
#include "lapack_impl.h"


/** ILAENV is called from the LAPACK routines to choose problem-dependent
 *  parameters for the local environment.
 *
 *  \param ispec
 *          Specifies the parameter to be returned as the value of
 *          ILAENV.
 *          - = 1: the optimal blocksize; if this value is 1, an unblocked
 *               algorithm will give the best performance.
 *          - = 2: the minimum block size for which the block routine
 *               should be used; if the usable block size is less than
 *               this value, an unblocked routine should be used.
 *          - = 3: the crossover point (in a block routine, for N less
 *               than this value, an unblocked routine should be used)
 *          - = 4: the number of shifts, used in the nonsymmetric
 *               eigenvalue routines (DEPRECATED)
 *          - = 5: the minimum column dimension for blocking to be used;
 *               rectangular blocks must have dimension at least k by m,
 *               where k is given by ILAENV(2,...) and m by ILAENV(5,...)
 *          - = 6: the crossover point for the SVD (when reducing an m by n
 *               matrix to bidiagonal form, if max(m,n)/min(m,n) exceeds
 *               this value, a QR factorization is used first to reduce
 *               the matrix to a triangular form.)
 *          - = 7: the number of processors
 *          - = 8: the crossover point for the multishift QR method
 *               for nonsymmetric eigenvalue problems (DEPRECATED)
 *          - = 9: maximum size of the subproblems at the bottom of the
 *               computation tree in the divide-and-conquer algorithm
 *               (used by xGELSD and xGESDD)
 *          - =10: ieee NaN arithmetic can be trusted not to trap
 *          - =11: infinity arithmetic can be trusted not to trap
 *          - 12 <= ISPEC <= 16:
 *               xHSEQR or one of its subroutines,
 *               see IPARMQ for detailed explanation
 *
 *  \param name
 *          The name of the calling subroutine, in either upper case or
 *          lower case.
 *
 *  \param opts
 *          The character options to the subroutine NAME, concatenated
 *          into a single character string.  For example, UPLO = 'U',
 *          TRANS = 'T', and DIAG = 'N' for a triangular routine would
 *          be specified as OPTS = 'UTN'.
 *
 *  \param n1      (input) INTEGER
 *  \param n2      (input) INTEGER
 *  \param n3      (input) INTEGER
 *  \param n4      (input) INTEGER
 *          Problem dimensions for the subroutine NAME; these may not all
 *          be required.
 * \param name_length fortran string brain damage, length of name
 * \param opts_length fortran string brain damage, length of opts
 */
AA_API int ilaenv_( const int *ispec, const char *name, const char *opts,
                      const int *n1, const int * n2, const int *n3, const int *n4,
                      int name_length, int opts_length );


#endif
