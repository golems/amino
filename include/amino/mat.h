/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2018, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@miens.edu>
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

#ifndef AMINO_MAT_H
#define AMINO_MAT_H

#include <cblas.h>

/**
 * @file mat.h
 *
 * Block matrix descriptors and linear algebra operations.
 *
 */

/**
 * Descriptor for a vector.
 */
struct aa_dvec {
    size_t len;   ///< Number of elements in vector
    double *data; ///< Pointer to data
    size_t inc;   ///< Increment between vector elements
};

/**
 * Descriptor for a block matrix.
 */
struct aa_dmat {
    size_t rows;    ///< number of rows in matrix
    size_t cols;    ///< number of columns
    double *data;   ///< Pointer to matrix data
    size_t ld;      ///< Leading dimension of matrix
};



/* Construction */

/**
 * Fill in a vector descriptor.
 */
AA_API void
aa_dvec_view( struct aa_dvec *vec, size_t len, double *data, size_t inc );

/**
 * Fill in a matrix descriptor.
 */
AA_API void
aa_dmat_view( struct aa_dmat *mat, size_t rows, size_t cols, double *data, size_t inc );

/**
 * Region-allocate a vector.
 *
 * When finished, pop the descriptor.
 */
AA_API struct aa_dvec *
aa_dvec_alloc( struct aa_mem_region *reg, size_t len );

/**
 * Region-allocate a matrix.
 *
 * When finished, pop the descriptor.
 */
AA_API struct aa_dmat *
aa_dmat_alloc( struct aa_mem_region *reg, size_t rows, size_t cols );

/**
 * Heap-allocate a vector.
 *
 * The descriptor and data are contained in a single malloc()'ed block.
 * When finished, call free() on the descriptor.
 */
AA_API struct aa_dvec *
aa_dvec_malloc( size_t len );

/**
 * Heap-allocate a matrix.
 *
 * The descriptor and data are contained in a single malloc()'ed block.
 * When finished, call free() on the descriptor.
 */
AA_API struct aa_dmat *
aa_dmat_malloc( size_t rows, size_t cols );

/* Level 1 BLAS */

/**
 * Swap x and y
 *
 * \f[ \mathbf{x} \leftrightarrow \mathbf{y} \f]
 */
AA_API void
aa_lb_dswap( struct aa_dvec *x, struct aa_dvec *y );

/**
 * Scale x by alpha.
 *
 * \f[ \mathbf{x} \leftarrow \alpha \mathbf{x} \f]
 */
AA_API void
aa_lb_dscal( double alpha, struct aa_dvec *x );

/**
 * Copy x to y.
 *
 * \f[ \mathbf{y} \leftarrow \mathbf{x} \f]
 */
AA_API void
aa_lb_dcopy( const struct aa_dvec *x, struct aa_dvec *y );


/**
 * Alpha x plus y.
 *
 * \f[ \mathbf{y} \leftarrow \alpha \mathbf{x} + \mathbf{y} \f]
 */
AA_API void
aa_lb_daxpy( double a, const struct aa_dvec *x, struct aa_dvec *y );

/**
 * Dot product
 *
 * \f[ \mathbf{x}^T \mathbf{y} \f]
 */
AA_API double
aa_lb_ddot( const struct aa_dvec *x, struct aa_dvec *y );

/**
 * Euclidean Norm
 *
 * \f[ \left\Vert \mathbf{x} \right\Vert_2  \f]
 */
AA_API double
aa_lb_dnrm2( const struct aa_dvec *x );

/* Level 2 BLAS */

/**
 * General Matrix-Vector multiply
 *
 * \f[ \mathbf{y} \leftarrow \alpha \mathbf{A}^{\rm op} \mathbf{x} + \beta \mathbf{y}  \f]
 */
AA_API void
aa_lb_dgemv( CBLAS_TRANSPOSE trans,
             double alpha, const struct aa_dmat *A,
             const struct aa_dvec *x,
             double beta, struct aa_dvec *y );



/* Level 3 BLAS */

/**
 * General Matrix-Matrix multiply
 *
 * \f[ \mathbf{y} \leftarrow \alpha \mathbf{A}^{\rm opA} \mathbf{B}^\rm{opB} + \beta \mathbf{C}  \f]
 */
AA_API void
aa_lb_dgemm( CBLAS_TRANSPOSE transA, CBLAS_TRANSPOSE transB,
             double alpha, const struct aa_dmat *A,
             const struct aa_dmat *B,
             double beta, struct aa_dmat *C );

#endif /* AMINO_MAT_H */
