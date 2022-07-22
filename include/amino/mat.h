/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2018-2019, Colorado School of Mines
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
 * Type for sizes of vectors and matrices.
 */
typedef size_t aa_la_size;

/**
 * Descriptor for a vector.
 */
struct aa_dvec {
    aa_la_size len;   ///< Number of elements in vector
    double *data; ///< Pointer to data
    aa_la_size inc;   ///< Increment between successive vector elements
};

/**
 * Descriptor for a block matrix.
 */
struct aa_dmat {
    aa_la_size rows;    ///< number of rows in matrix
    aa_la_size cols;    ///< number of columns
    double *data;   ///< Pointer to matrix data
    aa_la_size ld;      ///< Leading dimension of matrix
};


/** Return an element of a dvec.
 *
 * @param v an aa_dvec
 * @param i the index of the element in v to return
 */
#define AA_DVEC_REF(v,i) ((v)->data[(i) * (v)->inc])

/**
 * Reference a matrix entry.
 *
 * @param M matrix
 * @param i row
 * @param j column
 */
#define AA_DMAT_REF(M,i,j) (((M)->data)[(M)->ld*(j) + (i)])


/**
 * Callback type for linear algebra errors.
 */
typedef void
(aa_la_err_fun)( const char *message );

/**
 * Default handler function for linear algebra errors.
 */
AA_API void
aa_la_err( const char *message );

/**
 * Set the handler function for linear algebra errors.
 */
AA_API void
aa_la_set_err( aa_la_err_fun *fun );


/**
 * BLAS arguments for a vector
 */
#define AA_VEC_ARGS(X) (X->data), ((int)(X->inc))

/**
 * BLAS arguments for a matrix
 */
#define AA_MAT_ARGS(X) (X->data), ((int)(X->ld))

/* Construction */

/**
 * Fill in a vector descriptor.
 *
 * @param len  Number of elements in vector
 * @param data Pointer to vector data
 * @param inc  Increment between sucessive elements
 */
static inline struct aa_dvec
AA_DVEC_INIT( aa_la_size len, double *data, aa_la_size inc )
{
    struct aa_dvec vec = {len, data, inc};
    return vec;
}

/**
 * Fill in a vector descriptor.
 *
 * @param vec  Pointer to descriptor
 * @param len  Number of elements in vector
 * @param data Pointer to vector data
 * @param inc  Increment between sucessive elements
 */
static inline void
AA_DVEC_VIEW(struct aa_dvec *vec, aa_la_size len, double *data, aa_la_size inc)
{
    *vec = AA_DVEC_INIT(len,data,inc);
}

/**
 * Fill in a vector descriptor.
 *
 * @param vec  Pointer to descriptor
 * @param len  Number of elements in vector
 * @param data Pointer to vector data
 * @param inc  Increment between sucessive elements
 */
AA_API void
aa_dvec_view( struct aa_dvec *vec, aa_la_size len, double *data, aa_la_size inc );


/**
 * Fill in a matrix descriptor.
 *
 * @param mat   Pointer to descriptor
 * @param rows  Number of rows in matrix
 * @param cols  Number of colums in matrix
 * @param data  Pointer to vector data
 * @param ld    Leading dimension of matrix
 */
AA_API void
aa_dmat_view( struct aa_dmat *mat, aa_la_size rows, aa_la_size cols, double *data, aa_la_size ld );

/**
 * View a block of a matrix.
 */
AA_API void
aa_dmat_view_block( struct aa_dmat *dst,
                    const struct aa_dmat *src,
                    aa_la_size row_start, aa_la_size col_start,
                    aa_la_size rows, aa_la_size cols );

/**
 * View a slice of a vector.
 */
AA_API void
aa_dvec_slice( const struct aa_dvec *src,
               aa_la_size start,
               aa_la_size stop,
               aa_la_size step,
               struct aa_dvec *dst );


/**
 * View a block of a matrix.
 */
AA_API void
aa_dmat_block( const struct aa_dmat *src,
               aa_la_size row_start, aa_la_size col_start,
               aa_la_size row_end, aa_la_size col_end,
               struct aa_dmat *dst );

/**
 * View a row of a matrix as a vector.
 */
AA_API void
aa_dmat_row_vec( const struct aa_dmat *src, aa_la_size row, struct aa_dvec *dst );

/**
 * View a column of a matrix as a vector.
 */
AA_API void
aa_dmat_col_vec( const struct aa_dmat *src, aa_la_size col, struct aa_dvec *dst );

/**
 * View the diagonal of a matrix as a vector.
 */
AA_API void
aa_dmat_diag_vec( const struct aa_dmat *src, struct aa_dvec *dst );


/**
 * Fill in a matrix descriptor.
 *
 * @param rows  Number of rows in matrix
 * @param cols  Number of colums in matrix
 * @param data  Pointer to vector data
 * @param ld    Leading dimension of matrix
 */
static inline struct aa_dmat
AA_DMAT_INIT( aa_la_size rows, aa_la_size cols, double *data, aa_la_size ld )
{
    struct aa_dmat mat;
    mat.rows = rows;
    mat.cols = cols;
    mat.data = data;
    mat.ld = ld;
    return mat;
}

/**
 * Fill in a matrix descriptor.
 *
 * @param mat   Pointer to descriptor
 * @param rows  Number of rows in matrix
 * @param cols  Number of colums in matrix
 * @param data  Pointer to vector data
 * @param ld    Leading dimension of matrix
 */
static inline void
AA_DMAT_VIEW(struct aa_dmat *mat, aa_la_size rows, aa_la_size cols, double *data, aa_la_size ld)
{
    *mat = AA_DMAT_INIT(rows,cols,data,ld);
}

/**
 * Set VEC to a descriptor for the diagonal vector of MAT.
 */
#define AA_MAT_DIAG(VEC,MAT)                                    \
    aa_dvec_view((VEC), (MAT)->cols, (MAT)->data, 1+(MAT)->ld);

/**
 * Region-allocate a vector.
 *
 * When finished, pop the descriptor.
 */
AA_API struct aa_dvec *
aa_dvec_alloc( struct aa_mem_region *reg, aa_la_size len );


/**
 * Duplicate vector out of region
 *
 * When finished, pop the descriptor.
 */
AA_API struct aa_dvec *
aa_dvec_dup( struct aa_mem_region *reg, const struct aa_dvec *src);

/**
 * Duplicate vector in the heap.
 *
 * When finished, free the descriptor.
 */
AA_API struct aa_dvec *
aa_dvec_mdup(const struct aa_dvec *src);

/**
 * Duplicate matrix out of region
 *
 * When finished, pop the descriptor.
 */
AA_API struct aa_dmat *
aa_dmat_dup( struct aa_mem_region *reg, const struct aa_dmat *src);

/**
 * Duplicate matrix in the heap
 *
 * When finished, free the descriptor.
 */
AA_API struct aa_dmat *
aa_dmat_mdup(const struct aa_dmat *src);

/**
 * Region-allocate a matrix.
 *
 * When finished, pop the descriptor.
 */
AA_API struct aa_dmat *
aa_dmat_alloc( struct aa_mem_region *reg, aa_la_size rows, aa_la_size cols );

/**
 * Heap-allocate a vector.
 *
 * The descriptor and data are contained in a single malloc()'ed block.
 * When finished, call free() on the descriptor.
 */
AA_API struct aa_dvec *
aa_dvec_malloc( aa_la_size len );

/**
 * Heap-allocate a matrix.
 *
 * The descriptor and data are contained in a single malloc()'ed block.
 * When finished, call free() on the descriptor.
 */
AA_API struct aa_dmat *
aa_dmat_malloc( aa_la_size rows, aa_la_size cols );

/**
 * Zero a vector.
 */
AA_API void
aa_dvec_zero( struct aa_dvec *vec );

/**
 * Fill a vector.
 */
AA_API void
aa_dvec_set( struct aa_dvec *vec, double alpha );

/**
 * Fill a matrix diagonal and off-diagonal elements.
 *
 * @param[in] A        The matrix to fill
 * @param[in] alpha    off-diagonal entries of A
 * @param[in] beta     diagonal entries of A
 */
AA_API void
aa_dmat_set( struct aa_dmat *A, double alpha, double beta );

/**
 * Zero a matrix.
 */
AA_API void
aa_dmat_zero( struct aa_dmat *mat );

/* Level 1 BLAS */

/**
 * Swap x and y
 *
 * \f[ \mathbf{x} \leftrightarrow \mathbf{y} \f]
 */
AA_API void
aa_dvec_swap( struct aa_dvec *x, struct aa_dvec *y );

/**
 * Scale x by alpha.
 *
 * \f[ \mathbf{x} \leftarrow \alpha \mathbf{x} \f]
 */
AA_API void
aa_dvec_scal( double alpha, struct aa_dvec *x );

/**
 * Increment x by alpha.
 *
 * \f[ \mathbf{x} \leftarrow \alpha + \mathbf{x} \f]
 */
AA_API void
aa_dvec_inc( double alpha, struct aa_dvec *x );

/**
 * Copy x to y.
 *
 * \f[ \mathbf{y} \leftarrow \mathbf{x} \f]
 */
AA_API void
aa_dvec_copy( const struct aa_dvec *x, struct aa_dvec *y );


/**
 * Alpha x plus y.
 *
 * \f[ \mathbf{y} \leftarrow \alpha \mathbf{x} + \mathbf{y} \f]
 */
AA_API void
aa_dvec_axpy( double a, const struct aa_dvec *x, struct aa_dvec *y );

/**
 * Dot product
 *
 * \f[ \mathbf{x}^T \mathbf{y} \f]
 */
AA_API double
aa_dvec_dot( const struct aa_dvec *x, struct aa_dvec *y );

/**
 * Euclidean Norm
 *
 * \f[ \left\Vert \mathbf{x} \right\Vert_2  \f]
 */
AA_API double
aa_dvec_nrm2( const struct aa_dvec *x );

/* Level 2 BLAS */

/**
 * General Matrix-Vector multiply
 *
 * \f[ \mathbf{y} \leftarrow \alpha \mathbf{A}^{\rm op} \mathbf{x} + \beta \mathbf{y}  \f]
 */
AA_API void
aa_dmat_gemv( AA_CBLAS_TRANSPOSE trans,
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
aa_dmat_gemm( AA_CBLAS_TRANSPOSE transA, AA_CBLAS_TRANSPOSE transB,
              double alpha, const struct aa_dmat *A,
              const struct aa_dmat *B,
              double beta, struct aa_dmat *C );




/* LAPACK */

/**
 * Copies all or part of a two-dimensional matrix A to another
 * matrix B.
 *
 *  @param[in] uplo
 *          Specifies the part of the matrix A to be copied to B.
 *          - = 'U':      Upper triangular part
 *          - = 'L':      Lower triangular part
 *          - Otherwise:  All of the matrix A
 *
 *  @param[in] A
 *          dimension (LDA,N)
 *          The m by n matrix A.  If UPLO = 'U', only the upper triangle
 *          or trapezoid is accessed; if UPLO = 'L', only the lower
 *          triangle or trapezoid is accessed.
 *
 *  @param[out] B
 *          dimension (LDB,N)
 *          On exit, B = A in the locations specified by UPLO.
 *
 */
AA_API void
aa_dmat_lacpy( const char uplo[1],
               const struct aa_dmat *A,
               struct aa_dmat *B );



/* Matrix/Vector Functions */

/**
 * sum-square-differences of two vectors
 */
AA_API double
aa_dvec_ssd( const struct aa_dvec *x, const struct aa_dvec *y);


/**
 * Y += alpha * X
 */
AA_API void
aa_dmat_axpy( double alpha, const struct aa_dmat *X, struct aa_dmat *Y);

/**
 * sum-square-differences of two matrices
 */
AA_API double
aa_dmat_ssd( const struct aa_dmat *x, const struct aa_dmat *y);

/**
 * Scale the matrix A by alpha
 */
AA_API void
aa_dmat_scal( struct aa_dmat *A, double alpha );

/**
 * Increment the matrix A by alpha
 */
AA_API void
aa_dmat_inc( struct aa_dmat *A, double alpha );

/**
 * Euclidean Norm
 */
AA_API double
aa_dmat_nrm2( const struct aa_dmat *x );


/**
 * Matrix transpose.
 */
AA_API void
aa_dmat_trans( const struct aa_dmat *A, struct aa_dmat *At);


/**
 * Matrix inverse, in-place.
 */
AA_API int
aa_dmat_inv( struct aa_dmat *A);


/**
 * Pseudo-inverse.
 *
 * Singular values less than tol are ignored.  If tol < 0, then a sane
 * default is used.
 */
AA_API int
aa_dmat_pinv( const struct aa_dmat *A, double tol, struct aa_dmat *As);

/**
 * Damped pseudo-inverse.
 */
AA_API int
aa_dmat_dpinv( const struct aa_dmat *A, double k, struct aa_dmat *As);

/**
 * Dead-zone damped pseudo-inverse.
 */
AA_API int
aa_dmat_dzdpinv(  const struct aa_dmat *A, double s_min, struct aa_dmat *As);

/**
 * Copy a matrix
 */
AA_API void
aa_dmat_copy(  const struct aa_dmat *A, struct aa_dmat *B);


#endif /* AMINO_MAT_H */
