/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *
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
#ifndef AMINO_OPT_H
#define AMINO_OPT_H

/**
 * @file opt.h
 * @brief General optimization routines
 */

/**
 * @dir   opt
 * @brief Optimization SUpport
 */

/**
 * Optimization constraint type
 */
enum aa_opt_rel_type {
    AA_OPT_REL_EQ,  ///< equality constraint
    AA_OPT_REL_LEQ, ///< Less-than or equal constraint
    AA_OPT_REL_GEQ, ///< Greater-than or equal constraint
};


/**
 * Optimization direction.
 */
enum aa_opt_direction {
    AA_OPT_MAXIMIZE, ///< maximize objective function
    AA_OPT_MINIMIZE, ///< minimize objective function
};

/**
 * Type of optiziation variable.
 */
enum aa_opt_type {
    AA_OPT_CONTINUOUS, ///< continuous (float) variable
    AA_OPT_BINARY,     ///< binary variable
    AA_OPT_INTEGER     ///< integer variable
};

struct aa_opt_cx;

/**
 * Solve the optimization problem.
 */
AA_API int
aa_opt_solve( struct aa_opt_cx *cx, size_t n, double *x );


/**
 * Destroy the optimization context.
 */
AA_API int
aa_opt_destroy( struct aa_opt_cx *cx );

/**
 * Destroy the optimization direction.
 */
AA_API int
aa_opt_set_direction( struct aa_opt_cx *cx, enum aa_opt_direction );

/**
 * Destroy the quadratic objective function via compressed-row-storage
 * format.
 */
AA_API int
aa_opt_set_quad_obj_crs( struct aa_opt_cx *cx, size_t n,
                         const double *Q_values, int *Q_cols, int *Q_row_ptr );

/**
 * Destroy the optimization variable type.
 */
AA_API int
aa_opt_set_type( struct aa_opt_cx *cx, size_t i, enum aa_opt_type type );

/**
 * Optimization context constructor from general matrix format.
 */
typedef struct aa_opt_cx*
aa_opt_gmcreate_fun (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper
    );

/**
 * Create an optimization context for LP-Solve.
 */
AA_API struct aa_opt_cx* aa_opt_lpsolve_gmcreate (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper
    );


/**
 * Create an optimization context for CLP.
 */
AA_API struct aa_opt_cx* aa_opt_clp_gmcreate (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper
    );

/**
 * Create an optimization context for GLPK.
 */
AA_API struct aa_opt_cx* aa_opt_glpk_gmcreate (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper
    );

/**
 * Create an optimization context for LP-Solve.
 */
AA_API struct aa_opt_cx* aa_opt_lpsolve_crscreate (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper );


/**
 * Create an optimization context for CLP.
 */
AA_API struct aa_opt_cx* aa_opt_clp_crscreate (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper );


/**
 * Create an optimization context for GLPK.
 */
AA_API struct aa_opt_cx* aa_opt_glpk_crscreate (
    size_t m, size_t n,
    const double *A_values, int *A_cols, int *A_row_ptr,
    const double *b_lower, const double *b_upper,
    const double *c,
    const double *x_lower, const double *x_upper );

#endif //AMINO_OPT_H
