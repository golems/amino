/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2013, Georgia Tech Research Corporation
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

#ifndef AMINO_TEST_H
#define AMINO_TEST_H


AA_API void test( const char *name, int check ) ;
/** Test fuzzy equals. */
AA_API void test_feq( const char *name, double a, double b, double tol );
/** Test fuzzy less than. */
AA_API void test_flt( const char *name, double a, double b, double tol );
/** Test fuzzy greater than. */
AA_API void test_fgt( const char *name, double a, double b, double tol );

/** Simple assertion over a fuzzy equals. */
AA_API void afeq( double a, double b, double tol ) ;
/** Assertion over a fuzzy equals with an error message. */
AA_API void aafeq( const char *name, double a, double b, double tol ) ;

/** A fuzzy equals over two vectors. */
AA_API void aveq( const char * name, size_t n, const double *a, const double *b, double tol ) ;

/** A fuzzy equals over two rotation vectors. */
AA_API void arveq( const char * name, const double *a, const double *b, double tol );

AA_API void aneq( double a, double b, double tol ) ;

/* Set limits*/
AA_API void aa_test_ulimit( void );

/* Set random seed */
AA_API void aa_test_args(int argc, char *argv[]);


AA_API void aa_test_randv(double min, double max, size_t n, double *p);

AA_API void aa_test_quat_cmp(const char *name, const double *q1,
                             const double *q2, double tol);

AA_API void aa_test_qutr_cmp(const char *name, const double *E1,
                             const double *E2, double tol);


AA_API void aa_test_rotmat_cmp(const char *name, const double *R1,
                               const double *R2, double tol);

AA_API void aa_test_isrotmat(const char *name, const double *R, double tol);

/*
 * Compare two rotation vectors up to angle interval [0,pi]
 */
AA_API void aa_test_rotvec_cmp_pi(const char *name, const double *a,
                                  const double *b, double tol);

#endif
