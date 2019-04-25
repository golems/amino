/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
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
#ifndef AA_DIFFEQ_H
#define AA_DIFFEQ_H

/**
 * @file diffeq.h
 * @brief Numerical routines for differential equations
 *
 * @sa amino/mat.h
 */

#include "mat.h"

typedef double aa_de_scalar_field_fun(void *cx, const struct aa_dvec *x);

typedef void aa_de_vector_field_fun(void *cx, const struct aa_dvec *x, struct aa_dvec *y);


/**
 * Estimate the gradient by finite difference.
 *
 */
AA_API void aa_de_grad_fd( aa_de_scalar_field_fun *fun, void *cx,
                           const struct aa_dvec *x, double eps, struct aa_dvec *y );


/**
 * Estimate the Jacobian by finite difference.
 *
 */
AA_API void aa_de_jac_fd( aa_de_vector_field_fun *fun, void *cx,
                          const struct aa_dvec *x, double eps, struct aa_dmat *J );

#endif //AA_DIFFEQ_H
