/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
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


#include "amino.h"
#include "amino/opt/opt.h"

#include "opt_internal.h"

AA_API int
aa_opt_solve( struct aa_opt_cx *cx, size_t n, double *x )
{
    return cx->vtab->solve(cx,n,x);
}


AA_API int
aa_opt_destroy( struct aa_opt_cx *cx ) {
    return cx->vtab->destroy(cx);
}

AA_API int
aa_opt_set_direction( struct aa_opt_cx *cx, enum aa_opt_direction dir )
{
    return cx->vtab->set_direction(cx, dir);
}

AA_API int
aa_opt_set_quad_obj_crs( struct aa_opt_cx *cx, size_t n,
                         const double *Q_values, int *Q_cols, int *Q_row_ptr )
{
    return cx->vtab->set_quad_obj_crs( cx, n, Q_values, Q_cols, Q_row_ptr );
}


AA_API int
aa_opt_set_type( struct aa_opt_cx *cx, size_t i, enum aa_opt_type type )
{
    return cx->vtab->set_type(cx, i, type);
}


AA_API int
aa_opt_set_obj( struct aa_opt_cx *cx, size_t n, const double * c)
{
    return cx->vtab->set_obj(cx,n,c);
}

AA_API int
aa_opt_set_bnd( struct aa_opt_cx *cx, size_t n,
                const double * x_min, const double *x_max)
{
    return cx->vtab->set_bnd(cx,n,x_min,x_max);
}

AA_API int
aa_opt_set_cstr_gm( struct aa_opt_cx *cx,
                    size_t m, size_t n,
                    const double *A, size_t lda )
{
    return cx->vtab->set_cstr_gm( cx, m, n,
                                  A, lda );
}

AA_API int
aa_opt_set_cstr_bnd( struct aa_opt_cx *cx, size_t m,
                     const double * b_min, const double *b_max )
{
    return cx->vtab->set_cstr_bnd( cx, m, b_min, b_max );
}
