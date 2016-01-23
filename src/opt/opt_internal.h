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
#ifndef AMINO_OPT_INTERNAL_H
#define AMINO_OPT_INTERNAL_H

static inline int
aa_opt_is_eq( double l, double u)
{
    return aa_feq(l,u,0);
}

static inline int
aa_opt_is_lbound( double l )
{
    return -DBL_MAX < l && !isinf(l);
}

static inline int
aa_opt_is_ubound( double u )
{
    return DBL_MAX > u && !isinf(u);
}

static inline int
aa_opt_is_leq( int lb, int ub ) {
    return !lb && ub;
}

static inline int
aa_opt_is_geq( int lb, int ub ) {
    return lb && !ub;
}

static inline int
aa_opt_is_bound( int lb, int ub ) {
    return lb && ub;
}

static inline int
aa_opt_is_free( int lb, int ub ) {
    return !lb && !ub;
}


struct aa_opt_vtab {
    int (*solve)( struct aa_opt_cx *cx, size_t n, double *x );
    int (*destroy)( struct aa_opt_cx *cx );
    int (*set_direction)( struct aa_opt_cx *cx, enum aa_opt_direction dir );
    int (*set_quad_obj_crs)( struct aa_opt_cx *cx, size_t n,
                             const double *Q_values, int *Q_cols, int *Q_row_ptr );
};


struct aa_opt_cx {
    struct aa_opt_vtab *vtab;
    void *data;
};

static inline struct aa_opt_cx* cx_finish ( struct aa_opt_vtab *vtab, void *p )
{
    if( p ) {
        struct aa_opt_cx *cx = AA_NEW(struct aa_opt_cx);
        cx->vtab = vtab;
        cx->data = p;
        return cx;
    } else {
        return NULL;
    }
}

#endif /*AMINO_OPT_INTERNAL_H*/
