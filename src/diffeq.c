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

#ifdef HAVE_CONFIG
#include "config.h"
#endif

#include <math.h>
#include "amino.h"
#include "amino/mat_internal.h"
#include "amino/diffeq.h"

AA_API void aa_de_grad_fd( aa_de_scalar_field_fun *fun, void *cx,
                           const struct aa_dvec *x, double eps, struct aa_dvec *y )
{
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    size_t n = x->len;
    aa_lb_check_size( n, y->len );

    struct aa_dvec *p = aa_dvec_dup(reg,x);
    struct aa_dvec *m = aa_dvec_dup(reg,x);

    for( double *yy = y->data, *pp=p->data, *mm=m->data, *pe=p->data+n;
         pp < pe;
         yy+=y->inc, pp++, mm++ )
    {
        double orig = *pp;
        *pp += eps;
        *mm -= eps;

        *yy = (fun(cx,p) - fun(cx,m)) / (2*eps);

        *pp = orig;
        *mm = orig;
    }

    aa_mem_region_pop(reg,ptrtop);
}


AA_API void aa_de_jac_fd( aa_de_vector_field_fun *fun, void *cx,
                          const struct aa_dvec *x, double eps, struct aa_dmat *J )
{
    struct aa_mem_region *reg =  aa_mem_region_local_get();
    void *ptrtop = aa_mem_region_ptr(reg);

    size_t m = J->rows;
    size_t n = J->cols;
    aa_lb_check_size( n, x->len );

    struct aa_dvec *pv = aa_dvec_dup(reg,x);
    struct aa_dvec *mv = aa_dvec_dup(reg,x);
    struct aa_dvec *jtmp = aa_dvec_alloc(reg,m);

    struct aa_dvec jcol = AA_DVEC_INIT(m, J->data, 1);
    for( double *pp=pv->data, *mm=mv->data, *pe=pv->data+n;
         pp < pe;
         pp++, mm++, jcol.data+=J->ld )
    {
        double orig = *pp;
        *pp += eps;
        *mm -= eps;

        fun(cx,mv,jtmp);
        fun(cx,pv,&jcol);
        // jcol = (jcol - jtmp) / (2*eps)
        aa_lb_daxpy(-1, jtmp, &jcol);
        aa_lb_dscal(1.0 / (2*eps), &jcol);

        *pp = orig;
        *mm = orig;
    }

    aa_mem_region_pop(reg,ptrtop);
}


/* AA_API void aa_de_nrm2_grad( const struct aa_dvec *x,  struct aa_dvec *y ) */
/* { */
/* } */
