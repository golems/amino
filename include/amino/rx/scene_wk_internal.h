/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2018-2019, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@mines.edu>
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
 *   * Neither the name of the Rice University nor the names of its
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


#ifndef AMINO_RX_WK_INTERNAL_H
#define AMINO_RX_WK_INTERNAL_H

#include "amino.h"
#include "amino/opt/opt.h"
#include "amino/rx/scene_wk.h"
#include "amino/rx/scene_fk.h"
#include "amino/rx/scene_sub.h"

struct aa_rx_wk_opts {
    double s2min;   ///< minimum singular value for DLS via SVD
    double k_dls;   ///< damping constant for DLS via LU
    double gain_np;    ///< gain for nullspace projection
    double gain_angle; ///< gain for position error
    double gain_trans; ///< gain for position error
    enum aa_opt_lp_solver lp_solver; ///< linear program solver
};

static inline int
aa_rx_wk_get_js( const struct aa_rx_sg_sub *ssg,
                 const struct aa_rx_wk_opts * opts,
                 //const struct aa_dmat *TF_abs,
                 const struct aa_rx_fk *fk,
                 struct aa_dmat *J,
                 struct aa_dmat *Jstar )
{
    int r = -1;

    aa_rx_sg_sub_jac_vel_fill(ssg,fk,J);

    // Compute a damped pseudo inverse
    // TODO: Try DGECON to avoid damping when possible without taking the SVD
    if( opts->s2min > 0 ) {
        r = aa_dmat_dzdpinv( J, sqrt(opts->s2min), Jstar );
    } else  if (opts->k_dls > 0) {
        r = aa_dmat_dpinv( J, opts->k_dls, Jstar );
    } else {
        r = aa_dmat_pinv( J, -1, Jstar );
    }


    return r;
}

static inline int
aa_rx_wk_get_n( const struct aa_rx_sg_sub *ssg,
                const struct aa_rx_wk_opts * opts,
                const struct aa_dmat *J,
                const struct aa_dmat *Jstar,
                double alpha,
                struct aa_dmat *N )
{
    /* N = alpha(A^* A - I) */

    (void) ssg;
    (void) opts;


    // N = alpha*A^* A
    aa_lb_dgemm( CblasNoTrans, CblasNoTrans,
                 alpha, Jstar, J,
                 0.0, N );

    // N = N - alpha*I
    for( double *x = N->data, *e = N->data + N->cols*N->ld;
         x < e; x += N->ld+1 )
    {
        *x -= alpha;
    }

    return 0;
}

#endif /*AMINO_RX_WK_INTERNAL_H*/
