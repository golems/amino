/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
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

//#define AA_ALLOC_STACK_MAX
#include "config.h"
#include "amino.h"
#include "amino/test.h"
#include "amino/opt/lp.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>


void helper0( const char *name, aa_opt_gmcreate_fun fun) {

    double A[] = {120, 110, 1,  210, 30, 1};
    double b_u[] = {15000, 4000, 75};
    double b_l[] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};
    double c[] = {1, 1};

    double x_l[] = {0,0};
    double x_u[] = {1000,1000};

    double x[2];

    struct aa_opt_cx *cx = fun( 3,2,
                                A, 3,
                                b_l, b_u,
                                c,
                                x_l, x_u );


    aa_opt_set_direction(cx, AA_OPT_MAXIMIZE );
    int r = aa_opt_solve(cx,2,x);

    printf("r: %d\n", r );

    aa_dump_vec( stdout, x, 2 );

    double xref[] = {21.875, 53.125};
    assert(sizeof(xref) == sizeof(x));

    aafeq( name, xref[0] + xref[1], x[0]+x[1], 1e-3 );
}



void helper1( const char *name, aa_opt_gmcreate_fun fun) {

    double A[] = {.5, 1,
                  2, 2,
                  1, 4 };

    double b_u[] = {24, 60};
    double b_l[] = {-DBL_MAX, -DBL_MAX };
    double c[] = {6, 14, 13};

    double x_l[] = {0,0, 0};
    double x_u[] = {DBL_MAX,DBL_MAX,DBL_MAX};

    size_t n_x = sizeof(x_l) / sizeof(x_l[0]);
    size_t n_c = sizeof(b_u) / sizeof(b_u[0]);

    assert( n_x == sizeof(x_l)/sizeof(double) );
    assert( n_x == sizeof(x_u)/sizeof(double) );
    assert( n_x == sizeof(c)/sizeof(double) );
    assert( sizeof(b_l) == sizeof(b_u) );
    assert( n_x * sizeof(b_u)
            ==
            sizeof(A) );

    double x[n_x];

    struct aa_opt_cx *cx = fun( n_c, n_x,
                                A, n_c,
                                b_l, b_u,
                                c,
                                x_l, x_u );


    aa_opt_set_direction(cx, AA_OPT_MAXIMIZE );
    int r = aa_opt_solve(cx,n_x,x);

    printf("r: %d\n", r );

    aa_dump_vec( stdout, x, n_x );

    double xref[] = {36, 0, 6 };
    assert(sizeof(xref) == sizeof(x));

    aafeq( name,
           cblas_ddot( (int)n_x, xref, 1, c, 1 ),
           cblas_ddot( (int)n_x, x, 1, c, 1 ),
           1e-3 );
}


void helper2( const char *name, aa_opt_gmcreate_fun fun) {

    double A[] = {1, 1};

    double b_u[] = {DBL_MAX};
    double b_l[] = {20};
    double c[] = {4, 2};

    double x_l[] = {0, 0};
    double x_u[] = {20, 10};

    size_t n_x = sizeof(x_l) / sizeof(x_l[0]);
    size_t n_c = sizeof(b_u) / sizeof(b_u[0]);

    assert( n_x == sizeof(x_l)/sizeof(double) );
    assert( n_x == sizeof(x_u)/sizeof(double) );
    assert( n_x == sizeof(c)/sizeof(double) );
    assert( sizeof(b_l) == sizeof(b_u) );
    assert( n_x * sizeof(b_u)
            ==
            sizeof(A) );

    double x[n_x];

    struct aa_opt_cx *cx = fun( n_c, n_x,
                                A, n_c,
                                b_l, b_u,
                                c,
                                x_l, x_u );


    aa_opt_set_direction(cx, AA_OPT_MINIMIZE );
    int r = aa_opt_solve(cx,n_x,x);

    printf("r: %d\n", r );

    aa_dump_vec( stdout, x, n_x );

    double xref[] = {10,10};

    assert(sizeof(xref) == sizeof(x));

    aafeq( name,
           cblas_ddot( (int)n_x, xref, 1, c, 1 ),
           cblas_ddot( (int)n_x, x, 1, c, 1 ),
           1e-3 );
}

int main( int argc, char **argv ) {
    (void) argc; (void) argv;


#ifdef HAVE_LPSOLVE
    helper0("LP Solve", aa_opt_lpsolve_gmcreate);
    helper1("LP Solve", aa_opt_lpsolve_gmcreate);
    helper2("LP Solve", aa_opt_lpsolve_gmcreate);
#endif

#ifdef HAVE_GLPK
    helper0("GLPK", aa_opt_glpk_gmcreate);
    helper1("GLPK", aa_opt_glpk_gmcreate);
    helper2("GLPK", aa_opt_glpk_gmcreate);
#endif

#ifdef HAVE_CLP
    helper0("CLP", aa_opt_clp_gmcreate);
    helper1("CLP", aa_opt_clp_gmcreate);
    helper2("CLP", aa_opt_clp_gmcreate);
#endif

}
