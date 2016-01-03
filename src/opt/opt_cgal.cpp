/* -*- mode: C++; c-basic-offset: 4; -*- */
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
#include "amino/opt.h"


#include <iostream>
#include <CGAL/basic.h>
#include <CGAL/QP_models.h>
#include <CGAL/QP_functions.h>

#ifdef CGAL_USE_GMP
#include <CGAL/Gmpz.h>
typedef CGAL::Gmpz ET;
#else
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;
#endif
// program and solution types
typedef CGAL::Quadratic_program<ET> Program;
typedef CGAL::Quadratic_program_solution<ET> Solution;
typedef CGAL::Quotient<ET> Quotient;



AA_API int aa_opt_qp_solve_cgal (
    size_t m, size_t n,
    const double *A, size_t ldA,
    const double *b,
    const double *c, double c0,
    const double *D, size_t ldD,
    const double *l, const double *u,
    double *x )
{

    // aa_dump_mat(stderr, A, m, n);
    // aa_dump_vec(stderr, b, m );
    // aa_dump_vec(stderr, c, n );
    // aa_dump_mat(stderr, D, n, n);
    // aa_dump_vec(stderr, l, n);
    // aa_dump_vec(stderr, u, n);

    int mi = (int)m;
    int ni = (int)n;
    // by default, we have a nonnegative QP with Ax <= b
    Program qp (CGAL::SMALLER,
                l ? true : false,
                0,
                u ? true : false,
                1);

    // A
    for( int i = 0; i < mi; i ++ ) {
        for( int j = 0; j < ni; j ++ ) {
            qp.set_a(j, i, AA_MATREF(A, ldA, i, j));
        }
    }
    // b
    for( int i = 0; i < mi; i ++ ) {
        qp.set_b(i, b[i]);
    }
    // c
    for( int j = 0; j < ni; j ++ ) {
        qp.set_c(j, c[j]);
    }
    // c0
    qp.set_c0(c0);
    // D
    for( int j1 = 0; j1 < ni; j1 ++ ) {
        for( int j2 = 0; j2 < ni; j2 ++ ) {
            /* Apparently CGAL needs 2*D? */
            qp.set_d(j1, j1, 2*AA_MATREF(D, ldD, j1, j2));
        }
    }
    // L/U
    for( int j = 0; j < ni; j ++ ) {
        if(l) qp.set_l(j, true, l[j]);
        if(u) qp.set_u(j, true, u[j]);
    }


    // solve the program, using ET as the exact type

    //std::cout << qp;

    Solution s = CGAL::solve_quadratic_program(qp, ET());
    if( s.solves_quadratic_program(qp) ) {
        //printf("solved\n");

        //std::cout << s;

        // return result
        size_t j = 0;
        for( auto itr = s.variable_values_begin (); itr != s.variable_values_end(); itr++ ) {
            x[j] = CGAL::to_double(*itr);
            //printf("x[%d] = %f\n", j, x[j] );
            j++;
        }
        return 0;
    } else {
        // no solution
        return -1;
    }

}
