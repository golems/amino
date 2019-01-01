/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2018, Colorado School of Mines
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

// uncomment to check that local allocs actually get freed
// #define AA_ALLOC_STACK_MAX 0


#include <stdlib.h>
#include <cblas.h>

#include "amino.h"
#include "amino/mat.h"

#include "amino/test.h"


static void test_transpose()
{
    double X[3*2 + 1] = { 1,2,3, 4,5,6, 7};

    {
        double Y[3*2] = {0};
        double x0[] = {1,4, 2,5, 3,6};
        struct aa_dmat mX = AA_DMAT_INIT(3,2,X,3);
        struct aa_dmat mY = AA_DMAT_INIT(2,3,Y,2);
        aa_dmat_trans(&mX,&mY);
        aveq( "dmat_trans-0", 6, Y, x0, 1e-6 );
    }

    {
        double Y[3*3] = {0};
        double x1[] = {1,4,0, 2,5,0, 3,6,0};
        struct aa_dmat mX = AA_DMAT_INIT(3,2,X,3);
        struct aa_dmat mY = AA_DMAT_INIT(2,3,Y,3);
        aa_dmat_trans(&mX,&mY);
        aveq( "dmat_trans-1", 6, Y, x1, 1e-6 );
    }

    {
        double Y[2*2] = {0};
        double x2[] = {1,4, 2,5};
        struct aa_dmat mX = AA_DMAT_INIT(2,2,X,3);
        struct aa_dmat mY = AA_DMAT_INIT(2,2,Y,2);
        aa_dmat_trans(&mX,&mY);
        aveq( "dmat_trans-2", 3, Y, x2, 1e-6 );
    }

}

int main(void)
{

    {
        double Ad[] = {1,2,3,1024,   4,5,6,2048 };
        double xd[] = {7, 4096, 11, 4096};
        double yd[3];
        double rd[3] = {51, 69, 87};

        struct aa_dmat A;
        struct aa_dvec x,y;

        aa_dmat_view( &A, 3, 2, Ad, 4 );
        aa_dvec_view( &x, 2, xd, 2 );
        aa_dvec_view( &y, 3, yd, 1 );

        aa_lb_dgemv( CblasNoTrans, 1, &A, &x, 0, &y );

        aveq( "dgemv", 3,yd,rd,0);
    }

    test_transpose();

    printf("mat_test: OK\n");
    return 0;
}
