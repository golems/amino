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

static inline void
admeq( const char *name, const struct aa_dmat *A, const struct aa_dmat *B, double tol)
{
    aafeq( name, aa_dmat_ssd(A,B), 0, tol );
}

static inline void
adveq( const char *name, const struct aa_dvec *x, const struct aa_dvec *y, double tol)
{
    aafeq( name, aa_dvec_ssd(x,y), 0, tol );
}

static void test_transpose()
{
    double X[3*2 + 1] = { 1,2,3, 4,5,6, 7};

    {
        double Y[3*2] = {0};
        double x0[] = {1,4, 2,5, 3,6};
        struct aa_dmat mX = AA_DMAT_INIT(3,2,X,3);
        struct aa_dmat mY = AA_DMAT_INIT(2,3,Y,2);
        struct aa_dmat mr = AA_DMAT_INIT(2,3,x0,2);

        aa_dmat_trans(&mX,&mY);
        admeq( "dmat_trans-0", &mY, &mr, 1e-6 );
    }

    {
        double Y[3*3] = {0};
        double x1[] = {1,4,0, 2,5,0, 3,6,0};
        struct aa_dmat mX = AA_DMAT_INIT(3,2,X,3);
        struct aa_dmat mY = AA_DMAT_INIT(2,3,Y,3);
        struct aa_dmat mr = AA_DMAT_INIT(2,3,x1,3);
        aa_dmat_trans(&mX,&mY);
        admeq( "dmat_trans-1", &mY, &mr, 1e-6 );
    }

    {
        double Y[2*2] = {0};
        double x2[] = {1,4, 2,5};
        struct aa_dmat mX = AA_DMAT_INIT(2,2,X,3);
        struct aa_dmat mY = AA_DMAT_INIT(2,2,Y,2);
        struct aa_dmat mr = AA_DMAT_INIT(2,2,x2,2);
        aa_dmat_trans(&mX,&mY);
        admeq( "dmat_trans-2", &mY, &mr, 1e-6 );
    }

}


static void test_inv()
{

    double A[] = {1,3,0, 2,4,0 };
    double Ai[] = {-2, 1.5, 1, -.5};
    struct aa_dmat mA = AA_DMAT_INIT(2,2,A,3);
    struct aa_dmat mAi = AA_DMAT_INIT(2,2,Ai,2);
    aa_dmat_inv(&mA);
    admeq( "dmat_inv", &mA, &mAi, 1e-6 );

}

static void test_pinv()
{
    {

        double A[] = {1,2, 3,4 };
        double B[4];
        double Ai[] = {-2,1, 1.5,-.5};
        struct aa_dmat mA = AA_DMAT_INIT(2,2,A,2);
        struct aa_dmat mB = AA_DMAT_INIT(2,2,B,2);
        struct aa_dmat mAi = AA_DMAT_INIT(2,2,Ai,2);
        aa_dmat_pinv(&mA,-1,&mB);
        admeq( "dmat_pinv-0", &mB, &mAi, 1e-6 );
    }
    {
        double A[] = {1,2,3,4,5,6};
        double A_star[6] = {0};
        double R[] = {-1.3333, -.33333, 0.66667, 1.083333, 0.33333, -0.4166667};

        struct aa_dmat mA  = AA_DMAT_INIT(2,3,A,2);
        struct aa_dmat mAs = AA_DMAT_INIT(3,2,A_star,3);

        int r = aa_dmat_pinv( &mA, -1, &mAs );
        aveq( "la_pinv-1", 6, A_star, R, .0001 );
        assert(0 == r );
    }

    {
        double A[] = {1,3,5, 2,4,6 };
        double Ai[6];
        double R[] = {-1.3333,1.08333, -0.333333,0.3333333, 0.6666667, -0.416667};
        struct aa_dmat mA  = AA_DMAT_INIT(3,2,A,3);
        struct aa_dmat mAi = AA_DMAT_INIT(2,3,Ai,2);
        struct aa_dmat mR  = AA_DMAT_INIT(2,3,R,2);
        aa_dmat_pinv(&mA, -1, &mAi);
        admeq( "dmat_pinv-2", &mAi, &mR, 1e-3 );
    }

}

static void test_dpinv()
{
    //dpinv
    {
        double A[] = {1,2,3,4};
        double A_star[4] = {0};
        struct aa_dmat mA  = AA_DMAT_INIT(2,2,A,2);
        struct aa_dmat mAs = AA_DMAT_INIT(2,2,A_star,2);
        double R[] = {-2,1, 1.5,-.5};
        int r = aa_dmat_dpinv( &mA, 1e-4, &mAs );
        //printf("r0: %d\n", r);
        aveq( "la_dpinv-0", 4, A_star, R, 1e-2 );
        assert(0 == r );
    }
    {
        double A[] = {1,2,3,4,5,6};
        double A_star[6] = {0};
        double R[] = {-1.3333, -.33333, 0.66667, 1.083333, 0.33333, -0.4166667};

        struct aa_dmat mA  = AA_DMAT_INIT(2,3,A,2);
        struct aa_dmat mAs = AA_DMAT_INIT(3,2,A_star,3);

        int r = aa_dmat_dpinv( &mA, 1e-4, &mAs );
        //printf("r0: %d\n", r);
        aveq( "la_dpinv-1", 6, A_star, R, .001 );
        assert(0 == r );
    }

    {
        double A[] = {1,3,5, 2,4,6 };
        double A_star[6] = {0};
        double R[] = {-1.3333,1.08333, -0.333333,0.3333333, 0.6666667, -0.416667};
        //double R[6] = {-0.95256, 0.44785, -0.11224, 0.11159, 0.72808, -0.22467};

        struct aa_dmat mA  = AA_DMAT_INIT(3,2,A,3);
        struct aa_dmat mAs = AA_DMAT_INIT(2,3,A_star,2);

        int r = aa_dmat_dpinv( &mA, 1e-3, &mAs );
        //printf("r0: %d\n", r);
        aveq( "la_dpinv-2", 6, A_star, R, 1e-2 );
        assert(0 == r );
    }

    {
        double A[] = {1,2,3,4,5,6};
        double A_star[6] = {0};
        double R[6] = {-0.95256, 0.44785, -0.11224, 0.11159, 0.72808, -0.22467};

        struct aa_dmat mA  = AA_DMAT_INIT(3,2,A,3);
        struct aa_dmat mAs = AA_DMAT_INIT(2,3,A_star,2);

        int r = aa_dmat_dpinv( &mA, 1e-3, &mAs );
        //printf("r0: %d\n", r);
        aveq( "la_dpinv-3", 6, A_star, R, 1e-2 );
        assert(0 == r );
    }

}

static void test_dzdpinv()
{
    //dpinv
    {
        double A[] = {1,2,3,4};
        double A_star[4] = {0};
        struct aa_dmat mA  = AA_DMAT_INIT(2,2,A,2);
        struct aa_dmat mAs = AA_DMAT_INIT(2,2,A_star,2);
        double R[] = {  -2, 1,  1.5,  -0.5 };
        int r = aa_dmat_dzdpinv( &mA, .005, &mAs );
        //printf("r0: %d\n", r);
        aveq( "la_dzdpinv-0", 4, A_star, R, .0001 );
        assert(0 == r );
    }
    {
        double A[] = {1,2,3,4,5,6};
        double A_star[6] = {0};
        double R[] = {-1.3333, -.33333, 0.66667, 1.083333, 0.33333, -0.4166667};

        struct aa_dmat mA  = AA_DMAT_INIT(2,3,A,2);
        struct aa_dmat mAs = AA_DMAT_INIT(3,2,A_star,3);

        int r = aa_dmat_dzdpinv( &mA, .005, &mAs );
        //printf("r0: %d\n", r);
        aveq( "la_dzdpinv-1", 6, A_star, R, .0001 );
        assert(0 == r );
    }

    {
        double A[] = {1,3,5, 2,4,6 };
        double A_star[6] = {0};
        double R[] = {-1.3333,1.08333, -0.333333,0.3333333, 0.6666667, -0.416667};
        //double R[6] = {-0.95256, 0.44785, -0.11224, 0.11159, 0.72808, -0.22467};

        struct aa_dmat mA  = AA_DMAT_INIT(3,2,A,3);
        struct aa_dmat mAs = AA_DMAT_INIT(2,3,A_star,2);

        int r = aa_dmat_dzdpinv( &mA, .001, &mAs );
        //printf("r0: %d\n", r);

        assert(0 == r );
    }

}

static void test_scal()
{
    {
        double d[] = {1,2, 3,4};
        double d2[] = {2,4, 6,8};
        struct aa_dmat A = AA_DMAT_INIT(2,2,d,2);
        struct aa_dmat A2 = AA_DMAT_INIT(2,2,d2,2);
        aa_dmat_scal(&A,2);
        admeq( "dmat_scal-0", &A, &A2, 1e-6 );
    }
    {
        double d[] = {1,2,3,0, 4,5,6,0};
        double d2[] = {2,4,6, 8,10,12 };
        struct aa_dmat A = AA_DMAT_INIT(3,2,d,4);
        struct aa_dmat A2 = AA_DMAT_INIT(3,2,d2,3);
        aa_dmat_scal(&A,2);
        admeq( "dmat_scal-1", &A, &A2, 1e-6 );
    }
    {
        double d[] =  {1,2,0, 3,4,0, 5,6,0};
        double d2[] = {2,4, 6,8, 10,12};
        struct aa_dmat A = AA_DMAT_INIT(2,3,d,3);
        struct aa_dmat A2 = AA_DMAT_INIT(2,3,d2,2);
        aa_dmat_scal(&A,2);
        admeq( "dmat_scal-2", &A, &A2, 1e-6 );
    }
}
static void
test_inc()
{
    double xd[] = {1,2,3};
    double yd[] = {2,3,4};

    struct aa_dvec x = AA_DVEC_INIT(3,xd,1);
    struct aa_dvec y = AA_DVEC_INIT(3,yd,1);
    aa_lb_dinc(1,&x);


    aveq( "dinc", 3,xd,yd,0);

}

static void test_nrm2()
{
    double Ad[] = {1,2,3,1024,   4,5,6,2048 };
    struct aa_dmat A = AA_DMAT_INIT( 3, 2, Ad, 4 );
    aafeq( "mat-nrm2", aa_dmat_nrm2(&A),
           sqrt(1*1 + 2*2 + 3*3 + 4*4 + 5*5 + 6*6),
           1e-6 );

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
    test_inv();
    test_pinv();
    test_dpinv();
    test_dzdpinv();
    test_scal();
    test_inc();
    test_nrm2();

    printf("mat_test: OK\n");
    return 0;
}
