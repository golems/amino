/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2018-2019, Colorado School of Mines
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
#include "amino/mem.hpp"
#include "amino/mat.hpp"

#include "amino/test.h"

using namespace amino;


static inline void
admeq( const char *name, const DMat &A, const DMat &B, double tol)
{
    aafeq( name, DMat::ssd(A,B), 0, tol );
}


static void s_scal()
{
    RegionScope rs;

    double A[2] = {1, 2};
    DVec Av(2,A);

    DVec *Bv = DVec::alloc( rs.reg(), 2 );

    {
        *Bv = Av;
        assert(2 == Bv->len);
        assert(2 == Av.len);
        aveq( "DVec =", Bv->len, Bv->data, A, 0);
    }

    {
        double A2[2] = {2, 4};
        double A4[2] = {4, 8};

        *Bv = Av;
        aveq( "DVec = ", Bv->len, Bv->data, A, 0);
        *Bv *= 2;
        aveq( "DVec *= double", Bv->len, Bv->data, A2, 0);

        *Bv = 2*Av;
        aveq( "DVec = dscal 0", Bv->len, Bv->data, A2, 0);

        *Bv = (2*Av)*2;
        aveq( "DVec = dscal 1", Bv->len, Bv->data, A4, 0);

        *Bv = 2*(Av*2);
        aveq( "DVec = dscal 2", Bv->len, Bv->data, A4, 0);

        *Bv = 2*(Av+Av);
        aveq( "DVec = dscal 2", Bv->len, Bv->data, A4, 0);

        *Bv = (Av+Av)*2;
        aveq( "DVec = dscal 2", Bv->len, Bv->data, A4, 0);

        *Bv = 2*(Av+Av);
        aveq( "DVec = dscal 2", Bv->len, Bv->data, A4, 0);

        *Bv = 2*Av+(Av+Av);
        aveq( "DVec = dscal 2", Bv->len, Bv->data, A4, 0);

        *Bv = (Av+Av)+2*Av;
        aveq( "DVec = dscal 2", Bv->len, Bv->data, A4, 0);

        *Bv = Av+2*Av+Av;
        aveq( "DVec = dscal 2", Bv->len, Bv->data, A4, 0);

        *Bv = Av*2 + 2*Av;
        aveq( "DVec = dscal 2", Bv->len, Bv->data, A4, 0);

    }

}


static void s_axpy()
{
    RegionScope rs;

    double A[2] = {1, 2};
    DVec Av(2,A);

    DVec *Bv = DVec::alloc( rs.reg(), 2 );

    {
        double R[] = {2,4};
        *Bv = Av + Av;
        aveq( "DVec  axpy", Bv->len, Bv->data, R, 0);
    }

    {
        double R[] = {3,6};
        *Bv = (Av + Av) + Av;
        aveq( "DVec  axpy", Bv->len, Bv->data, R, 0);

        *Bv = Av + (Av + Av);
        aveq( "DVec  axpy", Bv->len, Bv->data, R, 0);


        *Bv = 2*Av + Av;
        aveq( "DVec  axpy", Bv->len, Bv->data, R, 0);

        *Bv = Av + 2*Av;
        aveq( "DVec  axpy", Bv->len, Bv->data, R, 0);
    }

}


static void s_mscal()
{
    {
        double d[] = {1,2, 3,4};
        double dp[] = {1,2, 3,4};
        double d2[] = {2,4, 6,8};
        double d4[] = {4,8, 12,16};
        DMat A(2,2,d,2);
        DMat Ap(2,2,dp,2);
        DMat A2(2,2,d2,2);
        DMat A4(2,2,d4,2);

        A *= 2;
        admeq( "dmat_scal-0", A, A2, 1e-6 );

        A = Ap*2;
        admeq( "dmat_scal-0", A, A2, 1e-6 );

        A = 2*Ap;
        admeq( "dmat_scal-0", A, A2, 1e-6 );


        A = 2*(Ap*2);
        admeq( "dmat_scal-0", A, A4, 1e-6 );

        A = (2*Ap)*2;
        admeq( "dmat_scal-0", A, A4, 1e-6 );

    }

    {
        double d[] = {1,2,3,0, 4,5,6,0};
        double dp[] = {1,2,3,0, 4,5,6,0};
        double d2[] = {2,4,6, 8,10,12 };
        DMat A(3,2,d,4);
        DMat Ap(3,2,dp,4);
        DMat A2(3,2,d2,3);
        A *= 2;
        admeq( "dmat_scal-1", A, A2, 1e-6 );

        A = Ap*2;
        admeq( "dmat_scal-1", A, A2, 1e-6 );

        A = 2*Ap;
        admeq( "dmat_scal-1", A, A2, 1e-6 );
    }
    {
        double d[] =  {1,2,0, 3,4,0, 5,6,0};
        double dp[] =  {1,2,0, 3,4,0, 5,6,0};
        double d2[] = {2,4, 6,8, 10,12};
        DMat A(2,3,d,3);
        DMat Ap(2,3,dp,3);
        DMat A2(2,3,d2,2);
        A *= 2;
        admeq( "dmat_scal-2", A, A2, 1e-6 );

        A = Ap*2;
        admeq( "dmat_scal-1", A, A2, 1e-6 );

        A = 2*Ap;
        admeq( "dmat_scal-1", A, A2, 1e-6 );
    }
}

static void s_transpose()
{
    double Xd[3*2 + 1] = { 1,2,3, 4,5,6, 7};

    {
        double Yd[3*2] = {0};
        double Rd[] = {1,4, 2,5, 3,6};
        DMat X(3,2,Xd,3);
        DMat Y(2,3,Yd,2);
        DMat R(2,3,Rd,2);
        Y = X.transpose();
        admeq( "dmat_trans-0", Y, R, 1e-6 );
    }

    {
        double Yd[3*3] = {0};
        double Rd[] = {1,4,0, 2,5,0, 3,6,0};
        DMat X(3,2,Xd,3);
        DMat Y(2,3,Yd,3);
        DMat R(2,3,Rd,3);
        Y = X.transpose();
        admeq( "dmat_trans-1", Y, R, 1e-6 );
    }

    {
        double Yd[2*2] = {0};
        double Rd[] = {1,4, 2,5};
        DMat X(2,2,Xd,3);
        DMat Y(2,2,Yd,2);
        DMat R(2,2,Rd,2);
        Y = X.transpose();
        admeq( "dmat_trans-1", Y, R, 1e-6 );
    }

}

static void s_inv()
{

    double Ad[] = {1,3,0, 2,4,0 };
    double Abd[4];
    double Aid[] = {-2, 1.5, 1, -.5};
    DMat A(2,2,Ad,3);
    DMat Ab(2,2,Abd,2);
    DMat Ai(2,2,Aid,2);

    Ab = A.inverse();
    admeq( "dmat_inv", Ab, Ai, 1e-6 );

    A.invert();
    admeq( "dmat_inv", A, Ai, 1e-6 );
}


int main(void)
{

    s_scal();
    s_axpy();
    s_mscal();
    s_transpose();
    s_inv();

    printf("MATPP: OK\n");
    return 0;
}
