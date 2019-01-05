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
        *Bv *= 2;
        aveq( "DVec *= double", Bv->len, Bv->data, A2, 0);

        *Bv = 2*Av;
        aveq( "DVec = dscal", Bv->len, Bv->data, A2, 0);

        *Bv = (2*Av)*2;
        aveq( "DVec = dscal", Bv->len, Bv->data, A4, 0);

        *Bv = 2*(Av*2);
        aveq( "DVec = dscal", Bv->len, Bv->data, A4, 0);

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

int main(void)
{

    s_scal();
    s_axpy();

    printf("MATPP: OK\n");
    return 0;
}
