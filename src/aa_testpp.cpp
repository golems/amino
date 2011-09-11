/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman
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


#include <amino.hpp>

using namespace amino;


//static void afeq( double a, double b, double tol ) {
    //assert( aa_feq(a,b,tol) );
//}


//static void aneq( double a, double b, double tol ) {
    //assert( !aa_feq(a,b,tol) );
//}

void mat() {
    // add
    {
        Vec<1> a;
        Vec<1> b;
        a[0] = 1;
        b[0] = 2;
        Vec<1> c = a + b;
        Vec<1> cp;
        cp[0] = 1 + 2;
        assert( c.eq(cp) );

        c = a*4;
        assert(aa_feq(c[0],4,0));

        c = a/4;
        assert(aa_feq(c[0],1.0/4,0));

        //c = a+4;
        //assert(aa_feq(c[0],1.0+4,0));

    }
}

void tf() {
    {
        Tf tf( 0, -1,  0, 1,
               1,  0,  0, 2,
               0,  0,  1, 3 );
        Mat<3> p(AA_FAR(3, 5, 7));
        Mat<3> qr(AA_FAR(-4,5,10));
        Mat<3> q = tf*p;
        assert( q.eq(qr) );
    }
}

int main( int argc, char **argv) {
    (void) argc; (void) argv;
    mat();
    tf();
    return 0;
}
