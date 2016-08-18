/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2012, Georgia Tech Research Corporation
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
#include "amino.h"
#include "amino/test.h"
#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>

static void test_ssd();
static void test_angle();
//static void test_cross();
//static void test_proj_orth();
//static void test_std();
static void test_meancov();


int main( int argc, char **argv ) {
    printf("Init la_ctest\n");
    (void) argc; (void) argv;


    // init
    srand((unsigned int)time(NULL)); // might break in 2038
    // some limits because linux (and sometimes our software) sucks
    {
        int r;
        struct rlimit lim;
        // address space
        lim.rlim_cur = (1<<30);
        lim.rlim_max = (1<<30);
        r = setrlimit( RLIMIT_AS, &lim );
        assert(0 == r );
        // cpu time
        lim.rlim_cur = 60;
        lim.rlim_max = 60;
        r = setrlimit( RLIMIT_CPU, &lim );
        assert(0 == r );
        // drop a core
        r = getrlimit( RLIMIT_CORE, &lim );
        assert(0==r);
        lim.rlim_cur = 100*1<<20;
        r = setrlimit( RLIMIT_CORE, &lim );
        assert(0==r);

    }

    test_ssd();
    test_angle();
    test_meancov();

    printf("Ending la_ctest\n");
}


static void test_ssd()
{
    {
        double a[] = {0,0};
        double b[] = {1,1};
        test_feq( "ssd1", 2, aa_la_d_ssd( sizeof(a)/sizeof(a[0]), a,1, b,1), 0 );
    }
    {
        double a[] = {0,0};
        double b[] = {1,2};
        test_feq( "ssd2", 5, aa_la_d_ssd( sizeof(a)/sizeof(a[0]), a,1, b,1), 0 );
    }
    {
        double a[] = {0,0};
        double b[] = {2,1};
        test_feq( "ssd3", 5, aa_la_d_ssd( sizeof(a)/sizeof(a[0]), a,1, b,1), 0 );
    }
    {
        double a[] = {0,0};
        double b[] = {2,2};
        test_feq( "ssd4", 8, aa_la_d_ssd( sizeof(a)/sizeof(a[0]), a,1, b,1), 0 );
    }
    {
        double a[] = {1,0};
        double b[] = {2,2};
        test_feq( "ssd5", 5, aa_la_d_ssd( sizeof(a)/sizeof(a[0]), a,1, b,1), 0 );
    }
}

static void test_angle()
{
    {
        double a[]={1,0};
        double b[]={0,1};
        test_feq("angle1", M_PI_2, aa_la_d_angle(2, a, 1, b, 1), 1e-6 );
    }
    {
        double a[]={1,0};
        double b[]={-1,0};
        test_feq("angle2", M_PI, aa_la_d_angle(2, a, 1, b, 1), 1e-6 );
    }
    {
        double a[]={2,0};
        double b[]={0,2};
        test_feq("angle1", M_PI_2, aa_la_d_angle(2, a, 1, b, 1), 1e-6 );
    }
    {
        double a[]={2,0};
        double b[]={-2,0};
        test_feq("angle1", M_PI, aa_la_d_angle(2, a, 1, b, 1), 1e-6 );
    }

}

static void test_meancov()
{

    double  A[] = {1.,2., 3.,3., 8.,12.}; // 2x3
    double e[4], mu[2], At[6];

    double mu_r[2] = {4, 5.6666667};
    double e_r[] = {13.,19.5, 19.5,30.333333};

    aa_la_d_colmean( 2, 3, A, 2, mu);
    aveq( "colmean", 2, mu, mu_r, 1e-6);

    aa_la_d_transpose(2,3,A,2, At,3);
    printf("\n");
    aa_dump_mat( stdout, A, 2,3);
    printf("\n");
    aa_dump_mat( stdout, At, 3,2);
    printf("\n");

    aa_la_d_rowmean( 3, 2, At, 3, mu);
    aveq( "rowmean", 2, mu, mu_r, 1e-6);

    aa_la_d_colcov( 2, 3, A, 2, mu, e, 2);
    aveq( "colcov", 4, e, e_r, 1e-6);
}


/* static void test_proj_orth() */
/* { */
/*     double  theta,a[2],p[2],o[2],rp[2],ro[2],s[2]; */
/*     double X[] = {1,0}; */
/*     for( double i = 0; i < 360; i ++ )  { */
/*         for( double j = 0; j <  100; j ++ ) { */
/*             theta = i*M_PI/180; */
/*             a[0] = j*cos(theta); */
/*             a[1] = j*sin(theta); */
/*             p=a; */
/*             o=a; */
/*             o(0) = 0.; */
/*             p(1) = 0.; */
/*             aa_la_proj_sub(a, X, rp); */
/*             aa_la_orth_sub(a, X, ro); */
/*             vtest("proj_sub", p, rp, .001 ); */
/*             vtest("orth_sub", o, ro, .001 ); */
/*             rp = aa_la_proj(a, X); */
/*             ro = aa_la_orth(a, X); */
/*             vtest("proj_sub", p, rp, .001 ); */
/*             vtest("orth_sub", o, ro, .001 ); */
/*             s = ro+rp; */
/*             call vtest("proj_orth", a, s, .001 ); */
/*         } */
/*     } */
/* } */
