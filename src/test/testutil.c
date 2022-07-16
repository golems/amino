//* -*- mode: C; c-basic-offset: 4 -*- */
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

#include "amino.h"
#include "amino/test.h"
#include "amino/tf.h"
#include "amino_internal.h"
#include <sys/resource.h>


void test( const char *name, int check ) {
    if( !check ) {
        fprintf( stderr, "FAILED: %s\n",name);
        abort();
    }
}

void afeq( double a, double b, double tol ) {
    assert( aa_feq(a,b,tol) );
}

void test_feq( const char *name, double a, double b, double tol )
{
    if( !aa_feq(a,b,tol) ) {
        fprintf( stderr, "FAILED: %s, %f != %f\n",name, a, b);
        abort();
    }
}

void aafeq( const char * name, double a, double b, double tol ) {
    if( !aa_feq( a, b, tol) ) {
        fprintf( stderr, "FAILED: %s\n",name);
        fprintf( stderr, "a: %f, b: %f\n", a, b);
        abort();
    }
}


void aveq( const char * name,
                  size_t n, const double *a, const double *b, double tol ) {

    if( !aa_veq(n, a, b, tol) ) {
        fprintf( stderr, "FAILED: %s\n",name);
        fprintf( stderr, "a: ");
        aa_dump_vec( stderr, a, n );
        fprintf( stderr, "b: ");
        aa_dump_vec( stderr, b, n );
        abort();
    }
}

AA_API void arveq( const char * name, const double *a, const double *b, double tol )
{
    double a_angle = aa_tf_vnorm(a);
    double b_angle = aa_tf_vnorm(b);
    double a_angle_norm = aa_ang_norm_pi(a_angle);
    double b_angle_norm = aa_ang_norm_pi(b_angle);
    double rel_angle_norm = aa_ang_norm_pi(a_angle_norm - b_angle_norm);
    double a_axis[3], b_axis[3], ma_axis[3];
    for (int i = 0; i < 3; i++) {
        a_axis[i] = a[i] / a_angle;
        ma_axis[i] = -a_axis[i];
        b_axis[i] = b[i] / b_angle;
    }

    /* printf("a_angle: %f\n", a_angle); */
    /* printf("b_angle: %f\n", b_angle); */
    /* printf("a_angle_norm: %f\n", a_angle_norm); */
    /* printf("b_angle_norm: %f\n", b_angle_norm); */
    /* printf("a_angle_norm: %f\n", a_angle_norm); */
    /* printf("rel_angle_norm: %f\n", rel_angle_norm); */

    /* fprintf(stderr, "a-axis: "); */
    /* aa_dump_vec(stderr, a_axis, 3); */
    /* fprintf(stderr, "b-axis: "); */
    /* aa_dump_vec(stderr, b_axis, 3); */

    /* Check angle */
    if (fabs(rel_angle_norm) > tol) goto FAIL;

    /* Skip ill-defined axis on small angles */
    if (fabs(a_angle_norm) < tol || fabs(b_angle_norm) < tol) return;

    /* check axis */
    if (aa_veq(3, a_axis, b_axis, tol)) return;

    /* Check negative axis near pi */
    if (fabs(fabs(a_angle_norm) - M_PI) < tol ||
        fabs(fabs(b_angle_norm) - M_PI) < tol) {
        if (aa_veq(3, ma_axis, b_axis, tol)) return;
        // else fail
    }

FAIL:
    fprintf(stderr, "FAILED: %s\n", name);
    fprintf(stderr, "a: ");
    aa_dump_vec(stderr, a, 3);
    fprintf(stderr, "b: ");
    aa_dump_vec(stderr, b, 3);
    abort();
}


AA_API void aa_test_rotvec_cmp_pi(const char *name, const double *a,
                                  const double *b, double tol)
{
    double a_norm[3], b_norm[3];
    // Normalize
    {
        double a_angle = aa_tf_vnorm(a);
        double b_angle = aa_tf_vnorm(b);
        double a_angle_norm = aa_ang_norm_pi(a_angle);
        double b_angle_norm = aa_ang_norm_pi(b_angle);

        if (a_angle > M_PI) {
            FOR_VEC(i) a_norm[i] = a[i] * a_angle_norm / a_angle;
            a = a_norm;
        }

        if (b_angle > M_PI) {
            FOR_VEC(i) b_norm[i] = b[i] * b_angle_norm / b_angle;
            b = b_norm;
        }
    }

    double a_angle = aa_tf_vnorm(a);
    double b_angle = aa_tf_vnorm(b);


    /* fprintf(stderr, "an: "); */
    /* aa_dump_vec(stderr, a, 3); */
    /* fprintf(stderr, "bn: "); */
    /* aa_dump_vec(stderr, b, 3); */


    /* printf("a_angle: %f\n", a_angle); */
    /* printf("b_angle: %f\n", b_angle); */

    /* fprintf(stderr, "a-axis: "); */
    /* aa_dump_vec(stderr, a_axis, 3); */
    /* fprintf(stderr, "b-axis: "); */
    /* aa_dump_vec(stderr, b_axis, 3); */


    /* check norm eq */
    if (aa_veq(3, a, b, tol)) return;

    /* Check negative near pi */
    if (fabs(fabs(a_angle) - M_PI) < tol ||
        fabs(fabs(b_angle) - M_PI) < tol) {
        double ma[3];
        FOR_VEC(i) ma[i] = -a[i];
        if (aa_veq(3, ma, b, tol)) return;
        // else fail
    }

// FAIL
    fprintf(stderr, "FAILED aa_test_rotvec_cmp_pi: %s\n", name);
    fprintf(stderr, "a: ");
    aa_dump_vec(stderr, a, 3);
    fprintf(stderr, "b: ");
    aa_dump_vec(stderr, b, 3);
    abort();
}

void aneq( double a, double b, double tol ) {
    assert( !aa_feq(a,b,tol) );
}


void test_flt( const char *name, double a, double b, double tol )
{
    if( a - tol >= b ) {
        fprintf( stderr, "FAILED: %s\n",name);
        fprintf( stderr, "a: %f, b: %f\n", a, b);
        abort();
    }
}

void test_fgt( const char *name, double a, double b, double tol )
{
    if (a + tol <= b) {
        fprintf( stderr, "FAILDED: %s\n", name);
        fprintf( stderr, "a: %f, b: %f\n", a, b);
        abort();
    }
}

void aa_test_ulimit( void ) {
    // some limits because
    {
        struct rlimit lim;
        // address space
        lim.rlim_cur = (1<<30);
        lim.rlim_max = (1<<30);
        if( setrlimit( RLIMIT_AS, &lim ) ) {
            perror("could not limit address space size");
        }

        // cpu time
        lim.rlim_cur = 60;
        lim.rlim_max = 60;
        if( setrlimit( RLIMIT_CPU, &lim ) ) {
            perror("could not limit cpu time");
        }

        // drop a core
        if( getrlimit( RLIMIT_CORE, &lim ) ) {
            perror("could not get core size limit");
        }

        lim.rlim_cur = 100*1<<20;
        if( setrlimit( RLIMIT_CORE, &lim ) ) {
            perror("could not limit core size");
        }

    }
}

AA_API void aa_test_args(int argc, char *argv[])
{
    time_t seed;
    if (argc > 1) {
        char *endptr = NULL, *strseed = argv[1];
        seed = strtol(strseed, &endptr, 10);
        if (endptr == strseed || *endptr != '\0') {
            fprintf(stderr, "Invalid seed: `%s'\n", strseed);
            exit(EXIT_FAILURE);
        }
    } else {
        seed = time(NULL);
    }

    printf("seed: %ld\n", seed);
    srand((unsigned int)seed); // might break in 2038
}


AA_API void aa_test_randv(double min, double max, size_t n, double *p)
{
    double range = max - min;
    aa_vrand(n, p);
    for(size_t i = 0; i < n; i ++ ) {
        p[i] = min + p[i]*range;
    }
}

AA_API void aa_test_quat_cmp(const char *name, const double *q1,
                             const double *q2, double tol)
{
    double r[4];
    aa_tf_qrel(q1, q2, r);
    aa_tf_qminimize(r);
    if (aa_tf_qangle(r) > tol) goto FAIL;

    return;

FAIL:
    fprintf(stderr, "FAILED: %s\n", name);
    fprintf(stderr, "a: ");
    aa_dump_vec(stderr, q1, 4);
    fprintf(stderr, "b: ");
    aa_dump_vec(stderr, q2, 4);
    fprintf(stderr, "r: ");
    aa_dump_vec(stderr, r, 4);
    abort();
}

AA_API void aa_test_qutr_cmp(const char *name, const double *E1,
                             const double *E2, double tol)
{
    double r[4];
    aa_tf_qrel(E1, E2, r);
    aa_tf_qminimize(r);
    if (aa_tf_qangle(r) > tol) goto FAIL;

    if (aa_tf_vssd(E1 + 4, E2 + 4) > tol) goto FAIL;

    return;

FAIL:
    fprintf(stderr, "FAILED: %s\n", name);
    fprintf(stderr, "a: ");
    aa_dump_vec(stderr, E1, 7);
    fprintf(stderr, "b: ");
    aa_dump_vec(stderr, E2, 7);
    fprintf(stderr, "r: ");
    aa_dump_vec(stderr, r, 4);
    abort();
}


AA_API void aa_test_rotmat_cmp(const char *name, const double *R1,
                               const double *R2, double tol)
{
    // Rotation matrices are unique for a given rotation
    aveq(name, 9, R1, R2, tol);
}

AA_API void aa_test_isrotmat(const char *name, const double *R, double tol)
{
    static const double I[9] = AA_TF_ROTMAT_IDENT_INITIALIZER;
    double Rt[9], RRt[9];
    aa_tf_rotmat_inv2(R, Rt);

    // Angles between columns and rows
    if (aa_tf_vdot(R, R + 3)      > tol ||
        aa_tf_vdot(R, R + 6)      > tol  ||
        aa_tf_vdot(R + 3, R + 6 ) > tol ||
        aa_tf_vdot(Rt, Rt + 3)    > tol ||
        aa_tf_vdot(Rt, Rt + 6)    > tol ||
        aa_tf_vdot(Rt + 3, Rt + 6) > tol) {
        fprintf(stderr, "Not orth\n");
        goto FAIL;
    }

    // Unit Columns and Rows
    if (fabs(aa_tf_vnorm(R) - 1)      > tol ||
        fabs(aa_tf_vnorm(R + 3) - 1 ) > tol ||
        fabs(aa_tf_vnorm(R + 6) - 1 ) > tol ||
        fabs(aa_tf_vnorm(Rt) - 1)     > tol ||
        fabs(aa_tf_vnorm(Rt + 3) - 1) > tol ||
        fabs(aa_tf_vnorm(Rt + 6) - 1) > tol) {
        fprintf(stderr, "Not normal\n");
        goto FAIL;
    }

    // Determinant
    {
        double d = aa_la_det3x3(R);
        if (d < 1 - tol || d > 1 + tol) {
            fprintf(stderr, "det: %f\n", d);
            goto FAIL;
        }
    }

    // Chain
    aa_tf_rotmat_mul(R, Rt, RRt);
    if (aa_la_ssd(9, RRt, I) > tol) {
        fprintf(stderr, "inverse test failed\n");
        goto FAIL;
    }

    return;

FAIL:
    fprintf(stderr, "FAILED: %s\n", name);
    aa_dump_vec(stderr, R, 9);

    abort();
}
