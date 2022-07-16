/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2013, Georgia Tech Research Corporation
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

static void rotmat_axang() {
    /* Had some numerical stability issues */
    double q[4] =  {0.538444,0.141454,0.510387,0.655419};
    double R[9];
    aa_tf_quat2rotmat(q, R); // get rotation matrix

    double qr[4];
    aa_tf_rotmat2quat(R, qr); // convert to quaternion
    aveq("quat", 4, q, qr, .001);

    double vr[3], vq[3];      // get rotation vectors
    aa_tf_quat2rotvec(q, vq);
    aa_tf_rotmat2rotvec(R, vr);
    aveq("vec", 3, vq, vr, .001);

    double qvr[4], qvq[4];    // convert back to quaternions
    aa_tf_rotvec2quat( vq, qvq );
    aa_tf_rotvec2quat( vr, qvr );
    aveq("quat2", 4, qvq, qvr, .001);
}

static void slerp() {
    double q[4], q1[4], u;
    aa_tf_qurand(q);
    u = aa_frand();
    aa_tf_qslerp(u, q, q, q1);
    aveq( "slerp-equiv", 4, q1, q, 1e-8 );
    aa_tf_qslerpalg(u, q, q, q1);
    aveq( "slerpalg-equiv", 4, q1, q, 1e-8 );
}



static void rotmat( void ) {
    double R0[9] = { 0, 1, 0,
                     0, 0, 1,
                     1, 0, 0 };
    assert(aa_tf_isrotmat(R0));
}


static void rotmat_lnv_test(const double e[3]) {
    /* fprintf(stderr, "--\n"); */

    // Rotation
    {
        double ee[9], q[4], rln[3], qe[4], eln[3];
        aa_tf_rotmat_expv(e, ee);
        aa_tf_rotvec2quat(e, q);
        aa_tf_qminimize(q);
        aa_tf_rotmat2quat(ee, qe);
        aa_tf_quat2rotvec(qe, eln);
        aa_tf_rotmat_lnv(ee, rln);

        /* fprintf(stderr, "R:\n"); */
        /* aa_dump_mat(stderr, ee, 3, 3); */
        /* fprintf(stderr, "\n"); */

        /* fprintf(stderr, "e: "); */
        /* aa_dump_vec(stderr, e, 3); */
        /* fprintf(stderr, "eln: "); */
        /* aa_dump_vec(stderr, eln, 3); */
        /* fprintf(stderr, "rln: "); */
        /* aa_dump_vec(stderr, rln, 3); */

        /* assert(aa_tf_isrotmat(ee)); */
        aveq("quat_exp", 4, q, qe, 1e-4);
        arveq("rotmat_expv", e, eln, 1e-4);
        arveq("rotmat_lnv", e, rln, 1e-4);
    }

    // Transformation
    {
        double v[6], ve[6];
        double T[12], E[7], ET[7], Tln[6],  Eln[6], T2[12];
        AA_MEM_CPY(v + 3, e, 3);
        v[0] = 1;
        v[1] = 2;
        v[2] = 3;
        for (int i = 0; i < 6; i++) ve[i] = v[i]/2;

        /* fprintf(stderr, "v:  "); */
        /* aa_dump_vec(stderr, v, 6); */

        aa_tf_tfmat_expv(v, T);
        aa_tf_qutr_expv(ve, E);
        aa_tf_tfmat2qutr(T, ET);


        /* fprintf(stderr, "E:  "); */
        /* aa_dump_vec(stderr, E, 7); */
        /* fprintf(stderr, "ET: "); */
        /* aa_dump_vec(stderr, ET, 7); */

        /* We don't recover the original log, but it does produce the same
         * matrix */
        aa_tf_tfmat_lnv(T, Tln);
        aa_tf_tfmat_expv(Tln, T2);

        aa_tf_qutr_lnv(E, Eln);
        for (int i = 0; i < 6; i++) Eln[i] *= 2;



        aa_tf_qminimize(ET);
        aa_tf_qminimize(E);
        aveq("tfmat/qutr_expv", 7, E, ET, 1e-3);
        aveq("qutr_lnv", 6, v, Eln, 1e-3);
        aveq("tfmat_lnv", 12, T, T2, 1e-3);
    }
}

static void rotmat_lnv()
{
    {
        /* The rotation angle is very close to pi */
        double e[3] = {1.808519650250, 0.065124237394, 2.567988211640};
        rotmat_lnv_test(e);
    }

    {
        /* The rotation angle is pi */
        double u[3] = {0.575671, 0.020730, 0.817418};
        aa_tf_vnormalize(u);
        double e[3] = {M_PI * u[0], M_PI * u[1], M_PI * u[2]};
        rotmat_lnv_test(e);
    }
    {
        /* The rotation angle is -pi */
        double u[3] = {0.575671, 0.020730, 0.817418};
        aa_tf_vnormalize(u);
        double  e[3] = {-M_PI * u[0], -M_PI * u[1], -M_PI * u[2]};
        rotmat_lnv_test(e);
    }
}

static void rotvec_axang_exp()
{
    double rv[3] = {0.000067, 0.000029, -0.000023};
    double R[9], q[4], aa[4], Raa[9];

    aa_tf_rotvec2rotmat(rv, R);
    aa_tf_rotvec2quat(rv, q);
    aa_tf_rotvec2axang(rv, aa);
    aa_tf_axang2rotmat(aa, Raa);

    double ln_R[3], ln_q[3], ln_aa[3], ln_Raa[3];

    aa_tf_quat2rotvec(q, ln_q);
    aa_tf_rotmat2rotvec(R, ln_R);
    aa_tf_axang2rotvec(aa, ln_aa);
    aa_tf_rotmat2rotvec(Raa, ln_Raa);

    aveq("rotvec_axang_exp-ln_q", 3, rv, ln_q, 1e-9);
    aveq("rotvec_axang_exp-ln_q", 3, rv, ln_q, 1e-9);
    aveq("rotvec_axang_exp-ln_R", 3, rv, ln_R, 1e-9);
    aveq("rotvec_axang_exp-ln_aa", 3, rv, ln_aa, 1e-9);
    aveq("rotvec_axang_exp-ln_Raa", 3, rv, ln_Raa, 1e-9);
}

int main() {
    srand((unsigned int)time(NULL)); // might break in 2038
    aa_test_ulimit();

    rotmat_axang();
    slerp();
    rotmat();
    rotmat_lnv();
    rotvec_axang_exp();
}
