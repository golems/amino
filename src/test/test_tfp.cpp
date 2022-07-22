/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2022, Colorado School of Mines
 * All rights reserved.
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
#include <complex.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <sys/resource.h>

using namespace ::amino;

/* Check sizes */

static_assert(sizeof(Vec2) == 2 * sizeof(double));
static_assert(sizeof(Cmplx) == 2 * sizeof(double));
static_assert(sizeof(RotMatP) == 4 * sizeof(double));

static_assert(sizeof(CmplxTran) == 4 * sizeof(double));
static_assert(sizeof(TfMatP) == 6 * sizeof(double));

static Vec2 rand_vec()
{
    return Vec2(aa_frand_minmax(-1, 1),aa_frand_minmax(-1, 1));
}

static double rand_angle()
{
    return aa_frand_minmax(-.99*M_PI, .99*M_PI);
}

static void
test_vec(const Vec2 V0) {
    aa_tf_vec2 v1 = V0;
    Vec2 V1 = v1;

    test_feq("vec x", V0.x(), V1.x(), 0);
    test_feq("vec y", V0.y(), V1.y(), 0);
    aa_test_vec2_cmp("vec", V0, v1, 0);
    aa_test_vec2_cmp("vec", V0, V1, 0);

    Vec2 V2(0,0);
    V2.x() = V0.x();
    V2.y() = V0.y();

    test_feq("vec.x()=", V0.x(), V2.x(), 0);
    test_feq("vec.y()=", V0.y(), V2.y(), 0);
    aa_test_vec2_cmp("vec", V0, V2, 0);
}

static void
test_rotconv(double theta, const Cmplx c, const RotMatP &R)
{
    AngleP Theta(theta);
    {  // To angle
        test_feq("angle2angle", theta, Theta.angle(), 0);
        test_feq("cmplx2angle", theta, c.angle(), 1e-9);
        test_feq("rotmatp2angle", theta, R.angle(), 1e-9);

        test_feq("AngleP(cmplx)", theta, AngleP(c).angle(), 1e-9);
    }

    {  // To complex
        Cmplx c_theta(Theta);
        Cmplx c_R(R);

        aa_test_cmplx_cmp("angle2cmplx", c, c_theta, 1e-9);
        aa_test_cmplx_cmp("rotmatp2cmplx", c, c_R, 1e-9);

        aa_test_cmplx_cmp("2cmplx 1", Cmplx::from_angle(theta), Cmplx(Theta), 0);
        aa_test_cmplx_cmp("2cmplx 2", Cmplx(c), Cmplx(c.c_cmplx()), 0);
        aa_test_cmplx_cmp("2cmplx 3", c_R, Cmplx(&R), 0);
    }

    {  // To rotmatp
        RotMatP R_theta(Theta);
        RotMatP R_c(c);
        aa_test_rotmatp_cmp("angle2rotmatp",  R.data, R_theta.data, 1e-9);
        aa_test_rotmatp_cmp("cmpxl2rotmatp",  R.data, R_c.data, 1e-9);


        RotMatP R0(R);
        RotMatP R1 = R;
        RotMatP R2(theta);
        R2 = R;

        RotMatP R3(c);
        RotMatP R4(c.c_cmplx());
        RotMatP R5(theta);
        RotMatP R6(Theta);

        aa_test_rotmatp_cmp("2rotmatp 0",  R.data, R0.data, 0);
        aa_test_rotmatp_cmp("2rotmatp 1",  R.data, R1.data, 0);
        aa_test_rotmatp_cmp("2rotmatp 2",  R.data, R2.data, 0);

        aa_test_rotmatp_cmp("2rotmatp 3",  R.data, R3.data, 1e-9);
        aa_test_rotmatp_cmp("2rotmatp 4",  R.data, R4.data, 1e-9);
        aa_test_rotmatp_cmp("2rotmatp 5",  R.data, R5.data, 1e-9);
        aa_test_rotmatp_cmp("2rotmatp 6",  R.data, R6.data, 1e-9);

        aa_test_rotmatp_cmp("2rotmatp 3/4",  R2.data, R4.data, 0);
        aa_test_rotmatp_cmp("2rotmatp 5/6",  R5.data, R6.data, 0);
    }
}

static void
test_ln(double theta, const Cmplx &c, const RotMatP &R)
{
    test_feq("cmplx pexp/ulog", theta, c.uln(), 1e-9);
    test_feq("rotmat pexp/ulog", theta, R.uln(), 1e-9);
}

static void
test_rot(const Cmplx c, const RotMatP &R, const Vec2 v)
{
    aa_test_vec2_cmp("rot", c*v, R*v, 1e-9);
}

static void
test_inv(double theta, const Cmplx c, const RotMatP R)
{
    Cmplx ci = c.conj();
    double theta_ci = ci.angle();
    Cmplx cci = c*ci;

    static const double cR_ident[] = AA_TF_ROTMATP_IDENT_INITIALIZER;
    static_assert(sizeof(cR_ident) == 4 * sizeof(double));
    static const RotMatP R_ident;
    aa_test_rotmatp_cmp("rotmatp ident",  cR_ident, R_ident.data, 0);

    RotMatP Ri = R.inv();
    double theta_ri = Ri.angle();
    RotMatP RRi = R*Ri;

    test_feq("rotmatp_inv", -theta, theta_ri, 1e-9);
    test_feq("cconj", -theta, theta_ci, 1e-9);
    aa_test_cmplx_cmp("cconj, mul", AA_TF_CMPLX_IDENT, cci, 1e-9);
    aveq("rotmatp_inv, mul", 4, R_ident.data, RRi.data, 1e-9);
}

static void
test_mul(double theta[2], const Cmplx c[2], const RotMatP R[2])
{
    Cmplx c2 = c[0]*c[1];
    Cmplx c2_c = aa_tf_cmul(c[0], c[1]);
    RotMatP R2 = R[0]*R[1];

    double theta2 = aa_ang_norm_pi(theta[0] + theta[1]);
    double theta2_c = c2.uln();
    double theta2_c1 = c2_c.uln();
    double theta2_R = R2.angle();

    test_feq("cmul", theta2, theta2_c, 1e-9);
    test_feq("cmul 1", theta2, theta2_c1, 1e-9);
    test_feq("rotmatp_mul", theta2, theta2_R, 1e-9);
}

static void
test_tf(const CmplxTran &E, const TfMatP &T, const aa_tf_vec2 p)

{
    aa_test_cmplx_cmp("tf", E*p, T*p, 1e-9);
}

static void
test_tf_conv(double theta, const Cmplx c, const Vec2 v, const RotMatP &R,
             const TfMatP &T)
{
    AngleP Theta(theta);
    CmplxTran E(c, v);
    {  // To Tfmat
        TfMatP T0(T);
        TfMatP T1(R, v);
        TfMatP T2(Theta, v);
        TfMatP T3(theta, v);
        TfMatP T4(E);
        TfMatP T5(c, v);
        TfMatP T6(c.c_cmplx(), v);
        aa_test_tfmatp_cmp("2tfmat 0", T.data, T0.data, 0);
        aa_test_tfmatp_cmp("2tfmat 1", T.data, T1.data, 0);
        aa_test_tfmatp_cmp("2tfmat 2", T.data, T2.data, 1e-9);
        aa_test_tfmatp_cmp("2tfmat 3", T.data, T3.data, 1e-9);
        aa_test_tfmatp_cmp("2tfmat 4", T.data, T4.data, 1e-9);
        aa_test_tfmatp_cmp("2tfmat 5", T.data, T5.data, 1e-9);
        aa_test_tfmatp_cmp("2tfmat 6", T.data, T6.data, 1e-9);

        aa_test_tfmatp_cmp("2tfmat 2/3", T2.data, T3.data, 0);
        aa_test_tfmatp_cmp("2tfmat 4/5", T4.data, T5.data, 0);
        aa_test_tfmatp_cmp("2tfmat 4/6", T4.data, T6.data, 0);
    }

    {  // To CV
        CmplxTran E0(E);
        CmplxTran E1(c,v);
        CmplxTran E2(c.c_cmplx(),v);
        CmplxTran E3(Theta,v);
        CmplxTran E4(theta,v);
        CmplxTran E5(R,v);
        CmplxTran E6(T);

        aa_test_cotr_cmp("2cotr 0", E.data, E0.data, 0);
        aa_test_cotr_cmp("2cotr 1", E.data, E1.data, 0);
        aa_test_cotr_cmp("2cotr 2", E.data, E2.data, 0);
        aa_test_cotr_cmp("2cotr 3", E.data, E3.data, 1e-9);
        aa_test_cotr_cmp("2cotr 4", E.data, E4.data, 1e-9);
        aa_test_cotr_cmp("2cotr 5", E.data, E5.data, 1e-9);
        aa_test_cotr_cmp("2cotr 6", E.data, E6.data, 1e-9);
    }
}
static void
test_tf_mul(const CmplxTran E[2],  const TfMatP T[2], Vec2 p) {
    CmplxTran E01 = T[0] * T[1];
    TfMatP T01 = E[0] * E[1];

    { // check equivalence
        CmplxTran E_T(T01);
        TfMatP T_E(E01);
        aa_test_tfmatp_cmp("tfmatp_mul", T01.data, T_E.data, 1e-9);
        aa_test_cotr_cmp("cotr_mul", E01.data, E_T.data, 1e-9);
    }

    { // check chaining
        Vec2 p_T01p = T01*p;
        Vec2 p_T1p = T[1]*p;
        Vec2 p_T0T1p = T[0] * p_T1p;

        Vec2 p_c01p = E01* p;
        Vec2 p_c1p = E[1]*p;
        Vec2 p_c0c1p = E[0]*p_c1p;

        aa_test_vec2_cmp("tf_mul/tf/mat-cv", p_T01p, p_c01p, 1e-9);
        aa_test_vec2_cmp("tf_mul/tf/mat-chain", p_T01p, p_T0T1p, 1e-9);
        aa_test_vec2_cmp("tf_mul/tf/cv-chain", p_c01p, p_c0c1p, 1e-9);
        aa_test_vec2_cmp("tf_mul/tf/mat-cv-chain", p_T1p, p_c1p, 1e-9);
    }
}

static void
test_tf_inv(const CmplxTran &E, const TfMatP &T)
{
    CmplxTran Ei = E.inv();
    TfMatP Ti = T.inv();

    {  // check equivalence
        CmplxTran Ei_T(Ti);
        TfMatP Ti_E(Ei);
        aa_test_tfmatp_cmp("tfmatp_inv", Ti.data, Ti_E.data, 1e-9);
        aa_test_cotr_cmp("cotr_inv", Ei.data, Ei_T.data, 1e-9);
    }

    // check identity
    {
        double cT_ident[] = AA_TF_TFMATP_IDENT_INITIALIZER;
        static_assert(sizeof(cT_ident) == 6 * sizeof(double));
        TfMatP TTi = T * Ti, TiT = Ti * T, T_ident;

        aa_test_tfmatp_cmp("tfmatp ident", cT_ident, T_ident.data, 0);
        aa_test_tfmatp_cmp("tfmatp_inv ident 0", T_ident.data, TTi.data, 1e-9);
        aa_test_tfmatp_cmp("tfmatp_inv ident 1", T_ident.data, TiT.data, 1e-9);
    }
    {
        CmplxTran EEi = E * Ei, EiE = Ei * E, E_ident;
        aa_test_cotr_cmp("cotr_inv ident 0", E_ident.data, EEi.data, 1e-9);
        aa_test_cotr_cmp("cotr_inv ident 1", E_ident.data, EiE.data, 1e-9);
    }
}

static void
test_vec2_constructors(const aa_tf_vec2 v)
{
    {
        Vec2 V1 = v;
        test_feq("Vec2(v).x()", V1.x(), AA_TF_VEC2_X(v), 0);
        test_feq("Vec2(v).y()", V1.y(), AA_TF_VEC2_Y(v), 0);
    }

    {
        Vec2 V1(1,2);
        test_feq("Vec2 = v, x", V1.x(), 1, 0);
        test_feq("Vec2 = v, y", V1.y(), 2, 0);
        V1 = v;
        test_feq("Vec2 = v, x", V1.x(), AA_TF_VEC2_X(v), 0);
        test_feq("Vec2 = v, y", V1.y(), AA_TF_VEC2_Y(v), 0);
    }

    {
        Vec2 V1(v);
        aa_tf_vec2 v1 = V1;
        test_feq("Vec2 = v, x", V1.x(), AA_TF_VEC2_X(v1), 0);
        test_feq("Vec2 = v, y", V1.y(), AA_TF_VEC2_Y(v1), 0);
    }

    {
        Vec2 V1(v);
        Vec2 V2 = {V1.x(), V2.y()};
        test_feq("Vec2 = v, x", V1.x(), V1.x(), 0);
        test_feq("Vec2 = v, y", V1.y(), V1.y(), 0);
    }
}

static void
test_cmplx_constructors(const aa_tf_cmplx c)
{
    {
        Cmplx C1 = c;
        test_feq("Cmplx(c).real()", C1.real(), AA_TF_CMPLX_REAL(c), 0);
        test_feq("Cmplx(c).y()", C1.imag(), AA_TF_CMPLX_IMAG(c), 0);
    }

    {
        Cmplx C1(1,2);
        test_feq("Cmplx = c, real", C1.real(), 1, 0);
        test_feq("Cmplx = c, imag", C1.imag(), 2, 0);
        C1 = c;
        test_feq("Cmplx = c, real", C1.real(), AA_TF_CMPLX_REAL(c), 0);
        test_feq("Cmplx = c, imag", C1.imag(), AA_TF_CMPLX_IMAG(c), 0);
    }

    {
        Cmplx C1(c);
        aa_tf_cmplx c1 = C1;
        test_feq("Cmplx = c, real", C1.real(), AA_TF_CMPLX_REAL(c1), 0);
        test_feq("Cmplx = c, imag", C1.imag(), AA_TF_CMPLX_IMAG(c1), 0);
    }
}

int main(int argc, char **argv)
{
    aa_test_args(argc, argv);
    aa_test_ulimit();

    /* Tests */
    for (int i = 0; i < 1000; i++) {
        Vec2 p[2];
        double theta[2];
        RotMatP R[2];
        Cmplx c[2];

        /* Rotation */
        for (int k = 0; k < 2; k++) {
            p[k] = rand_vec();
            theta[k] = rand_angle();
            c[k] = Cmplx::from_angle(theta[k]);
            R[k] = RotMatP(theta[k]);
        }

        for (int k = 0; k < 2; k++) {
            test_rotconv(theta[k], c[k], R[k]);
            test_vec(p[k]);
            test_ln(theta[k], c[k], R[k]);
            test_rot(c[k], R[k], p[k]);
            test_inv(theta[k], c[k], R[k]);

            test_vec2_constructors(p[k]);
            test_cmplx_constructors(c[k]);
        }

        test_mul(theta, c, R);

        /* Transformation */
        TfMatP T[2];
        CmplxTran E[2];
        Vec2 v[2];

        /* Random transforms, same rotations, new translation */
        for (int k = 0; k < 2; k++) {
            v[k] = rand_vec();
            T[k] = TfMatP(R[k], v[k]);
            E[k] = CmplxTran(c[k], v[k]);
        }

        for (int k = 0; k < 2; k++) {
            test_tf_conv(theta[k], c[k], v[k], R[k], T[k]);
            test_tf(E[k], T[k], p[k]);
            test_tf_inv(E[k], T[k]);
            test_tf_mul(E, T, p[k]);
        }
    }

    return 0;
}
