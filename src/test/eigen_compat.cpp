/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
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

#include <cblas.h>
#include <stdlib.h>

// #include <iostream>

#include "amino.h"

#include "amino.hpp"
#include "amino/tf.hpp"


#include "amino/eigen_compat.hpp"

#include "amino/test.h"

using namespace amino;

static void s_aveq3(const struct aa_tf_vec3 *a, const ::Eigen::Vector3d *b,
                    double tol)
{
    afeq(a->x, b->x(), tol);
    afeq(a->y, b->y(), tol);
    afeq(a->z, b->z(), tol);
}

void test_quat(const Quat &aq)
{
    // to eigen
    ::Eigen::Quaternion<double> eq;
    ::amino::conv(&aq, &eq);

    afeq(aq.x, eq.x(), 1e-6);
    afeq(aq.y, eq.y(), 1e-6);
    afeq(aq.z, eq.z(), 1e-6);
    afeq(aq.w, eq.w(), 1e-6);
}

void test_rotmat(const Quat &h, const Vec3 &p)
{
    // construct amino rotmat
    RotMat R(h);
    Vec3 q = R * p;
    Vec3 qh = h.rotate(p);
    aveq("quat/rotmat rotate", 3, q.data, qh.data, 1e-9);

    // convert vectors
    ::Eigen::Vector3d ep, eq;
    ::amino::conv(&p, &ep);
    ::amino::conv(&q, &eq);
    s_aveq3(&p, &ep, 1e-6);
    s_aveq3(&q, &eq, 1e-6);

    ::Eigen::Matrix<double, 3, 3> eR;
    ::amino::conv(&R, &eR);
    ::Eigen::Vector3d eqt = eR * ep;
    s_aveq3(&q, &eqt, 1e-6);
}

void test_tf(const QuatTran &E, const Vec3 &p)
{
    TfMat T(E);

    Vec3 qT = T.transform(p);
    Vec3 qE = E.transform(p);

    aveq("qutr/tf", 3, qT.data, qE.data, 1e-6);

    ::Eigen::Vector3d ep;
    ::amino::conv(&p, &ep);
    ::Eigen::Transform<double, 3, ::Eigen::Isometry> eT;
    ::amino::conv(&E, &eT);
    ::Eigen::Vector3d eqt = eT * ep;

    s_aveq3(&qE, &eqt, 1e-6);
}

int main(void)
{
    time_t seed = time(NULL);
    printf("seed: %ld\n", seed);

    Vec3 v(1, 2, 3);
    {
        Quat aq;
        aa_tf_qurand(aq.data);
        test_quat(aq);
        test_rotmat(aq, v);
    }

    {
        QuatTran E;
        aa_tf_qutr_rand(E.data);
        test_tf(E, v);
    }

    printf("eigen_compat: OK\n");
    return 0;
}
