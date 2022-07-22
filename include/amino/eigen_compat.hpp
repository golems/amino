/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
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
#ifndef AA_EIGEN_COMPAT_H
#define AA_EIGEN_COMPAT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace amino {

/** Convert an amino quaternion to an Eigen quaternion */
template <typename T>
void conv(const aa_tf_quat *src, ::Eigen::Quaternion<T> *dst)
{
    dst->x() = src->x;
    dst->y() = src->y;
    dst->z() = src->z;
    dst->w() = src->w;
}

/** Convert an amino vector to an Eigen translation */
template <typename T>
void conv(const aa_tf_vec3 *src, ::Eigen::Translation<T, 3> *dst)
{
    dst->x() = src->x;
    dst->y() = src->y;
    dst->z() = src->z;
}

/** Convert an amino vector to an Eigen column vector */
template <typename T>
void conv(const aa_tf_vec3 *src, ::Eigen::Matrix<T, 3, 1> *dst)
{
    dst->x() = src->x;
    dst->y() = src->y;
    dst->z() = src->z;
}

/** Convert an amino rotation matrix to an Eigen matrix */
template <typename T>
void conv(const aa_tf_rotmat *src, ::Eigen::Matrix<T, 3, 3> *dst)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            (*dst)(i, j) = src->data[j * 3 + i];
        }
    }
}

/** Convert an amino transformation matrix to an Eigen Isometry */
template <typename T>
void conv(const aa_tf_tfmat *src,
          ::Eigen::Transform<T, 3, ::Eigen::Isometry> *dst)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            (*dst)(i, j) = src->data[j * 3 + i];
        }
    }
}

/** Convert an amino transformation matrix to an Eigen Affine */
template <typename T>
void conv(const aa_tf_tfmat *src,
          ::Eigen::Transform<T, 3, ::Eigen::Affine> *dst)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            (*dst)(i, j) = src->data[j * 3 + i];
        }
    }
}

/** Convert an amino quaternion-translation to an Eigen Affine */
template <typename T>
void conv(const aa_tf_qv *src, ::Eigen::Transform<T, 3, ::Eigen::Affine> *dst)
{
    struct aa_tf_tfmat tmp;
    aa_tf_qutr2tfmat(src->data, tmp.data);
    ::amino::conv(&tmp, dst);
}

/** Convert an amino quaternion-translation to an Eigen Isometry */
template <typename T>
void conv(const aa_tf_qv *src, ::Eigen::Transform<T, 3, ::Eigen::Isometry> *dst)
{
    struct aa_tf_tfmat tmp;
    aa_tf_qutr2tfmat(src->data, tmp.data);
    ::amino::conv(&tmp, dst);
}

}  // namespace amino

#endif  // AA_EIGEN_COMPAT_H
