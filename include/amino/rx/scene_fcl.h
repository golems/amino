/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifndef AMINO_RX_SCENE_FCL_H
#define AMINO_RX_SCENE_FCL_H

/**
 * @file scene_fcl.h
 * @brief FCL-specific collision checking
 */

/* Utility */
#ifdef __cplusplus

// #include <fcl/math/transform.h>

#include "amino/tf.hpp"
#include "amino/eigen_compat.hpp"

namespace amino {
namespace fcl {

typedef double fcl_scalar;

typedef ::fcl::CollisionGeometry<fcl_scalar> CollisionGeometry ;
typedef ::fcl::CollisionObject<fcl_scalar> CollisionObject ;
typedef ::fcl::Vector3<fcl_scalar> Vec3;
typedef ::fcl::OBBRSS<fcl_scalar> OBBRSS;

typedef ::fcl::Box<fcl_scalar> Box;
typedef ::fcl::Sphere<fcl_scalar> Sphere;
typedef ::fcl::Cylinder<fcl_scalar> Cylinder;

typedef ::fcl::CollisionRequest<fcl_scalar> CollisionRequest;
typedef ::fcl::CollisionResult<fcl_scalar> CollisionResult;
typedef ::fcl::Contact<fcl_scalar> Contact;
typedef ::fcl::BroadPhaseCollisionManager<fcl_scalar> BroadPhaseCollisionManager;
typedef ::fcl::DynamicAABBTreeCollisionManager<fcl_scalar> DynamicAABBTreeCollisionManager;

typedef ::fcl::DistanceRequest<fcl_scalar> DistanceRequest;
typedef ::fcl::DistanceResult<fcl_scalar> DistanceResult;



static inline ::fcl::Transform3<::amino::fcl::fcl_scalar>
qutr2fcltf( const double E[7] )
{
    const double *q = E + AA_TF_QUTR_Q;
    const double *v = E + AA_TF_QUTR_V;

    ::amino::QuatTran oE = QuatTran::from_qv(E);
    ::fcl::Transform3<::amino::fcl::fcl_scalar> result;
    ::amino::conv(&oE, &result);
    return result;


    // return ::fcl::Transform3f(::fcl::Quaternionf( q[AA_TF_QUAT_W],
    //                                                q[AA_TF_QUAT_X],
    //                                                q[AA_TF_QUAT_Y],
    //                                                q[AA_TF_QUAT_Z]),
    //                           ::fcl::Vec3f(v[0], v[1], v[2]));
}



} /* namespace fcl */
} /* namespace amino */


#endif /* __cplusplus */

#endif /*AMINO_RX_SCENE_FCL_H*/
