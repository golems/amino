/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#ifndef AMINO_INTERNAL_H
#define AMINO_INTERNAL_H

#define XYZ AA_TF_QUAT_XYZ
#define W AA_TF_QUAT_W

#define REAL AA_TF_DUQU_REAL
#define DUAL AA_TF_DUQU_DUAL
#define REAL_XYZ AA_TF_DUQU_REAL_XYZ
#define DUAL_XYZ AA_TF_DUQU_DUAL_XYZ
#define REAL_W AA_TF_DUQU_REAL_W
#define DUAL_W AA_TF_DUQU_DUAL_W

#define OMEGA AA_TF_DX_W
#define V AA_TF_DX_V

#define FOR_VEC(i) for( size_t i = 0; i < 3; i ++ )
#define FOR_QUAT(i) for( size_t i = 0; i < 4; i ++ )
#define FOR_DUQU(i) for( size_t i = 0; i < 8; i ++ )

static inline double
dot3( const double a[AA_RESTRICT 3], const double b[AA_RESTRICT 3] )
{ return a[0]*b[0] + a[1]*b[1] + a[2]*b[2] ; }


#define DECLARE_QUAT_XYZW          \
    const size_t x = AA_TF_QUAT_X; \
    const size_t y = AA_TF_QUAT_Y; \
    const size_t z = AA_TF_QUAT_Z; \
    const size_t w = AA_TF_QUAT_W;

#endif /* AMINO_INTERNAL_H */
