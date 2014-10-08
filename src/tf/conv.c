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

#define _GNU_SOURCE
#include "amino.h"


AA_API void
aa_tf_duqu2tfmat( const double d[AA_RESTRICT 8], double T[AA_RESTRICT 12] )
{
    aa_tf_quat2rotmat( &d[AA_TF_DUQU_REAL], T );
    aa_tf_duqu_trans( d, T+9);
}

AA_API void
aa_tf_tfmat2duqu( const double T[AA_RESTRICT 12], double d[AA_RESTRICT 8] )
{
    double q[4];
    aa_tf_rotmat2quat( T, q );
    aa_tf_qv2duqu( q, T+9, d );
}

AA_API void
aa_tf_xangle2rotmat( double theta, double R[AA_RESTRICT 9] )
{
    double s = sin(theta), c = cos(theta);
    R[0] = 1;
    R[1] = 0;
    R[2] = 0;

    R[3] = 0;
    R[4] = c;
    R[5] = s;

    R[6] = 0;
    R[7] = -s;
    R[8] = c;
}

AA_API void
aa_tf_yangle2rotmat( double theta, double R[AA_RESTRICT 9] )
{
    double s = sin(theta), c = cos(theta);
    R[0] = c;
    R[1] = 0;
    R[2] = -s;

    R[3] = 0;
    R[4] = 1;
    R[5] = 0;

    R[6] = s;
    R[7] = 0;
    R[8] = c;
}

AA_API void
aa_tf_zangle2rotmat( double theta, double R[AA_RESTRICT 9] )
{
    double s = sin(theta), c = cos(theta);
    R[0] = c;
    R[1] = s;
    R[2] = 0;

    R[3] = -s;
    R[4] = c;
    R[5] = 0;

    R[6] = 0;
    R[7] = 0;
    R[8] = 1;
}

AA_API void
aa_tf_quat2rotmat( const double q[AA_RESTRICT 4], double R[AA_RESTRICT 9] )
{
    double b[3] = { 2*q[0],
                    2*q[1],
                    2*q[2] };
    double a[3] = { q[3] * b[0],
                    q[3] * b[1],
                    q[3] * b[2] };
    aa_tf_skewsym_scal_c( q, a, b, R );
}
