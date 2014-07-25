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

/* Transform */

AA_API void aa_tf_rotmat_rot( const double R[AA_RESTRICT 9],
                              const double p0[AA_RESTRICT 3],
                              double p1[AA_RESTRICT 3] )
{
    p1[0] =  R[0]*p0[0] + R[3]*p0[1] + R[6]*p0[2];
    p1[1] =  R[1]*p0[0] + R[4]*p0[1] + R[7]*p0[2];
    p1[2] =  R[2]*p0[0] + R[5]*p0[1] + R[8]*p0[2];
}

AA_API void aa_tf_tfmat_tf( const double T[AA_RESTRICT 12],
                      const double p0[AA_RESTRICT 3],
                      double p1[AA_RESTRICT 3] )
{
    aa_tf_tfmat2_tf( T,T+9, p0, p1 );
}

AA_API void aa_tf_tfmat2_tf( const double R[AA_RESTRICT 9],
                             const double v[AA_RESTRICT 3],
                             const double p0[AA_RESTRICT 3],
                             double p1[AA_RESTRICT 4] )
{

    p1[0] =  R[0]*p0[0] + R[3]*p0[1] + R[6]*p0[2] + v[0];
    p1[1] =  R[1]*p0[0] + R[4]*p0[1] + R[7]*p0[2] + v[1];
    p1[2] =  R[2]*p0[0] + R[5]*p0[1] + R[8]*p0[2] + v[2];
}


/* Multiply */
AA_API void aa_tf_rotmat_mul( const double R1[AA_RESTRICT 9],
                              const double R2[AA_RESTRICT 9],
                              double R3[AA_RESTRICT 9] )
{
    aa_tf_rotmat_rot( R1, R2, R3 );
    aa_tf_rotmat_rot( R1, R2+3, R3+3 );
    aa_tf_rotmat_rot( R1, R2+6, R3+6 );
}

AA_API void aa_tf_tfmat_mul( const double T0[AA_RESTRICT 12],
                             const double T1[AA_RESTRICT 12],
                             double T[AA_RESTRICT 12] )
{
    aa_tf_tfmat2_mul( T0, T0+9,
                      T1, T1+9,
                      T,  T+9 );
}

AA_API void aa_tf_tfmat2_mul( const double R0[AA_RESTRICT 9],
                              const double v0[AA_RESTRICT 3],
                              const double R1[AA_RESTRICT 9],
                              const double v1[AA_RESTRICT 3],
                              double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] )
{
    aa_tf_rotmat_mul(R0,R1,R);
    aa_tf_tfmat2_tf( R0, v0, v1, v );
}


/* Inverting Multiply */

/* AA_API void aa_tf_rotmat_imul( const double R0[AA_RESTRICT 9], */
/*                                const double R1[AA_RESTRICT 9], */
/*                                double R[AA_RESTRICT 9] ); */

/* AA_API void aa_tf_rotmat_muli( const double R0[AA_RESTRICT 9], */
/*                                const double R1[AA_RESTRICT 9], */
/*                                double R[AA_RESTRICT 9] ); */

/* AA_API void aa_tf_tfmat_imul( const double T0[AA_RESTRICT 12], */
/*                               const double T1[AA_RESTRICT 12], */
/*                               double T[AA_RESTRICT 12] ); */

/* AA_API void aa_tf_tfmat_muli( const double T0[AA_RESTRICT 12], */
/*                               const double T1[AA_RESTRICT 12], */
/*                               double T[AA_RESTRICT 12] ); */

/* AA_API void aa_tf_tfmat2_imul( const double R0[AA_RESTRICT 9], */
/*                                const double v0[AA_RESTRICT 3], */
/*                                const double R1[AA_RESTRICT 9], */
/*                                const double v1[AA_RESTRICT 3], */
/*                                double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] ); */

/* AA_API void aa_tf_tfmat2_muli( const double R0[AA_RESTRICT 9], */
/*                                const double v0[AA_RESTRICT 3], */
/*                                const double R1[AA_RESTRICT 9], */
/*                                const double v1[AA_RESTRICT 3], */
/*                                double R[AA_RESTRICT 9], double v[AA_RESTRICT 3] ); */
