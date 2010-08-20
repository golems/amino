/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef AMINO_TF_H
#define AMINO_TF_H

/**************/
/* Transforms */
/**************/

AA_CDECL void aa_tf_12( const double T[12], const double p0[3], double p1[3] );
//AA_CDECL void aa_tf_16( const double T[16], const double p0[4], double p1[4] );
AA_CDECL void aa_tf_93( const double R[9], const double v[3], const double p0[3], double p1[4] );
AA_CDECL void aa_tf_q3( const double quat[4], const double v[3], const double p0[3], double p1[4] );

AA_CDECL void aa_tf_12inv( double T[12] );
//AA_CDECL void aa_tf_16inv( double T[16] );
AA_CDECL void aa_tf_93inv( double R[9], double v[3] );
AA_CDECL void aa_tf_q3inv( double q[4], double v[3] );

AA_CDECL void aa_tf_12chain( const double T1[12], const double T1[12], double T[12] );
//AA_CDECL void aa_tf_16chain( double T1[16], const double T1[16], double T[16] );
AA_CDECL void aa_tf_93chain( const double R0[9], const double v0[3],
                             const double R1[9], const double v1[3],
                             double R[9], double v[3] );
AA_CDECL void aa_tf_q3chain( const double q0[4], const double v0[3],
                             const double q1[4], const double v1[3],
                             const double q[4], const double v[3] );

/** Normalize Quaternion */
AA_CDECL void aa_tf_qnorm( double q[4] );

/** Quaternion conjugate */
AA_CDECL void aa_tf_qconj( const double q[4], double r[4] );

/** Quaternion inverse */
AA_CDECL void aa_tf_qinv( const double q[4], double r[4] );

/** Quaternion addition. */
AA_CDECL void aa_tf_qadd( const double a[4], const double b[4], double c[4] );

/** Quaternion subtraction. */
AA_CDECL void aa_tf_qsub( const double a[4], const double b[4], double c[4] );

/** Quaternion multiplication. */
AA_CDECL void aa_tf_qmul( const double a[4], const double b[4], double c[4] );

/** Quaternion SLERP. */
AA_CDECL void aa_tf_qslerp( double t, const double a[4], const double b[4], double c[4] );

/** Quaternion to axis-angle. */
AA_CDECL void aa_tf_quat2axang( const double q[4], double axang[4] );


/** axis-angle to quaternion. */
AA_CDECL void aa_tf_axang2quat( const double axang[4], double q[4] );

AA_CDECL void aa_tf_axang_make( double x, double y, double z, double theta, double axang[4] );

AA_CDECL void aa_tf_axang_permute( const double aa0[4], double aa1[0], double aa2[4], double aa3[4] );

AA_CDECL void aa_tf_axang2rotvec( const double axang[4], double rotvec[3] );

AA_CDECL void aa_tf_rotvec2axang( const double rotvec[3], double axang[4] );

AA_CDECL void aa_tf_rotvec2quat( const double rotvec[3], double q[4] );
AA_CDECL void aa_tf_quat2rotvec( const double q[4], double rotvec[4] );


AA_CDECL void aa_tf_quat2rotmat( const double quat[4], double rotmat[9] );
AA_CDECL void aa_tf_rotmat2quat( const double rotmat[9], double quat[4] );

#endif //AMINO_TF_H
