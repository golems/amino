/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
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

AA_API void
aa_tf_duqu_vel2twist( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                      double t[AA_RESTRICT 8] )
{
    AA_MEM_CPY( &t[REAL_XYZ], &dx[OMEGA], 3 );
    t[REAL_W] = 0;

    double p[3];
    aa_tf_duqu_trans(d, p );
    aa_tf_cross( p, &dx[OMEGA], &t[DUAL_XYZ] );
    FOR_VEC(i) t[DUAL_XYZ+i] += dx[V+i];
    t[DUAL_W] = 0;
}

AA_API void
aa_tf_duqu_twist2vel( const double d[AA_RESTRICT 8], const double t[AA_RESTRICT 8],
                                  double dx[AA_RESTRICT 6] )
{
    double p[3], pxw[3];
    AA_MEM_CPY( &dx[OMEGA], &t[XYZ], 3 );
    aa_tf_duqu_trans(d,p);
    aa_tf_cross(p, &t[REAL_XYZ], pxw );
    FOR_VEC(i) dx[V+i] = t[DUAL_XYZ+i] - pxw[i];
}

AA_API void
aa_tf_duqu_twist2diff( const double d[AA_RESTRICT 8], const double t[AA_RESTRICT 8],
                       double dd[AA_RESTRICT 8] )
{
    aa_tf_duqu_mul(t,d,dd);
    for( size_t i = 0; i < 8; i ++ ) dd[i] /= 2;
}

AA_API void
aa_tf_duqu_diff2twist( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                       double twist[AA_RESTRICT 8] )
{
    double dx[6];
    aa_tf_duqu_diff2vel(d, dd, dx);
    aa_tf_duqu_vel2twist(d, dx, twist);
}

AA_API void
aa_tf_duqu_vel2diff( const double d[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                     double dd[AA_RESTRICT 8] )
{
    double t[8];
    aa_tf_duqu_vel2twist( d, dx, t );
    aa_tf_duqu_twist2diff( d, t, dd );

}

AA_API void
aa_tf_duqu_diff2vel( const double d[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                     double dx[AA_RESTRICT 6] )
{
    double t1[4], t2[4];
    // rotation
    aa_tf_qdiff2vel( &d[REAL], &dd[REAL], &dx[OMEGA] );
    // translation
    // dx/dt = 2 * ( d_dual/dt conj(r) + d_dual conj(d_real/dt) )
    aa_tf_qmulc( &dd[DUAL], &d[REAL], t1 );
    aa_tf_qmulc( &d[DUAL], &dd[REAL], t2 );
    FOR_VEC(i) dx[V+i] = 2 * (t1[XYZ+i] + t2[XYZ+i]);
}

AA_API void
aa_tf_duqu_stwist( const double d0[AA_RESTRICT 8], const double twist[AA_RESTRICT 8],
                   double dt, double d1[AA_RESTRICT 6] )
{
    double twist1[8], etwist[8];
    for( size_t i = 0; i < 8; i ++ ) twist1[i] = dt/2 * twist[i];
    aa_tf_duqu_exp( twist1, etwist );
    aa_tf_duqu_mul( etwist, d0, d1 );
}

AA_API void
aa_tf_duqu_svel( const double d0[AA_RESTRICT 8], const double dx[AA_RESTRICT 6],
                             double dt, double d1[AA_RESTRICT 6] )
{
    double twist[8];
    aa_tf_duqu_vel2twist( d0, dx, twist );
    aa_tf_duqu_stwist(d0, twist, dt, d1 );
}

AA_API void
aa_tf_duqu_sdiff( const double d0[AA_RESTRICT 8], const double dd[AA_RESTRICT 8],
                  double dt, double d1[AA_RESTRICT 6] )
{
    double w[8];
    aa_tf_duqu_diff2twist(d0, dd, w);
    aa_tf_duqu_stwist(d0, w, dt, d1 );
}
