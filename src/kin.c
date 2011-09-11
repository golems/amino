/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
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


#include <amino.h>


AA_API int aa_kin_planar2_ik_theta2( const double l[2],
                                     const double x[2],
                                     double theta_a[2],
                                     double theta_b[2] ) {

    // help out the optimizer
    const double x1=x[0], x2=x[1], l1=l[0], l2=l[1];

    theta_a[0] = atan2((x2*x2*x2*x2+x2*x2*x1*x1+x1*sqrt(-x2*x2*(x2*x2+x1*x1-l1*l1+2.0*l2*l1-l2*l2)*(x2*x2+x1*x1-l1*l1-2.0*l2*l1-l2*l2))-l2*l2*x2*x2+x2*x2*l1*l1)/(x2*x2+x1*x1)/x2/l1, (x2*x2*x1+x1*x1*x1-l2*l2*x1+x1*l1*l1-sqrt(-x2*x2*(x2*x2+x1*x1-l1*l1+2.0*l2*l1-l2*l2)*(x2*x2+x1*x1-l1*l1-2.0*l2*l1-l2*l2)))/(x2*x2+x1*x1)/l1);

    theta_b[0] = atan2((x2*x2*x2*x2+x2*x2*x1*x1-x1*sqrt(-x2*x2*(x2*x2+x1*x1-l1*l1+2.0*l2*l1-l2*l2)*(x2*x2+x1*x1-l1*l1-2.0*l2*l1-l2*l2))-l2*l2*x2*x2+x2*x2*l1*l1)/(x2*x2+x1*x1)/x2/l1, (x2*x2*x1+x1*x1*x1-l2*l2*x1+x1*l1*l1+sqrt(-x2*x2*(x2*x2+x1*x1-l1*l1+2.0*l2*l1-l2*l2)*(x2*x2+x1*x1-l1*l1-2.0*l2*l1-l2*l2)))/(x2*x2+x1*x1)/l1);

      theta_a[1] = atan2((x2*x2*x1*x1-x1*sqrt(-x2*x2*(x2*x2+x1*x1-l1*l1+2.0*l2*l1-l2*l2)*(x2*x2+x1*x1-l1*l1-2.0*l2*l1-l2*l2))+l2*l2*x2*x2+x2*x2*x2*x2-x2*x2*l1*l1)/(x2*x2+x1*x1)/l2/x2, (l2*l2*x1-x1*l1*l1+x2*x2*x1+x1*x1*x1+sqrt(-x2*x2*(x2*x2+x1*x1-l1*l1+2.0*l2*l1-l2*l2)*(x2*x2+x1*x1-l1*l1-2.0*l2*l1-l2*l2)))/(x2*x2+x1*x1)/l2);

      theta_b[1] = atan2((x2*x2*x1*x1+x1*sqrt(-x2*x2*(x2*x2+x1*x1-l1*l1+2.0*l2*l1-l2*l2)*(x2*x2+x1*x1-l1*l1-2.0*l2*l1-l2*l2))+l2*l2*x2*x2+x2*x2*x2*x2-x2*x2*l1*l1)/(x2*x2+x1*x1)/l2/x2, (l2*l2*x1-x1*l1*l1+x2*x2*x1+x1*x1*x1-sqrt(-x2*x2*(x2*x2+x1*x1-l1*l1+2.0*l2*l1-l2*l2)*(x2*x2+x1*x1-l1*l1-2.0*l2*l1-l2*l2)))/(x2*x2+x1*x1)/l2);

      return
          !( aa_feq( l1*cos(theta_a[0]) + l2*cos(theta_a[1]), x1, .001 ) &&
             aa_feq( l1*sin(theta_a[0]) + l2*sin(theta_a[1]), x2, .001 ) &&
             aa_feq( l1*cos(theta_b[0]) + l2*cos(theta_b[1]), x1, .001 ) &&
             aa_feq( l1*sin(theta_b[0]) + l2*sin(theta_b[1]), x2, .001 ) );
}
