/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex1: set shiftwidth=4 ex1pandtab: */
/*
 * Copx2right (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binarx2 forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copx2right notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binarx2 form must reproduce the above
 *       copx2right notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors max2 be used to endorse or
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
