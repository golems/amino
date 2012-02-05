#if 0
/* Copyright (c) 2012, Georgia Tech Research Corporation
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
#endif

#if defined AA_LA_TYPE_DOUBLE

#define AA_LA_TYPE double
#define AA_CBLAS_NAME( name ) cblas_d ## name
#define AA_LAPACK_NAME( name ) d ## name ## _
#define AA_LAPACK_PREFIX_STR "D"
#define AA_CLA_NAME( name ) aa_cla_d ## name
#define AA_LA_NAME( name ) aa_la_d ## name
#define AA_LA_FDEC( rettype, name, ... )                                \
    AA_API rettype aa_la_d ## name ## _( __VA_ARGS__ );                 \
    static rettype (* const aa_la_d ## name)( __VA_ARGS__ ) = aa_la_d ## name ## _;

#elif defined AA_LA_TYPE_FLOAT

#define AA_LA_TYPE float
#define AA_CBLAS_NAME( name ) cblas_s ## name
#define AA_LAPACK_NAME( name ) s ## name ## _
#define AA_LAPACK_PREFIX_STR "S"
#define AA_CLA_NAME( name ) aa_cla_s ## name
#define AA_LA_NAME( name ) aa_la_s ## name
#define AA_LA_FDEC( rettype, name, ... )                                \
    AA_API rettype aa_la_s ## name ## _( __VA_ARGS__ );                 \
    static rettype (* const aa_la_s ## name)( __VA_ARGS__ ) = aa_la_s ## name ## _;

#else

#error "Need to define AA_LA_TYPE_?"

#endif
