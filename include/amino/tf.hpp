/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#ifndef AA_TF_HPP
#define AA_TF_HPP

namespace amino {

struct DualQuat : aa_tf_duqu {
    DualQuat() {}

    DualQuat(const double S[8]) {
        memcpy(this->data, S, 8*sizeof(S[0]));
    }
    DualQuat(const double q[4], const double v[3]) {
        from_qv(q,v);
    }

    void from_qv(const double q[4], const double v[3]) {
        aa_tf_qv2duqu(q,v,this->data);
    }
    void from_tfmat(const double T[12] ) {
        aa_tf_tfmat2duqu(T,this->data);
    }
};


struct QuatVec : aa_tf_qv {
    QuatVec() {}

    QuatVec(const double a_r[4], const double a_v[3])
    {
        memcpy(this->r.data, a_r, 4*sizeof(this->r.data[0]));
        memcpy(this->v.data, a_v, 3*sizeof(this->v.data[0]));
    }
    QuatVec(const struct aa_tf_duqu *S) {
        from_duqu(S->data);
    }
    QuatVec(const struct aa_tf_duqu S) {
        from_duqu(S.data);
    }

    void from_duqu(const double S[8]) {
        aa_tf_duqu2qv( S, this->r.data, this->v.data );
    }
    void from_tfmat(const double T[12]) {
        aa_tf_tfmat2duqu( T, this->data );
    }
};

}

#endif //AA_MEM_HPP
