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



struct Vec3 : aa_tf_vec3 {
    Vec3() {}

    Vec3( const struct aa_tf_duqu *S ) :  aa_tf_vec3(from_duqu(S->data))  {}
    Vec3( const struct aa_tf_duqu &S ) :  aa_tf_vec3(from_duqu(S.data))   {}
    Vec3( const struct aa_tf_tfmat *T ) : aa_tf_vec3(from_tfmat(T->data)) {}
    Vec3( const struct aa_tf_tfmat &T ) : aa_tf_vec3(from_tfmat(T.data))  {}

    static aa_tf_vec3 from_vec3( const double a_x[3] ) {
        aa_tf_vec3 V;
        memcpy( V.data, a_x, 3*sizeof(V.data[0]) );
        return V;
    }
    static aa_tf_vec3 from_duqu( const double S[8] ) {
        aa_tf_vec3 V;
        aa_tf_duqu_trans( S, V.data );
        return V;
    }
    static aa_tf_vec3 from_tfmat( const double T[12] ) {
        return from_vec3( T+9 );
    }
};

/*------------------------------*/
/*-------- ORIENTATIONS --------*/
/*------------------------------*/

struct Quat : aa_tf_quat {
    Quat() {}
    Quat( const aa_tf_quat *p ) :   aa_tf_quat(from_quat(p->data)) {}
    Quat( const aa_tf_quat &p ) :   aa_tf_quat(from_quat(p.data)) {}
    Quat( const aa_tf_rotmat *p ) : aa_tf_quat(from_rotmat(p->data)) {}
    Quat( const aa_tf_rotmat &p ) : aa_tf_quat(from_rotmat(p.data)) {}
    Quat( const aa_tf_axang *p ) :  aa_tf_quat(from_axang(p->data)) {}
    Quat( const aa_tf_axang &p ) :  aa_tf_quat(from_axang(p.data)) {}

    static aa_tf_quat from_quat( const double x[4] ) {
        aa_tf_quat y;
        memcpy(y.data, x, 4*sizeof(y.data[0]));
        return y;
    }
    static aa_tf_quat from_rotmat( const double x[9] ) {
        aa_tf_quat y;
        aa_tf_rotmat2quat(x, y.data);
        return y;
    }
    static aa_tf_quat from_axang( const double x[4] ) {
        aa_tf_quat y;
        aa_tf_axang2quat(x, y.data);
        return y;
    }
    static aa_tf_quat from_rotvec( const double x[3] ) {
        aa_tf_quat y;
        aa_tf_rotvec2quat(x, y.data);
        return y;
    }
};

struct RotMat : aa_tf_rotmat {
    RotMat() {}

    RotMat( const aa_tf_quat *p ) :   aa_tf_rotmat(from_quat(p->data)) {}
    RotMat( const aa_tf_quat &p ) :   aa_tf_rotmat(from_quat(p.data)) {}
    RotMat( const aa_tf_rotmat *p ) : aa_tf_rotmat(from_rotmat(p->data)) {}
    RotMat( const aa_tf_rotmat &p ) : aa_tf_rotmat(from_rotmat(p.data)) {}
    RotMat( const aa_tf_axang *p ) :  aa_tf_rotmat(from_axang(p->data)) {}
    RotMat( const aa_tf_axang &p ) :  aa_tf_rotmat(from_axang(p.data)) {}

    static aa_tf_rotmat from_quat( const double x[4] ) {
        aa_tf_rotmat y;
        aa_tf_quat2rotmat(x, y.data);
        return y;
    }
    static aa_tf_rotmat from_rotmat( const double x[9] ) {
        aa_tf_rotmat y;
        memcpy(y.data, x, 9*sizeof(y.data[0]));
        return y;
    }
    static aa_tf_rotmat from_axang( const double x[4] ) {
        aa_tf_rotmat y;
        aa_tf_axang2rotmat(x, y.data);
        return y;
    }
    static aa_tf_rotmat from_rotvec( const double x[3] ) {
        aa_tf_rotmat y;
        aa_tf_rotvec2rotmat(x, y.data);
        return y;
    }
};

struct AxisAngle : aa_tf_axang {
    AxisAngle() {}

    AxisAngle( const aa_tf_quat *p ) :   aa_tf_axang(from_quat(p->data)) {}
    AxisAngle( const aa_tf_quat &p ) :   aa_tf_axang(from_quat(p.data)) {}
    AxisAngle( const aa_tf_rotmat *p ) : aa_tf_axang(from_rotmat(p->data)) {}
    AxisAngle( const aa_tf_rotmat &p ) : aa_tf_axang(from_rotmat(p.data)) {}
    AxisAngle( const aa_tf_axang *p ) :  aa_tf_axang(from_axang(p->data)) {}
    AxisAngle( const aa_tf_axang &p ) :  aa_tf_axang(from_axang(p.data)) {}

    static aa_tf_axang from_quat( const double x[4] ) {
        aa_tf_axang y;
        aa_tf_quat2axang(x, y.data);
        return y;
    }
    static aa_tf_axang from_rotmat( const double x[9] ) {
        aa_tf_axang y;
        aa_tf_rotmat2axang(x, y.data);
        return y;
    }
    static aa_tf_axang from_axang( const double x[4] ) {
        aa_tf_axang y;
        memcpy(y.data, x, 4*sizeof(y.data[0]));
        return y;
    }
    static aa_tf_axang from_rotvec( const double x[3] ) {
        aa_tf_axang y;
        aa_tf_rotvec2axang(x, y.data);
        return y;
    }
};

/*-----------------------------*/
/*------ TRANSFORMATIONS ------*/
/*-----------------------------*/

struct DualQuat : aa_tf_duqu {
    DualQuat() {}

    DualQuat(const struct aa_tf_duqu *S) :  aa_tf_duqu(from_duqu(S->data))  {}
    DualQuat(const struct aa_tf_duqu &S) :  aa_tf_duqu(from_duqu(S.data))   {}
    DualQuat(const struct aa_tf_qv *S) :    aa_tf_duqu(from_qv(S->r.data, S->v.data)) {}
    DualQuat(const struct aa_tf_qv &S) :    aa_tf_duqu(from_qv(S.r.data, S.v.data))  {}
    DualQuat(const struct aa_tf_tfmat *T) : aa_tf_duqu(from_tfmat(T->data)) {}
    DualQuat(const struct aa_tf_tfmat &T) : aa_tf_duqu(from_tfmat(T.data))  {}

    static aa_tf_duqu from_duqu( const double s[8] ) {
        DualQuat S;
        memcpy(S.data, s, 8*sizeof(s[0]));
        return S;
    }
    static aa_tf_duqu from_qv(const double q[4], const double v[3]) {
        DualQuat S;
        aa_tf_qv2duqu(q,v,S.data);
        return S;
    }
    static aa_tf_duqu from_tfmat(const double T[12] ) {
        DualQuat S;
        aa_tf_tfmat2duqu(T,S.data);
        return S;
    }
};


struct QuatVec : aa_tf_qv {
    QuatVec() {}

    QuatVec(const struct aa_tf_qv *S) :    aa_tf_qv(from_qv(S->r.data, S->v.data)) {}
    QuatVec(const struct aa_tf_qv &S) :    aa_tf_qv(from_qv(S.r.data, S.v.data))  {}
    QuatVec(const struct aa_tf_duqu *S) :  aa_tf_qv(from_duqu(S->data))  {}
    QuatVec(const struct aa_tf_duqu &S) :  aa_tf_qv(from_duqu(S.data))   {}
    QuatVec(const struct aa_tf_tfmat *T) : aa_tf_qv(from_tfmat(T->data)) {}
    QuatVec(const struct aa_tf_tfmat &T) : aa_tf_qv(from_tfmat(T.data))  {}

    static aa_tf_qv from_qv(const double a_r[4], const double a_v[3])
    {
        QuatVec qv;
        memcpy(qv.r.data, a_r, 4*sizeof(qv.r.data[0]));
        memcpy(qv.v.data, a_v, 3*sizeof(qv.v.data[0]));
        return qv;
    }
    static aa_tf_qv from_duqu(const double s[8]) {
        QuatVec qv;
        aa_tf_duqu2qv( s, qv.r.data, qv.v.data );
        return qv;
    }
    static aa_tf_qv from_tfmat(const double t[12]) {
        QuatVec qv;
        aa_tf_tfmat2duqu( t, qv.data );
        return qv;
    }
};


struct TfMat : aa_tf_tfmat {
    TfMat() {}
    TfMat(const struct aa_tf_tfmat *T) : aa_tf_tfmat(from_tfmat(T->data)) {}
    TfMat(const struct aa_tf_tfmat &T) : aa_tf_tfmat(from_tfmat(T.data))  {}
    TfMat(const struct aa_tf_duqu *S) :  aa_tf_tfmat(from_duqu(S->data))  {}
    TfMat(const struct aa_tf_duqu &S) :  aa_tf_tfmat(from_duqu(S.data))   {}
    TfMat(const struct aa_tf_qv *S) :    aa_tf_tfmat(from_qv(S->r.data, S->v.data)) {}
    TfMat(const struct aa_tf_qv &S) :    aa_tf_tfmat(from_qv(S.r.data, S.v.data))  {}

    static aa_tf_tfmat from_duqu( const double s[8] ) {
        aa_tf_tfmat T;
        aa_tf_duqu2tfmat(s, T.data);
        return T;
    }
    static aa_tf_tfmat from_qv(const double q[4], const double v[3]) {
        aa_tf_tfmat T;
        aa_tf_qv2tfmat(q,v,T.data);
        return T;
    }
    static aa_tf_tfmat from_tfmat(const double t[12] ) {
        aa_tf_tfmat T;
        memcpy(T.data, t, 12*sizeof(T.data[0]));
        return T;
    }
};

}

#endif //AA_MEM_HPP
