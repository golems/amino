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
    Vec3( double _x, double _y, double _z ) : aa_tf_vec3(from_xyz(_x,_y,_z)) {}
    Vec3( const double *_xyz ) : aa_tf_vec3(from_vec3(_xyz)) {}

    static aa_tf_vec3 from_xyz( double x, double y, double z ) {
        aa_tf_vec3 V;
        V.x = x;
        V.y = y;
        V.z = z;
        return V;
    }

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

struct XAngle {
    double value;
    XAngle(double v) : value(v) {}
};

struct YAngle {
    double value;
    YAngle(double v) : value(v) {}
};

struct ZAngle {
    double value;
    ZAngle(double v) : value(v) {}
};

struct Quat : aa_tf_quat {
    Quat() {}
    Quat( const aa_tf_quat *p ) :   aa_tf_quat(from_quat(p->data)) {}
    Quat( const aa_tf_quat &p ) :   aa_tf_quat(from_quat(p.data)) {}
    Quat( const aa_tf_rotmat *p ) : aa_tf_quat(from_rotmat(p->data)) {}
    Quat( const aa_tf_rotmat &p ) : aa_tf_quat(from_rotmat(p.data)) {}
    Quat( const aa_tf_axang *p ) :  aa_tf_quat(from_axang(p->data)) {}
    Quat( const aa_tf_axang &p ) :  aa_tf_quat(from_axang(p.data)) {}
    Quat( const XAngle &p ) :  aa_tf_quat(from_xangle(p.value)) {}
    Quat( const YAngle &p ) :  aa_tf_quat(from_yangle(p.value)) {}
    Quat( const ZAngle &p ) :  aa_tf_quat(from_zangle(p.value)) {}

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
    static aa_tf_quat from_axang( const double a[3], double angle ) {
        double x[4] = {a[0], a[1], a[2], angle};
        return from_axang(x);
    }
    static aa_tf_quat from_rotvec( const double x[3] ) {
        aa_tf_quat y;
        aa_tf_rotvec2quat(x, y.data);
        return y;
    }
    static aa_tf_quat from_xangle( const double v ) {
        aa_tf_quat y;
        aa_tf_xangle2quat(v, y.data);
        return y;
    }
    static aa_tf_quat from_yangle( const double v ) {
        aa_tf_quat y;
        aa_tf_yangle2quat(v, y.data);
        return y;
    }
    static aa_tf_quat from_zangle( const double v ) {
        aa_tf_quat y;
        aa_tf_zangle2quat(v, y.data);
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
    RotMat( const XAngle &p ) :       aa_tf_rotmat(from_xangle(p.value)) {}
    RotMat( const YAngle &p ) :       aa_tf_rotmat(from_yangle(p.value)) {}
    RotMat( const ZAngle &p ) :       aa_tf_rotmat(from_zangle(p.value)) {}

    RotMat( double r11, double r12, double r13,
            double r21, double r22, double r23,
            double r31, double r32, double r33 ) :
        aa_tf_rotmat(from_rotmat(r11, r12, r13,
                                 r21, r22, r23,
                                 r31, r32, r33))
    {}


    static aa_tf_rotmat from_rotmat( double r11, double r12, double r13,
                                     double r21, double r22, double r23,
                                     double r31, double r32, double r33 )
    {
        aa_tf_rotmat R;
        R.data[0] = r11;
        R.data[1] = r21;
        R.data[2] = r31;

        R.data[3] = r12;
        R.data[4] = r22;
        R.data[5] = r32;

        R.data[6] = r13;
        R.data[7] = r23;
        R.data[8] = r33;
        return R;
    }


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

    static aa_tf_rotmat from_xangle( const double v ) {
        aa_tf_rotmat y;
        aa_tf_xangle2rotmat(v, y.data);
        return y;
    }
    static aa_tf_rotmat from_yangle( const double v ) {
        aa_tf_rotmat y;
        aa_tf_yangle2rotmat(v, y.data);
        return y;
    }
    static aa_tf_rotmat from_zangle( const double v ) {
        aa_tf_rotmat y;
        aa_tf_zangle2rotmat(v, y.data);
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
    AxisAngle( double x, double y, double z, double theta ) :
        aa_tf_axang(from_axang(x,y,z,theta))
    {}
    AxisAngle( const double *_axis, double _angle ) :
        aa_tf_axang(from_axang(_axis,_angle))
    {}

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

    static aa_tf_axang from_axang( double x, double y, double z, double theta ) {
        double n = sqrt( x*x + y*y + z*z );
        double a[4] = {x/n, y/n, z/n, theta};
        return from_axang(a);
    }
    static aa_tf_axang from_axang( const double x[4] ) {
        aa_tf_axang y;
        memcpy(y.data, x, 4*sizeof(y.data[0]));
        return y;
    }
    static aa_tf_axang from_axang( const double axis[3], double angle ) {
        aa_tf_axang y;
        y.axis.x = axis[0];
        y.axis.y = axis[1];
        y.axis.z = axis[2];
        y.angle = angle;
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

    DualQuat(const double *S) :  aa_tf_duqu(from_duqu(S))  {}
    DualQuat(const struct aa_tf_duqu *S) :  aa_tf_duqu(from_duqu(S->data))  {}
    DualQuat(const struct aa_tf_duqu &S) :  aa_tf_duqu(from_duqu(S.data))   {}
    DualQuat(const struct aa_tf_qv *S) :    aa_tf_duqu(from_qv(S->r.data, S->v.data)) {}
    DualQuat(const struct aa_tf_qv &S) :    aa_tf_duqu(from_qv(S.r.data, S.v.data))  {}
    DualQuat(const struct aa_tf_quat *r, const struct aa_tf_vec3 *v) :
        aa_tf_duqu(from_qv(r->data, v->data))
    {}
    DualQuat(const struct aa_tf_quat &r, const struct aa_tf_vec3 &v) :
        aa_tf_duqu(from_qv(r.data, v.data))
    {}
    DualQuat(const struct aa_tf_tfmat *T) : aa_tf_duqu(from_tfmat(T->data)) {}
    DualQuat(const struct aa_tf_tfmat &T) : aa_tf_duqu(from_tfmat(T.data))  {}

    DualQuat(const XAngle &r, const struct aa_tf_vec3 &v) : aa_tf_duqu(from_xxyz(r.value, v.x, v.y, v.z)) {}
    DualQuat(const YAngle &r, const struct aa_tf_vec3 &v) : aa_tf_duqu(from_yxyz(r.value, v.x, v.y, v.z)) {}
    DualQuat(const ZAngle &r, const struct aa_tf_vec3 &v) : aa_tf_duqu(from_zxyz(r.value, v.x, v.y, v.z)) {}
    DualQuat(const struct aa_tf_vec3 &v) : aa_tf_duqu(from_xyz(v.x, v.y, v.z)) {}

    DualQuat(const aa_tf_axang &r, const struct aa_tf_vec3 &v) :
        aa_tf_duqu(from_qv(Quat::from_axang(r.data).data,
                           v.data))
    {}

    aa_tf_vec3 translation() {
        aa_tf_vec3 V;
        aa_tf_duqu_trans( this->data, V.data );
        return V;
    }

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

    static aa_tf_duqu from_xxyz(double theta, double x, double y, double z) {
        DualQuat S;
        aa_tf_xxyz2duqu(theta, x, y, z, S.data);
        return S;
    }
    static aa_tf_duqu from_yxyz(double theta, double x, double y, double z) {
        DualQuat S;
        aa_tf_yxyz2duqu(theta, x, y, z, S.data);
        return S;
    }
    static aa_tf_duqu from_zxyz(double theta, double x, double y, double z) {
        DualQuat S;
        aa_tf_zxyz2duqu(theta, x, y, z, S.data);
        return S;
    }
    static aa_tf_duqu from_xyz(double x, double y, double z) {
        DualQuat S;
        aa_tf_xyz2duqu(x, y, z, S.data);
        return S;
    }
};


struct QuatVec : aa_tf_qv {
    QuatVec() {}

    QuatVec(const struct aa_tf_qv *S) :    aa_tf_qv(from_qv(S->r.data, S->v.data)) {}
    QuatVec(const struct aa_tf_qv &S) :    aa_tf_qv(from_qv(S.r.data, S.v.data))  {}
    QuatVec(const struct aa_tf_quat *_r, const struct aa_tf_vec3 *_v) :
        aa_tf_qv(from_qv(_r->data, _v->data))
    {}
    QuatVec(const struct aa_tf_quat &_r, const struct aa_tf_vec3 &_v) :
        aa_tf_qv(from_qv(_r.data, _v.data))
    {}
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
    TfMat(const struct aa_tf_quat *_r, const struct aa_tf_vec3 *_v) :
        aa_tf_tfmat(from_qv(_r->data, _v->data))
    {}
    TfMat(const struct aa_tf_quat &_r, const struct aa_tf_vec3 &_v) :
        aa_tf_tfmat(from_qv(_r.data, _v.data))
    {}

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

/*---- OPERATORS -----*/


/* Vec3 */
static inline struct aa_tf_vec3
operator+(const struct aa_tf_vec3 &a, const struct aa_tf_vec3 &b) {
    struct aa_tf_vec3 c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

static inline struct aa_tf_vec3
operator/(const struct aa_tf_vec3 &a, double b ) {
    struct aa_tf_vec3 c;
    c.x = a.x / b;
    c.y = a.y / b;
    c.z = a.z / b;
    return c;
}


static inline struct aa_tf_rotmat
operator*(const struct aa_tf_rotmat &a, const struct aa_tf_rotmat &b) {
    struct aa_tf_rotmat c;
    aa_tf_9mul(a.data, b.data, c.data);
    return c;
}

static inline struct aa_tf_quat
operator*(const struct aa_tf_quat &a, const struct aa_tf_quat &b) {
    struct aa_tf_quat c;
    aa_tf_qmul(a.data, b.data, c.data);
    return c;
}

static inline struct aa_tf_tfmat
operator*(const struct aa_tf_tfmat &a, const struct aa_tf_tfmat &b) {
    struct aa_tf_tfmat c;
    aa_tf_12chain(a.data, b.data, c.data);
    return c;
}

static inline struct aa_tf_duqu
operator*(const struct aa_tf_duqu &a, const struct aa_tf_duqu &b) {
    struct aa_tf_duqu c;
    aa_tf_duqu_mul(a.data, b.data, c.data);
    return c;
}

static inline struct aa_tf_qv
operator*(const struct aa_tf_qv &a, const struct aa_tf_qv &b) {
    struct aa_tf_qv c;
    aa_tf_qv_chain(a.r.data, a.v.data,
                   b.r.data, b.v.data,
                   c.r.data, c.v.data);
    return c;
}

};

#endif //AA_MEM_HPP
