/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2022, Colorado School of Mines
 * All rights reserved.
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

#ifndef AMINO_PLANAR_HPP
#define AMINO_PLANAR_HPP

/**
 * @file planar.hpp
 *
 * Object types for SE(2) representations.
 *
 * The class definitions in this file provide convenience methods to
 * simplify type conversions between various orientation and
 * transformation representations.  The underlying operatations (and
 * many others) are declared in tfp.h.
 *
 * \sa tfp.h for low level definitions of SE(3) operations.
 */

#include <complex>

namespace amino {

/**
 * Base type for complex numbers and 2D vectors.
 */
struct BaseCmplx : ::std::complex<double> {
    /** Convenience typedef for C++ complex numbers. */
    typedef ::std::complex<double> type;

    /** Construct a zero-valued complex number. */
    BaseCmplx() {}

    /** Construct from real and imaginary values. */
    BaseCmplx(double real, double imag) : type(real, imag) {}

    /** Construct from a C++ complex number. */
    BaseCmplx(type c) : type(c) {}

    /** Construct from a C complex number. */
    BaseCmplx(const aa_tf_cmplx& v) : type(from(v)) {}

    /** Convert to a C complex number. */
    static aa_tf_cmplx c_cmplx(type c) {
        return *reinterpret_cast<const aa_tf_cmplx*>(&c);
    }

    /** Convert to a C complex number. */
    aa_tf_cmplx c_cmplx() const {
        return c_cmplx(*this);
    }

    /** Implicit conversion to a C complex number. */
    operator aa_tf_cmplx() const { return c_cmplx(); }

    /** Extract real part of a C complex number. */
    static double c_real (const aa_tf_cmplx& v) {
        return reinterpret_cast<const type*>(&v)->real();
    }

    /** Extract imaginary part of a C complex number. */
    static double c_imag (const aa_tf_cmplx& v) {
        return reinterpret_cast<const type*>(&v)->imag();
    }

    /** Convert C complex number to C++ complex number. */
    static type from(const aa_tf_cmplx& v) {
        return type(c_real(v), c_imag(v));
    }

protected:
    /** Assignable reference to the real part. */
    double& real_ref() { return reinterpret_cast<double(&)[2]>(*this)[0]; }

    /** Assignable reference to the imaginary part. */
    double& imag_ref() { return reinterpret_cast<double(&)[2]>(*this)[1]; }
};

/**
 * A 2D vector represented using a complex number.
 */
struct Vec2 : BaseCmplx {
    /** Construct a zero vector. */
    Vec2() {}

    /** Construct from x and y components. */
    Vec2(double x, double y) : BaseCmplx(x, y) {}

    /** Construct from a C complex number. */
    Vec2(aa_tf_vec2 v) : BaseCmplx(v) {}

    /** Return value of the x component. */
    double x() const { return real(); }

    /** Return value of the y component. */
    double y() const { return imag(); }

    /** Return assignable reference to the x component. */
    double& x() { return real_ref(); }

    /** Return assignable reference to the y component. */
    double& y() { return imag_ref(); }
};

/** A planar rotation angle. */
struct AngleP {
    /** Zero rotation. */
    AngleP() {}

    /** Construct rotation of angle theta. */
    AngleP(double theta) : angle_(theta) {}

    /** Construct rotation angle of the C complex number. */
    AngleP(aa_tf_cmplx c) : angle_(aa_tf_cmplx2angle(c)) {}

    /** Construct rotation angle of the C++ complex number. */
    AngleP(BaseCmplx::type c) : angle_(atan2(c.imag(), c.real())) {}

    /** Construct rotation angle from a rotation matrix. */
    AngleP(const aa_tf_rotmatp* R) : angle_(aa_tf_rotmatp2angle(R->data)) {}

    /** Construct rotation angle from a rotation matrix. */
    AngleP(const aa_tf_rotmatp& R) : angle_(aa_tf_rotmatp2angle(R.data)) {}

    /** Return the rotation angle. */
    double angle() const { return angle_; };

    /** Return assignable reference to the rotation angle. */
    double& angle() { return angle_; };

   private:
    double angle_;
};

/** A complex number. */
struct Cmplx : BaseCmplx {
    /** Construct zero-valued complex number. */
    Cmplx() {}

    /** Construct from real and imaginary components. */
    Cmplx(double real, double imag) : BaseCmplx(real, imag) {}

    /** Construct from a C++ complex number. */
    Cmplx(type c) : BaseCmplx(c) {}

    /** Construct from a C complex number. */
    Cmplx(aa_tf_cmplx c) : BaseCmplx(c) {}

    /** Construct from a rotation matrix. */
    Cmplx(const aa_tf_rotmatp& R) : BaseCmplx(from(R)) {}

    /** Construct from a rotation matrix. */
    Cmplx(const aa_tf_rotmatp* R) : BaseCmplx(from(R)) {}

    /** Construct from a rotation angle. */
    Cmplx(const AngleP& a) : BaseCmplx(from(a)) {}

    /** Construct from a rotation angle. */
    Cmplx(const AngleP* a) : BaseCmplx(from(a)) {}

    /** Create complex number from a rotation angle. */
    static Cmplx from_angle(double theta) {
        return aa_tf_angle2cmplx(theta);
    }

    /** Create complex number from a rotation matrix. */
    static Cmplx from_rotmatp(const double *R) {
        return aa_tf_rotmatp2cmplx(R);
    }

    /** Create complex number from a rotation matrix. */
    static Cmplx from(const aa_tf_rotmatp* R) { return from_rotmatp(R->data); }
    /** Create complex number from a rotation matrix. */
    static Cmplx from(const aa_tf_rotmatp& R) { return from_rotmatp(R.data); }
    /** Create complex number from a rotation angle. */
    static Cmplx from(const AngleP& a) { return from_angle(a.angle()); }
    /** Create complex number from a rotation angle. */
    static Cmplx from(const AngleP* a) { return from_angle(a->angle()); }

    /** Return the identity element. */
    static aa_tf_cmplx ident() { return 1; }

    /** Return the angle of the complex number. */
    double angle() const { return aa_tf_cmplx2angle(*this); }

    /** Return the logarithm of the complex number. */
    double uln() const { return aa_tf_culn(*this); }

    /** Return the complex conjugate. */
    Cmplx conj() const { return aa_tf_cconj(*this); }
};

/** A planar rotation matrix.
 *
 * Column-major order.
 */
struct RotMatP : aa_tf_rotmatp {
    /** Construct the identity rotation. */
    RotMatP () : aa_tf_rotmatp(ident())  {}

    /** Construct from another rotation matrix. */
    RotMatP(const aa_tf_rotmatp& R) : aa_tf_rotmatp(R) {}
    /** Construct from another rotation matrix. */
    RotMatP(const aa_tf_rotmatp* R) : aa_tf_rotmatp(*R) {}
    /** Construct from another rotation matrix. */
    RotMatP(double r11, double r12, double r21, double r22)
        : aa_tf_rotmatp(from_rotmatp(r11, r12, r21, r22))
    {}

    /** Construct from a C++ complex number. */
    RotMatP(Cmplx::type c) : aa_tf_rotmatp(from(c)) {}
    /** Construct from a C complex number. */
    RotMatP(aa_tf_cmplx c) : aa_tf_rotmatp(from(c)) {}
    /** Construct from a rotation angle. */
    RotMatP(double angle) : aa_tf_rotmatp(from(angle)) {}
    /** Construct from a rotation angle. */
    RotMatP(const AngleP &angle) : aa_tf_rotmatp(from(angle)) {}
    /** Construct from a rotation angle. */
    RotMatP(const AngleP *angle) : aa_tf_rotmatp(from(angle)) {}

    /** Create a rotation matrix from a rotation angle. */
    static aa_tf_rotmatp from_angle(double theta) {
        aa_tf_rotmatp R;
        aa_tf_angle2rotmatp(theta, R.data);
        return R;
    }

    /** Create a rotation matrix from elements. */
    static aa_tf_rotmatp
    from_rotmatp (double r11, double r12, double r21, double r22)
    {
        aa_tf_rotmatp R;
        R.data[0] = r11;
        R.data[1] = r21;
        R.data[2] = r12;
        R.data[3] = r22;
        return R;
    }

    /** Create a rotation matrix from another rotation matrix. */
    static aa_tf_rotmatp
    from_rotmatp (const double *R) {
        return from_rotmatp(R[0], R[2], R[1], R[3]);
    }

    /** Create a rotation matrix from a complex number. */
    static aa_tf_rotmatp
    from_cmplx (aa_tf_cmplx c) {
        aa_tf_rotmatp R;
        aa_tf_cmplx2rotmatp(c, R.data);
        return R;
    }

    /** Create a rotation matrix from a complex number. */
    static aa_tf_rotmatp
    from_cmplx (Cmplx::type c) {
        return from_cmplx(Cmplx::c_cmplx(c));
    }

    /** Create a rotation matrix from a complex number. */
    static aa_tf_rotmatp from(aa_tf_cmplx c) { return from_cmplx(c); }
    /** Create a rotation matrix from a complex number. */
    static aa_tf_rotmatp from(Cmplx::type c) { return from_cmplx(c); }
    /** Create a rotation matrix from a rotation angle. */
    static aa_tf_rotmatp from(double angle) { return from_angle(angle); }
    /** Create a rotation matrix from a rotation angle. */
    static aa_tf_rotmatp from(const AngleP& angle) {
        return from_angle(angle.angle());
    }
    /** Create a rotation matrix from a rotation angle. */
    static aa_tf_rotmatp from(const AngleP* angle) { return from_angle(angle->angle()); }

    /** Create the identity rotation matrix. */
    static aa_tf_rotmatp ident() {
        static const double R[] = AA_TF_ROTMATP_IDENT_INITIALIZER;
        return from_rotmatp(R);
    }

    /** Return the rotation angle. */
    double angle() const { return aa_tf_rotmatp2angle(this->data); }

    /** Return the logarithm of the rotation matrix. */
    double uln() const { return aa_tf_rotmatp_uln(this->data); }

    /** Return the inverse of the rotation matrix. */
    aa_tf_rotmatp inv() const {
        aa_tf_rotmatp Ri;
        aa_tf_rotmatp_inv2(this->data, Ri.data);
        return Ri;
    }
};

/**
 * Planar transformation represented with a complex number and translation vector.
 */
struct CmplxTran : aa_tf_cv {
    /** Construct the identity transformation. */
    CmplxTran() : aa_tf_cv(ident()) {}
    /** Construct from a C complex number and translation vector. */
    CmplxTran(aa_tf_cmplx c_, aa_tf_vec2 v_) : aa_tf_cv(from(c_, v_)) {}
    /** Construct from a C++ complex number and translation vector. */
    CmplxTran(Cmplx::type c_, aa_tf_vec2 v_) : aa_tf_cv(from(c_, v_)) {}
    /** Construct from rotation angle and translation vector. */
    CmplxTran(double angle, aa_tf_vec2 v_) : aa_tf_cv(from(angle, v_)) {}
    /** Construct from rotation angle and translation vector. */
    CmplxTran(const AngleP &angle, aa_tf_vec2 v_) : aa_tf_cv(from(angle, v_)) {}
    /** Construct from rotation angle and translation vector. */
    CmplxTran(const AngleP *angle, aa_tf_vec2 v_) : aa_tf_cv(from(angle, v_)) {}
    /** Construct from rotation matrix and translation vector. */
    CmplxTran(const RotMatP &R, aa_tf_vec2 v_) : aa_tf_cv(from(R, v_)) {}
    /** Construct from rotation matrix and translation vector. */
    CmplxTran(const RotMatP *R, aa_tf_vec2 v_) : aa_tf_cv(from(R, v_)) {}
    /** Construct from another complex number and translation vector. */
    CmplxTran(const aa_tf_cv &cv) : aa_tf_cv(from(cv)) {}
    /** Construct from another complex number and translation vector. */
    CmplxTran(const aa_tf_cv *cv) : aa_tf_cv(from(cv)) {}
    /** Construct from a transformation matrix. */
    CmplxTran(const struct aa_tf_tfmatp &T) : aa_tf_cv(from(T)) {}
    /** Construct from a transformation matrix. */
    CmplxTran(const struct aa_tf_tfmatp *T) : aa_tf_cv(from(T)) {}

    /** Return the identity transformation. */
    static aa_tf_cv ident() {
        static const double data[] = AA_TF_COTR_IDENT_INITIALIZER;
        return from_cv(data);
    }

    /** Create a transformation from another complex number and translation. */
    static aa_tf_cv from_cv(const double *ptr) {
        aa_tf_cv e;
        e.data[0] = ptr[0];
        e.data[1] = ptr[1];
        e.data[2] = ptr[2];
        e.data[3] = ptr[3];
        return e;
    }

    /** Create a transformation from another complex number and translation. */
    static aa_tf_cv from(aa_tf_cmplx c, aa_tf_vec2 v)
    {
        aa_tf_cv e;
        e.c = c;
        e.v = v;
        return e;
    }

    /** Create a transformation from another complex number and translation. */
    static aa_tf_cv from(Cmplx::type c, aa_tf_vec2 v) {
        return from(Cmplx::c_cmplx(c), v);
    }

    /** Create a transformation from a transformation matrix. */
    static aa_tf_cv from_tfmatp(const double *T) {
        aa_tf_cv e;
        aa_tf_tfmatp2cv(T, &e.c, &e.v);
        return e;
    }

    /** Create a transformation from a rotation angle and translation vector. */
    static aa_tf_cv from(double angle, aa_tf_vec2 v) {
        return from(Cmplx::from_angle(angle), v);
    }
    /** Create a transformation from a rotation angle and translation vector. */
    static aa_tf_cv from(const AngleP &angle, aa_tf_vec2 v) {
        return from(angle.angle(), v);
    }
    /** Create a transformation from a rotation angle and translation vector. */
    static aa_tf_cv from(const AngleP *angle, aa_tf_vec2 v) {
        return from(angle->angle(), v);
    }

    /** Create a transformation from another complex number and translation vector. */
    static aa_tf_cv from(const aa_tf_cv& cv) { return from(cv.c, cv.v); }
    /** Create a transformation from another complex number and translation vector. */
    static aa_tf_cv from(const aa_tf_cv* cv) { return from(*cv); }
    /** Create a transformation from a transformation matrix. */
    static aa_tf_cv from(const aa_tf_tfmatp& T) { return from_tfmatp(T.data); }
    /** Create a transformation from a transformation matrix. */
    static aa_tf_cv from(const aa_tf_tfmatp* T) { return from(*T); }

    /** Create a transformation from a rotation matrix and translation vector. */
    static aa_tf_cv from(const aa_tf_rotmatp *R, aa_tf_vec2 v) {
        aa_tf_cv e;
        e.c = aa_tf_rotmatp2cmplx(R->data);
        e.v = v;
        return e;
    }

    /** Create a transformation from a rotation matrix and translation vector. */
    static aa_tf_cv from(const aa_tf_rotmatp &R, aa_tf_vec2 v) {
        return from(&R, v);
    }

    /** Return the inverse transformation. */
    aa_tf_cv inv() const {
        aa_tf_cv e;
        aa_tf_cv_inv(this->c, this->v, &e.c, &e.v);
        return e;
    }
};

/**
 * A Planar transformation matrix
 */
struct TfMatP : aa_tf_tfmatp {
    /** Construct the identity transformation. */
    TfMatP() : aa_tf_tfmatp(ident()) {}

    /** Construct from a rotation matrix and translation vector. */
    TfMatP(const aa_tf_rotmatp* R_, aa_tf_vec2 v_) : aa_tf_tfmatp(from(R_, v_))
    {}
    /** Construct from a rotation matrix and translation vector. */
    TfMatP(const aa_tf_rotmatp& R_, aa_tf_vec2 v_) : aa_tf_tfmatp(from(R_, v_))
    {}
    /** Construct from a complex number and translation vector. */
    TfMatP(aa_tf_cmplx c, aa_tf_vec2 v_) : aa_tf_tfmatp(from(c,v_)) {}
    /** Construct from a complex number and translation vector. */
    TfMatP(Cmplx::type c, aa_tf_vec2 v_) : aa_tf_tfmatp(from(c,v_)) {}

    /** Construct from a rotation angle and translation vector. */
    TfMatP(double angle, aa_tf_vec2 v_) : aa_tf_tfmatp(from(angle,v_)) {}
    /** Construct from a rotation angle and translation vector. */
    TfMatP(const AngleP &angle, aa_tf_vec2 v_) : aa_tf_tfmatp(from(angle,v_)) {}
    /** Construct from a rotation angle and translation vector. */
    TfMatP(const AngleP *angle, aa_tf_vec2 v_) : aa_tf_tfmatp(from(angle,v_)) {}

    /** Construct from another transformation matrix. */
    TfMatP(const aa_tf_tfmatp *T) : aa_tf_tfmatp(*T) {}
    /** Construct from another transformation matrix. */
    TfMatP(const aa_tf_tfmatp &T) : aa_tf_tfmatp(T) {}
    /** Construct from a complex number and translation vector. */
    TfMatP(const aa_tf_cv* cv) : aa_tf_tfmatp(from(cv)) {}
    /** Construct from a complex number and translation vector. */
    TfMatP(const aa_tf_cv& cv) : aa_tf_tfmatp(from(cv)) {}

    /** Return the identity transformation. */
    static aa_tf_tfmatp ident() {
        static const double data[] = AA_TF_TFMATP_IDENT_INITIALIZER;
        return from_tfmatp(data);
    }

    /** Create a transformation matrix from values at ptr. */
    static aa_tf_tfmatp from_tfmatp(const double * ptr) {
        aa_tf_tfmatp T;
        memcpy(T.data, ptr, sizeof(T));
        return T;
    }

    /** Create a transformation matrix from a complex number and translation vector. */
    static aa_tf_tfmatp from_cv(aa_tf_cmplx c, aa_tf_vec2 v) {
        aa_tf_tfmatp T;
        aa_tf_cmplx2rotmatp(c, T.R.data);

        T.v = v;
        return T;
    }

    /** Create a transformation matrix from a complex number and translation vector. */
    static aa_tf_tfmatp from_cv(Cmplx::type c, aa_tf_vec2 v) {
        return from_cv(Cmplx::c_cmplx(c), v);
    }


    /** Create a transformation matrix from another transformation matrix. */
    static aa_tf_tfmatp from(const aa_tf_tfmatp* T) {
        return from_tfmatp(T->data);
    }

    /** Create a transformation matrix from another transformation matrix. */
    static aa_tf_tfmatp from(const aa_tf_tfmatp& T) {
        return from_tfmatp(T.data);
    }

    /** Create a transformation matrix from a rotation matrix and translation vector. */
    static aa_tf_tfmatp from(const aa_tf_rotmatp &R, aa_tf_vec2 v) {
        aa_tf_tfmatp T;
        T.R = R;
        T.v = v;
        return T;
    }

    /** Create a transformation matrix from a rotation matrix and translation vector. */
    static aa_tf_tfmatp from(const aa_tf_rotmatp *R, aa_tf_vec2 v) {
        return from(*R, v);
    }

    /** Create a transformation matrix from a complex number and translation vector. */
    static aa_tf_tfmatp from(aa_tf_cmplx c, aa_tf_vec2 v) {
        return from_cv(c, v);
    }

    /** Create a transformation matrix from a complex number and translation vector. */
    static aa_tf_tfmatp from(Cmplx::type c, aa_tf_vec2 v) {
        return from_cv(c, v);
    }

    /** Create a transformation matrix from a rotation angle and translation vector. */
    static aa_tf_tfmatp from(double angle, aa_tf_vec2 v) {
        return from_cv(Cmplx::from_angle(angle), v);
    }

    /** Create a transformation matrix from a rotation angle and translation vector. */
    static aa_tf_tfmatp from(const AngleP &angle, aa_tf_vec2 v) {
        return from(angle.angle(), v);
    }

    /** Create a transformation matrix from a rotation angle and translation vector. */
    static aa_tf_tfmatp from(const AngleP* angle, aa_tf_vec2 v) {
        return from(*angle, v);
    }

    /** Create a transformation matrix from a complex number and translation vector. */
    static aa_tf_tfmatp from(const aa_tf_cv& cv) { return from(cv.c, cv.v); }

    /** Create a transformation matrix from a complex number and translation vector. */
    static aa_tf_tfmatp from(const aa_tf_cv* cv) { return from(*cv); }

    /** Return the inverse transformation. */
    aa_tf_tfmatp inv() const {
        aa_tf_tfmatp T;
        aa_tf_tfmatp_inv2(this->data, T.data);
        return T;
    }
};

/** Rotate a 2D vector using a complex number. */
static inline aa_tf_vec2
operator*(const Cmplx& c, const Vec2& v)
{
    return aa_tf_crot(c,v);
}

/** Rotate a 2D vector using a rotation matrix. */
static inline aa_tf_vec2
operator*(const aa_tf_rotmatp& R, const Vec2& v)
{
    return aa_tf_rotmatp_rot(R.data,v);
}

/** Multiply (chain) two rotation matrices. */
static inline aa_tf_rotmatp
operator*(const aa_tf_rotmatp& R1, const aa_tf_rotmatp& R2)
{
    aa_tf_rotmatp R12;
    aa_tf_rotmatp_mul(R1.data,R2.data, R12.data);
    return R12;
}

/** Transform a 2D vector using a transformation matrix. */
static inline aa_tf_vec2
operator*(const aa_tf_tfmatp& T, const aa_tf_vec2& p)
{
    return aa_tf_tfmatp_tf(T.data, p);
}

/** Transform a 2D vector using a complex number and translation vector. */
static inline aa_tf_vec2
operator*(const aa_tf_cv& E, const aa_tf_vec2& p)
{
    return aa_tf_cv_tf(E.c, E.v, p);
}


/** Multiply (chain) two transformation matrices. */
static inline aa_tf_tfmatp
operator*(const aa_tf_tfmatp& T1, const aa_tf_tfmatp& T2)
{
    aa_tf_tfmatp T12;
    aa_tf_tfmatp_mul(T1.data, T2.data, T12.data);
    return T12;
}

/** Chain two complex-numbers and translation vectors. */
static inline aa_tf_cv
operator*(const aa_tf_cv& E1, const aa_tf_cv& E2)
{
    aa_tf_cv E12;
    aa_tf_cv_mul(E1.c, E1.v, E2.c, E2.v, &E12.c, &E12.v);
    return E12;
}

}  // namespace amino

#endif /* AMINO_TFP_HPP */
