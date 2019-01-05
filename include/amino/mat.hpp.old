/* -*- mode: C++; c-basic-offset: 4 -*- */
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

#ifndef AMINO_MAT_HPP
#define AMINO_MAT_HPP

// in namespace amino

/**
 * \file mat.hpp
 * \author Neil T. Dantam
 *
 * Don't use this stuff yet.  It's probably awful.
 */


template <size_t ROWS, size_t COLS = 1>
class Mat {
public:
    double _array[ROWS*COLS];
    Mat() {};
    Mat( const double * src) {
        load(src);
    };
    Mat( const Mat<ROWS,COLS>  &src) {
        load(&src[0]);
    };
    ~Mat(){};
    void load(const double *src) {
        aa_fcpy( _array, src, ROWS*COLS );
    }
    void store(double *dst) const {
        aa_fcpy( dst, _array, ROWS*COLS );
    }
    double &operator()( size_t i ) const { return _array[i]; }
    double &operator()( size_t i ) { return _array[i]; }
    double &operator[]( size_t i ) { return _array[i]; }
    const double &operator[]( size_t i ) const { return _array[i]; }
    double &operator()( size_t i, size_t j ) const {
        return AA_MATREF(_array, ROWS, i, j);
    }
    double &operator()( size_t i, size_t j ) {
        return AA_MATREF(_array, ROWS, i, j);
    }
    size_t size() const { return ROWS*COLS; }
    bool eq( const Mat<ROWS, COLS> &other ) {
        return aa_veq( ROWS*COLS, &_array[0], &other[0], AA_EPSILON );
    }
    Mat<COLS,ROWS> transpose() const {
        Mat<COLS,ROWS> mt;
        aa_la_transpose2( ROWS, COLS, _array, &mt[0] );
    }

    double norm() const { return aa_la_norm( ROWS*COLS, _array ); }
    double dot(Mat<ROWS,COLS> &other) const {
        return aa_la_dot( ROWS*COLS, _array, other );
    }
    double ssd(Mat<ROWS,COLS> &other) const {
        return aa_la_ssd( ROWS*COLS, _array, other );
    }
    double dist(Mat<ROWS,COLS> &other) const {
        return aa_la_dist( ROWS*COLS, _array, other );
    }
};


template <size_t ROWS >
class Vec : public Mat<ROWS,1> {
public:
    Vec() {};
    Vec(const Mat<ROWS,1> &other) :
        Mat<ROWS,1>(other)
    {};
    Vec( const double * src) : Mat<ROWS,1>(src) {};
    ~Vec(){};
    Vec<ROWS> operator=( const Mat<ROWS,1> &other ) {
        aa_fcpy( &this->_array[0], &other[0], ROWS );
        return *this;
    }
};

// Matrix Operators
template <size_t ROWS, size_t COLS>
static inline Mat<ROWS,COLS> operator*( double alpha,
                                        const Mat<ROWS,COLS> b ) {
    Mat<ROWS, COLS> c;
    aa_la_smul( ROWS*COLS, alpha, &b[0], &c[0] );
    return c;
}
template <size_t ROWS, size_t COLS>
static inline Mat<ROWS,COLS> operator*( const Mat<ROWS,COLS> b ,
                                        double alpha ) {
    return alpha * b;
}
template <size_t ROWS, size_t COLS>
static inline Mat<ROWS,COLS> operator/( const Mat<ROWS,COLS> b ,
                                        double alpha ) {
    return 1.0/alpha * b;
}
template <size_t ROWS, size_t COLS>
static inline Mat<ROWS,COLS> operator/( double alpha, const Mat<ROWS,COLS> b ) {
    Mat<ROWS, COLS> c;
    aa_la_sdiv( ROWS*COLS, alpha, &b[0], &c[0] );
    return c;
}


template <size_t ROWS, size_t COLS>
static inline Mat<ROWS,COLS> operator+( const Mat<ROWS,COLS> a,
                                        const Mat<ROWS,COLS> b ) {
    Mat<ROWS, COLS> c;
    aa_la_vadd( ROWS*COLS, &a[0], &b[0], &c[0] );
    return c;
}

template <size_t ROWS, size_t COLS>
static inline Mat<ROWS,COLS> operator-( const Mat<ROWS,COLS> a,
                                        const Mat<ROWS,COLS> b ) {
    Mat<ROWS, COLS> c;
    aa_la_vsub( ROWS*COLS, &a[0], &b[0], &c[0] );
    return c;
}


class Tf {
public:
    double _array[12];
    Tf() { load(AA_TF_IDENT); }
    Tf(const double src[12] ) { load(src); }
    Tf( double R11, double R12, double R13, double v1,
        double R21, double R22, double R23, double v2,
        double R31, double R32, double R33, double v3 ) {
        _array[0] = R11;
        _array[1] = R21;
        _array[2] = R31;
        _array[3] = R12;
        _array[4] = R22;
        _array[5] = R32;
        _array[6] = R13;
        _array[7] = R23;
        _array[8] = R33;
        _array[9] = v1;
        _array[10] = v2;
        _array[11] = v3;
    }
    Tf(const Tf &other) { load(other._array); }
    ~Tf() {};

    double &operator[]( size_t i ) { return _array[i]; }
    const double &operator[]( size_t i ) const { return _array[i]; }

    /// load store ops
    void load(const double src[12]) {
        aa_fcpy( _array, src, 12 );
    }
    void load_rotmat(const double src[9]) {
        aa_fcpy( _array, src, 9 );
    }
    void load_vec(const double src[3]) {
        aa_fcpy( _array+9, src, 3 );
    }
    void store(double dst[12]) const {
        aa_fcpy( dst, _array, 12 );
    }
    void store_rotmat(double dst[9]) const {
        aa_fcpy( dst, _array, 9 );
    }
    void store_vec(double dst[3]) const {
        aa_fcpy( dst, _array+9, 3 );
    }

    Tf inv() const {
        Tf tfi;
        aa_tf_12inv( _array, &tfi[0] );
        return tfi;
    }

};


static inline Mat<3> operator*( const Tf tf,
                                const Mat<3> v ) {
    Mat<3> v1;
    aa_tf_12(tf._array, &v[0], &v1[0]);
    return v1;
}


static inline Tf operator*( const Tf tf1,
                            const Tf tf2 ) {
    Tf tf;
    aa_tf_12chain(&tf1[0], &tf2[0], &tf[0]);
    return tf;
}



#endif //AMINO_MAT_HPP
