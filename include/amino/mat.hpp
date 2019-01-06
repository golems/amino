/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@mines.edu>
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

#include "mat.h"
#include "mem.hpp"

// in namespace amino

/**
 * @file mat.hpp
 * @author Neil T. Dantam
 *
 * Don't use this stuff yet.  It's probably awful.
 */


namespace amino {


/************************/
/* EXPRESSION TEMPLATES */
/************************/

template <typename E>
class DVecExp {
  public:
    void eval(struct aa_dvec &target) const {
        static_cast<E const&>(*this).eval(target);
    }
};

template <typename E>
class DMatExp {
  public:
    void eval(struct aa_dmat &target) const {
        static_cast<E const&>(*this).eval(target);
    }
};


template <typename E>
void VecEval( E const &e, struct aa_dvec &target) {
    e.eval(target);
}

template <>
void VecEval(aa_dvec const &e, struct aa_dvec &target) {
    aa_lb_dcopy( &e, &target );
}

template <typename E>
void MatEval(E const &e, struct aa_dmat &target) {
    e.eval(target);
}

template <>
void MatEval(aa_dmat const &e, struct aa_dmat &target) {
    aa_lb_dlacpy("A", &e, &target);
}



template <typename E>
class DVecExpScal : public DVecExp< DVecExpScal<E> > {
public:
    double const alpha;
    E const &x;
    DVecExpScal(double alpha_, E const& x_) :
        alpha(alpha_), x(x_) { }

    void eval( struct aa_dvec &target ) const {
        VecEval(x,target);
        aa_lb_dscal( alpha, &target );
    }
};

template <typename E>
class DVecExpAxpy : public DVecExp< DVecExpAxpy<E> > {
public:
    double const alpha;
    struct aa_dvec const &x;
    E const &y;

    DVecExpAxpy(double alpha_, struct aa_dvec const &x_, E const &y_):
        alpha(alpha_), x(x_), y(y_) { }

    void eval( struct aa_dvec &target ) const {
        VecEval(y,target);
        aa_lb_daxpy(alpha,&x,&target);
    }
};

template <typename E>
class DMatExpScal : public DMatExp< DMatExpScal<E> > {
public:
    double const alpha;
    E const &x;
    DMatExpScal(double alpha_, E const& x_) :
        alpha(alpha_), x(x_) { }

    void eval( struct aa_dmat &target ) const {
        MatEval(x,target);
        aa_dmat_scal( &target, alpha );
    }
};

class DMatExpTranspose {
public:
    struct aa_dmat const &x;
    DMatExpTranspose(const aa_dmat & x_) :
        x(x_) { }

    void eval( struct aa_dmat &target ) const {
        aa_dmat_trans(&x, &target);
    }
};

class DMatExpInverse {
public:
    struct aa_dmat const &x;
    DMatExpInverse(const aa_dmat & x_) :
        x(x_) { }

    void eval( struct aa_dmat &target ) const {
        aa_lb_dlacpy("A", &x, &target);
        aa_dmat_inv(&target);
    }
};


/**
 * Object type for vectors
 */
class DVec : public ::aa_dvec {
public:
    DVec(size_t len_, double *data_, size_t inc_ ) {
        aa_dvec_view( this, len_, data_, inc_ );
    }

    DVec(size_t len_, double *data_ ) {
        aa_dvec_view(this, len_, data_, 1 );
    }

    static DVec * alloc(struct aa_mem_region *reg, size_t len) {
        return new(reg) DVec( len, new(reg) double[len] );
    }

    static DVec * alloc_local( size_t len) {
        struct aa_mem_region *reg = aa_mem_region_local_get();
        return alloc(reg,len);
    }

    void eval( struct aa_dvec &target ) const {
        aa_lb_dcopy( this, &target );
    }

    template <typename E>
    DVec & operator= (E const  &rhs) {
        VecEval<E>(rhs,*this);
        return *this;
    }

    DVec & operator= (struct aa_dvec const  &rhs) {
        VecEval<aa_dvec>(rhs,*this);
        return *this;
    }

    DVec & operator= (DVec const  &rhs) {
        VecEval<DVec>(rhs,*this);
        return *this;
    }

    DVec & operator= (double rhs) {
        aa_dvec_set(this,rhs);
        return *this;
    }

    DVec & operator*= (double alpha) {
        aa_lb_dscal(alpha,this);
        return *this;
    }


};

/**
 * Object type for matrices
 */
class DMat : public ::aa_dmat {
public:
    DMat(size_t rows_, size_t cols_, double *data_, size_t ld_ ) {
        aa_dmat_view( this, rows_, cols_, data_, ld_ );
    }

    DMat(size_t rows_, size_t cols_, double *data_ ) {
        aa_dmat_view( this, rows_, cols_, data_, rows_ );
    }

    DMat & operator*= (double alpha) {
        aa_dmat_scal(this, alpha);
        return *this;
    }

    void eval( struct aa_dmat &target ) const {
        aa_lb_dlacpy("A", this, &target);
    }

    template <typename Exp>
    DMat & operator= (Exp const  &rhs) {
        MatEval<Exp>(rhs,*this);
        return *this;
    }


    DMatExpTranspose transpose() const {
        return DMatExpTranspose(*this);
    }

    DMatExpInverse inverse() const {
        return DMatExpInverse(*this);
    }

    void invert() {
        aa_dmat_inv(this);
    }

    static double ssd(const DMat &x, const DMat &y ) {
        return aa_dmat_ssd(&x,&y);
    }



};


/***************/
/* EXPRESSIONS */
/***************/

/* Vector Scaling */
DVecExpScal<aa_dvec>
operator*(double alpha, aa_dvec const &x) {
    return DVecExpScal<aa_dvec>(alpha,x);
}

DVecExpScal<aa_dvec>
operator*(aa_dvec const &x, double alpha) { return alpha*x; }


template <typename E> DVecExpScal<E>
operator*(double alpha,  DVecExpScal<E> const &x ) {
    return DVecExpScal<E>(alpha*x.alpha,x.x);
}

template <typename E> DVecExpScal<E>
operator*( DVecExpScal<E> const &x, double alpha ) { return alpha*x; }


template <typename E> DVecExpScal< DVecExp<E> >
operator*(double alpha,  DVecExp<E> const &x ) {
    return DVecExpScal< DVecExp<E> >( alpha, x );
}

template <typename E> DVecExpScal< DVecExp<E> >
operator*(DVecExp<E> const &x, double alpha ) { return alpha * x; }

/* Matrix Scaling */

DMatExpScal<aa_dmat>
operator*(double alpha, aa_dmat const &x) {
    return DMatExpScal<aa_dmat>(alpha,x);
}

DMatExpScal<aa_dmat>
operator*( aa_dmat const &x, double alpha) {
    return alpha * x;
}

template <typename E> DMatExpScal<E>
operator*(double alpha,  DMatExpScal<E> const &x ) {
    return DMatExpScal<E>(alpha*x.alpha,x.x);
}

template <typename E> DMatExpScal<E>
operator*( DMatExpScal<E> const &x, double alpha ) {
    return alpha*x;
}

template <typename E> DMatExpScal< DMatExp<E> >
operator*(double alpha,  DMatExp<E> const &x ) {
    return DMatExpScal<E>(alpha,x);
}

template <typename E> DMatExpScal< DMatExp<E> >
operator*( DMatExp<E> const &x, double alpha ) {
    return alpha*x;
}


/* Vector Addition */

DVecExpAxpy<struct aa_dvec>
operator+(struct aa_dvec const &x, struct aa_dvec const &y) {
    return DVecExpAxpy<struct aa_dvec>(1,x,y);
}

DVecExpAxpy<DVec>
operator+(DVec const &x, DVec const &y) {
    return DVecExpAxpy<DVec>(1,x,y);
}


template <typename E> DVecExpAxpy< DVecExp<E> >
operator+(struct aa_dvec const &x,  DVecExp<E>  const &y) {
    return DVecExpAxpy< DVecExp<E> >(1,x,y);
}

template <typename E> DVecExpAxpy< DVecExp<E> >
operator+(DVecExp<E> const &y,struct aa_dvec const &x) { return x + y; }



template <typename E> DVecExpAxpy< DVecExp<E> >
operator+( DVecExpScal<struct aa_dvec> const &a,  DVecExp<E>  const &b ) {
    return DVecExpAxpy< DVecExp<E> >(a.alpha,a.x,b);
}

template <typename E> DVecExpAxpy<E>
operator+(E const &b, DVecExpScal<struct aa_dvec>  const &a) {
    return DVecExpAxpy< E >(a.alpha,a.x,b);
}


};


#endif //AMINO_MAT_HPP
