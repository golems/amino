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
class DVecExpScal {
public:
    double const alpha;
    E const &x;
    DVecExpScal(double alpha_, E const& x_) :
        alpha(alpha_), x(x_) { }

    void eval( struct aa_dvec &target ) const {
        x.eval(target);
        aa_lb_dscal( alpha, &target );
    }
};

template <typename E>
class DVecExpAxpy {
public:
    double const alpha;
    struct aa_dvec const &x;
    E const &y;

    DVecExpAxpy(double alpha_, struct aa_dvec const &x_, E const &y_):
        alpha(alpha_), x(x_), y(y_) { }

    void eval( struct aa_dvec &target ) const {
        y.eval(target);
        aa_lb_daxpy(alpha,&x,&target);
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

    template <typename Exp>
    DVec & operator= (Exp const  &rhs) {
        rhs.eval(*this);
        return *this;
    }

    DVec & operator= (struct aa_dvec const  &rhs) {
        aa_lb_dcopy( &rhs, this );
        return *this;
    }

    DVec & operator= (DVec const  &rhs) {
        aa_lb_dcopy( &rhs, this );
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


/***************/
/* EXPRESSIONS */
/***************/

/* Scaling */
template <typename E> DVecExpScal<E>
operator*(double alpha, E const &x) {
    return DVecExpScal<E>(alpha,x);
}

template <typename E> DVecExpScal<E>
operator*(E const &x, double alpha) {
    return alpha*x;
}

template <typename E> DVecExpScal<E>
operator*(double alpha,  DVecExpScal<E> const &x ) {
    return DVecExpScal<E>(alpha*x.alpha,x.x);
}

template <typename E> DVecExpScal<E>
operator*( DVecExpScal<E> const &x, double alpha ) {
    return alpha*x;
}


/* Addition */

DVecExpAxpy<struct aa_dvec>
operator+(struct aa_dvec const &x, struct aa_dvec const &y) {
    return DVecExpAxpy<struct aa_dvec>(1,x,y);
}

DVecExpAxpy<DVec>
operator+(DVec const &x, DVec const &y) {
    return DVecExpAxpy<DVec>(1,x,y);
}

template <typename E> DVecExpAxpy<E>
operator+(struct aa_dvec const &x, E const &y) {
    return DVecExpAxpy<E>(1,x,y);
}

template <typename E> DVecExpAxpy<E>
operator+(E const &y,struct aa_dvec const &x) {
    return x + y;
}

template <typename E> DVecExpAxpy<E>
operator+( DVecExpScal<DVec> const &a, E const &b ) {
    return DVecExpAxpy<E>(a.alpha,a.x,b);
}

template <typename E> DVecExpAxpy<E>
operator+(E const &a, DVecExpScal<DVec>  const &b) {
    return b+a;
}

template <typename E> DVecExpAxpy<E>
operator+( DVecExpScal<struct aa_dvec> const &a, E const &b ) {
    return DVecExpAxpy<E>(a.alpha,a.x,b);
}

template <typename E> DVecExpAxpy<E>
operator+(E const &a, DVecExpScal<struct aa_dvec>  const &b) {
    return b+a;
}



};


#endif //AMINO_MAT_HPP
