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

#ifdef AA_MATPP_TRACE
#define AA_MATPP_PRINT(thing) printf(thing)
#else
#define AA_MATPP_PRINT(thing)
#endif

namespace amino {

/************/
/* WRAPPERS */
/************/

namespace la {

static inline void
scal( struct aa_dvec *x, double alpha ) {
    AA_MATPP_PRINT("dscal\n");
    aa_dvec_scal(alpha, x);
}

static inline void
scal( struct aa_dmat *x, double alpha ) {
    AA_MATPP_PRINT("dscal\n");
    aa_dmat_scal(x,alpha);
}

static inline void
copy( const struct aa_dvec *x, struct aa_dvec *y )
{
    AA_MATPP_PRINT("dcopy\n");
    aa_dvec_copy( x, y );
}

static inline void
axpy( double a, const struct aa_dvec *x, struct aa_dvec *y )
{
    AA_MATPP_PRINT("daxpy\n");
    aa_dvec_axpy(a,x,y);
}

static inline void
lacpy( const char uplo[1],
       const struct aa_dmat *A,
       struct aa_dmat *B )
{
    AA_MATPP_PRINT("dlacpy\n");
    aa_lb_dlacpy(uplo,A,B);
}

static inline void
transpose( const struct aa_dmat *A, struct aa_dmat *B )
{
    AA_MATPP_PRINT("transpose\n");
    aa_dmat_trans(A, B);
}

static inline void
gemv( CBLAS_TRANSPOSE trans,
      double alpha, const struct aa_dmat *A,
      const struct aa_dvec *x,
      double beta, struct aa_dvec *y )
{
    AA_MATPP_PRINT("dgemv\n");
    aa_lb_dgemv(trans,alpha,A,x,beta,y);
}

static inline void
gemm( CBLAS_TRANSPOSE transA, CBLAS_TRANSPOSE transB,
      double alpha, const struct aa_dmat *A,
      const struct aa_dmat *B,
      double beta, struct aa_dmat *C )
{
    AA_MATPP_PRINT("dgemm\n");
    aa_lb_dgemm( transA, transB, alpha, A, B, beta, C );
}


}

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
    la::copy( &e, &target );
}

template <typename E>
void MatEval(E const &e, struct aa_dmat &target) {
    e.eval(target);
}

template <>
void MatEval(aa_dmat const &e, struct aa_dmat &target) {
    la::lacpy("A", &e, &target);
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
        la::scal( &target, alpha );
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
        la::axpy(alpha,&x,&target);
    }
};

class DVecExpMV1 : public DVecExp< DVecExpMV1 > {
public:
    double const alpha;
    CBLAS_TRANSPOSE const trans;
    struct aa_dmat const &A;
    struct aa_dvec const &x;

    DVecExpMV1( CBLAS_TRANSPOSE trans_,
                double alpha_, const struct aa_dmat &A_,
                const struct aa_dvec &x_ ) :
        trans(trans_),
        alpha(alpha_),
        A(A_),
        x(x_)
        {}

    void eval( struct aa_dvec &target ) const {
        la::gemv(trans, alpha, &A, &x, 0, &target);
    }
};

template <typename E>
class DVecExpMV2 : public DVecExp< DVecExpMV2<E> > {
public:
    double const alpha;
    CBLAS_TRANSPOSE const trans;
    struct aa_dmat const &A;
    struct aa_dvec const &x;
    double const beta;
    E const &y;

    DVecExpMV2( CBLAS_TRANSPOSE trans_,
                double alpha_, const struct aa_dmat &A_,
                const struct aa_dvec &x_,
                double beta_,
                E const &y_) :
        trans(trans_),
        alpha(alpha_),
        A(A_),
        x(x_),
        beta(beta_),
        y(y_)
        {}


    void eval( struct aa_dvec &target ) const {
        VecEval<E>(y,target);
        la::gemv(trans, alpha, &A, &x, beta, &target);
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
        la::scal( &target, alpha );
    }
};

class DMatExpMM1 : public DMatExp< DMatExpMM1 > {
public:
    double const alpha;
    CBLAS_TRANSPOSE const transA;
    CBLAS_TRANSPOSE const transB;
    struct aa_dmat const &A;
    struct aa_dmat const &B;

    DMatExpMM1( CBLAS_TRANSPOSE transA_, CBLAS_TRANSPOSE transB_,
                double alpha_,
                const struct aa_dmat &A_,
                const struct aa_dmat &B_ ) :
        transA(transA_),
        transB(transB_),
        alpha(alpha_),
        A(A_),
        B(B_)
        {}

    void eval( struct aa_dmat &target ) const {
        la::gemm(transA, transB, alpha, &A, &B, 0, &target);
    }
};

template <typename E>
class DMatExpMM2 : public DMatExp< DMatExpMM2<E> > {
public:
    double const alpha;
    CBLAS_TRANSPOSE const transA;
    CBLAS_TRANSPOSE const transB;
    struct aa_dmat const &A;
    struct aa_dmat const &B;
    double beta;
    E const &C;

    DMatExpMM2( CBLAS_TRANSPOSE transA_, CBLAS_TRANSPOSE transB_,
                double alpha_,
                const struct aa_dmat &A_,
                const struct aa_dmat &B_,
                double beta_,
                E const &C_ ) :
        transA(transA_),
        transB(transB_),
        alpha(alpha_),
        A(A_),
        B(B_),
        beta(beta_),
        C(C_)
        {}

    void eval( struct aa_dmat &target ) const {
        MatEval<E>(C,target);
        la::gemm(transA, transB, alpha, &A, &B, beta, &target);
    }
};


class DMatExpTranspose : public DMatExp< DMatExpTranspose > {
public:
    struct aa_dmat const &x;
    DMatExpTranspose(const aa_dmat & x_) :
        x(x_) { }

    void eval( struct aa_dmat &target ) const {
        la::transpose(&x,&target);
    }
};

class DMatExpInverse : public DMatExp< DMatExpInverse > {
public:
    struct aa_dmat const &x;
    DMatExpInverse(const aa_dmat & x_) :
        x(x_) { }

    void eval( struct aa_dmat &target ) const {
        la::lacpy("A", &x, &target);
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

    DVec(aa_dvec *v) : aa_dvec(*v) { }
    DVec(aa_dvec &v) : aa_dvec(v) { }

    static DVec * alloc(struct aa_mem_region *reg, size_t len) {
        return new(reg) DVec( len, new(reg) double[len] );
    }

    static DVec * alloc_local( size_t len) {
        struct aa_mem_region *reg = aa_mem_region_local_get();
        return alloc(reg,len);
    }

    size_t   len()  const { return aa_dvec::len; }
    double * data() const { return aa_dvec::data; }
    size_t   inc()  const { return aa_dvec::inc; }

    void eval( struct aa_dvec &target ) const {
        la::copy( this, &target );
    }

    double &operator[](size_t i) {
        return data()[i*inc()];
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
        la::scal(this, alpha);
        return *this;
    }

    DVec & operator/= (double alpha) {
        la::scal(this, 1/alpha);
        return *this;
    }

    DVec & operator+= (aa_dvec const &x) {
        la::axpy(1,&x,this);
        return *this;
    }

    DVec & operator-= (aa_dvec const &x) {
        la::axpy(-1,&x,this);
        return *this;
    }

    DVec & operator+= (DVecExpScal<aa_dvec> const &x) {
        la::axpy(x.alpha,&x.x,this);
        return *this;
    }

    DVec & operator-= (DVecExpScal<aa_dvec> const &x) {
        la::axpy(-x.alpha,&x.x,this);
        return *this;
    }

    static double ssd(const DVec &x, const DVec &y ) {
        return aa_dvec_ssd(&x,&y);
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

    static DMat * alloc(struct aa_mem_region *reg, size_t rows, size_t cols) {
        return new(reg) DMat( rows, cols,
                              new(reg) double[rows*cols],
                              rows);
    }

    static DMat * alloc_local(size_t rows, size_t cols) {
        struct aa_mem_region *reg = aa_mem_region_local_get();
        return alloc(reg,rows,cols);
    }


    size_t rows() const { return aa_dmat::rows; }
    size_t cols() const { return aa_dmat::rows; }
    double * data() const { return aa_dmat::data; }
    size_t ld()   const { return aa_dmat::ld; }

    static DVec row_vec(struct aa_dmat *A, size_t j) {
        aa_dvec v;
        aa_dmat_row_vec(A,j,&v);
        return DVec(v);
    }

    static DVec col_vec(struct aa_dmat *A, size_t i) {
        aa_dvec v;
        aa_dmat_col_vec(A,i,&v);
        return DVec(v);
    }

    DVec row_vec(size_t j) {
        return DMat::row_vec(this, j);
    }

    DVec col_vec(size_t i) {
        return DMat::col_vec(this, i);
    }

    DMat & operator*= (double alpha) {
        la::scal(this, alpha);
        return *this;
    }

    void eval( struct aa_dmat &target ) const {
        la::lacpy("A", this, &target);
    }

    template <typename Exp>
    DMat & operator= (Exp const  &rhs) {
        MatEval<Exp>(rhs,*this);
        return *this;
    }


    DMatExpTranspose transpose() const {
        return DMatExpTranspose(*this);
    }

    DMatExpTranspose t() const {
        return this->transpose();
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
    return DMatExpScal< DMatExp<E> >(alpha,x);
}

template <typename E> DMatExpScal< DMatExp<E> >
operator*( DMatExp<E> const &x, double alpha ) {
    return alpha*x;
}


DMatExpScal< DMatExpTranspose >
operator*( double alpha, DMatExpTranspose const &At ) {
    return DMatExpScal< DMatExpTranspose >(alpha,At);
}

DMatExpScal< DMatExpTranspose >
operator*( DMatExpTranspose const &At, double alpha ) {
    return alpha*At;
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

template <typename E> DVecExpAxpy< DVecExp<E> >
operator+( DVecExp<E> const &b, DVecExpScal<struct aa_dvec>  const &a) {
    return DVecExpAxpy<  DVecExp<E> >(a.alpha,a.x,b);
}

DVecExpAxpy< DVecExpScal< aa_dvec > >
operator+( DVecExpScal< aa_dvec > const &b, DVecExpScal<aa_dvec>  const &a) {
    return DVecExpAxpy<  DVecExpScal<aa_dvec> >(a.alpha,a.x,b);
}


/* Matrix-Vector Multiplication */

DVecExpMV1
operator*(aa_dmat const &A, aa_dvec const &x) {
    return DVecExpMV1(CblasNoTrans, 1, A, x);
}

DVecExpMV1
operator*(double alpha, DVecExpMV1 const &y) {
    return DVecExpMV1(y.trans, alpha*y.alpha, y.A, y.x);
}

DVecExpMV1
operator*(DVecExpMV1 const &y, double alpha) {
    return alpha*y;
}

DVecExpMV1
operator*(DMatExpTranspose const &A, aa_dvec const &x) {
    return DVecExpMV1(CblasTrans, 1, A.x, x);
}

DVecExpMV1
operator*(DMatExpTranspose const &A, DVecExpScal<aa_dvec> const &x) {
    return x.alpha * (A*x.x);
}

DVecExpMV1
operator*(DMatExpScal<aa_dmat> const &A, aa_dvec const &x) {
    return A.alpha * (A.x*x);
}

DVecExpMV1
operator*(aa_dmat const &A, DVecExpScal<aa_dvec> const &x) {
    return x.alpha*(A*x.x);
}

DVecExpMV1
operator*(DMatExpScal<aa_dmat> const &A, DVecExpScal<aa_dvec> const &x) {
    return (A.alpha*x.alpha)*(A.x*x.x);
}

/* MV and addition */

template <typename E>
DVecExpMV2< DVecExp<E> >
operator+(DVecExpMV1 const &b,  DVecExp<E>  const &y) {
    return DVecExpMV2< DVecExp<E> >( b.trans, b.alpha, b.A, b.x, 1, y);
}

template <typename E>
DVecExpMV2< DVecExp<E> >
operator+( DVecExp<E>  const &y, DVecExpMV1 const &b ) {
    return b+y;
}


DVecExpMV2<aa_dvec>
operator+(DVecExpMV1 const &b,  aa_dvec  const &y) {
    return DVecExpMV2< aa_dvec >( b.trans, b.alpha, b.A, b.x, 1, y);
}

DVecExpMV2<aa_dvec>
operator+( aa_dvec  const &y, DVecExpMV1 const &b ) {
    return b+y;
}

template <typename E> DVecExpMV2<E>
operator+(DVecExpMV1 const &b, DVecExpScal<E> const &y) {
    return DVecExpMV2<E>( b.trans, b.alpha, b.A, b.x, y.alpha, y.x);
}

template <typename E> DVecExpMV2<E>
operator+( DVecExpScal<E> const &y, DVecExpMV1 const &b) {
    return b+y;
}

template <typename E> DVecExpMV2<E>
operator*( double alpha, DVecExpMV2<E> const &b) {
    return DVecExpMV2<E>( b.trans, alpha*b.alpha, b.A, b.x,
                          alpha*b.beta, b.y);
}

template <typename E> DVecExpMV2<E>
operator*( DVecExpMV2<E> const &b, double alpha ) {
    return alpha * b;
}


/* MM */

DMatExpMM1
operator*( aa_dmat const &A, aa_dmat const &B ) {
    return DMatExpMM1(CblasNoTrans, CblasNoTrans, 1, A, B);
}

DMatExpMM1
operator*( DMatExpTranspose const &A, aa_dmat const &B ) {
    return DMatExpMM1(CblasTrans, CblasNoTrans, 1, A.x, B);
}

DMatExpMM1
operator*( aa_dmat const &A, DMatExpTranspose  const &B ) {
    return DMatExpMM1(CblasNoTrans, CblasTrans, 1, A, B.x);
}

DMatExpMM1
operator*( DMatExpTranspose const &A, DMatExpTranspose const &B ) {
    return DMatExpMM1(CblasTrans, CblasTrans, 1, A.x, B.x);
}


DMatExpMM1
operator*( double a, DMatExpMM1 const &C )  {
    return DMatExpMM1(C.transA, C.transB, a*C.alpha, C.A, C.B);
}

DMatExpMM1
operator*( DMatExpMM1 const &C, double a ) {
    return a*C;
}



template<typename E>
DMatExpMM1
operator*( DMatExpScal< E > const &A, aa_dmat const &B) {
    return A.alpha * (A.x*B);
}

template<typename E>
DMatExpMM1
operator*( DMatExpScal< E > const &A, DMatExpTranspose const &B) {
    return A.alpha * (A.x*B);
}

template<typename E>
DMatExpMM1
operator*( aa_dmat const &A, DMatExpScal< E > const &B) {
    return B.alpha * (A*B.x);
}

template<typename E>
DMatExpMM1
operator*( DMatExpTranspose const &A, DMatExpScal< E > const &B) {
    return B.alpha * (A*B.x);
}

template<typename Ea, typename Eb>
DMatExpMM1
operator*( DMatExpScal< Ea > const &A,
           DMatExpScal< Eb > const &B) {
    return (A.alpha*B.alpha) * (A.x*B.x);
}

/* MM2 */

DMatExpMM2< aa_dmat >
operator+(DMatExpMM1 const &M,  aa_dmat  const &C) {
    return DMatExpMM2< aa_dmat >( M.transA, M.transB,
                                     M.alpha, M.A, M.B,
                                     1, C );
}

DMatExpMM2< aa_dmat >
operator+( aa_dmat  const &C, DMatExpMM1 const &M) { return M+C; }

template <typename E> DMatExpMM2< DMatExp<E> >
operator+(DMatExpMM1 const &M,  DMatExp<E>  const &C) {
    return DMatExpMM2< DMatExp<E> >( M.transA, M.transB,
                                     M.alpha, M.A, M.B,
                                     1, C );
}

template <typename E> DMatExpMM2< DMatExp<E> >
operator+(DMatExp<E>  const &C, DMatExpMM1 const &M) { return M+C; }


template <typename E> DMatExpMM2< E >
operator+(DMatExpMM1 const &M,  DMatExpScal<E>  const &C) {
    return DMatExpMM2< E >( M.transA, M.transB,
                            M.alpha, M.A, M.B,
                            C.alpha, C.x );
}

template <typename E> DMatExpMM2< E >
operator+( DMatExpScal<E>  const &C, DMatExpMM1 const &M) {
    return M + C;
}

template <typename E> DMatExpMM2< E >
operator*(double alpha, DMatExpMM2<E> const &M ) {
    return DMatExpMM2< E >( M.transA, M.transB,
                            alpha*M.alpha, M.A, M.B,
                            alpha*M.beta, M.C );
}

template <typename E> DMatExpMM2< E >
operator*( DMatExpMM2<E> const &M, double alpha ) {
    return alpha*M;
}

}
;


#endif //AMINO_MAT_HPP
