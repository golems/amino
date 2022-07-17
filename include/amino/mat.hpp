/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019-2020, Colorado School of Mines
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

#include <initializer_list>
#include <algorithm>
#include <memory>


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
    aa_dmat_lacpy(uplo,A,B);
}

static inline void
transpose( const struct aa_dmat *A, struct aa_dmat *B )
{
    AA_MATPP_PRINT("transpose\n");
    aa_dmat_trans(A, B);
}

static inline void
gemv( AA_CBLAS_TRANSPOSE trans,
      double alpha, const struct aa_dmat *A,
      const struct aa_dvec *x,
      double beta, struct aa_dvec *y )
{
    AA_MATPP_PRINT("dgemv\n");
    aa_dmat_gemv(trans,alpha,A,x,beta,y);
}

static inline void
gemm( AA_CBLAS_TRANSPOSE transA, AA_CBLAS_TRANSPOSE transB,
      double alpha, const struct aa_dmat *A,
      const struct aa_dmat *B,
      double beta, struct aa_dmat *C )
{
    AA_MATPP_PRINT("dgemm\n");
    aa_dmat_gemm( transA, transB, alpha, A, B, beta, C );
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
static void VecEval( E const &e, struct aa_dvec &target) {
    e.eval(target);
}

template <>
void VecEval(aa_dvec const &e, struct aa_dvec &target) {
    la::copy( &e, &target );
}

template <typename E>
static void MatEval(E const &e, struct aa_dmat &target) {
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
    AA_CBLAS_TRANSPOSE const trans;
    struct aa_dmat const &A;
    struct aa_dvec const &x;

    DVecExpMV1( AA_CBLAS_TRANSPOSE trans_,
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
    AA_CBLAS_TRANSPOSE const trans;
    struct aa_dmat const &A;
    struct aa_dvec const &x;
    double const beta;
    E const &y;

    DVecExpMV2( AA_CBLAS_TRANSPOSE trans_,
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
    AA_CBLAS_TRANSPOSE const transA;
    AA_CBLAS_TRANSPOSE const transB;
    struct aa_dmat const &A;
    struct aa_dmat const &B;

    DMatExpMM1( AA_CBLAS_TRANSPOSE transA_, AA_CBLAS_TRANSPOSE transB_,
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
    AA_CBLAS_TRANSPOSE const transA;
    AA_CBLAS_TRANSPOSE const transB;
    struct aa_dmat const &A;
    struct aa_dmat const &B;
    double beta;
    E const &C;

    DMatExpMM2( AA_CBLAS_TRANSPOSE transA_, AA_CBLAS_TRANSPOSE transB_,
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

    typedef unsigned char byte;
    typedef aa_la_size size_type;

    DVec(size_type len_, double *data_, size_type inc_ = 1 ) {
        AA_DVEC_VIEW(this, len_, data_, inc_);
    }

    DVec(struct aa_mem_region *reg, size_type len_) {
        AA_DVEC_VIEW(this, len_, new(reg) double[len_], 1);
    }

    DVec(struct aa_mem_region *reg, const ::std::initializer_list<double> x) {
        AA_DVEC_VIEW(this, x.size(), new(reg) double[x.size()], 1);
        ::std::copy(x.begin(), x.end(), this->data);
    }

    DVec(aa_dvec *v) : aa_dvec(*v) { }
    DVec(aa_dvec &v) : aa_dvec(v) { }


    // Allocation
    template <template<typename> typename A >
    static DVec * alloc(A<byte> &a, size_type len) {
        static const size_type n_obj = AA_ALIGN2(sizeof(DVec),AA_MEMREG_ALIGN);
        void *mem = a.allocate(n_obj + sizeof(double)*len);
        double *data = reinterpret_cast<double*>( static_cast<byte*>(mem) + n_obj );
        return new(mem) DVec(len, data, 1);
    }

    static DVec * alloc(struct aa_mem_region *reg, size_type len) {
        auto r =  RegionAllocator<byte>(reg);
        return DVec::alloc(r, len);
    }

    static DVec * alloc_local( size_type len) {
        struct aa_mem_region *reg = aa_mem_region_local_get();
        return alloc(reg,len);
    }

    // Assign
    template <typename I>
    static void assign(::aa_dvec *v, const I &first, const I &last) {
        ::std::copy(first, last, iterator(v->data, v->inc));
    }

    template <typename X>
    static void assign(::aa_dvec *v, const X &x ) {
        assign(v, x.begin(), x.end());
    }


    // Init
    template <typename A, typename X>
    static DVec *init(A a, const X &x) {
        DVec *v = DVec::alloc(a, x.size());
        DVec::assign(v, x);
        return v;
    }

    template <typename A>
    static DVec *init(A a, const  ::std::initializer_list<double>  &x) {
        DVec *v = DVec::alloc(a, x.size());
        DVec::assign(v, x);
        return v;
    }

    template <typename A, typename X>
    static DVec init(A a, const X &first, const X &last) {
        DVec v = DVec::alloc(a, last - first);
        DVec::assign(v, first, last);
        return v;
    }

    // Viewing
    static DVec view(size_type len, double *data, size_type inc = 1) {
        return DVec(len, data, inc);
    }

    static const DVec view(size_type len, const double *data, size_type inc = 1) {
        return DVec::view(len, const_cast<double*>(data));
    }

    static DVec view(struct aa_dvec *x) {
        return DVec(x);
    }

    static const DVec view(const struct aa_dvec *x) {
        return DVec::view(const_cast<aa_dvec*>(x));
    }

    // Slicing
    DVec slice(size_type start, size_type stop, size_type step = 1) {
        struct aa_dvec tmp;
        aa_dvec_slice(this, start, stop, stop, &tmp);
        return DVec(tmp);
    }

    const DVec slice(size_type start, size_type stop, size_type step = 1) const {
        return const_cast<DVec*>(this)->slice(start,stop,step);
    }

    // Accessors
    size_type   size()  const { return aa_dvec::len; }
    // size_type   len()  const { return aa_dvec::len; }
    // double * data() const { return aa_dvec::data; }
    // size_type   inc()  const { return aa_dvec::inc; }

    void eval( struct aa_dvec &target ) const {
        la::copy( this, &target );
    }

    const double &operator[](size_type i) const {
        return data[i*inc];
    }

    double &operator[](size_type i) {
        return data[i*inc];
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


    // Iteration

private:
    template <class C, typename D, typename V>
    class base_iterator {
    public:
        typedef std::forward_iterator_tag iterator_category;
        typedef D value_type;
        typedef size_type difference_type;
        typedef D *pointer;
        typedef D &reference;

        base_iterator( D *data_, size_type inc_) :
            data(data_),
            inc(inc_)
        {}


        C &operator++ () {
            this->data += this->inc;
            return *static_cast<C*>(this);
        }

        C operator++ (int) {
            C tmp(data,inc);
            operator++();
            return tmp;
        }

        C &operator-- () {
            this->data -= this->inc;
            return *static_cast<C*>(this);
        }

        C operator-- (int) {
            C tmp(data,inc);
            operator--();
            return tmp;
        }

        C &operator+= (size_type s) {
            this->data += (s*this->inc);
            return *static_cast<C*>(this);
        }

        D &operator* () { return *this->data; }

        bool operator<(const C &other) const {
            return this->data < other.data;
        }
        bool operator<=(const C &other) const {
            return this->data <= other.data;
        }

        bool operator>(const C &other) const {
            return this->data > other.data;
        }

        bool operator>=(const C &other) const {
            return this->data >= other.data;
        }

        bool operator==(const C &other) const {
            return this->data == other.data;
        }

        bool operator!=(const C &other) const {
            return this->data != other.data;
        }

        V slice(size_type len, size_type step = 1) const {
            return DVec(len, this->data, this->inc*step);
        }

    protected:
        D *data;
        size_type inc;
    };

    class const_iterator : public base_iterator<const_iterator, const double, const DVec> {
    public:
        const_iterator( const double *data_, size_type inc_) :
            base_iterator(data_, inc_)
        {}
    };

    class iterator : public base_iterator<iterator, double, const DVec> {
    public:
        iterator( double *data_, size_type inc_) :
            base_iterator(data_, inc_)
        {}
    };
public:

    static const_iterator begin(const struct aa_dvec *x) {
        return const_iterator(x->data, x->inc);
    }

    static const_iterator end(const struct aa_dvec *x) {
        return const_iterator(x->data + x->inc*x->len, x->inc);
    }

    static iterator begin(struct aa_dvec *x) {
        return iterator(x->data, x->inc);
    }

    static iterator end(struct aa_dvec *x) {
        return iterator(x->data + x->inc*x->len, x->inc);
    }

    const_iterator begin() const { return DVec::begin(this); }
    const_iterator end() const { return DVec::end(this); }
    iterator begin() { return DVec::begin(this); }
    iterator end() { return DVec::end(this); }
};


/**
 * Object type for matrices
 */
class DMat : public ::aa_dmat {
public:

    typedef unsigned char byte;
    typedef aa_la_size size_type;

    DMat(size_type rows_, size_type cols_, double *data_, size_type ld_ ) {
        AA_DMAT_VIEW(this, rows_, cols_, data_, ld_);
    }

    DMat(size_type rows_, size_type cols_, double *data_ ) {
        AA_DMAT_VIEW(this, rows_, cols_, data_, rows_);
    }

    DMat(struct aa_mem_region *reg, size_type rows_, size_type cols_)  {
        AA_DMAT_VIEW(this, rows_, cols_, new(reg) double[rows_*cols_], rows_);
    }

    DMat(struct aa_dmat *A) : aa_dmat(*A)
    {}

    DMat(struct aa_dmat &A) : aa_dmat(A)
    {}

    // Allocation

    static DMat * alloc(struct aa_mem_region *reg, size_type rows, size_type cols) {
        static const size_type n_obj = AA_ALIGN2(sizeof(DMat),AA_MEMREG_ALIGN);
        void *mem = aa_mem_region_alloc(reg, n_obj + sizeof(double)*rows*cols);
        return new(mem) DMat(rows, cols,
                             reinterpret_cast<double*>(reinterpret_cast<byte*>(mem) + n_obj),
                             rows);
    }

    static DMat * alloc_local(size_type rows, size_type cols) {
        struct aa_mem_region *reg = aa_mem_region_local_get();
        return alloc(reg,rows,cols);
    }


    static DMat * col_mat(struct aa_mem_region *reg,
                          const ::std::initializer_list<::std::initializer_list<double> > x ) {
        auto it = x.begin();
        size_type cols = x.size();
        size_type rows = it->size();
        DMat *y =  DMat::alloc(reg,rows,cols);

        for( auto & col : y->col_container() )
        {
            ::std::copy(it->begin(), it->end(), col.data);
            it++;
        }

        return y;
    }

    static DMat * row_mat(struct aa_mem_region *reg,
                          const ::std::initializer_list<::std::initializer_list<double> > x ) {
        auto it = x.begin();
        size_type rows = x.size();
        size_type cols = it->size();
        DMat *y =  DMat::alloc(reg,rows,cols);
        for( auto &v : y->row_container() ) {
            ::std::copy(it->begin(), it->end(), v.begin());
            it++;
        }

        return y;
    }

    // viewing

    static DMat view(aa_dmat *A) {
        return DMat(A);
    }

    static const DMat view(const aa_dmat *A) {
        return DMat::view(const_cast<aa_dmat*>(A));
    }



    // Accessors
    // size_type rows() const { return aa_dmat::rows; }
    // size_type cols() const { return aa_dmat::cols; }
    // double * data() const { return aa_dmat::data; }
    // size_type ld()   const { return aa_dmat::ld; }

    static DVec row_vec(struct aa_dmat *A, size_type j) {
        aa_dvec v;
        aa_dmat_row_vec(A,j,&v);
        return DVec(v);
    }

    static DVec col_vec(struct aa_dmat *A, size_type i) {
        aa_dvec v;
        aa_dmat_col_vec(A,i,&v);
        return DVec(v);
    }

    static DMat block(struct aa_dmat *A,
                      size_type row_start, size_type col_start,
                      size_type row_end, size_type col_end) {
        aa_dmat B;
        aa_dmat_block(A, row_start, col_start, row_end, col_end, &B);
        return DMat(B);
    }

    DVec row_vec(size_type j) {
        return DMat::row_vec(this, j);
    }

    DVec col_vec(size_type i) {
        return DMat::col_vec(this, i);
    }


    const double &operator()(size_type i, size_type j) const {
        const struct aa_dmat *m = this;
        return AA_DMAT_REF(m, i, j);
    }

    double &operator()(size_type i, size_type j) {
        struct aa_dmat *m = this;
        return AA_DMAT_REF(m, i, j);
    }


    DMat & operator*= (double alpha) {
        la::scal(this, alpha);
        return *this;
    }

    DMat block( size_type row_start, size_type col_start,
                size_type row_end, size_type col_end) {
        return DMat::block(this, row_start, col_start, row_end, col_end);
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


private:

    // iterators

    template <class C, typename D, typename V>
    class base_iterator {
    public:
        typedef std::forward_iterator_tag iterator_category;
        typedef V value_type;
        typedef size_type difference_type;
        typedef V *pointer;
        typedef V &reference;

        base_iterator(size_type len_, D *data_, size_type inc_) :
            vec(len_, const_cast<double*>(data_), inc_)
        {}


        C tmp () {
            return C (vec.len(), vec.data(), vec.inc());
        }

        double *&data() const {
            const struct aa_dvec * cr = & this->vec;
            struct aa_dvec * r = const_cast<aa_dvec*>(cr);
            return  r->data;
        }

        bool operator<(const C &other) const {
            return this->data() < other.data();
        }

        bool operator<=(const C &other) const {
            return this->data() <= other.data();
        }

        bool operator>(const C &other) const {
            return this->data() > other.data();
        }

        bool operator>=(const C &other) const {
            return this->data() >= other.data();
        }

        bool operator==(const C &other) const {
            return this->data() == other.data();
        }

        bool operator!=(const C &other) const {
            return this->data() != other.data();
        }

        V &operator* () { return this->vec; }
    protected:
        V vec;
    };

    template <class C, typename D, typename V>
    class base_row_iterator : public base_iterator <C,D,V> {
    public:
        base_row_iterator(size_type len_, D *data_, size_type inc_) :
            base_iterator<C,D,V>(len_, data_, inc_)
        {}

        C &operator++ () {
            this->data()++;
            return *static_cast<C*>(this);
        }

        C operator++ (int) {
            C tmp = this->tmp();
            operator++();
            return tmp;
        }

        C &operator-- () {
            this->data()--;
            return this->vec;
        }

        C operator-- (int) {
            C tmp = this->tmp();
            operator--();
            return tmp;
        }

        C &operator+= (size_type s) {
            this->data() += s;
            return this->vec;
        }

        C &operator-= (size_type s) {
            this->data() -= s;
            return this->vec;
        }
    };

    class const_row_iterator : public base_row_iterator<const_row_iterator, const double, const DVec> {
    public:
        const_row_iterator(size_type len_, const double *data_, size_type inc_) :
            base_row_iterator(len_, data_, inc_)
        {}
    };

    class row_iterator : public base_row_iterator<row_iterator, double, DVec> {
    public:
        row_iterator( size_type len_, double *data_, size_type inc_) :
            base_row_iterator(len_, data_, inc_)
        {}
    };


    template <typename M, typename I>
    class base_row_container {
    public:
        base_row_container( M *mat_ ) : mat(mat_)
            { }

        I begin() { return DMat::row_begin(mat); }
        I end() { return DMat::row_end(mat); }
    private:
        M *mat;
    };

    template <class C, typename D, typename V>
    class base_col_iterator : public base_iterator <C,D,V> {
    public:
        base_col_iterator(size_type len_, D *data_, size_type ld_) :
            base_iterator<C,D,V>(len_, data_, 1),
            ld(ld_)
        {}

        C &operator++ () {
            this->data() += ld;
            return *static_cast<C*>(this);
        }

        C operator++ (int) {
            C tmp = this->tmp();
            operator++();
            return tmp;
        }

        C &operator-- () {
            this->data() -= ld;
            return this->vec;
        }

        C operator-- (int) {
            C tmp = this->tmp();
            operator--();
            return tmp;
        }

        C &operator+= (size_type s) {
            this->data() += s*ld;
            return this->vec;
        }

        C &operator-= (size_type s) {
            this->data() -= s*ld;
            return this->vec;
        }

    protected:
        size_type ld;
    };

    class const_col_iterator : public base_col_iterator<const_col_iterator, const double, const DVec> {
    public:
        const_col_iterator(size_type len_, const double *data_, size_type ld_) :
            base_col_iterator(len_, data_, ld_)
        {}
    };

    class col_iterator : public base_col_iterator<col_iterator, double, DVec> {
    public:
        col_iterator( size_type len_, double *data_, size_type ld_) :
            base_col_iterator(len_, data_, ld_)
        {}
    };

    template <typename M, typename I>
    class base_col_container {
    public:
        base_col_container( M *mat_ ) : mat(mat_)
            { }

        I begin() { return DMat::col_begin(mat); }
        I end() { return DMat::col_end(mat); }
    private:
        M *mat;
    };

public:
    static const_row_iterator row_begin(const struct aa_dmat *x) {
        return const_row_iterator(x->cols, x->data, x->ld);
    }

    static const_row_iterator row_end(const struct aa_dmat *x) {
        return const_row_iterator(x->cols, x->data+x->rows, x->ld);
    }

    static row_iterator row_begin(struct aa_dmat *x) {
        return row_iterator(x->cols, x->data, x->ld);
    }

    static row_iterator row_end(struct aa_dmat *x) {
        return row_iterator(x->cols, x->data+x->rows, x->ld);
    }

    base_row_container<aa_dmat,row_iterator> row_container() {
        return base_row_container<aa_dmat,row_iterator> (this);
    }

    base_row_container<const aa_dmat, const_row_iterator> row_container() const {
        return base_row_container<const aa_dmat, const_row_iterator> (this);
    }

    static const_col_iterator col_begin(const struct aa_dmat *x) {
        return const_col_iterator(x->rows, x->data, x->ld);
    }

    static const_col_iterator col_end(const struct aa_dmat *x) {
        return const_col_iterator(x->rows, x->data+x->cols*x->ld, x->ld);
    }

    static col_iterator col_begin(struct aa_dmat *x) {
        return col_iterator(x->rows, x->data, x->ld);
    }

    static col_iterator col_end(struct aa_dmat *x) {
        return col_iterator(x->rows, x->data+x->cols*x->ld, x->ld);
    }

    base_col_container<aa_dmat,col_iterator> col_container() {
        return base_col_container<aa_dmat,col_iterator> (this);
    }

    base_col_container<const aa_dmat, const_col_iterator> col_container() const {
        return base_col_container<const aa_dmat, const_col_iterator> (this);
    }

};

/***************/
/* EXPRESSIONS */
/***************/

/* Vector Scaling */
DVecExpScal<aa_dvec>
static operator*(double alpha, aa_dvec const &x) {
    return DVecExpScal<aa_dvec>(alpha,x);
}

DVecExpScal<aa_dvec>
static operator*(aa_dvec const &x, double alpha) { return alpha*x; }


template <typename E> DVecExpScal<E>
static operator*(double alpha,  DVecExpScal<E> const &x ) {
    return DVecExpScal<E>(alpha*x.alpha,x.x);
}

template <typename E> DVecExpScal<E>
static operator*( DVecExpScal<E> const &x, double alpha ) { return alpha*x; }


template <typename E> DVecExpScal< DVecExp<E> >
static operator*(double alpha,  DVecExp<E> const &x ) {
    return DVecExpScal< DVecExp<E> >( alpha, x );
}

template <typename E> DVecExpScal< DVecExp<E> >
static operator*(DVecExp<E> const &x, double alpha ) { return alpha * x; }

/* Matrix Scaling */

DMatExpScal<aa_dmat>
static operator*(double alpha, aa_dmat const &x) {
    return DMatExpScal<aa_dmat>(alpha,x);
}

DMatExpScal<aa_dmat>
static operator*( aa_dmat const &x, double alpha) {
    return alpha * x;
}

template <typename E> DMatExpScal<E>
static operator*(double alpha,  DMatExpScal<E> const &x ) {
    return DMatExpScal<E>(alpha*x.alpha,x.x);
}

template <typename E> DMatExpScal<E>
static operator*( DMatExpScal<E> const &x, double alpha ) {
    return alpha*x;
}

template <typename E> DMatExpScal< DMatExp<E> >
static operator*(double alpha,  DMatExp<E> const &x ) {
    return DMatExpScal< DMatExp<E> >(alpha,x);
}

template <typename E> DMatExpScal< DMatExp<E> >
static operator*( DMatExp<E> const &x, double alpha ) {
    return alpha*x;
}


DMatExpScal< DMatExpTranspose >
static operator*( double alpha, DMatExpTranspose const &At ) {
    return DMatExpScal< DMatExpTranspose >(alpha,At);
}

DMatExpScal< DMatExpTranspose >
static operator*( DMatExpTranspose const &At, double alpha ) {
    return alpha*At;
}

/* Vector Addition */

DVecExpAxpy<struct aa_dvec>
static operator+(struct aa_dvec const &x, struct aa_dvec const &y) {
    return DVecExpAxpy<struct aa_dvec>(1,x,y);
}

DVecExpAxpy<DVec>
static operator+(DVec const &x, DVec const &y) {
    return DVecExpAxpy<DVec>(1,x,y);
}


template <typename E> DVecExpAxpy< DVecExp<E> >
static operator+(struct aa_dvec const &x,  DVecExp<E>  const &y) {
    return DVecExpAxpy< DVecExp<E> >(1,x,y);
}

template <typename E> DVecExpAxpy< DVecExp<E> >
static operator+(DVecExp<E> const &y,struct aa_dvec const &x) { return x + y; }



template <typename E> DVecExpAxpy< DVecExp<E> >
static operator+( DVecExpScal<struct aa_dvec> const &a,  DVecExp<E>  const &b ) {
    return DVecExpAxpy< DVecExp<E> >(a.alpha,a.x,b);
}

template <typename E> DVecExpAxpy< DVecExp<E> >
static operator+( DVecExp<E> const &b, DVecExpScal<struct aa_dvec>  const &a) {
    return DVecExpAxpy<  DVecExp<E> >(a.alpha,a.x,b);
}

DVecExpAxpy< DVecExpScal< aa_dvec > >
static operator+( DVecExpScal< aa_dvec > const &b, DVecExpScal<aa_dvec>  const &a) {
    return DVecExpAxpy<  DVecExpScal<aa_dvec> >(a.alpha,a.x,b);
}


/* Matrix-Vector Multiplication */

DVecExpMV1
static operator*(aa_dmat const &A, aa_dvec const &x) {
    return DVecExpMV1(CblasNoTrans, 1, A, x);
}

DVecExpMV1
static operator*(double alpha, DVecExpMV1 const &y) {
    return DVecExpMV1(y.trans, alpha*y.alpha, y.A, y.x);
}

DVecExpMV1
static operator*(DVecExpMV1 const &y, double alpha) {
    return alpha*y;
}

DVecExpMV1
static operator*(DMatExpTranspose const &A, aa_dvec const &x) {
    return DVecExpMV1(CblasTrans, 1, A.x, x);
}

DVecExpMV1
static operator*(DMatExpTranspose const &A, DVecExpScal<aa_dvec> const &x) {
    return x.alpha * (A*x.x);
}

DVecExpMV1
static operator*(DMatExpScal<aa_dmat> const &A, aa_dvec const &x) {
    return A.alpha * (A.x*x);
}

DVecExpMV1
static operator*(aa_dmat const &A, DVecExpScal<aa_dvec> const &x) {
    return x.alpha*(A*x.x);
}

DVecExpMV1
static operator*(DMatExpScal<aa_dmat> const &A, DVecExpScal<aa_dvec> const &x) {
    return (A.alpha*x.alpha)*(A.x*x.x);
}

/* MV and addition */

template <typename E>
DVecExpMV2< DVecExp<E> >
static operator+(DVecExpMV1 const &b,  DVecExp<E>  const &y) {
    return DVecExpMV2< DVecExp<E> >( b.trans, b.alpha, b.A, b.x, 1, y);
}

template <typename E>
DVecExpMV2< DVecExp<E> >
static operator+( DVecExp<E>  const &y, DVecExpMV1 const &b ) {
    return b+y;
}


DVecExpMV2<aa_dvec>
static operator+(DVecExpMV1 const &b,  aa_dvec  const &y) {
    return DVecExpMV2< aa_dvec >( b.trans, b.alpha, b.A, b.x, 1, y);
}

DVecExpMV2<aa_dvec>
static operator+( aa_dvec  const &y, DVecExpMV1 const &b ) {
    return b+y;
}

template <typename E> DVecExpMV2<E>
static operator+(DVecExpMV1 const &b, DVecExpScal<E> const &y) {
    return DVecExpMV2<E>( b.trans, b.alpha, b.A, b.x, y.alpha, y.x);
}

template <typename E> DVecExpMV2<E>
static operator+( DVecExpScal<E> const &y, DVecExpMV1 const &b) {
    return b+y;
}

template <typename E> DVecExpMV2<E>
static operator*( double alpha, DVecExpMV2<E> const &b) {
    return DVecExpMV2<E>( b.trans, alpha*b.alpha, b.A, b.x,
                          alpha*b.beta, b.y);
}

template <typename E> DVecExpMV2<E>
static operator*( DVecExpMV2<E> const &b, double alpha ) {
    return alpha * b;
}


/* MM */

DMatExpMM1
static operator*( aa_dmat const &A, aa_dmat const &B ) {
    return DMatExpMM1(CblasNoTrans, CblasNoTrans, 1, A, B);
}

DMatExpMM1
static operator*( DMatExpTranspose const &A, aa_dmat const &B ) {
    return DMatExpMM1(CblasTrans, CblasNoTrans, 1, A.x, B);
}

DMatExpMM1
static operator*( aa_dmat const &A, DMatExpTranspose  const &B ) {
    return DMatExpMM1(CblasNoTrans, CblasTrans, 1, A, B.x);
}

DMatExpMM1
static operator*( DMatExpTranspose const &A, DMatExpTranspose const &B ) {
    return DMatExpMM1(CblasTrans, CblasTrans, 1, A.x, B.x);
}


DMatExpMM1
static operator*( double a, DMatExpMM1 const &C )  {
    return DMatExpMM1(C.transA, C.transB, a*C.alpha, C.A, C.B);
}

DMatExpMM1
static operator*( DMatExpMM1 const &C, double a ) {
    return a*C;
}



template<typename E>
DMatExpMM1
static operator*( DMatExpScal< E > const &A, aa_dmat const &B) {
    return A.alpha * (A.x*B);
}

template<typename E>
DMatExpMM1
static operator*( DMatExpScal< E > const &A, DMatExpTranspose const &B) {
    return A.alpha * (A.x*B);
}

template<typename E>
DMatExpMM1
static operator*( aa_dmat const &A, DMatExpScal< E > const &B) {
    return B.alpha * (A*B.x);
}

template<typename E>
DMatExpMM1
static operator*( DMatExpTranspose const &A, DMatExpScal< E > const &B) {
    return B.alpha * (A*B.x);
}

template<typename Ea, typename Eb>
DMatExpMM1
static operator*( DMatExpScal< Ea > const &A,
           DMatExpScal< Eb > const &B) {
    return (A.alpha*B.alpha) * (A.x*B.x);
}

/* MM2 */

DMatExpMM2< aa_dmat >
static operator+(DMatExpMM1 const &M,  aa_dmat  const &C) {
    return DMatExpMM2< aa_dmat >( M.transA, M.transB,
                                     M.alpha, M.A, M.B,
                                     1, C );
}

DMatExpMM2< aa_dmat >
static operator+( aa_dmat  const &C, DMatExpMM1 const &M) { return M+C; }

template <typename E> DMatExpMM2< DMatExp<E> >
static operator+(DMatExpMM1 const &M,  DMatExp<E>  const &C) {
    return DMatExpMM2< DMatExp<E> >( M.transA, M.transB,
                                     M.alpha, M.A, M.B,
                                     1, C );
}

template <typename E> DMatExpMM2< DMatExp<E> >
static operator+(DMatExp<E>  const &C, DMatExpMM1 const &M) { return M+C; }


template <typename E> DMatExpMM2< E >
static operator+(DMatExpMM1 const &M,  DMatExpScal<E>  const &C) {
    return DMatExpMM2< E >( M.transA, M.transB,
                            M.alpha, M.A, M.B,
                            C.alpha, C.x );
}

template <typename E> DMatExpMM2< E >
static operator+( DMatExpScal<E>  const &C, DMatExpMM1 const &M) {
    return M + C;
}

template <typename E> DMatExpMM2< E >
static operator*(double alpha, DMatExpMM2<E> const &M ) {
    return DMatExpMM2< E >( M.transA, M.transB,
                            alpha*M.alpha, M.A, M.B,
                            alpha*M.beta, M.C );
}

template <typename E> DMatExpMM2< E >
static operator*( DMatExpMM2<E> const &M, double alpha ) {
    return alpha*M;
}

}
;


#endif //AMINO_MAT_HPP
