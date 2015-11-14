/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011-2012, Georgia Tech Research Corporation
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

#include "amino/def.h"

void AA_NAME(la,transpose) ( size_t m, size_t n,
                             const AA_TYPE *A, size_t lda,
                             AA_TYPE *B, size_t ldb ) {
    for( size_t i=0, ia=0; i < n; i++, ia+=lda ) {
        for( size_t j=0, ib=0; j < m; j++, ib+=ldb ) {
            B[ib+i] = A[ia+j];
        }
    }
}


#define LA_WORK(REGION, WORK, LWORK, EXP)                               \
    {                                                                   \
        int LWORK = -1;                                                 \
        for(;;) {                                                       \
            AA_TYPE *WORK =                                             \
                (AA_TYPE*)aa_mem_region_tmpalloc( REGION, sizeof(AA_TYPE)* \
                                                  (size_t)(LWORK < 0 ? 1 : LWORK) ); \
            EXP;                                                        \
            if( LWORK >= 0 ) break;                                     \
            LWORK = (int) WORK[0];                                      \
        }                                                               \
    }


static void AA_NAME(la,opt_hungarian) (
    size_t n, AA_TYPE *A, size_t lda,
    ssize_t *row_assign,
    ssize_t *col_assign )
{
    ssize_t *row_cover = (ssize_t*)aa_mem_region_local_alloc(sizeof(ssize_t)*n);   // length n
    ssize_t *col_cover = (ssize_t*)aa_mem_region_local_alloc(sizeof(ssize_t)*n);   // length n
    ssize_t *mask = (ssize_t*)aa_mem_region_local_alloc(sizeof(ssize_t)*n*n);      // length n*n
    ssize_t *path = (ssize_t*)aa_mem_region_local_alloc(sizeof(ssize_t)*2*n*n);    // length 2*n*n

    const AA_TYPE max_cost =
        AA_NAME(la,mat_max)(n,n,A,lda,NULL,NULL);
    //int mask[n*n];
    size_t zerorc[2] = {0};
    memset(mask,0,n*n*sizeof(mask[0]));
    memset(row_cover,0,n*sizeof(row_cover[0]));
    memset(col_cover,0,n*sizeof(col_cover[0]));
    // -- Step 1 ----
    // for each column
    for( size_t j = 0; j < n; j ++ ) {
        // find smallest element
        AA_TYPE xmin =
            AA_MATREF( A, lda,
                       AA_NAME(la,minloc)(n, &AA_MATREF(A, lda, 0, j), 1),
                       j );
        // subtract from all entries in the column
        for( size_t i = 0; i < n; i++ ) AA_MATREF(A,lda,i,j) -= xmin;
    }
    // for each row
    for( size_t i = 0; i < n; i ++ ) {
        // find smallest element
        size_t jmin = AA_NAME(la,minloc)(n, &AA_MATREF(A, lda, i, 0),
                                         lda);
        AA_TYPE xmin = AA_MATREF( A, lda, i, jmin );
        // subtract from all entries in the row
        for( size_t j = 0; j < n; j++ ) AA_MATREF(A,lda,i,j) -= xmin;
    }

    // -- Step 2 ----
    // Mark mask with star for each zero in cost
    for( size_t j = 0; j < n; j ++ ) {
        for( size_t i = 0; i < n; i++ ) {
            if( 0 >= AA_MATREF(A,lda,i,j) &&
                0 == row_cover[i] &&
                0 == col_cover[j]
                )
            {
                row_cover[i] = col_cover[j] = 1;
                AA_MATREF(mask,n,i,j) = 1;
                break;
            }
        }
    }

    memset(row_cover,0,n*sizeof(row_cover[0]));
    memset(col_cover,0,n*sizeof(col_cover[0]));

    int step = 3;
    do {
        switch( step ) {
        case 3:
            assert( 3 == step );
            // cover starred columns, check if all columns covered
            for (size_t j = 0; j < n; j++ ) {
                for( size_t i = 0; i < n; i ++ ) {
                    if( 1 == AA_MATREF(mask,n,i,j) ) {
                        assert( 0 >= AA_MATREF(A,lda,i,j) );
                        col_cover[j] = 1;
                    }
                }
            }
            { //check if all columns covered
                size_t count = 0;
                for (size_t j = 0; j < n; j++ ) {
                    if( col_cover[j] ) count++;
                }
                if( count >= n )
                    step = 7;
                else
                    step = 4;
            }
            break;
        case 4: ;
            assert( 4 == step );
            // Find uncovered zero and prime it or goto 6.
            // If star in row, cover row, uncover star's column,
            // repeat for all uncovered zeros.  Otherwise, goto 5.

            while(4 == step) {
                // find uncovered zero for rowcol
                for( size_t j = 0; j < n; j++ ) {
                    for( size_t i = 0;  i < n; i++ ) {
                        if( 0 >= AA_MATREF(A, lda, i, j) &&
                            0 == row_cover[i] &&
                            0 == col_cover[j] )
                        {
                            // prime the found uncovered zero
                            AA_MATREF(mask,n,i,j) = 2;
                            for( size_t k = 0; k < n; k ++ ) {
                                if( 1 == AA_MATREF( mask, n, i, k ) ) {
                                    row_cover[i] = 1; // cover star's row
                                    col_cover[k] = 0; // uncover star's column
                                    goto STEP4_BREAK2; // break from nested loop
                                }
                            }
                            // nothing is starred in the row
                            zerorc[0] = i;
                            zerorc[1] = j;
                            step = 5;

                            goto STEP4_BREAK2; // break out of nested loop

                        }
                    }
                }
                // step 6 when no uncovered zeros
                step = 6;
            STEP4_BREAK2:; // come here from inner loops
            }
            break;
        case 5: ;
            assert( 5 == step );
            // construct series of alternating primes and stars
            size_t count = 0;
            {
                AA_MATREF(path,n*n,count,0) = (ssize_t)zerorc[0];
                AA_MATREF(path,n*n,count,1) = (ssize_t)zerorc[1];
                // loop till we can't find a starred column
                for( size_t i = 0; i < n; ) {
                    //dump_imat( path, 2, 1+count);
                    assert( count < n*n );
                    // find star n column
                    for (i = 0; i < n; i ++ ) {
                        if( 1 == AA_MATREF(mask,n,i,
                                           (size_t)AA_MATREF(path,n*n,
                                                             count,1) ) )
                        {
                            count++;
                            // row of starred zero
                            AA_MATREF(path,n*n,count,0) = (ssize_t)i;
                            // col of starred zero
                            AA_MATREF(path,n*n,count,1) =
                                AA_MATREF(path,n*n,count-1,1);

                            for (size_t j = 0; j < n; j ++ ) {
                                if( 2 == AA_MATREF(mask,n,
                                                   (size_t)AA_MATREF(path, n*n,
                                                                     count, 0),
                                                   j) )
                                {
                                    count++;
                                    // row of primed zero
                                    AA_MATREF(path,n*n,count,0) =
                                        AA_MATREF(path,n*n,count-1,0);
                                    // col of primed zero
                                    AA_MATREF(path,n*n,count,1) =
                                        (ssize_t)j;
                                    break;
                                }
                            }
                        }
                    }
                }
                // convert path
                //printf("convert path: %d\n", count);
                //dump_imat( path, 2, 1+count);
                for( size_t i = 0; i <= count; i ++ ) {
                    ssize_t *ptr = &AA_MATREF(mask, n,
                                              (size_t)AA_MATREF(path,n*n,i,0),
                                              (size_t)AA_MATREF(path,n*n,i,1));
                    *ptr = (1==*ptr) ? 0 : 1;
                }
                //dump_imat( mask, n, n );
            }
            // clear covers
            memset(row_cover,0,n*sizeof(row_cover[0]));
            memset(col_cover,0,n*sizeof(col_cover[0]));
            // erase primes
            for( size_t j = 0; j < n; j ++ ) {
                for( size_t i = 0; i < n; i ++ ) {
                    ssize_t *ptr = &AA_MATREF(mask,n,i,j);
                    if( 2 ==  *ptr ) *ptr = 0;
                }
            }
            step = 3;
            break;
        case 6: ;
            assert( 6 == step );
            // find smallest uncovered value in cost
            AA_TYPE minval = max_cost;
            for( size_t j = 0; j < n; j ++ ) {
                for( size_t i = 0; i < n; i ++ ) {
                    if( 0 == row_cover[i] &&
                        0 == col_cover[j] &&
                        AA_MATREF(A,lda,i,j) < minval ) {
                        minval = AA_MATREF(A,lda,i,j);
                    }
                }
            }
            // add to every element of the covered rows
            // subtract from every elemented of uncovered columns
            for( size_t j = 0; j < n; j ++ ) {
                for( size_t i = 0; i < n; i ++ ) {
                    if( 1 == row_cover[i] )
                        AA_MATREF(A,lda,i,j) += minval;
                    if( 0 == col_cover[j] )
                        AA_MATREF(A,lda,i,j) -= minval;
                }
            }
            step = 4;
            break;
        }
    } while (step < 7 );

    // compute assignments
    for( size_t j = 0; j < n; j ++ ) {
        for( size_t i = 0; i < n; i ++ ) {
            if( 1 == AA_MATREF(mask,n,i,j) ) {
                if( row_assign )
                    row_assign[i] = (int)j;
                if( col_assign )
                    col_assign[j] = (int)i;
            }
        }
    }

    aa_mem_region_local_pop(row_cover);
}

AA_API void AA_NAME(la,assign_hungarian_max2min) (
    size_t m, size_t n,
    AA_TYPE *A, size_t lda )
{
    AA_TYPE max = AA_NAME(la,mat_max)(m,n,A,lda,NULL,NULL);
    for( size_t j = 0; j < n; j ++ ) {
        for( size_t i = 0; i < m; i ++ ) {
            AA_MATREF(A,lda,i,j) = max - AA_MATREF(A,lda,i,j);
        }
    }
}

AA_API void AA_NAME(la,assign_hungarian) (
    size_t m, size_t n, const AA_TYPE *A, size_t lda,
    ssize_t *row_assign,
    ssize_t *col_assign )
{

    size_t p = AA_MAX(m,n);
    size_t q = AA_MIN(m,n);
    ssize_t *alt_row = row_assign, *alt_col = col_assign;


    /* copy A into work */
    AA_TYPE *B = (AA_TYPE*)aa_mem_region_local_alloc( sizeof(AA_TYPE)*p*p);
    AA_CLA_NAME(lacpy)( 0, (int)m, (int)n, A, (int)lda, B, (int)p );

    /* zero pad work */
    if( m > n ) {
        AA_CLA_NAME(laset)(0, (int)p, (int)(p-q), 0, 0,
                           B+p*n, (int)p );
        alt_col = (ssize_t*)aa_mem_region_local_alloc( sizeof(AA_TYPE*)*p);
    } else if( m < n ) {
        AA_CLA_NAME(laset)(0, (int)(p-q), (int)p, 0, 0,
                           B+m, (int)p );
        alt_row = (ssize_t*)aa_mem_region_local_alloc( sizeof(AA_TYPE*)*p);
    } /* else we're square */

    /* run hungarian */
    AA_NAME(la,opt_hungarian)( p, B, p, alt_row, alt_col );

    /* fixup assignments */
    if( m > n ) {
        AA_MEM_CPY( col_assign, alt_col, n );
        for( size_t i = 0; i < m; i ++ ) {
            if( row_assign[i] >= (ssize_t)n ) {
                row_assign[i] = -1;
            }
        }
    } else if( m < n ) {
        AA_MEM_CPY( row_assign, alt_row, m );
        for( size_t i = 0; i < n; i ++ ) {
            if( col_assign[i] >= (ssize_t)m ) {
                col_assign[i] = -1;
            }
        }
    } /* else we're square */


    aa_mem_region_local_pop(B);
}

AA_API void AA_NAME(la,lerp)
( size_t n, AA_TYPE u,
  const AA_TYPE *v1, size_t inc1,
  const AA_TYPE *v2, size_t inc2,
  AA_TYPE *vu, size_t incu )
{
    for( size_t i=0, j=0, k=0;
         n--;
         i+=inc1, j+=inc2, k+=incu
        )
    {
        vu[k] = v1[i] + u * (v2[j] - v1[i]);
    }
}

void AA_NAME(la,colmean)
( size_t m, size_t n,
  const AA_TYPE *A, size_t lda,
  AA_TYPE *x)
{
    memset( x, 0, sizeof(x[0])*m );
    for( size_t i=0, j=0; i < n; i++, j+=lda ) {
        AA_CBLAS_NAME(axpy)( (int)m, 1.0, A+j, (int)1,
                             x, 1 );
    }
    AA_CBLAS_NAME(scal)((int)m, ((AA_TYPE)1.0)/(AA_TYPE)n,
                        x, 1);
}


void AA_NAME(la,rowmean)
( size_t m, size_t n,
  const AA_TYPE *A, size_t lda,
  AA_TYPE *x)
{
    memset( x, 0, sizeof(x[0])*n );
    for( size_t i=0; i < m; i++ ) {
        AA_CBLAS_NAME(axpy)( (int)n, 1.0, A+i, (int)lda,
                             x, 1 );
    }
    AA_CBLAS_NAME(scal)((int)n, ((AA_TYPE)1.0)/(AA_TYPE)m,
                        x, 1);
}

void AA_NAME(la,colcov)
( size_t m, size_t n,
  const AA_TYPE *A, size_t lda,
  const AA_TYPE *x,
  AA_TYPE *E, size_t lde ) {
    for( size_t j = 0; j < m*lde; j+=lde ) {
        memset( E+j, 0, sizeof(E[0])*m );
    }
    for( size_t j = 0; j < n*lda; j+=lda )
    {
        /* t := - mu + A_i */
        AA_TYPE t[m];
        memcpy( t, A+j, sizeof(t[0])*m );
        AA_CBLAS_NAME(axpy) ((int)m, -1.0, x, (int)1,
                             t, 1 );
        /* E += t * t' */
        AA_CBLAS_NAME(syr)( CblasColMajor, CblasUpper,
                            (int)m, 1.0, t, 1,
                            E, (int)lde );
    }
    AA_CBLAS_NAME(scal) ( (int)(m*m),
                          (AA_TYPE)1.0/(AA_TYPE)(n-1),
                          E, 1 );
    /* fill lower half */
    for( size_t i = 0; i < m; i ++ ) {
        for( size_t j = i+1; j < m; j ++ ) {
            AA_MATREF(E, m, j, i) = AA_MATREF(E, m, i, j);
        }
    }
}




AA_API void AA_NAME(la,lls)
( size_t m, size_t n, size_t p,
  const AA_TYPE *A, size_t lda,
  const AA_TYPE *b, size_t ldb,
  AA_TYPE *x, size_t ldx ) {

    int mi=(int)m, ni=(int)n, pi=(int)p;
    AA_TYPE rcond=-1;
    struct aa_mem_region *reg = aa_mem_region_local_get();

    size_t ldbp = AA_MAX(m,n);
    AA_TYPE *Ap = AA_MEM_REGION_NEW_N(reg, AA_TYPE, m*n);
    AA_TYPE *bp = AA_MEM_REGION_NEW_N(reg, AA_TYPE, ldbp*p);
    AA_MEM_ZERO(bp, ldbp*p);


    AA_CLA_NAME(lacpy)(0, (int)m,(int)n, A, (int)lda, Ap, (int)m);
    AA_CLA_NAME(lacpy)(0, (int)m,(int)p, b, (int)ldb, bp, (int)ldbp);

    int rank, info;
    size_t liwork = (size_t)AA_CLA_NAME(gelsd_miniwork)(mi,ni);
    size_t ls = AA_MIN(m,n);
    AA_TYPE *S = AA_MEM_REGION_NEW_N(reg, AA_TYPE, ls);
    int *iwork = AA_MEM_REGION_NEW_N(reg, int, liwork);

    LA_WORK( reg, work, lwork,
             info = AA_CLA_NAME(gelsd)( mi, ni, pi,
                                        Ap, mi, bp, (int)ldbp,
                                        S, &rcond, &rank,
                                        work, lwork, iwork ) );
    AA_CLA_NAME(lacpy)(0, (int)n,(int)p, bp, (int)ldbp, x, (int)ldx);
    aa_mem_region_pop(reg, Ap);
}

AA_API int AA_NAME(la,qr)
(size_t m, size_t n, const AA_TYPE *A, size_t lda,
 AA_TYPE *Q, size_t ldq,
 AA_TYPE *R, size_t ldr)
{
    size_t mn = AA_MIN(m,n);
    int mi = (int)m, ni=(int)n;

    struct aa_mem_region *reg = aa_mem_region_local_get();
    AA_TYPE *tau = AA_MEM_REGION_NEW_N(reg, AA_TYPE, mn);
    AA_TYPE *Ap = (AA_TYPE*)AA_MEM_REGION_NEW_N(reg, AA_TYPE, m*n);

    AA_CLA_NAME(lacpy)(0, mi, ni, A, (int)lda,
                       Ap, mi);

    int info;
    //printf("\n\n");
    //aa_dump_mat( stdout, Ap, mi, n );

    LA_WORK( reg, work, lwork,
             info = AA_CLA_NAME(geqrf)(mi, ni, Ap, (int)mi,
                                       tau, work, lwork) );
    //printf("\n\n");
    //aa_dump_mat( stdout, Ap, mi, n );

    if( R ) {
        /* Fill R */
        AA_CLA_NAME(laset)('L', mi, ni,
                           0, 0,
                           R, ldr );
        AA_CLA_NAME(lacpy)('U', mn, ni, Ap, (int)lda,
                           R, ldr);

    }

    if( Q ) {
        /* Fill Q */
        AA_CLA_NAME(lacpy)('L', mi, ni, Ap, (int)lda,
                           Q, ldq);
        LA_WORK( reg, work, lwork,
                 AA_CLA_NAME(orgqr)(mi, mi, mn,
                                    Q, ldq, tau,
                                    work, lwork ) );
    }

    aa_mem_region_pop(reg, tau);
}


AA_API int AA_NAME(la,svd)
( size_t m, size_t n, const AA_TYPE *A, size_t lda,
  AA_TYPE *U, size_t ldu,
  AA_TYPE *S,
  AA_TYPE *Vt, size_t ldvt ) {

    int mi = (int)m, ni=(int)n;
    struct aa_mem_region *reg = aa_mem_region_local_get();

    AA_TYPE *Ap = AA_MEM_REGION_NEW_N( reg, AA_TYPE, m*n );
    AA_CLA_NAME(lacpy)(0, mi, ni, A, (int)lda,
                       Ap, mi);

    const char *jobu = (U && ldu > 0) ? "A" : "N";
    const char *jobvt = (Vt && ldvt > 0) ? "A" : "N";
    int info;

    LA_WORK( reg, work, lwork,
             AA_LAPACK_NAME(gesvd)( jobu, jobvt, &mi, &ni,
                                    Ap, &mi,
                                    S, U, &mi,
                                    Vt, &ni,
                                    &work[0], &lwork, &info ) );

    aa_mem_region_pop(reg, Ap);

    //finish
    return info;
}


AA_API int AA_NAME(la,eev)
( size_t n, const AA_TYPE *A, size_t lda,
  AA_TYPE *wr,
  AA_TYPE *wi,
  AA_TYPE *Vl, size_t ldvl,
  AA_TYPE *Vr, size_t ldvr )
{
    int info = -1;
    int ni = (int)n;
    int ldai = (int)lda;
    struct aa_mem_region *reg = aa_mem_region_local_get();


    const char *jobvl = Vl ? "V" : "N";
    int ldvli = Vl ? (int)ldvl : 1;
    const char *jobvr = Vr ? "V" : "N";
    int ldvri = Vr ? (int)ldvr : 1;

    AA_TYPE *Ap = AA_MEM_REGION_NEW_N( reg, AA_TYPE, n*n );
    AA_CLA_NAME(lacpy)(0, ni, ni, A, (int)lda,
                       Ap, ni);

    LA_WORK( reg, work, lwork,
             AA_LAPACK_NAME(geev)( jobvl, jobvr,
                                   &ni, Ap, &ldai,
                                   wr, wi,
                                   Vl, &ldvli,
                                   Vr, &ldvri,
                                   work, &lwork, &info ) );

    aa_mem_region_pop(reg, Ap);
    return info;
}



int AA_NAME(la,compar)( const void *_a, const void *_b )
{
    double a = *(double*)_a;
    double b = *(double*)_b;
    if( a < b ) return -1;
    if( a > b ) return 1;
    return 0;
}

AA_TYPE AA_NAME(la,nmedian)( size_t n, AA_TYPE *x )
{
    if( 0 == n ) {
        return 0; // is this reasonable?
    } else if( 1 == n ) {
        return *x;
    } else {
        // this is not great
        // TODO: linear time select algorithm
        aa_aheap_sort( x, n, sizeof(AA_TYPE), AA_NAME(la,compar) );
        size_t i = n/2;
        if ( 1 == n%2 ) { // odd
            return x[i];
        } else { // even
            return (x[i] + x[i-1])/2;
        }
    }
}

AA_TYPE AA_NAME(la,median)( size_t n, const AA_TYPE *x, size_t incx )
{
    AA_TYPE *y = AA_MEM_REGION_LOCAL_NEW_N(AA_TYPE,n);
    AA_CBLAS_NAME(copy) ( (int)n, x, (int)incx, y, 1 );
    return AA_NAME(la,nmedian_pop)( n, y );
}

AA_TYPE AA_NAME(la,mad)( size_t n, const AA_TYPE u, const AA_TYPE *x, size_t incx )
{
    AA_TYPE *P = AA_MEM_REGION_LOCAL_NEW_N(AA_TYPE,n);

    // compute distances
    for( size_t i = 0; i < n; i ++ ) {
        P[i] = (AA_TYPE)fabs( u - x[i*incx] );
    }

    return AA_NAME(la,nmedian_pop)( n, P );
}

AA_TYPE AA_NAME(la,mad2)( size_t m, size_t n, const AA_TYPE *u, const AA_TYPE *A, size_t lda )
{
    AA_TYPE *P = AA_MEM_REGION_LOCAL_NEW_N(AA_TYPE,n);

    // compute distances
    for( size_t i = 0; i < n; i ++ ) {
        P[i] = (AA_TYPE)sqrt( AA_NAME(la,ssd)(m, AA_MATCOL(A,lda,i), 1, u, 1) );
    }
    return AA_NAME(la,nmedian_pop)( n, P );
}


AA_TYPE AA_NAME(la,ssd) (
    size_t n,
    const AA_TYPE *x, size_t incx,
    const AA_TYPE *y, size_t incy
    )
{
    double a = 0;
    for( size_t i = 0, j=0;
         n--;
         i+=incx, j+=incy
        )
    {
        double t = x[i] - y[j];
        a += t*t;
    }
    return (AA_TYPE)a;
}


/* Method from: Kahan, Willliam. How futile are mindless assessments
 * of roundoff in floating-point computation. 2006
 */
AA_TYPE AA_NAME(la,angle) (
    size_t n,
    const AA_TYPE *x, size_t incx,
    const AA_TYPE *y, size_t incy
    )
{
    double nx = AA_CBLAS_NAME(nrm2)((int)n, x, (int)incx);
    double ny = AA_CBLAS_NAME(nrm2)((int)n, y, (int)incy);
    double s=0, c=0;

    for( size_t i = 0, j=0;
         n--;
         i+=incx, j+=incy
        )
    {
        double a = ny*x[i];
        double b = nx*y[j];
        double ts = a-b;
        double tc = a+b;
        s += ts*ts;
        c += tc*tc;
    }
    return (AA_TYPE)(2*atan2( sqrt(s), sqrt(c)));
}

AA_API void AA_NAME(la, colfit) (
    size_t m, size_t n,
    const AA_TYPE *A, size_t lda, AA_TYPE *x
    )
{

    struct aa_mem_region *reg = aa_mem_region_local_get();
    AA_TYPE *At = (AA_TYPE*)
        aa_mem_region_alloc(reg, sizeof(AA_TYPE)*m*n);
    AA_TYPE *b = (AA_TYPE*)
        aa_mem_region_alloc(reg, sizeof(AA_TYPE)*n);
    AA_TYPE *xout = (AA_TYPE*)
        aa_mem_region_alloc(reg, sizeof(AA_TYPE)*m);

    // construct normed A,b matrix
    for( size_t i = 0, j=0;
         i < n;
         j+=lda, i++ )
    {
        // Copy normed columns of A into At
        const AA_TYPE *aa = A+j; // column of A
        double g = AA_CBLAS_NAME(nrm2)( (int)m, aa, 1 );

        b[i] = (AA_TYPE)(-1/g);
        for( size_t q=0,k=i; q<m; q++,k+=n ) {
            At[k] = (AA_TYPE)(aa[q]/g);
        }
    }
    // solve
    AA_NAME(la,lls)( n, m, 1,
                     At, n, b, n, xout, m );
    // normalize
    double d = AA_CBLAS_NAME(nrm2)((int)m, xout, (int)1);
    for( size_t i = 0; i < m; i ++ ) {
        x[i] = (AA_TYPE)(xout[i]/d);
    }
    x[m] = (AA_TYPE)(1.0/d);

    aa_mem_region_pop(reg, At);
}

AA_API AA_TYPE AA_NAME(la, vecstd) (
    size_t n,
    const AA_TYPE *x, size_t incx,
    AA_TYPE mu)
{
    double a=0;
    for( size_t p=n,i=0;
         p--;
         i+=incx )
    {
        double t=x[i]-mu;
        a += t*t;
    }
    return (AA_TYPE)sqrt( a/(double)(n-1) );
}

#include "amino/undef.h"
