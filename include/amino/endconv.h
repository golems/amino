/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008-2011, Georgia Tech Research Corporation
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

#ifndef _ENDCONV_H
#define _ENDCONV_H

/**
 * \file endconv.h
 *
 * \brief Routines to convert numbers to different endian formats.
 *
 * \author Neil Dantam
 *
 */

/* See the README and the doxygen docs for usage info. */



/****************/
/** PROTOTYPES **/
/****************/

/// Memory Load: convert little endian src to native type
static inline int64_t aa_endconv_ld_le_i64( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_i64( void *dst, int64_t src);
/// Memory Load: convert big-endian src to native
static inline int64_t aa_endconv_ld_be_i64( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_i64( void *dst, int64_t src);
/// convert native representation to little-endian
static inline int64_t aa_endconv_h_to_le_i64( int64_t i_host);
/// convert little-endian representation to native
static inline int64_t aa_endconv_le_to_h_i64( int64_t i_le);
/// convert native representation to big-endian
static inline int64_t aa_endconv_h_to_be_i64( int64_t i_host);
/// convert big-endian representation to native
static inline int64_t aa_endconv_be_to_h_i64( int64_t i_be);


/// Memory Load: convert little endian src to native type
static inline uint64_t aa_endconv_ld_le_u64( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_u64( void *dst, uint64_t src);
/// Memory Load: convert big-endian src to native
static inline uint64_t aa_endconv_ld_be_u64( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_u64( void *dst, uint64_t src);
/// convert native representation to little-endian
static inline uint64_t aa_endconv_h_to_le_u64( uint64_t i_host);
/// convert little-endian representation to native
static inline uint64_t aa_endconv_le_to_h_u64( uint64_t i_le);
/// convert native representation to big-endian
static inline uint64_t aa_endconv_h_to_be_u64( uint64_t i_host);
/// convert big-endian representation to native
static inline uint64_t aa_endconv_be_to_h_u64( uint64_t i_be);


/// Memory Load: convert little endian src to native type
static inline int32_t aa_endconv_ld_le_i32( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_i32( void *dst, int32_t src);
/// Memory Load: convert big-endian src to native
static inline int32_t aa_endconv_ld_be_i32( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_i32( void *dst, int32_t src);
/// convert native representation to little-endian
static inline int32_t aa_endconv_h_to_le_i32( int32_t i_host);
/// convert little-endian representation to native
static inline int32_t aa_endconv_le_to_h_i32( int32_t i_le);
/// convert native representation to big-endian
static inline int32_t aa_endconv_h_to_be_i32( int32_t i_host);
/// convert big-endian representation to native
static inline int32_t aa_endconv_be_to_h_i32( int32_t i_be);

/// Memory Load: convert little endian src to native type
static inline uint32_t aa_endconv_ld_le_u32( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_u32( void *dst, uint32_t src);
/// Memory Load: convert big-endian src to native
static inline uint32_t aa_endconv_ld_be_u32( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_u32( void *dst, uint32_t src);
/// convert native representation to little-endian
static inline uint32_t aa_endconv_h_to_le_u32( uint32_t i_host);
/// convert little-endian representation to native
static inline uint32_t aa_endconv_le_to_h_u32( uint32_t i_le);
/// convert native representation to big-endian
static inline uint32_t aa_endconv_h_to_be_u32( uint32_t i_host);
/// convert big-endian representation to native
static inline uint32_t aa_endconv_be_to_h_u32( uint32_t i_be);


/// Memory Load: convert little endian src to native type
static inline int16_t aa_endconv_ld_le_i16( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_i16( void *dst, int16_t src);
/// Memory Load: convert big-endian src to native
static inline int16_t aa_endconv_ld_be_i16( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_i16( void *dst, int16_t src);
/// convert native representation to little-endian
static inline int16_t aa_endconv_h_to_le_i16( int16_t i_host);
/// convert little-endian representation to native
static inline int16_t aa_endconv_le_to_h_i16( int16_t i_le);
/// convert native representation to big-endian
static inline int16_t aa_endconv_h_to_be_i16( int16_t i_host);
/// convert big-endian representation to native
static inline int16_t aa_endconv_be_to_h_i16( int16_t i_be);

/// Memory Load: convert little endian src to native type
static inline uint16_t aa_endconv_ld_le_u16( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_u16( void *dst, uint16_t src);
/// Memory Load: convert big-endian src to native
static inline uint16_t aa_endconv_ld_be_u16( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_u16( void *dst, uint16_t src);
/// convert native representation to little-endian
static inline uint16_t aa_endconv_h_to_le_u16( uint16_t i_host);
/// convert little-endian representation to native
static inline uint16_t aa_endconv_le_to_h_u16( uint16_t i_le);
/// convert native representation to big-endian
static inline uint16_t aa_endconv_h_to_be_u16( uint16_t i_host);
/// convert big-endian representation to native
static inline uint16_t aa_endconv_be_to_h_u16( uint16_t i_be);


/// Memory Load: convert little endian src to native type
static inline double aa_endconv_ld_le_d( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_d( void *dst, double src);
/// Memory Load: convert big-endian src to native
static inline double aa_endconv_ld_be_d( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_d( void *dst, double src);

/// Memory Load: convert little endian src to native type
static inline double aa_endconv_ld_le_d( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_d( void *dst, double src);
/// Memory Load: convert big-endian src to native
static inline double aa_endconv_ld_be_d( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_d( void *dst, double src);

/// Memory Load: convert little endian src to native type
static inline float aa_endconv_ld_le_s( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_s( void *dst, float src);
/// Memory Load: convert big-endian src to native
static inline float aa_endconv_ld_be_s( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_s( void *dst, float src);

/// Memory Load: convert little endian src to native type
static inline float aa_endconv_ld_le_s( void *src );
/// convert native to src to little-endian dst
static inline void aa_endconv_st_le_s( void *dst, float src);
/// Memory Load: convert big-endian src to native
static inline float aa_endconv_ld_be_s( void *src );
/// convert native src to big-endian dst
static inline void aa_endconv_st_be_s( void *dst, float src);



/*************/
/*  HELPERS  */
/*************/

/// copy src to dst in reverse byte order
static inline void aa_endconv_reverse( void * dst , void *src, size_t len ) {
    size_t i;
    for( i = 0; i < len; i ++ ) {
        size_t j = len - i - 1;
        uint8_t s = ((uint8_t*)src)[j];
        //printf("reversing: src[%d] -> dst[%d]: %x\n", j, i, s );
        ((uint8_t*)dst)[i] = s;
    }
}


/// st src to dst with dst being little endian
static inline void aa_endconv_st_le( void *dst, void *src, size_t len ) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
    memcpy( dst, src, len );
#elif __BYTE_ORDER == __BIG_ENDIAN
    aa_endconv_reverse( dst, src, len );
#else
#error "Unknown Endian Convention"
#endif
}

/// st src to dst with dst being big endian
static inline void aa_endconv_st_be( void *dst, void *src, size_t len ) {
#if __BYTE_ORDER == __LITTLE_ENDIAN
    aa_endconv_reverse( dst, src, len );
#elif __BYTE_ORDER == __BIG_ENDIAN
    memcpy( dst, src, len );
#else
#error "Unknown Endian Convention"
#endif
}

/*****************/
/*  DEFINITIONS  */
/*****************/


// typing is work, so let's make some macros to do our function defs

// this should probably be unrolled by a smart optimizer
/// definer macro for a reverse expression function
#define AA_ENDCONV_MAKE_REVERSE( type, suffix )                 \
    static inline type aa_endconv_reverse_##suffix( type x ) {  \
        type y = 0;                                             \
        type v;                                                 \
        type j;                                                 \
        size_t i;                                               \
        for( i = 0  ; i < 8*sizeof(type); i+=8 ) {              \
            j = (type) (8*sizeof(type) - i - 8);                \
            v = (type) (((x >> i) & 0xff) << j);                \
            y |= v;                                             \
        }                                                       \
        return y;                                               \
    };


// definer macros for aa_endconv_h_to_{l,b}e functions
#if __BYTE_ORDER == __LITTLE_ENDIAN

/// Make host to little-endian conversion function
#define AA_ENDCONV_MAKE_H_TO_LE( type, suffix )                         \
    static inline type aa_endconv_h_to_le_##suffix( type v ) {  return v; }
/// Make host to big-endian conversion function
#define AA_ENDCONV_MAKE_H_TO_BE( type, suffix )                 \
    static inline type aa_endconv_h_to_be_##suffix( type v ) {  \
        return aa_endconv_reverse_##suffix( v );                \
    };

/// Make host to little-endian conversion function
#define AA_ENDCONV_MAKE_LE_TO_H( type, suffix )                         \
    static inline type aa_endconv_le_to_h_##suffix( type v ) {  return v; }
/// Make host to big-endian conversion function
#define AA_ENDCONV_MAKE_BE_TO_H( type, suffix )                 \
    static inline type aa_endconv_be_to_h_##suffix( type v ) {  \
        return aa_endconv_reverse_##suffix( v );                \
    };

#elif __BYTE_ORDER == __BIG_ENDIAN

#define AA_ENDCONV_MAKE_H_TO_LE( type, suffix )                 \
    static inline type aa_endconv_h_to_le_##suffix( type v ) {  \
        return aa_endconv_reverse_##suffix( v );                \
    };
#define AA_ENDCONV_MAKE_H_TO_BE( type, suffix )                         \
    static inline type aa_endconv_h_to_le_##suffix( type v ) {  return v; }

#define AA_ENDCONV_MAKE_LE_TO_H( type, suffix )                 \
    static inline type aa_endconv_le_to_h_##suffix( type v ) {  \
        return aa_endconv_reverse_##suffix( v );                \
    };
#define AA_ENDCONV_MAKE_BE_TO_H( type, suffix )                         \
    static inline type aa_endconv_be_to_h_##suffix( type v ) {  return v; }

#else
#error "Unknown Endian Convention"
#endif

/// macro make to load/store function defs
#define AA_ENDCONV_MAKE_LDST_DEF(type,suffix)                           \
    static inline void aa_endconv_st_le_##suffix( void *dst, type src ) { \
        aa_endconv_st_le( dst, &src, sizeof(type) ); }                  \
    static inline void aa_endconv_st_be_##suffix( void *dst, type src ) { \
        aa_endconv_st_be( dst, &src, sizeof(type) ); }                  \
    static inline type aa_endconv_ld_le_##suffix( void *src ) {         \
        type r;                                                         \
        aa_endconv_st_le( &r, src, sizeof(type) );                      \
        return r;}                                                      \
    static inline type aa_endconv_ld_be_##suffix( void *src ) {         \
        type r;                                                         \
        aa_endconv_st_be( &r, src, sizeof(type) );                      \
        return r; }                                                     \
/// macro to make conversion function defs
#define AA_ENDCONV_MAKE_CONV_DEF(type,suffix)   \
    AA_ENDCONV_MAKE_REVERSE( type, suffix )     \
    AA_ENDCONV_MAKE_H_TO_LE( type, suffix )     \
    AA_ENDCONV_MAKE_H_TO_BE( type, suffix )     \
    AA_ENDCONV_MAKE_LE_TO_H( type, suffix )     \
    AA_ENDCONV_MAKE_BE_TO_H( type, suffix )     \


/// define functions for 16-bit signed int
AA_ENDCONV_MAKE_LDST_DEF( int16_t,  i16 );
/// define functions for 16-bit signed int
AA_ENDCONV_MAKE_CONV_DEF( int16_t,  i16 );
/// define functions for 16-bit unsigned int
AA_ENDCONV_MAKE_LDST_DEF( uint16_t, u16 );
/// define functions for 16-bit unsigned int
AA_ENDCONV_MAKE_CONV_DEF( uint16_t, u16 );

/// define functions for 32-bit signed int
AA_ENDCONV_MAKE_LDST_DEF( int32_t,  i32 );
/// define functions for 32-bit signed int
AA_ENDCONV_MAKE_CONV_DEF( int32_t,  i32 );

/// define functions for 32-bit unsigned int
AA_ENDCONV_MAKE_LDST_DEF( uint32_t, u32 );
/// define functions for 32-bit unsigned int
AA_ENDCONV_MAKE_CONV_DEF( uint32_t, u32 );

/// define functions for 64-bit signed int
AA_ENDCONV_MAKE_LDST_DEF( int64_t,  i64 );
/// define functions for 64-bit signed int
AA_ENDCONV_MAKE_CONV_DEF( int64_t,  i64 );

/// define functions for 64-bit unsigned int
AA_ENDCONV_MAKE_LDST_DEF( uint64_t, u64 );
/// define functions for 64-bit unsigned int
AA_ENDCONV_MAKE_CONV_DEF( uint64_t, u64 );

/// define functions for 64-bit floating point
AA_ENDCONV_MAKE_LDST_DEF( double,   d );
/// define functions for 32-bit floating point
AA_ENDCONV_MAKE_LDST_DEF( float,    s );


#endif
