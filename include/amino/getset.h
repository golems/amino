/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

#ifndef AMINO_GETSET_H
#define AMINO_GETSET_H

#define AA_DEC_SETTER( struct_type, type, field_name )           \
    AA_API void                                                  \
    struct_type ## _set_ ## field_name( struct struct_type *obj, \
                                        type field_name )


#define AA_DEC_VEC_SETTER( struct_type, field_name )                    \
    AA_API void                                                         \
    struct_type ## _set_ ## field_name( struct struct_type *obj,        \
                                        const double field_name[3] )

#define AA_DEC_VEC3_SETTER( struct_type, field_name, x, y, z)           \
    AA_API void                                                         \
    struct_type ## _set_ ## field_name ## 3( struct struct_type *obj,   \
                                             double x,                  \
                                             double y,                  \
                                             double z )

#define AA_DEF_SETTER( struct_type, field_type, field_name )     \
    AA_DEC_SETTER( struct_type, field_type, field_name )         \
    {                                                            \
        obj->field_name = field_name;                            \
    }

#define AA_DEF_BOOL_SETTER( struct_type, field_name )            \
    AA_DEC_SETTER( struct_type, int, field_name )                \
    {                                                            \
        obj->field_name = field_name ? 1 : 0;                    \
    }

#define AA_DEF_FLOAT_SETTER( struct_type, field_name )           \
    AA_DEC_SETTER( struct_type, double, field_name )             \
    {                                                            \
        obj->field_name = (float)field_name;                     \
    }

#define AA_DEF_FLOAT3_SETTER( struct_type, field_name )                 \
    AA_DEC_VEC3_SETTER( struct_type, field_name, x, y, z )              \
    {                                                                   \
        obj->field_name[0] = (float)x;                                  \
        obj->field_name[1] = (float)y;                                  \
        obj->field_name[2] = (float)z;                                  \
    }                                                                   \
                                                                        \
    AA_DEC_VEC_SETTER( struct_type, field_name)                         \
    {                                                                   \
        struct_type ## _set_ ## field_name ## 3( obj,                   \
                                                 field_name[0],         \
                                                 field_name[1],         \
                                                 field_name[2] );       \
    }

#define AA_DEF_VEC3_SETTER( struct_type, field_name )                   \
    AA_DEC_VEC3_SETTER( struct_type, field_name, x, y, z )              \
    {                                                                   \
        obj->field_name[0] = x;                                         \
        obj->field_name[1] = y;                                         \
        obj->field_name[2] = z;                                         \
    }                                                                   \
                                                                        \
    AA_DEC_VEC_SETTER( struct_type, field_name)                         \
    {                                                                   \
        struct_type ## _set_ ## field_name ## 3( obj,                   \
                                                 field_name[0],         \
                                                 field_name[1],         \
                                                 field_name[2] );       \
    }

#define AA_DEF_GETTER( struct_type, field_type, field_name )            \
    AA_API field_type                                                   \
    struct_type ## _get_ ## field_name(const struct struct_type *obj)   \
    {                                                                   \
        return obj->field_name;                                         \
    }

#endif /*AMINO_GETSET_H*/
