/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder the names of its
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

#include <libgen.h>

#include "amino.h"

#include "wavefront_internal.h"


#include <ctype.h>

AA_VECTOR_DEF( double, dvec_type )
AA_VECTOR_DEF( int32_t, ivec_type )
AA_VECTOR_DEF( char *, svec_type )

AA_VECTOR_DEF( struct aa_rx_wf_mtl*, mtlvec_type )

struct aa_rx_wf_obj {
    char *filename;
    char *dirname;
    char *dirname_data;

    dvec_type vertex;
    dvec_type normal;
    dvec_type texture_vertex;

    ivec_type vertex_indices;
    ivec_type normal_indices;
    ivec_type uv_indices;
    ivec_type texture_indices;


    svec_type mtl_files;
    svec_type objects;
    svec_type materials;

    mtlvec_type mtl;

    int32_t current_material;

};

static void svec_type_push_dup( svec_type *v, const char *s ) {
    svec_type_push(v, strdup(s));
}

static void svec_type_destroy( svec_type *v ) {
    for( size_t i = 0; i < v->size; i ++ ) {
        free(v->data[i]);
    }
    free(v->data);
}

static void mtlvec_type_destroy( mtlvec_type *v ) {
    for( size_t i = 0; i < v->size; i ++ ) {
        aa_rx_wf_mtl_destroy(v->data[i]);
    }
    free(v->data);
}

AA_API struct aa_rx_wf_obj *
aa_rx_wf_obj_create()
{
    struct aa_rx_wf_obj * obj = AA_NEW0(struct aa_rx_wf_obj);

    dvec_type_init( &obj->vertex, 64 );
    dvec_type_init( &obj->normal, 64 );

    ivec_type_init( &obj->vertex_indices, 64 );
    ivec_type_init( &obj->normal_indices, 64 );
    ivec_type_init( &obj->uv_indices, 64 );
    ivec_type_init( &obj->texture_indices, 64 );

    svec_type_init( &obj->mtl_files, 1 );
    svec_type_init( &obj->objects, 1 );
    svec_type_init( &obj->materials, 1 );

    mtlvec_type_init( &obj->mtl, 1 );

    return obj;
}

AA_API double
aa_rx_wf_parse_float( const char *str )
{
    /* skip blanks */
    while( *str &&  isblank(*str) ) str++;

    char *e, *g;
    double r = (double)strtol( str, &e, 10 );
    long f = ('.' == *e)
        ? strtol( e+1, &g, 10 )
        : 0;

    if( f ) {
        long k = g - e - 1;
        if( k ) {
            double x = (double)f;
            while(k--) x/=10;
            if( r < 0 || ('-' == *str) )
                r -= x;
            else
                r += x;
        }
    }

    /* printf("t: %s\n", str); */
    /* printf("r: %f\n", r); */
    /* printf("a: %f\n", atof(str)); */

    return r;
}

AA_API void
aa_rx_wf_obj_destroy( struct aa_rx_wf_obj * obj)
{
    free( obj->vertex.data );
    free( obj->normal.data );
    free( obj->vertex_indices.data );
    free( obj->normal_indices.data );
    free( obj->uv_indices.data );
    free( obj->texture_indices.data );

    svec_type_destroy( &obj->mtl_files );
    svec_type_destroy( &obj->objects );
    svec_type_destroy( &obj->materials );

    mtlvec_type_destroy( &obj->mtl );

    aa_checked_free(obj->filename);
    aa_checked_free(obj->dirname_data);

    free(obj);
}

AA_API void
aa_rx_wf_obj_push_vertex( struct aa_rx_wf_obj *obj, double f )
{
    //printf("push %f\n", f);
    dvec_type_push( &obj->vertex, f );
}

AA_API void
aa_rx_wf_obj_push_texture_vertex( struct aa_rx_wf_obj *obj, double f )
{
    dvec_type_push( &obj->texture_vertex, f );
}

AA_API void
aa_rx_wf_obj_push_normal( struct aa_rx_wf_obj *obj, double f )
{
    dvec_type_push( &obj->normal, f );
}

AA_API void
aa_rx_wf_obj_push_face( struct aa_rx_wf_obj *obj,
                        struct aa_rx_wf_obj_face *face )
{
    for( int i = 0; i < 3; i ++ ) {
        ivec_type_push( &obj->vertex_indices, face->v[i]);
        ivec_type_push( &obj->normal_indices, face->n[i]);
        ivec_type_push( &obj->uv_indices, face->t[i]);
    }

    ivec_type_push( &obj->texture_indices, obj->current_material);

}

AA_API void
aa_rx_wf_obj_push_object( struct aa_rx_wf_obj *obj,
                          const char *object )
{
    svec_type_push_dup(&obj->objects, object );
}

AA_API int
aa_rx_wf_obj_push_mtl( struct aa_rx_wf_obj *obj,
                       const char *mtl_file )
{
    svec_type_push_dup(&obj->mtl_files, mtl_file );

    aa_mem_region_t *reg = aa_mem_region_local_get();
    char *buf = aa_mem_region_printf(reg, "%s/%s", obj->dirname, mtl_file);
    struct aa_rx_wf_mtl *mtl = aa_rx_wf_mtl_parse(buf);
    aa_mem_region_pop(reg, buf);

    if( mtl ) {
        mtlvec_type_push( &obj->mtl, mtl );
        return 0;
    } else {
        return -1;
    }


}


AA_API size_t
aa_rx_wf_obj_mtl_count( struct aa_rx_wf_obj *obj )
{
    return obj->mtl_files.size;
}


AA_API const char *
aa_rx_wf_obj_get_mtl_filename( struct aa_rx_wf_obj *obj, size_t i )
{
    if( i <= obj->mtl_files.size ) {
        return obj->mtl_files.data[i];
    } else {
        return NULL;
    }
}

AA_API const struct aa_rx_wf_mtl *
aa_rx_wf_obj_get_mtl( struct aa_rx_wf_obj *obj, size_t i )
{
    if( i <= obj->mtl.size ) {
        return obj->mtl.data[i];
    } else {
        return NULL;
    }
}

AA_API void
aa_rx_wf_obj_use_material( struct aa_rx_wf_obj *obj,
                           const char *material )
{
    for( size_t i = 0; i < obj->materials.size; i ++ ) {
        if( 0 == strcmp(material, obj->materials.data[i]) ) {
            /* Found the material */
            obj->current_material = (int32_t)i;
            return;
        }
    }

    /* Material not found */
    obj->current_material = (int32_t)obj->materials.size;
    svec_type_push_dup( &obj->materials, material);
}

AA_API size_t
aa_rx_wf_obj_material_count( struct aa_rx_wf_obj *obj )
{
    return obj->materials.size;
}


AA_API const char *
aa_rx_wf_obj_get_material_name( struct aa_rx_wf_obj *obj, size_t i )
{
    if( i < obj->materials.size ) {
        return obj->materials.data[i];
    } else {
        return NULL;
    }
}

AA_API void
aa_rx_wf_obj_get_vertices( const struct aa_rx_wf_obj *obj,
                           const double **vertices, size_t *n )
{
    *vertices = obj->vertex.data;
    *n = obj->vertex.size;
}

AA_API void
aa_rx_wf_obj_get_normals( const struct aa_rx_wf_obj *obj,
                          const double **normals, size_t *n )
{
    *normals = obj->normal.data;
    *n = obj->normal.size;
}

AA_API void
aa_rx_wf_obj_get_vertex_indices( const struct aa_rx_wf_obj *obj,
                                 const int32_t **v, size_t *n )
{
    *v = obj->vertex_indices.data;
    *n = obj->vertex_indices.size;
}

AA_API void
aa_rx_wf_obj_get_normal_indices( const struct aa_rx_wf_obj *obj,
                                 const int32_t **v, size_t *n ) {
    *v = obj->normal_indices.data;
    *n = obj->normal_indices.size;
}
AA_API void
aa_rx_wf_obj_get_uv_indices( const struct aa_rx_wf_obj *obj,
                             const int32_t **v, size_t *n ) {
    *v = obj->uv_indices.data;
    *n = obj->uv_indices.size;
}

AA_API void
aa_rx_wf_obj_get_texture_indices( const struct aa_rx_wf_obj *obj,
                                  const int32_t **v, size_t *n ) {
    *v = obj->texture_indices.data;
    *n = obj->texture_indices.size;
}

AA_API void
aa_rx_wf_obj_set_filename( struct aa_rx_wf_obj *obj, const char *filename )
{
    obj->filename = strdup(filename);
    obj->dirname_data = strdup(filename);

    obj->dirname = dirname( obj->dirname_data );

}

AA_API const char *
aa_rx_wf_obj_get_filename( struct aa_rx_wf_obj *obj )
{
    return obj->filename;
}
