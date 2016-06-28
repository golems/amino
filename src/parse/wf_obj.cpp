/* -*- mode: C++; c-basic-offset: 4; -*- */
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

#include "amino.h"
#include "wavefront_internal.h"

#include <vector>
#include <string>
#include <algorithm>

struct aa_rx_wf_obj {
    std::vector<double> vertex;
    std::vector<double> normal;
    std::vector<size_t> uv;
    std::vector<struct aa_rx_wf_obj_face*> face;

    std::vector<std::string> mtl_files;
    std::vector<std::string> objects;

    std::vector<std::string> materials;

    size_t current_material;

};


AA_API struct aa_rx_wf_obj *
aa_rx_wf_obj_create()
{
    struct aa_rx_wf_obj * obj = new aa_rx_wf_obj;

    return obj;
}


AA_API void
aa_rx_wf_obj_destroy( struct aa_rx_wf_obj * obj)
{
    delete obj;
}

AA_API void
aa_rx_wf_obj_push_vertex( struct aa_rx_wf_obj *obj, double f )
{
    obj->vertex.push_back(f);
}

AA_API void
aa_rx_wf_obj_push_normal( struct aa_rx_wf_obj *obj, double f )
{
    obj->normal.push_back(f);
}

AA_API void
aa_rx_wf_obj_push_face( struct aa_rx_wf_obj *obj,
                        struct aa_rx_wf_obj_face *face )
{
    face->material = obj->current_material;
    obj->face.push_back(face);
}

AA_API void
aa_rx_wf_obj_push_object( struct aa_rx_wf_obj *obj,
                          const char *object )
{
    obj->objects.push_back(object);
}

AA_API void
aa_rx_wf_obj_push_material( struct aa_rx_wf_obj *obj,
                            const char *mtl_file )
{
    obj->mtl_files.push_back(mtl_file);
}

AA_API void
aa_rx_wf_obj_use_material( struct aa_rx_wf_obj *obj,
                           const char *material )
{
    auto itr = std::find(obj->materials.begin(), obj->materials.end(), material);
    if( obj->materials.end() == itr ) {
        obj->current_material = obj->materials.size();
        obj->materials.push_back(material);
    } else {
        obj->current_material = itr - obj->materials.begin();
    }
}

AA_API size_t
aa_rx_wf_obj_material_count( struct aa_rx_wf_obj *obj )
{
    return obj->materials.size();
}

AA_API const char *
aa_rx_wf_obj_get_material_name( struct aa_rx_wf_obj *obj, size_t i )
{
    if( i < obj->materials.size() ) {
        return obj->materials[i].c_str();
    } else {
        return NULL;
    }
}

AA_API void
aa_rx_wf_obj_get_vertices( const struct aa_rx_wf_obj *obj,
                           const double **vertices, size_t *n )
{
    *vertices = obj->vertex.data();
    *n = obj->vertex.size();
}

AA_API void
aa_rx_wf_obj_get_normals( const struct aa_rx_wf_obj *obj,
                          const double **normals, size_t *n )
{
    *normals = obj->normal.data();
    *n = obj->normal.size();
}

AA_API void
aa_rx_wf_obj_get_faces( const struct aa_rx_wf_obj *obj,
                        struct aa_rx_wf_obj_face * const **faces , size_t *n )
{
    *faces = obj->face.data();
    *n = obj->face.size();
}
