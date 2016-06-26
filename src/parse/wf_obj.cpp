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

struct aa_rx_wf_obj {
    std::vector<float> vertex;
    std::vector<float> normal;
    std::vector<size_t> uv;
    std::vector<aa_rx_wf_obj_face*> face;

    std::vector<std::string> mtl_files;
    std::vector<std::string> objects;

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
aa_rx_wf_obj_push_vertex( struct aa_rx_wf_obj *obj, float f )
{
    obj->vertex.push_back(f);
}

AA_API void
aa_rx_wf_obj_push_normal( struct aa_rx_wf_obj *obj, float f )
{
    obj->normal.push_back(f);
}

AA_API void
aa_rx_wf_obj_push_face( struct aa_rx_wf_obj *obj,
                        struct aa_rx_wf_obj_face *face )
{
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
