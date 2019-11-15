/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
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

#include "config.h"
#include "amino.h"

#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"
#include "amino/rx/octree_geom.hpp"

#include "amino/rx/rxtype.h"
#include "amino/rx/rxtype_internal.h"

#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"

#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_gl_internal.h"

#include <math.h>
#include <unistd.h>


void add_box ( GLfloat* values, GLfloat* normals, unsigned* indices,
               GLfloat s, GLfloat* c, size_t idx )
{
    GLfloat a[2] = {1,-1};
    size_t ii[4][2] = {{0,0}, {0,1}, {1,1}, {1,0}};
    GLfloat d[3] = {s, s, s};



    // Z
    size_t n = 0;
    printf("before making z\n");
    for( size_t k = 0; k < 2; k ++ ) {

        for( size_t ell = 0; ell<4; ell++ ) {
            size_t i = ii[ell][0];
            size_t j = ii[ell][1];
            values[n*3 + 0] = a[i]*d[0]+c[0];
            values[n*3 + 1] = a[j]*d[1]+c[1];
            values[n*3 + 2] = a[k]*d[2]+c[2];
            normals[n*3+0] = 0;
            normals[n*3+1] = 0;
            normals[n*3+2] = a[k];
            n++;
        }
    }
    // X
    printf("before making x\n");
    for( size_t k = 0; k < 2; k ++ ) {
        for( size_t ell = 0; ell<4; ell++ ) {
            size_t i = ii[ell][0];
            size_t j = ii[ell][1];
            values[n*3 + 0] = a[k]*d[0]+c[0];
            values[n*3 + 1] = a[i]*d[1]+c[1];
            values[n*3 + 2] = a[j]*d[2]+c[2];
            normals[n*3+0] = a[k];
            normals[n*3+1] = 0;
            normals[n*3+2] = 0;
            n++;
        }
    }
    // Y
    printf("before making y\n");
    for( size_t k = 0; k < 2; k ++ ) {
        for( size_t ell = 0; ell<4; ell++ ) {
            size_t i = ii[ell][0];
            size_t j = ii[ell][1];
            values[n*3 + 0] = a[i]*d[0]+c[0];
            values[n*3 + 1] = a[k]*d[1]+c[1];
            values[n*3 + 2] = a[j]*d[2]+c[2];
            normals[n*3+0] = 0;
            normals[n*3+1] = a[k];
            normals[n*3+2] = 0;
            n++;
        }
    }
    printf("before making indicies: %ld\n", idx);
    {
        unsigned nn = 0;
        for( unsigned axis = 0; axis < 3; axis ++ ) {
            for( size_t k = 0; k < 2; k ++ ) {
                indices[idx++] = nn;
                indices[idx++] = nn+1;
                indices[idx++] = nn+2;
                indices[idx++] = nn;
                indices[idx++] = nn+3;
                indices[idx++] = nn+2;
                nn += 4;
            }
        }
    }
}



void init_octree ( struct aa_rx_geom_octree *geom )
{
    octomap::OcTree* tree = geom->shape->otree;

    size_t num_nodes = tree->getNumLeafNodes();
    size_t num_val = 6*4*3;
    GLfloat* values = AA_NEW0_AR(GLfloat, num_val*num_nodes);
    GLfloat* normals = AA_NEW0_AR(GLfloat, num_val*num_nodes);
    unsigned* indices = AA_NEW0_AR(unsigned, num_val/2*num_nodes);

    struct aa_rx_mesh *mesh = aa_rx_mesh_create();
    mesh->vertices=NULL;
    mesh->normals=NULL;
    mesh->indices=NULL;
    fprintf(stderr,"created mesh\n");

    size_t x=0;
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(),
            end=tree->end_leafs(); it!= end; ++it)
    {
        if (it->getOccupancy() > 0.5){
            fprintf(stderr,"adding value %ld out of %ld\n", x, num_nodes);
            GLfloat d = (GLfloat) cbrt(it.getSize());
            GLfloat c[3] = {(GLfloat) it.getX(),(GLfloat) it.getY(),(GLfloat) it.getZ()};

            size_t idx = x*num_val/2;
            add_box(&values[num_val*x], &normals[num_val*x],
                    &indices[num_val/2*x], d, c, idx);

            x++;
        }
    }
    fprintf(stderr,"filled values itr: %ld\n", x);


    size_t num_vert = x*num_val;
    fprintf(stderr,"before setting verticies x: %ld\n", num_vert);
    aa_rx_mesh_set_vertices( mesh, num_vert, values, 0 );
    fprintf(stderr,"set verticies\n");
    aa_rx_mesh_set_normals( mesh, num_vert, normals, 0 );
    fprintf(stderr,"set normals\n");
    size_t num_idx = num_val/6*x;
    aa_rx_mesh_set_indices( mesh, num_idx, indices, 0 );
    fprintf(stderr,"set indicies\n");


    aa_rx_mesh_set_texture(mesh, &geom->base.opt);
    fprintf(stderr,"set texture\n");
    tri_mesh( &geom->base, mesh );
    aa_rx_mesh_destroy(mesh);

}
