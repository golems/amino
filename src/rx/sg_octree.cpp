/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@mines.edu>
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
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/octree_geom.hpp"
#include "amino/mat_internal.h"

AA_API void aa_rx_sg_add_octree(
    struct aa_rx_sg *scene_graph,
    const char *parent,
    struct aa_rx_octree* ocTree, struct aa_rx_geom_opt* opt)
{
    size_t x=0;
    size_t len = 8;
    size_t base = 10;
    double q[4] ={0,0,0,1};

    octomap::OcTree* tree = ocTree->otree;
    for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(),
            end=tree->end_leafs(); it!= end; ++it)
    {
        if (it->getOccupancy() > 0.5){
            char name[len];
            sprintf(name, "octree%ld",x);

            double v[3] = { it.getX(), it.getY(), it.getZ() };

            aa_rx_sg_add_frame_fixed(scene_graph, parent, name, q,v);

            x++;
            if (x % base==0) {
                len++;
                base *=10;

            }
            if ( opt ){
                double s_len = cbrt(it.getSize());
                static const double dimension[3] = {s_len, s_len, s_len};
                struct aa_rx_geom* geom = aa_rx_geom_box(opt, dimension);

                aa_rx_geom_attach(scene_graph, name, geom);

            }
        }
    }
}
