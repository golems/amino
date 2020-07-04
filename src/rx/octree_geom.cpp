/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Matthew Schack <mschack@mines.edu>
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

#include "amino/rx/rxtype.h"
#include "amino/rx/rxtype_internal.h"

#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"
#include "amino/rx/octree_geom.hpp"

#include "sg_convenience.h"

#define ALLOC_GEOM(TYPE, var, type_value, geom_opt )            \
    TYPE *var = AA_NEW0(TYPE);                                  \
    AA_MEM_CPY(&g->base->opt, geom_opt, 1);                      \
    var->base->type = type_value;                                \
    var->base->gl_buffers = NULL;                                \
    var->base->refcount = 1;


struct aa_rx_geom *
aa_rx_geom_octree (
                   struct aa_rx_geom_opt *opt,
                   struct aa_rx_octree *octree )
{
  ALLOC_GEOM( struct aa_rx_geom_octree, g,
              AA_RX_OCTREE, opt);
  g->shape = octree;
  return g->base;
}

AA_API struct aa_rx_octree* aa_rx_geom_read_octree_from_file( const char* file)
{

    octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(file);
    octomap::OcTree* oTree = dynamic_cast<octomap::OcTree*>(tree);

    aa_rx_octree *aa_oct = new aa_rx_octree;
    aa_oct->otree = oTree;
    return aa_oct;
}
