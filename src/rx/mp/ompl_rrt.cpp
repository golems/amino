/* -*- mode: C++; c-basic-offset: 4; -*- */
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


#include "amino.h"
#include "amino/rx/rxerr.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_kin_internal.h"
#include "amino/rx/scene_collision.h"
#include "amino/rx/scene_planning.h"
#include "amino/rx/scene_ompl.h"

#include <ompl/base/Planner.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>


struct aa_rx_mp_rrt_attr
{
    unsigned is_bidirectional : 1;
};


AA_API struct aa_rx_mp_rrt_attr*
aa_rx_mp_rrt_attr_create(void)
{
    struct aa_rx_mp_rrt_attr * a = AA_NEW(struct aa_rx_mp_rrt_attr);
    a->is_bidirectional = 1;
    return a;
}


AA_API void
aa_rx_mp_rrt_attr_destroy(struct aa_rx_mp_rrt_attr* a)
{
    free(a);
}


AA_API void
aa_rx_mp_rrt_attr_set_bidirectional( struct aa_rx_mp_rrt_attr* attrs,
                                     int is_bidirectional )
{
    attrs->is_bidirectional = is_bidirectional ? 1 : 0;
}


AA_API void
aa_rx_mp_set_rrt( struct aa_rx_mp* mp,
                  const struct aa_rx_mp_rrt_attr *attr )
{
    if( attr->is_bidirectional ) {
        auto p = new ompl::geometric::RRTConnect(mp->space_information);
        mp->set_planner(p);
    } else {
        auto p = new ompl::geometric::RRT(mp->space_information);
        mp->set_planner(p);
    }

}
