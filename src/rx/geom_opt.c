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
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"





struct aa_rx_geom_opt* aa_rx_geom_opt_create()
{
    struct aa_rx_geom_opt *a = AA_NEW0(struct aa_rx_geom_opt);
    aa_rx_geom_opt_set_alpha(a, 1);
    aa_rx_geom_opt_set_color(a, .5, .5, .5);

    return a;
}

void
aa_rx_geom_opt_destroy(struct aa_rx_geom_opt* opt)
{
    free(opt);
}

void
aa_rx_geom_opt_set_no_shadow (
    struct aa_rx_geom_opt *opt,
    int no_shadow )
{
    opt->no_shadow = no_shadow ? 1 : 0;
}


void
aa_rx_geom_opt_set_color (
    struct aa_rx_geom_opt *opt,
    double red, double blue, double green )
{
    opt->color[0] = red;
    opt->color[1] = blue;
    opt->color[2] = green;
}


void
aa_rx_geom_opt_set_alpha (
    struct aa_rx_geom_opt *opt,
    double alpha )
{
    opt->color[3] = alpha;
}

void
aa_rx_geom_opt_set_visual (
    struct aa_rx_geom_opt *opt,
    int visual )
{
    opt->visual = visual ? 1 : 0;
}

void
aa_rx_geom_opt_set_collision (
    struct aa_rx_geom_opt *opt,
    int collision )
{
    opt->collision = collision ? 1 : 0;

}

void
aa_rx_geom_opt_set_specular (
    struct aa_rx_geom_opt *opt,
    double specular[3] )
{
    for( size_t i = 0; i < 3; i ++ ) {
        opt->specular[i] = specular[i];
    }
}
