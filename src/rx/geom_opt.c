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
#include "amino/rx/rxtype.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_geom_internal.h"
#include "amino/getset.h"





struct aa_rx_geom_opt* aa_rx_geom_opt_create()
{
    struct aa_rx_geom_opt *a = AA_NEW0(struct aa_rx_geom_opt);
    aa_rx_geom_opt_set_alpha(a, 1);
    aa_rx_geom_opt_set_color3(a, .5, .5, .5);
    aa_rx_geom_opt_set_visual(a,1);
    aa_rx_geom_opt_set_collision(a,1);
    aa_rx_geom_opt_set_scale(a,1.0);

    return a;
}


void
aa_rx_geom_opt_destroy(struct aa_rx_geom_opt* opt)
{
    free(opt);
}

AA_DEF_BOOL_SETTER( aa_rx_geom_opt, no_shadow );
AA_DEF_BOOL_SETTER( aa_rx_geom_opt, visual );
AA_DEF_BOOL_SETTER( aa_rx_geom_opt, collision );


AA_DEF_VEC3_SETTER( aa_rx_geom_opt, color );
AA_DEF_VEC3_SETTER( aa_rx_geom_opt, specular );

void
aa_rx_geom_opt_set_alpha (
    struct aa_rx_geom_opt *opt,
    double alpha )
{
    opt->color[3] = alpha;
}

AA_API int
aa_rx_geom_opt_get_no_shadow ( struct aa_rx_geom_opt *opt )
{
    return opt->no_shadow;
}
AA_API int
aa_rx_geom_opt_get_visual ( struct aa_rx_geom_opt *opt )
{
    return opt->visual;
}
AA_API int
aa_rx_geom_opt_get_collision ( struct aa_rx_geom_opt *opt )
{
    return opt->collision;
}

AA_API double
aa_rx_geom_opt_get_color_red ( struct aa_rx_geom_opt *opt )
{
    return opt->color[0];
}
AA_API double
aa_rx_geom_opt_get_color_blue ( struct aa_rx_geom_opt *opt )
{
    return opt->color[1];
}
AA_API double
aa_rx_geom_opt_get_color_green ( struct aa_rx_geom_opt *opt )
{
    return opt->color[2];
}
AA_API double
aa_rx_geom_opt_get_alpha ( struct aa_rx_geom_opt *opt )
{
    return opt->color[3];
}

AA_API double
aa_rx_geom_opt_get_specular_red ( struct aa_rx_geom_opt *opt )
{
    return opt->specular[0];
}
AA_API double
aa_rx_geom_opt_get_specular_blue ( struct aa_rx_geom_opt *opt )
{
    return opt->specular[1];
}
AA_API double
aa_rx_geom_opt_get_specular_green ( struct aa_rx_geom_opt *opt )
{
    return opt->specular[2];
}

AA_API void
aa_rx_geom_opt_set_scale (
    struct aa_rx_geom_opt *opt,
    double scale )
{
    opt->scale = scale;
}

AA_API double
aa_rx_geom_opt_get_scale ( const struct aa_rx_geom_opt *opt )
{
    return opt->scale;
}
