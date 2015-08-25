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
 *   AND ON ANY HEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef AMINO_RX_SCENE_GEOM_INTERNAL_H
#define AMINO_RX_SCENE_GEOM_INTERNAL_H

/**
 * Opaque structure for geometry options.
 */
struct aa_rx_geom_opt
{
    int no_shadow;
    double color[3];
    double alpha;
    bool visual;
    bool collision;
};


struct aa_rx_geom_base {
    struct aa_rx_geom_opt opt;
    enum aa_rx_geom_shape shape;
};

struct aa_rx_geom_box {
    struct aa_rx_geom_base base;
    double dimension[3];
};

struct aa_rx_geom_sphere {
    struct aa_rx_geom_base base;
    double radius;
};

struct aa_rx_geom_cylinder {
    struct aa_rx_geom_base base;
    double height;
    double radius;
};

struct aa_rx_geom_cone {
    struct aa_rx_geom_base base;
    double height;
    double start_radius;
    double end_radius;
};

struct aa_rx_mesh {
    std::vector<float> vertex_vectors;
    std::vector<size_t> vertex_indices;

    std::vector<float> normal_vectors;
    std::vector<size_t> normal_vectors;

    std::vector<float> uv_vectors;
    std::vector<size_t> uv_vectors;
}

#endif /*AMINO_RX_SCENE_GEOM_INTERNAL_H*/
