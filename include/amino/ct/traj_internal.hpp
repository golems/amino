/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016 Rice University
 * All rights reserved.
 *
 * Author(s): Zachary K. Kingston <zak@rice.edu>
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

#ifndef AMINO_CT_TRAJ_INTERNAL_HPP
#define AMINO_CT_TRAJ_INTERNAL_HPP

/**
 * @file traj_internal.hpp
 */

#ifdef __cplusplus
extern "C" {
#endif


typedef int (*aa_ct_seg_eval_fun)(struct aa_ct_seg *seg,
                                  struct aa_ct_state *state, double t);

/**
 * Trajectory segment.
 */
struct aa_ct_seg;
struct aa_ct_seg {
    int type; ///< Type label for disambiguation
    int (*eval)(struct aa_ct_seg *seg,
                struct aa_ct_state *state, double t); ///< Evaluate function
    struct aa_ct_seg *prev, *next; ///< Links to next and previous segments
    void *cx; ///< Segment context
};


/**
 * Add a reference to a segment to a segment list. The reference will be kept in
 * the list.
 *
 * @param list List to add segment to
 * @param seg  Segment to add to list
 */
void aa_ct_seg_list_add(struct aa_ct_seg_list *list, struct aa_ct_seg *seg);


void aa_ct_seg_list_add_cx( struct aa_ct_seg_list *list,
                            aa_ct_seg_eval_fun eval,
                            void *cx );


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

struct aa_ct_pt_list {
    struct aa_mem_region reg;
    amino::RegionList<struct aa_ct_pt *>::allocator alloc; ///< Allocator
    amino::RegionList<struct aa_ct_pt *>::type list;       ///< List

    aa_ct_pt_list(struct aa_mem_region *_reg) : alloc(_reg), list(alloc) {
        aa_mem_region_init(&reg, 512);
    };

    ~aa_ct_pt_list(void) {
        list.~list();
        aa_mem_region_destroy(&reg);
    }
};

struct aa_ct_seg_list {
    struct aa_mem_region reg;
    amino::RegionList<struct aa_ct_seg *>::allocator alloc;  ///< Allocator
    amino::RegionList<struct aa_ct_seg *>::type list;        ///< List
    amino::RegionList<struct aa_ct_seg *>::iterator it;      ///< Iterator
    int it_on;                                               ///< Iterator init?

    size_t n_q;
    double duration;

    aa_ct_seg_list(struct aa_mem_region *_reg) :
        alloc(_reg),
        list(alloc)
    {
        aa_mem_region_init(&reg, 512);
        it_on = 0;
    }

    ~aa_ct_seg_list(void) {
        list.~list();
        aa_mem_region_destroy(&reg);
    }
};

#endif

#endif
