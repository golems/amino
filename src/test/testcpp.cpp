/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
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

#include "amino.hpp"
#include <iostream>


void allocator() {
    aa_mem_region_t reg;
    aa_mem_region_init( &reg, 1024 );

    /*--- List ---*/
    amino::RegionList<int>::allocator alloc(&reg);
    amino::RegionList<int>::type list(alloc);

    list.push_back(1);
    list.push_back(2);
    list.push_back(3);
    printf("List:\n");
    for( amino::RegionList<int>::iterator p = list.begin(); p != list.end(); p++ ) {
        printf("> %d\n", *p);

    }

    /*--- Vector ---*/
    amino::RegionVector<int>::allocator alloc1(&reg);
    amino::RegionVector<int>::type vector(3, 0, alloc1);

    vector[0] = 10;
    vector[1] = 20;
    vector[2] = 30;

    printf("Vector:\n");
    for( amino::RegionVector<int>::iterator p = vector.begin(); p != vector.end(); p++ ) {
        printf("> %d\n", *p);
    }

    /*--- Map ---*/
    amino::RegionMap<int,int>::allocator alloc2(&reg);
    amino::RegionMap<int,int>::type map(std::less<int>(), alloc2);

    map[1] = 10;
    map[2] = 20;
    map[3] = 30;

    printf("Map:\n> %d %d %d\n", map[1], map[2], map[3] );


    /*--- Stats ---*/
    printf("Stats:\n"
           "> start: 0x%x\n"
           "> head:  0x%x\n"
           "> used:  %lu\n",
           reg.node->d,
           reg.head,
           reg.head - reg.node->d
        );


    aa_mem_region_destroy( &reg );


}

int main( int argc, char **argv) {
    (void)argc; (void)argv;
    allocator();
    return 0;
}
