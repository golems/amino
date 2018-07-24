/* -*- mode: C++; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2012, Georgia Tech Research Corporation
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

#include "amino.h"
#include "amino/test.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"
#include <assert.h>



static void scara( struct aa_rx_sg *sg );
static void check_scara( struct aa_rx_sg *sg );
static void check_tf( struct aa_rx_sg *sg );

int main(void)
{
    struct aa_rx_sg *sg = aa_rx_sg_create();
    scara(sg);
    aa_rx_sg_init(sg);

    check_scara(sg);
    check_tf(sg);



    aa_rx_sg_destroy(sg);

    return 0;
}


#define l0 .2
#define l1 .5
#define l2 .7

static void scara( struct aa_rx_sg *sg )
{
    /* Link Lengths */
    static const double L0[3] = {l0, 0, 0};
    static const double L1[3] = {l1, 0, 0};
    static const double L2[3] = {l2, 0, 0};
    static const double L3[3] = {0, 0, 0};


    double q_ident[4] = AA_TF_QUAT_IDENT_INITIALIZER;

    /* Construct a scara manipulator */
    aa_rx_sg_add_frame_revolute( sg,
                                 "q0", "q1",
                                 q_ident, L1,
                                 NULL, aa_tf_vec_z, 0);

    aa_rx_sg_add_frame_revolute( sg,
                                 "", "q0",
                                 q_ident, L0,
                                 NULL, aa_tf_vec_z, 0);

    aa_rx_sg_add_frame_prismatic( sg,
                                  "q2", "q3",
                                  q_ident, L3,
                                  NULL, aa_tf_vec_z, 0);

    aa_rx_sg_add_frame_revolute( sg,
                                 "q1", "q2",
                                 q_ident, L2,
                                 NULL, aa_tf_vec_z, 0);
}

static void check_scara( struct aa_rx_sg *sg )
{
    /* Check that the scene graph topology is reasonable */
    aa_rx_frame_id fid0 = aa_rx_sg_frame_id(sg,"q0");
    aa_rx_frame_id fid1 = aa_rx_sg_frame_id(sg,"q1");
    aa_rx_frame_id fid2 = aa_rx_sg_frame_id(sg,"q2");
    aa_rx_frame_id fid3 = aa_rx_sg_frame_id(sg,"q3");


    assert( 0 == strcmp( "q0", aa_rx_sg_frame_name(sg, fid0) ) );
    assert( 0 == strcmp( "q1", aa_rx_sg_frame_name(sg, fid1) ) );
    assert( 0 == strcmp( "q2", aa_rx_sg_frame_name(sg, fid2) ) );
    assert( 0 == strcmp( "q3", aa_rx_sg_frame_name(sg, fid3) ) );
    assert( AA_RX_FRAME_ROOT == aa_rx_sg_frame_parent(sg, fid0) );
    assert( fid0 == aa_rx_sg_frame_parent(sg, fid1) );
    assert( fid1 == aa_rx_sg_frame_parent(sg, fid2) );
    assert( fid2 == aa_rx_sg_frame_parent(sg, fid3) );
    assert( fid0 < fid1 );
    assert( fid1 < fid2 );
    assert( fid2 < fid3 );

    assert( 4 == aa_rx_sg_frame_count(sg) );
    assert( 4 == aa_rx_sg_config_count(sg) );

    // aa_rx_config_id cid0 = aa_rx_sg_config_id(sg,"q0");
    // aa_rx_config_id cid1 = aa_rx_sg_config_id(sg,"q1");
    // aa_rx_config_id cid2 = aa_rx_sg_config_id(sg,"q2");
    // aa_rx_config_id cid3 = aa_rx_sg_config_id(sg,"q3");

    // assert( cid0 == aa_rx_sg_frame_config_id(sg, fid0) );
    // assert( cid1 == aa_rx_sg_frame_config_id(sg, fid1) );
    // assert( cid2 == aa_rx_sg_frame_config_id(sg, fid2) );
    // assert( cid3 == aa_rx_sg_frame_config_id(sg, fid3) );
}

static void check_tf( struct aa_rx_sg *sg )
{
    aa_rx_config_id cid0 = aa_rx_sg_config_id(sg,"q0");
    aa_rx_config_id cid1 = aa_rx_sg_config_id(sg,"q1");
    aa_rx_config_id cid2 = aa_rx_sg_config_id(sg,"q2");
    aa_rx_config_id cid3 = aa_rx_sg_config_id(sg,"q3");

    // aa_rx_frame_id fid0 = aa_rx_sg_frame_id(sg,"q0");
    // aa_rx_frame_id fid1 = aa_rx_sg_frame_id(sg,"q1");
    // aa_rx_frame_id fid2 = aa_rx_sg_frame_id(sg,"q2");
    // aa_rx_frame_id fid3 = aa_rx_sg_frame_id(sg,"q3");

    size_t frame_cnt =  aa_rx_sg_frame_count(sg);
    size_t config_cnt =  aa_rx_sg_config_count(sg);
    double q[config_cnt];
    double TF_rel[7*frame_cnt];
    double TF_abs[7*frame_cnt];

    /* q = 0 */
    {
        q[cid0] = 0;
        q[cid1] = 0;
        q[cid2] = 0;
        q[cid3] = 0;
        aa_rx_sg_tf( sg, config_cnt, q, frame_cnt, TF_rel, 7, TF_abs, 7 );
        double E_ref[] = {0,0,0,1, l0, 0, 0,
                          0,0,0,1, l0+l1, 0, 0,
                          0,0,0,1, l0+l1+l2, 0, 0,
                          0,0,0,1, l0+l1+l2, 0, 0 };
        aveq( "chain 0", 7*4, E_ref, TF_abs, 1e-6 );
    }

    /* q1 = M_PI_2 */
    {
        q[cid1] = M_PI_2;
        aa_rx_sg_tf( sg, config_cnt, q, frame_cnt, TF_rel, 7, TF_abs, 7 );
        double s = sin(M_PI_4);
        double E_ref[] = {0,0,0,1, l0, 0, 0,
                          0,0,s,s, l0+l1, 0, 0,
                          0,0,s,s, l0+l1, l2, 0,
                          0,0,s,s, l0+l1, l2, 0 };
        aveq( "chain 0", 7*4, E_ref, TF_abs, 1e-6 );
    }

    /* q3 = 1.5 */
    {
        q[cid3] = 1.5;
        aa_rx_sg_tf( sg, config_cnt, q, frame_cnt, TF_rel, 7, TF_abs, 7 );
        double s = sin(M_PI_4);
        double E_ref[] = {0,0,0,1, l0, 0, 0,
                          0,0,s,s, l0+l1, 0, 0,
                          0,0,s,s, l0+l1, l2, 0,
                          0,0,s,s, l0+l1, l2, q[cid3] };
        aveq( "chain 0", 7*4, E_ref, TF_abs, 1e-6 );
    }
}
