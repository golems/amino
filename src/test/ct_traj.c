/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016, Rice University
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

#include "amino.h"
#include "amino/test.h"

#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scenegraph_internal.h"

#include "amino/ct/state.h"
#include "amino/ct/traj.h"

void
test_tjX(void)
{
    struct aa_mem_region reg;
    aa_mem_region_init(&reg, 512);

    struct aa_ct_pt_list *pt_list = aa_ct_pt_list_create(&reg);

    struct aa_ct_state pt1 = {0}, pt2 = {0}, pt3 = {0};
    double x1[7] = {0}, x2[7] = {0}, x3[7] = {0};
    pt1.X = x1;
    pt2.X = x2;
    pt3.X = x3;

    x1[0] = 1;
    x2[1] = 1;
    x3[0] = 1;

    x1[4] = 0;
    x2[4] = 1;
    x3[4] = 0;

    aa_ct_pt_list_add(pt_list, &pt1);
    aa_ct_pt_list_add(pt_list, &pt2);
    aa_ct_pt_list_add(pt_list, &pt3);

    double dXlim[6], ddXlim[6];
    struct aa_ct_state limits;
    limits.dX = dXlim;
    limits.ddX = ddXlim;

    for (size_t j = 0; j < 6; j++) {
        limits.dX[j] = 1;
        limits.ddX[j] = 1;
    }

    struct aa_ct_seg_list *seg_list =
        aa_ct_tjX_pb_generate(&reg, pt_list, &limits);

    struct aa_ct_state state;
    double X[7], dX[6];
    state.X = X;
    state.dX = dX;

    for (double t = 0; aa_ct_seg_list_eval(seg_list, &state, t); t += 0.1) {
        aa_ct_seg_list_eval(seg_list, &state, t);
        // aa_dump_vec(stdout, state.X, 7);
    }

    aa_ct_seg_list_destroy(seg_list);
    aa_ct_pt_list_destroy(pt_list);
    aa_mem_region_destroy(&reg);
}

void
test_tjq_check( struct aa_mem_region *reg,
                struct aa_ct_pt_list *pt_list,
                struct aa_ct_seg_list *seg_list,
                struct aa_ct_state *limits,
                int dq, int ddq )
{
    (void) pt_list;
    size_t n_q = aa_ct_seg_list_n_q(seg_list);
    struct aa_ct_state *vstate = aa_ct_state_alloc(reg, n_q, 0);
    const struct aa_ct_state *state0 = aa_ct_pt_list_start_state(pt_list);
    const struct aa_ct_state *state1 = aa_ct_pt_list_final_state(pt_list);
    double sdq[n_q];
    AA_MEM_CPY(sdq, state0->q, n_q);
    aa_ct_seg_list_eval(seg_list, vstate, 0);
    aveq("Traj q 0", n_q, state0->q, vstate->q, 1e-3);
    double t = 0;
    double dt = .001;
    double duration = aa_ct_seg_list_duration(seg_list);

    while( aa_ct_seg_list_eval(seg_list, vstate, t) ) {
        if( dq )
            aveq("Traj q integrate dq", n_q, sdq, vstate->q, 1e-2);

        for( size_t i = 0; i < n_q; i ++ ) {
            /* Finite */
            test( "Traj q normal q", isfinite(vstate->q[i]) );
            if( dq )
                test( "Traj q normal dq", isfinite(vstate->dq[i]) );
            if( ddq )
                test( "Traj q normal ddq", isfinite(vstate->ddq[i]) );

            /* Limits */
            if( dq )
                test_flt( "Traj q limit dq",  fabs(vstate->dq[i]),  limits->dq[i], 1e-3 );
            if( ddq )
                test_flt( "Traj q limit ddq", fabs(vstate->ddq[i]), limits->ddq[i], 1e-3 );

            /* Integrate */
            //sdq[i] = vstate->q[i] + dt*vstate->dq[i];
            sdq[i] += dt*vstate->dq[i];
        }
        t += dt;
    }

    test_flt("Traj q dur",  duration, t, 0 );

    int r = aa_ct_seg_list_eval(seg_list, vstate, duration);
    test( "Traj dur eval", AA_CT_SEG_IN == r );
    aveq("Traj q Final", n_q, state1->q, vstate->q, 1e-3);
}

void
test_tjq(size_t n_p)
{
    size_t n_q = 4;

    struct aa_mem_region reg;
    aa_mem_region_init(&reg, 512);

    /* Construct Points */
    struct aa_ct_pt_list *pt_list = aa_ct_pt_list_create(&reg);

    struct aa_ct_state state[n_p];
    AA_MEM_ZERO(state,n_p);

    for (size_t i = 0; i < n_p; i++) {
        state[i].q = AA_MEM_REGION_NEW_N(&reg,double,n_q);
        state[i].n_q = n_q;

        aa_vrand(n_q, state[i].q);

        aa_ct_pt_list_add(pt_list, &state[i]);
    }

    /* Set Limits */
    double dqlim[n_q], ddqlim[n_q];
    struct aa_ct_state limits;
    limits.n_q = n_q;
    limits.dq = dqlim;
    limits.ddq = ddqlim;

    for (size_t j = 0; j < n_q; j++) {
        limits.dq[j] = aa_frand() + .5;
        limits.ddq[j] = aa_frand() + .5;
    }

    /* Generate Trajectory */
    {
        //struct aa_ct_seg_list *seg_list =
            //aa_ct_tjq_pb_generate(&reg, pt_list, &limits);

    }

    {
        struct aa_ct_seg_list *seg_list =
            aa_ct_tjq_lin_generate(&reg, pt_list, &limits);
        test_tjq_check( &reg, pt_list, seg_list,
                        &limits, 1, 0 );
        /* aa_ct_seg_list_plot( seg_list, n_q, .01, */
        /*                      1, 0 ); */
    }

    /* Evaluate and Check */

    /* struct aa_ct_state *vstate = aa_ct_state_alloc(&reg, n_q, 0); */
    /* aa_ct_seg_list_eval(seg_list, vstate, 0); */
    /* aveq("PBlend 0", n_q, state[0].q, vstate->q, 1e-3); */
    /* double t = 0; */

    /* while( aa_ct_seg_list_eval(seg_list, vstate, t) ) { */
    /*     /\* Test limits *\/ */
    /*     for( size_t i = 0; i < n_q; i ++ ) { */
    /*         test( "Pblend normal q", isfinite(vstate->q[i]) ); */
    /*         test( "Pblend normal dq", isfinite(vstate->dq[i]) ); */
    /*         test( "Pblend normal ddq", isfinite(vstate->ddq[i]) ); */

    /*         test_flt( "Pblend limit dq",  fabs(vstate->dq[i]),  limits.dq[i], 1e-3 ); */
    /*         test_flt( "Pblend limit ddq", fabs(vstate->ddq[i]), limits.ddq[i], 1e-3 ); */
    /*     } */
    /*     t += .01; */
    /* } */
    /* aveq("PBlend Final", n_q, state[n_p-1].q, vstate->q, 1e-3); */

    //aa_ct_seg_list_destroy(seg_list);
    aa_ct_pt_list_destroy(pt_list);
    aa_mem_region_destroy(&reg);
}

/**
 * Test making a parabolic blend trajectory
 */
int
main(void)
{
    time_t seed = time(NULL);
    printf("ct_traj seed: %ld\n", seed);
    srand((unsigned int)seed); // might break in 2038
    aa_test_ulimit();

    test_tjX();

    for( size_t i = 0; i < 100; i ++ ) {
        size_t n_p = 2 + (size_t)(10*aa_frand());
        test_tjq(n_p);
        //test_tjq(2);
    }
}
