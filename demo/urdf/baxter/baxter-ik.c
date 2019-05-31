/* -*- mode: C; c-basic-offset: 4; -*- */
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

#include "baxter-demo.h"
#include "amino/rx/rxerr.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_gl.h"

int main(int argc, char *argv[])
{
    (void)argc; (void)argv;

    // Initialize scene graph
    struct aa_rx_sg *scenegraph = baxter_demo_load_baxter(NULL);
    aa_rx_sg_init(scenegraph);

    /*--- setup window ---*/
    struct aa_rx_win * win = baxter_demo_setup_window(scenegraph);
    struct aa_gl_globals *globals = aa_rx_win_gl_globals(win);
    aa_gl_globals_set_show_visual(globals, 1);
    aa_gl_globals_set_show_collision(globals, 0);


    /* --- Solve IK --- */
    aa_rx_frame_id tip_id = aa_rx_sg_frame_id(scenegraph, "right_w2");
    struct aa_rx_sg_sub *ssg = aa_rx_sg_chain_create( scenegraph, AA_RX_FRAME_ROOT, tip_id);
    size_t n_q = aa_rx_sg_config_count(scenegraph);
    size_t n_qs = aa_rx_sg_sub_config_count(ssg);

    assert( 7 == n_qs );

    // start and seed position
    double qstart_all[n_q];
    double qstart_s[7] = {
        .05 * M_PI, // s0
        -.25 * M_PI, // s1
        0, // e0
        .25*M_PI, // e1
        0, // w0
        .25*M_PI, // w1
        0 // w2
    };
    struct aa_dvec qv_start_all = AA_DVEC_INIT(n_q, qstart_all, 1);
    struct aa_dvec qv_start_sub = AA_DVEC_INIT(n_qs, qstart_s, 1);
    aa_dvec_zero(&qv_start_all);
    aa_rx_sg_sub_config_scatter(ssg, &qv_start_sub, &qv_start_all);


    // solver goal
    double q_solution[n_qs];
    struct aa_dvec qv_solution = AA_DVEC_INIT(n_qs, q_solution, 1);
    //double E_ref[7];

    double E_ref[7] = {0, 1, 0, 0,
                       .8, -.25, .3051};
    struct aa_dmat E_mat = AA_DMAT_INIT(7, 1, E_ref, 7);
    {
        double E0[7] = {0, 1, 0, 0,
                        0.6, -0.0, 0.3};
        double E1[7] = {0, 0, 0, 1,
                        0, 0, 0 };
        aa_tf_xangle2quat(-.5*M_PI, E1 );
        aa_tf_qutr_mul( E0, E1, E_ref );
    }

    // solver options
    struct aa_rx_ik_parm *ikp = aa_rx_ik_parm_create();
    aa_rx_ik_parm_set_debug( ikp, 1 ); // print debugging output
    aa_rx_ik_parm_center_configs( ikp, ssg, .1 );
    aa_rx_ik_parm_take_seed( ikp, n_q, qstart_all, AA_MEM_BORROW );

    aa_rx_ik_parm_set_ik_algo(ikp,
                                //AA_RX_IK_JPINV
                                AA_RX_IK_SQP
        );


    /* Default workspace objective */
    aa_rx_ik_parm_set_obj(ikp, aa_rx_ik_opt_err_qlnpv);

    /* An alternate workspace objective */
    //aa_rx_ik_parm_set_obj(ikp, aa_rx_ik_opt_err_dqln);

    /* A jointspace objective and workspace constraint */
    /* { */
    /*     aa_rx_ik_parm_set_obj(ikp,  aa_rx_ik_opt_err_jcenter); */
    /*     aa_rx_ik_parm_set_eqct(ikp, aa_rx_ik_opt_err_qlnpv, 1e-9); */
    /*     /\* Need to update the tolerances since we cannot zero the joint error *\/ */
    /*     aa_rx_ik_parm_set_tol_dq(ikp,1e-6); */
    /*     aa_rx_ik_parm_set_tol_obj_abs(ikp,-1); */
    /* } */

    struct aa_rx_ik_cx * cx = aa_rx_ik_cx_create(ssg, ikp);
    aa_rx_ik_set_start(cx, &qv_start_all);
    aa_rx_ik_set_seed(cx, &qv_start_sub);

    aa_rx_ik_set_restart_time(cx, 1); // restart for 1 second

    aa_tick("Begin Inverse Kinematics...\n");
    int r = aa_rx_ik_solve( cx, &E_mat, &qv_solution );
    aa_tock();

    if( r ) {
        /* NO IK */
        char *e = aa_rx_errstr( aa_mem_region_local_get(), r );
        fprintf(stderr, "Oops, IK failed: `%s' (0x%x)\n", e, r);
        aa_mem_region_local_pop(e);
    } else {

        // set all-config joint position
        aa_rx_sg_sub_config_scatter(ssg, &qv_solution, &qv_start_all);
        aa_rx_win_set_config( win, qv_start_all.len, qv_start_all.data );

        /*--- Do Display ---*/
        aa_rx_win_run();
    }

    /*--- Cleanup ---*/
    aa_rx_sg_destroy(scenegraph);
    aa_rx_win_destroy(win);
    SDL_Quit();

    return 0;
}
