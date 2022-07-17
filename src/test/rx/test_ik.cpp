/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2022, Colorado School of Mines
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

#include "config.h"

#define GL_GLEXT_PROTOTYPES

#include <stdio.h>
#include <math.h>
#include <getopt.h>
#include <unistd.h>



#include "amino.h"


#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_gl.h"
#include "amino/rx/scene_win.h"
#include "amino/rx/scene_geom.h"
#include "amino/rx/scene_ik.h"
#include "amino/rx/scene_fk.h"

#ifdef __cplusplus
#include "amino/mat.hpp"
#include "amino/mem.hpp"
#include "amino/tf.hpp"
#endif

AA_API struct aa_rx_sg * aa_rx_dl_sg__7dof(struct aa_rx_sg *sg, const char *root);

static int SCREEN_WIDTH = 800;
static int SCREEN_HEIGHT = 600;

int main(int argc, char *argv[])
{
    const char *end_effector="hand";

    long unsigned opt_iterations = 1;
    double opt_timeout = 0;

    int gui = 0;
    int iterations = 1000;
    time_t seed = time(NULL);

    // Reduce iterations under valgrind
    if(getenv("AMINO_VALGRIND")) iterations /= 10;


    double allowed_failures = .05;

    /* Parse Options */
    {
        int c;
        opterr = 0;

        while ((c = getopt(argc, argv, "gs:i:f:?")) != -1) {
            switch(c) {
            case 'g':
                gui = 1;
                break;
            case 's': {
                char *endptr = NULL;
                seed = strtol(optarg, &endptr, 10);
                if (endptr == optarg || *endptr != '\0') {
                    fprintf(stderr, "Invalid seed: `%s'\n", optarg);
                    exit(EXIT_FAILURE);
                }
                break;
            }
            case 'i': {
                char *endptr = NULL;
                iterations = (int)strtol(optarg, &endptr, 10);
                if (endptr == optarg || *endptr != '\0' || iterations <= 0) {
                    fprintf(stderr, "Invalid iterations: `%s'\n", optarg);
                    exit(EXIT_FAILURE);
                }
                break;
            }
            case 'f': {
                char *endptr = NULL;
                allowed_failures = strtod(optarg, &endptr);
                if (endptr == optarg || *endptr != '\0' ||
                    allowed_failures < 0 || allowed_failures > 1) {
                    fprintf(stderr, "Invalid allowed failure rate: `%s'\n",
                            optarg);
                    exit(EXIT_FAILURE);
                }
                break;
            }
            case '?':
                puts("Usage: test_ik [OPTIONS] \n"
                     "Test IK options"
                     "\n"
                     "Options:\n"
                     "  -g             Display scene in a GUI\n"
                     "  -i             Test Iterations\n"
                     "  -s             Random Seed\n"
                     "  -f             Allowed failure rate in interval [0.0, 1.0]\n"
                     "\n"
                     "\n"
                     "Report bugs to " PACKAGE_BUGREPORT "\n" );
                exit(EXIT_SUCCESS);
                break;
            default:
                exit(EXIT_FAILURE);
            }
        }
    }

    /* Random Seed */
    printf("seed: %ld\n", seed);
    srand((unsigned int)seed); // might break in 2038

    /* Initialize scene graph */
    struct aa_rx_sg *scenegraph = aa_rx_dl_sg__7dof(NULL, "");
    aa_rx_sg_init(scenegraph);


    aa_rx_frame_id id = aa_rx_sg_frame_id(scenegraph, end_effector);
    if (AA_RX_FRAME_NONE == id) {
        fprintf(stderr, "Could not find frame `%s'\n", end_effector);
        exit(EXIT_FAILURE);
    }
    struct aa_rx_sg_sub *sub =
        aa_rx_sg_chain_create(scenegraph, AA_RX_FRAME_ROOT, id);

    /* Center configurations */
    amino::RegionScope rs_main;

    size_t m_all = aa_rx_sg_config_count(scenegraph);
    amino::DVec q_all(rs_main, m_all), min_all(rs_main, m_all), max_all(rs_main, m_all);
    size_t m_sub = aa_rx_sg_sub_config_count(sub);
    amino::DVec q_sub(rs_main, m_sub), min_sub(rs_main, m_sub), max_sub(rs_main, m_sub);

    for (size_t i = 0; i < m_all; i++) {
        //aa_rx_sg_get_limit_pos(scenegraph, (aa_rx_config_id)i, &min[i], &max[i]);
        min_all[i] = -M_PI;
        max_all[i] = M_PI;
        q_all[i] = (max_all[i] + min_all[i]) / 2;
    }
    aa_rx_sg_sub_config_gather(sub, &min_all, &min_sub);
    aa_rx_sg_sub_config_gather(sub, &max_all, &max_sub);

    printf("min: ");
    aa_dump_vec(stdout, min_sub.data, min_sub.len);
    printf("max: ");
    aa_dump_vec(stdout, max_sub.data, max_sub.len);

    /* setup window */
    struct aa_rx_win *win = NULL;
    if (gui) {
        win = aa_rx_win_default_create("Amino: IK Test", SCREEN_WIDTH, SCREEN_HEIGHT);
        //aa_rx_win_set_sg(win, scenegraph);
        aa_rx_win_set_sg_sub(win, sub);
        aa_rx_win_set_bconfig(win, &q_all);
        aa_rx_win_run_async();
    }

    /* Setup the solver */
    struct aa_rx_fk *fk = aa_rx_fk_alloc(scenegraph, rs_main);
    struct aa_rx_ik_parm *ko = aa_rx_ik_parm_create();
    /* aa_rx_ik_parm_set_max_iterations(ko, opt_iterations); */
    /* aa_rx_ik_parm_set_timeout(ko, opt_timeout); */
    struct aa_rx_ik_cx *ik = aa_rx_ik_cx_create(sub, ko);

    amino::DVec q_sub_sol(rs_main, m_sub);

    /* Loop to run the tests */
    int solved = 0, failed = 0;

    for (int i = 0;
         i < iterations && (NULL == win || aa_rx_win_is_running(win)); i++) {
        // FK at random configuration
        {
            for( size_t k = 0; k < m_sub; k++ ) {
                q_sub[k] = aa_frand_minmax(min_sub[k], max_sub[k]);
            }

            aa_rx_sg_sub_config_scatter(sub, &q_sub, &q_all);
            aa_rx_fk_all(fk, &q_all);
        }

        // Solve the IK
        {
            amino::QuatTran E;
            aa_rx_fk_get_abs_qutr(fk, id, E.data);
            amino::DMat E_mat(7, 1, E.data, 7);

            int r = aa_rx_ik_solve(ik, &E_mat, &q_sub_sol);
            int x = aa_rx_ik_check(ik, &E_mat, &q_sub_sol);

            if (0 == x) solved++;
            else failed++;
        }

        // If window, set the configuration and sleep
        if(win) {
            aa_rx_sg_sub_config_scatter(sub, &q_sub_sol, &q_all);
            aa_rx_win_set_bconfig(win, &q_all);
            aa_dump_vec(stdout, q_sub.data, q_sub.len);
            usleep(250000);
        }
    }

    double solved_rate = (double) solved / (solved + failed);
    double failed_rate = 1 - solved_rate;
    printf("Solved: %d (%.2f%%)\n", solved, 100*solved_rate);
    printf("Failed: %d (%.2f%%)\n", failed, 100*failed_rate);

    /* Cleanup */
    if(win) {
        aa_rx_win_destroy(win);
        SDL_Quit();
    }

    aa_rx_ik_parm_destroy(ko);
    aa_rx_ik_cx_destroy(ik);
    aa_rx_sg_sub_destroy(sub);
    aa_rx_sg_destroy(scenegraph);

    return failed_rate > allowed_failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
