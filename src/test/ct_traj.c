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

#include "amino/ct/traj.h"


static void
gnuplot_file(const char *filename, size_t n_l)
{
    FILE *pipe = popen("gnuplot -persistent", "w");
    // fprintf(pipe,"set terminal svg enhanced background rgb 'white'\n");
    fprintf(pipe, "plot ");
    for (size_t i = 0; i < n_l; i++)
        fprintf(pipe, "\"%s\" using 1:%lu title '%lu' with lines%s",
                filename, i + 2, i, (i == (n_l - 1)) ? "\n" : ", ");
    fclose(pipe);
}

int
main(void)
{
    struct aa_mem_region reg;
    aa_mem_region_init(&reg, 1024);

    size_t n_q = 4;

    // Initialize trajectory
    struct aa_ct_traj traj;
    aa_ct_traj_init(&traj, n_q, &reg, aa_ct_pb_traj_generate);

    // Setup limits
    struct aa_ct_pb_limits limits = {
        .dqmax = AA_MEM_REGION_NEW_N(&reg, double, n_q),
        .ddqmax = AA_MEM_REGION_NEW_N(&reg, double, n_q)
    };

    for (size_t i = 0; i < n_q; i++) {
        limits.dqmax[i] = 1;
        limits.ddqmax[i] = 1;
    }

    for (size_t i = 0; i < 4; i++) {
        struct aa_ct_pb_trajpt *pt =
            AA_MEM_REGION_NEW(&reg, struct aa_ct_pb_trajpt);

        pt->q = AA_MEM_REGION_NEW_N(&reg, double, n_q);

        for (size_t j = 0; j < n_q; j++)
            pt->q[j] = (double) rand() / RAND_MAX;

        aa_ct_trajpt_add(&traj, (struct aa_ct_trajpt *) pt);
    }

    aa_ct_traj_generate(&traj, &limits);

    const char *qfile = "/tmp/trajq.temp";
    const char *dqfile = "/tmp/trajv.temp";
    FILE *qtemp = fopen(qfile, "w");
    FILE *dqtemp = fopen(dqfile, "w");

    struct aa_ct_pb_trajval val = {
        .q = AA_MEM_REGION_NEW_N(&reg, double, n_q),
        .dq = AA_MEM_REGION_NEW_N(&reg, double, n_q)
    };

    for (double t = 0; aa_ct_traj_value(&traj, &val, t); t += 0.01) {
        fprintf(qtemp, "%lf ", t);
        fprintf(dqtemp, "%lf ", t);

        aa_dump_vec(qtemp, val.q, n_q);
        aa_dump_vec(dqtemp, val.dq, n_q);
    }

    fclose(qtemp);
    fclose(dqtemp);

    gnuplot_file(qfile, n_q);
    gnuplot_file(dqfile, n_q);

    return 0;
}
