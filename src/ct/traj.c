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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>

#include <amino/ct/traj.h>
#include <amino/ct/traj_internal.h>

AA_API void
aa_ct_traj_init(struct aa_ct_traj *traj, size_t n_q,
                aa_mem_region_t *reg,
                int (*generate)(struct aa_ct_traj *traj, void *cx))
{
    *traj = (struct aa_ct_traj) {0};
    traj->n_q = n_q;

    traj->reg = reg;
    traj->generate = generate;

    traj->points = aa_mem_rlist_alloc(reg);
    traj->segments = aa_mem_rlist_alloc(reg);
}

AA_API int
aa_ct_traj_generate(struct aa_ct_traj *traj, void *cx)
{
    return traj->generate(traj, cx);
}

int
aa_ct_traj_value_it(struct aa_ct_traj *traj, void *v, double t) {
    // Iterate through segment list until no more segments remain
    do {
        struct aa_ct_trajseg *seg =
            (struct aa_ct_trajseg *) traj->last_seg->data;

        // Evaluate segment at time t. Returns 1 if t is contained.
        if (seg->value(traj, seg, v, t))
            return 1;

        traj->last_seg = traj->last_seg->next;
    } while (traj->last_seg != NULL);

    return 0;
}

AA_API int
aa_ct_traj_value(struct aa_ct_traj *traj, void *v, double t)
{
    // Initialize iterator
    if (!traj->last_seg)
        traj->last_seg = traj->segments->head;

    // Add time offset
    t += traj->t_o;
    int r = aa_ct_traj_value_it(traj, v, t);

    // Linear scan if nothing found 
    if (r == 0) {
        traj->last_seg = traj->segments->head;
        r = aa_ct_traj_value_it(traj, v, t);
    }

    return r;
}

AA_API void
aa_ct_trajpt_add(struct aa_ct_traj *traj, struct aa_ct_trajpt *pt)
{
    // Set point index
    if (traj->tail_pt) {
        traj->tail_pt->next = pt;
        pt->i = traj->tail_pt->i + 1;
    } else
        pt->i = 0;

    // Link in to list
    pt->prev = traj->tail_pt;
    traj->tail_pt = pt;

    aa_mem_rlist_enqueue_ptr(traj->points, pt);
}

void
aa_ct_trajseg_add(struct aa_ct_traj *traj, struct aa_ct_trajseg *seg)
{
    // Set segment index
    if (traj->tail_seg) {
        traj->tail_seg->next = seg;
        seg->i = traj->tail_seg->i + 1;
    } else
        seg->i = 0;

    // Link in to list
    seg->prev = traj->tail_seg;
    traj->tail_seg = seg;

    aa_mem_rlist_enqueue_ptr(traj->segments, seg);
}

double
aa_ct_pb_trajseg_limit(double *a, double *b, double *m, size_t n_q)
{
    double mv = DBL_MIN;
    for (size_t i = 0; i < n_q; i++) {
        double v = fabs(((a) ? a[i] : 0) - ((b) ? b[i] : 0)) / ((m) ? m[i] : 1);
        mv = (mv < v) ? v : mv;
    }

    return mv;
}

int
aa_ct_pb_trajseg_value(struct aa_ct_traj *traj, struct aa_ct_trajseg *seg,
                       void *v, double t)
{
    struct aa_ct_pb_trajval *value = (struct aa_ct_pb_trajval *) v;

    struct aa_ct_pb_trajseg *c = (struct aa_ct_pb_trajseg *) seg;
    struct aa_ct_pb_trajseg *p = (struct aa_ct_pb_trajseg *) seg->prev;
    struct aa_ct_pb_trajseg *n = (struct aa_ct_pb_trajseg *) seg->next;

    double dt = t - c->t_s;

    if ((c->t_s - c->b / 2) <= t && t <= (c->t_s + c->b / 2)) {
        // Blend segment
        for (size_t i = 0; i < traj->n_q; i++) {
            double prev_dq = (p) ? p->dq[i] : 0;

            value->q[i] = c->pt->q[i] + prev_dq * dt + \
                c->ddq[i] * pow((dt + c->b / 2), 2) / 2;

            value->dq[i] = prev_dq + c->ddq[i] * (dt + c->b / 2);
        }
    } else if (n && (c->t_s + c->b / 2) <= t \
               && t <= (c->t_s + c->dt - n->b / 2)) {
        // Linear segment
        for (size_t i = 0; i < traj->n_q; i++) {
            value->q[i] = c->pt->q[i] + c->dq[i] * dt;

            value->dq[i] = c->dq[i];
        }
    } else {
        // Who knows what happened man
        return 0;
    }

    return 1;
}

struct aa_ct_pb_trajseg *
aa_ct_pb_trajseg_create(struct aa_ct_traj *traj)
{
    struct aa_ct_pb_trajseg *seg =
        AA_MEM_REGION_NEW(traj->reg, struct aa_ct_pb_trajseg);

    *seg = (struct aa_ct_pb_trajseg) {
        .value = aa_ct_pb_trajseg_value,
        .prev = NULL,
        .next = NULL,
        .dq = AA_MEM_REGION_NEW_N(traj->reg, double, traj->n_q),
        .ddq = AA_MEM_REGION_NEW_N(traj->reg, double, traj->n_q),
        .dt = DBL_MAX,
        .b = DBL_MAX
    };

    bzero(seg->dq, sizeof(double) * traj->n_q);
    bzero(seg->ddq, sizeof(double) * traj->n_q);

    return seg;
}

void
aa_ct_pb_trajseg_update(struct aa_ct_pb_trajseg *seg, double *ddqmax, size_t n_q)
{
    struct aa_ct_pb_trajpt *c = (struct aa_ct_pb_trajpt *) seg->pt;
    struct aa_ct_pb_trajpt *n = (struct aa_ct_pb_trajpt *) c->next;

    // If this isn't the last segment update forward velocities
    if (n) 
        for (size_t i = 0; i < n_q; i++)
            seg->dq[i] = (n->q[i] - c->q[i]) / seg->dt;

    // Calculate acceleration based on new velocities
    struct aa_ct_pb_trajseg *p = (struct aa_ct_pb_trajseg *) seg->prev;
    seg->b = aa_ct_pb_trajseg_limit(seg->dq, (p) ? p->dq : NULL, ddqmax, n_q);

    for (size_t i = 0; i < n_q; i++)
        seg->ddq[i] = (seg->dq[i] - ((p) ? p->dq[i] : 0)) / seg->b;

    // Update start and end times. Assume later segments will also be updated.
    if (!p) {
        seg->t_s = 0;
        seg->t_f = seg->dt;
    } else {
        seg->t_s = p->t_f;
        seg->t_f = seg->t_s + seg->dt;
    }

    if (!n)
        seg->t_f += seg->b / 2;
}

AA_API int
aa_ct_pb_traj_generate(struct aa_ct_traj *traj, void *cx)
{
    struct aa_ct_pb_limits *limits = (struct aa_ct_pb_limits *) cx;

    // Initialize segments
    struct aa_ct_pb_trajpt *curr_pt;
    struct aa_ct_pb_trajseg *curr_seg;
    for (curr_pt = (struct aa_ct_pb_trajpt *) traj->points->head->data;
         curr_pt != NULL; curr_pt = (struct aa_ct_pb_trajpt *) curr_pt->next) {
        curr_seg = aa_ct_pb_trajseg_create(traj);
        curr_seg->pt = curr_pt;
        aa_ct_trajseg_add(traj, (struct aa_ct_trajseg *) curr_seg);

        struct aa_ct_pb_trajpt *next = (struct aa_ct_pb_trajpt *) curr_pt->next;
        curr_seg->dt = (next)
            ? aa_ct_pb_trajseg_limit(next->q, curr_pt->q,
                                     limits->dqmax, traj->n_q)
            : DBL_MAX;

        aa_ct_pb_trajseg_update(curr_seg, limits->ddqmax, traj->n_q);
    }

    for (;;) {
        bool flag = false;
        double f[traj->n_q], *fp = f;
        struct aa_ct_pb_trajseg *prev_seg = NULL, *next_seg = NULL;
        for (curr_seg = (struct aa_ct_pb_trajseg *) traj->segments->head->data;
             curr_seg != NULL; curr_seg = next_seg, prev_seg = curr_seg, fp++) {
            next_seg = (struct aa_ct_pb_trajseg *) curr_seg->next;

            // Check for blend region overlap in next and previous segment
            bool prev_overlap = (prev_seg)
                ? (prev_seg->dt < curr_seg->b                     
                   && (2 * prev_seg->dt - prev_seg->b) < curr_seg->b)
                : false;

            bool next_overlap = (next_seg)
                ? (curr_seg->dt < curr_seg->b                          
                   && (2 * curr_seg->dt - next_seg->b) < curr_seg->b)
                : false;

            // Calculate reduction constant if overlapping
            if (prev_overlap || next_overlap) {
                double prev_dt = (prev_seg) ? prev_seg->dt : DBL_MAX; 
                *fp = sqrt(fmin(prev_dt, curr_seg->dt) / curr_seg->b);
                flag = true;
            } else
                *fp = 1;
        }

        // If done, set time offset and exit
        if (!flag) {
            curr_seg = (struct aa_ct_pb_trajseg *) traj->segments->head->data;
            traj->t_o = -curr_seg->b / 2;

            break;
        }

        // If overlap, update all segments
        size_t i = 0;
        for (curr_seg = (struct aa_ct_pb_trajseg *) traj->segments->head->data;
             curr_seg != NULL; i++) {
            curr_seg->dt /= (i < traj->n_q - 1) ? fmin(f[i + 1], f[i]) : f[i];
            aa_ct_pb_trajseg_update(curr_seg, limits->ddqmax, traj->n_q);
        }
    }

    return 0;
}
