/* -*- mode: C; c-basic-offset: 4; -*- */
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

#include <amino.hpp>

#include <amino/ct/state.h>
#include <amino/ct/traj.h>
#include <amino/ct/traj_internal.hpp>

using namespace amino;

/**
 * Generic function to add an element to a list container.
 *
 * @param list List to add el to
 * @param el   Element to add
 */
template <typename T, typename U>
static inline void
aa_ct_list_add(T *list, U *el)
{
    el->next = NULL;
    if (list->list.size())
        list->list.back()->next = el;

    el->prev = list->list.back();
    list->list.push_back(el);
}

/**
 * Point lists
 */
AA_API struct aa_ct_pt_list *
aa_ct_pt_list_create(struct aa_mem_region *reg)
{
    return new(reg) struct aa_ct_pt_list(reg);
}

AA_API void
aa_ct_pt_list_add(struct aa_ct_pt_list *list, struct aa_ct_state *state)
{
    if (list->list.size()) {
        struct aa_ct_pt *p_pt = list->list.back();

        if (aa_ct_state_eq(&p_pt->state, state))
            return;
    }

    struct aa_ct_pt *pt = new(&list->reg) struct aa_ct_pt;
    aa_ct_state_clone(&list->reg, &pt->state, state);

    aa_ct_list_add(list, pt);
}

AA_API void
aa_ct_pt_list_destroy(struct aa_ct_pt_list *list)
{
    list->~aa_ct_pt_list();
};

AA_API void
aa_ct_pt_list_dump(FILE *stream, struct aa_ct_pt_list *list)
{
    size_t i = 0;
    struct aa_ct_pt *c_pt = list->list.front();
    for (; c_pt != NULL; c_pt = c_pt->next) {
        fprintf(stream, "PT %lu: \n", i++);
        aa_ct_state_dump(stream, &c_pt->state);
        fputc('\n', stream);
    }
}

/**
 * Segment lists
 */
AA_API void
aa_ct_seg_list_add(struct aa_ct_seg_list *list, struct aa_ct_seg *seg)
{
    aa_ct_list_add(list, seg);
}

AA_API int
aa_ct_seg_list_eval(struct aa_ct_seg_list *list, struct aa_ct_state *state,
                    double t)
{
    if (!list->it_on) {
        list->it = list->list.begin();
        list->it_on = 1;
    }

    do {
        struct aa_ct_seg *seg = *list->it;

        int r;
        if ((r = seg->eval(seg, state, t)))
            return r;

        list->it++;
    } while (list->it != list->list.end());

    list->it_on = 0;
    return 0;
}

AA_API void
aa_ct_seg_list_destroy(struct aa_ct_seg_list *list)
{
    list->~aa_ct_seg_list();
};

/**
 * Pipe a file to gnuplot to display.
 *
 * @param filename Filename to plot
 * @param n_l      Number of lines contained in file
 */
static void
aa_ct_gnuplot_file(const char *filename, size_t n_l, const char *ylabel)
{
    FILE *pipe = popen("gnuplot -persistent", "w");
    // fprintf(pipe,"set terminal svg enhanced background rgb 'white'\n");
    fprintf(pipe, "set xlabel \"time (s)\"\n");
    if( ylabel ) fprintf(pipe, "set ylabel \"%s\"\n", ylabel);
    fprintf(pipe, "plot ");
    for (size_t i = 0; i < n_l; i++)
        fprintf(pipe, "\"%s\" using 1:%lu title '%lu' with lines%s",
                filename, i + 2, i, (i == (n_l - 1)) ? "\n" : ", ");
    fclose(pipe);
}

AA_API void
aa_ct_seg_list_plot(struct aa_ct_seg_list *list, size_t n_q, double dt)
{
    const char *qfile = "/tmp/aa_ct_seg_list_q.temp";
    const char *dqfile = "/tmp/aa_ct_seg_list_dq.temp";

    FILE *qtemp = fopen(qfile, "w");
    FILE *dqtemp = fopen(dqfile, "w");

    struct aa_ct_state state;
    double q[n_q], dq[n_q];
    state.n_q = n_q;
    state.q = q;
    state.dq = dq;

    for (double t = 0; aa_ct_seg_list_eval(list, &state, t); t += dt) {
        aa_ct_seg_list_eval(list, &state, t);

        fprintf(qtemp, "%lf ", t);
        fprintf(dqtemp, "%lf ", t);

        aa_dump_vec(qtemp, state.q, n_q);
        aa_dump_vec(dqtemp, state.dq, n_q);
    }

    fclose(qtemp);
    fclose(dqtemp);

    aa_ct_gnuplot_file(qfile, n_q, "Position");
    aa_ct_gnuplot_file(dqfile, n_q, "Velocity");
}


/**
 * Parabolic blend trajectory
 */

/**
 * Parabolic blend trajectory segment context.
 */
struct aa_ct_seg_pb_cx {
    size_t n_q;  ///< Number of configurations

    double *q;   ///< Waypoint position
    double *dq;  ///< Velocity
    double *ddq; ///< Acceleration

    double t;    ///< Start time
    double dt;   ///< Duration of segment
    double b;    ///< Blend time
};

/**
 * Get the context for the neighboring segments.
 *
 * @param seg Segment to grab neighbors of
 * @param p   Previous segment context
 * @param c   Current segment context
 * @param n   Next segment context
 */
void
aa_ct_tj_pb_nbrs(struct aa_ct_seg *seg, struct aa_ct_seg_pb_cx **p,
                 struct aa_ct_seg_pb_cx **c, struct aa_ct_seg_pb_cx **n)
{
    *c = (struct aa_ct_seg_pb_cx *) seg->cx;
    *n = (seg->next) ? (struct aa_ct_seg_pb_cx *) seg->next->cx : NULL;
    *p = (seg->prev) ? (struct aa_ct_seg_pb_cx *) seg->prev->cx : NULL;
}

/**
 * Evaluate a parabolic blend trajectory segment.
 *
 * @return 0 if not in segment, 1 if.
 */
int
aa_ct_tj_pb_eval(struct aa_ct_seg *seg, struct aa_ct_state *state, double t)
{
    struct aa_ct_seg_pb_cx *c_cx, *p_cx, *n_cx;
    aa_ct_tj_pb_nbrs(seg, &p_cx, &c_cx, &n_cx);

    double dt = t - c_cx->t;
    if ((c_cx->t - c_cx->b / 2) <= t && t <= (c_cx->t + c_cx->b / 2)) {
        // Blend region
        for (size_t i = 0; i < c_cx->n_q; i++) {
            double p_dq = (p_cx) ? p_cx->dq[i] : 0;

            state->q[i] = c_cx->q[i] + p_dq * dt + \
                c_cx->ddq[i] * pow((dt + c_cx->b / 2), 2) / 2;
            state->dq[i] = p_dq + c_cx->ddq[i] * (dt + c_cx->b / 2);
        }

    } else if (n_cx && ((c_cx->t + c_cx->b / 2) <= t
                        && t <= (c_cx->t + c_cx->dt - n_cx->b / 2))) {
        // Linear region
        for (size_t i = 0; i < c_cx->n_q; i++) {
            state->q[i] = c_cx->q[i] + c_cx->dq[i] * dt;
            state->dq[i] = c_cx->dq[i];
        }

    } else {
        // Off-the-rails region
        return 0;
    }

    return 1;
}

/**
 * Calculates the maximum (a - b) / m value for a n_q vector.
 *
 * @return The maximum value.
 */
double
aa_ct_tj_pb_limit(double *a, double *b, double *m, size_t n_q)
{
    double mv = DBL_MIN;
    for (size_t i = 0; i < n_q; i++) {
        double v = fabs(((a) ? a[i] : 0) - ((b) ? b[i] : 0)) / ((m) ? m[i] : 1);
        mv = (mv < v) ? v : mv;
    }

    return mv;
}

/**
 * Create a new parabolic blend segment allocated from a memory region.
 *
 * @param reg Region to allocate from
 * @param pt  Point to base segment from
 * @param limits Kinematic limits
 *
 * @return Returns a newly allocated and initialized segment.
 */
struct aa_ct_seg *
aa_ct_tj_pb_new(struct aa_mem_region *reg, struct aa_ct_pt *pt,
                struct aa_ct_state *limits)
{
    size_t n_q = pt->state.n_q;
    struct aa_ct_seg *seg = new(reg) struct aa_ct_seg();
    seg->eval = aa_ct_tj_pb_eval;

    struct aa_ct_seg_pb_cx *cx = new(reg) struct aa_ct_seg_pb_cx();
    bzero(cx, sizeof(struct aa_ct_seg_pb_cx));

    cx->n_q = n_q;
    cx->q = AA_MEM_REGION_NEW_N(reg, double, n_q);
    memcpy(cx->q, pt->state.q, sizeof(double) * n_q);

    cx->dq = AA_MEM_REGION_NEW_N(reg, double, n_q);
    cx->ddq = AA_MEM_REGION_NEW_N(reg, double, n_q);

    cx->t = 0;
    cx->dt = DBL_MAX;
    if (pt->next)
        cx->dt = aa_ct_tj_pb_limit(pt->next->state.q, cx->q, limits->dq, n_q);
    cx->b = 0;

    seg->cx = (void *) cx;
    return seg;
}

/**
 * Update a segment based on new dt.
 *
 * @param seg    Segment to update
 * @param limits Kinematic limits
 */
void
aa_ct_tj_pb_update(struct aa_ct_seg *seg, struct aa_ct_state *limits)
{
    struct aa_ct_seg_pb_cx *c_cx, *p_cx, *n_cx;
    aa_ct_tj_pb_nbrs(seg, &p_cx, &c_cx, &n_cx);

    // If this isn't the last segment update forward velocities
    if (n_cx)
        for (size_t i = 0; i < c_cx->n_q; i++)
            c_cx->dq[i] = (n_cx->q[i] - c_cx->q[i]) / c_cx->dt;

    // Calculate acceleration based on new velocities
    c_cx->b = aa_ct_tj_pb_limit(c_cx->dq, (p_cx) ? p_cx->dq : NULL,
                                limits->ddq, c_cx->n_q);
 
    for (size_t i = 0; i < c_cx->n_q; i++)
        c_cx->ddq[i] = (c_cx->dq[i] - ((p_cx) ? p_cx->dq[i] : 0)) / c_cx->b;
 
    // Update start time. Assume earlier segments have been updated.
    if (p_cx)
        c_cx->t = p_cx->t + p_cx->dt;
    else
        c_cx->t = c_cx->b / 2;
}

AA_API struct aa_ct_seg_list *
aa_ct_tjq_pb_generate(struct aa_mem_region *reg, struct aa_ct_pt_list *pt_list,
                      struct aa_ct_state *limits)
{
    struct aa_ct_seg_list *list = new(reg) struct aa_ct_seg_list(reg);

    // Populate segment list with one segment per point
    struct aa_ct_pt *c_pt = pt_list->list.front();
    for (; c_pt != NULL; c_pt = c_pt->next)
        aa_ct_seg_list_add(list, aa_ct_tj_pb_new(&list->reg, c_pt, limits));

    bool flag;
    size_t n = pt_list->list.size();

    // Iterate and update segments until no overlap
    do {
        size_t i = 0;
        double f[n];
        flag = false;

        struct aa_ct_seg *c_seg = list->list.front();
        for (; c_seg != NULL; c_seg = c_seg->next, i++) {
            // Update each segment
            aa_ct_tj_pb_update(c_seg, limits);

            struct aa_ct_seg_pb_cx *c_cx, *p_cx, *n_cx;
            aa_ct_tj_pb_nbrs(c_seg, &p_cx, &c_cx, &n_cx);

            bool overlap = false;

            // Check for backwards overlap
            if (p_cx)
                overlap |= p_cx->dt < c_cx->b \
                    && (2 * p_cx->dt - p_cx->b) < c_cx->b;

            // Check for forwards overlap
            if (n_cx)
                overlap |= c_cx->dt < c_cx->b \
                    && (2 * c_cx->dt - n_cx->b) < c_cx->b;

            // Calculate scaling constant if overlap
            if (overlap) {
                double p_dt = (p_cx) ? p_cx->dt : DBL_MAX;
                f[i] = sqrt(fmin(p_dt, c_cx->dt) / c_cx->b);
                flag = true;
            } else
                f[i] = 1;
        }

        i = 0;
        c_seg = list->list.front();
        for (; c_seg != NULL; c_seg = c_seg->next, i++)
            // Scale each region's time based on calculated constant
            ((struct aa_ct_seg_pb_cx *) c_seg->cx)->dt /= \
                (i < n - 1) ? fmin(f[i + 1], f[i]) : 1;
    } while (flag);

    return list;
}


int
aa_ct_tjX_pb_eval(struct aa_ct_seg *seg, struct aa_ct_state *state, double t)
{
    double *tq = state->q;
    double *tdq = state->dq;

    double X[6], dX[6];
    state->q = X;
    state->dq = dX;

    int r = aa_ct_tj_pb_eval(seg, state, t);

    if (r) {
        aa_tf_rotvec2quat(&state->q[3], state->X);
        state->X[4] = state->q[0];
        state->X[5] = state->q[1];
        state->X[6] = state->q[2];
    }

    state->q = tq;
    state->dq = tdq;

    return r;
}

// TODO: This is a hack, replace with a real function later.
AA_API struct aa_ct_seg_list *
aa_ct_tjX_pb_generate(struct aa_mem_region *reg, struct aa_ct_pt_list *pt_list,
                      struct aa_ct_state *limits)
{
    struct aa_ct_pt_list *Xpt_list = aa_ct_pt_list_create(reg);

    struct aa_ct_pt *c_pt = pt_list->list.front();
    struct aa_ct_pt *p_pt = NULL;
    for (; c_pt != NULL; c_pt = c_pt->next) {
        struct aa_ct_state state;
        bzero(&state, sizeof(struct aa_ct_state));

        double q[6];
        state.n_q = 6;

        for (size_t i = 0; i < 6; i++) {
            double *X = c_pt->state.X;
            memcpy(q, &X[4], sizeof(double) * 3);

            if (p_pt)
                aa_tf_quat2rotvec_near(X, p_pt->state.q + 3, &q[3]);
            else
                aa_tf_quat2rotvec(X, &q[3]);
        }

        state.q = q;
        aa_ct_pt_list_add(Xpt_list, &state);

        p_pt = Xpt_list->list.back();
    }

    struct aa_ct_state Xlimits;
    Xlimits.n_q = 6;
    Xlimits.dq = limits->dX;
    Xlimits.ddq = limits->ddX;

    struct aa_ct_seg_list *Xseg_list =
        aa_ct_tjq_pb_generate(reg, Xpt_list, &Xlimits);

    struct aa_ct_seg *c_seg = Xseg_list->list.front();
    for (; c_seg != NULL; c_seg = c_seg->next) {
        c_seg->eval = aa_ct_tjX_pb_eval;
    }

    return Xseg_list;
}
