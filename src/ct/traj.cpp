/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2016-2017 Rice University
 * All rights reserved.
 *
 * Author(s): Zachary K. Kingston <zak@rice.edu>
 *            Neil T. Dantam <ntd@rice.edu>
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

#include <math.h>

#include <amino.hpp>

#include <amino/ct/state.h>
#include <amino/ct/traj.h>
#include <amino/ct/traj_internal.hpp>

using namespace amino;

/**
 * Generic function to add an element to the back of a list container.
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
 * Generic function to add an element to the front of a list container.
 *
 * @param list List to add el to
 * @ param el  Element to add to the front.
 */
template <typename T, typename U>
static inline void
aa_ct_list_add_front(T *list, U *el)
{
    el->prev = NULL;
    if (list->list.size()) {
        list->list.front()->prev = el;
    }

    el->next = list->list.front();
    list->list.insert(list->list.begin(), el);
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
    /* Don't add the same point if it's already at the end. */
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
aa_ct_pt_list_add_front(struct aa_ct_pt_list *list, struct aa_ct_state *state)
{
    /* Don't add the same point if it's already at the beginning. */
    if (list->list.size()) {
        struct aa_ct_pt *f_pt = list->list.front();

        if (aa_ct_state_eq(&f_pt->state, state))
            return;
    }

    struct aa_ct_pt *pt = new(&list->reg) struct aa_ct_pt;
    aa_ct_state_clone(&list->reg, &pt->state, state);

    aa_ct_list_add_front(list, pt);
}


void aa_ct_pt_list_add_qutr(struct aa_ct_pt_list *list, const double E[7])
{
    struct aa_ct_state state;
    AA_MEM_ZERO(&state,1);
    state.X = (double*)E;

    aa_ct_pt_list_add(list,&state);
}

void aa_ct_pt_list_add_q(struct aa_ct_pt_list *list, size_t n_q, const double *q)
{
    struct aa_ct_state state;
    AA_MEM_ZERO(&state,1);
    state.q = (double*)q;
    state.n_q = n_q;

    aa_ct_pt_list_add(list,&state);
}

AA_API void
aa_ct_pt_list_destroy(struct aa_ct_pt_list *list)
{
    list->~aa_ct_pt_list();
};


const struct aa_ct_state *
aa_ct_pt_list_start_state(const struct aa_ct_pt_list *list)
{
    return &list->list.front()->state;
}


const struct aa_ct_state *
aa_ct_pt_list_final_state(const struct aa_ct_pt_list *list)
{
    return &list->list.back()->state;
}

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


void aa_ct_seg_list_add_cx( struct aa_ct_seg_list *list,
                            aa_ct_seg_eval_fun eval,
                            void *cx )
{
    struct aa_ct_seg *seg = AA_MEM_REGION_NEW(&list->reg, aa_ct_seg);
    seg->cx = cx;
    seg->eval = eval;
    aa_ct_seg_list_add( list, seg );

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
    return AA_CT_SEG_OUT;
}

int aa_ct_seg_list_eval_q(struct aa_ct_seg_list *list, double t, size_t n, double *q)
{
    struct aa_ct_state state;
    AA_MEM_ZERO(&state,1);
    state.n_q = n;
    state.q = q;
    return aa_ct_seg_list_eval(list,&state,t);
}

int aa_ct_seg_list_eval_dq(struct aa_ct_seg_list *list, double t, size_t n, double *q, double *dq)
{
    struct aa_ct_state state;
    AA_MEM_ZERO(&state,1);
    state.n_q = n;
    state.q = q;
    state.dq = dq;
    return aa_ct_seg_list_eval(list,&state,t);
}

AA_API void
aa_ct_seg_list_destroy(struct aa_ct_seg_list *list)
{
    list->~aa_ct_seg_list();
};

size_t aa_ct_seg_list_n_q(const struct aa_ct_seg_list *list)
{
    return list->n_q;
}

double aa_ct_seg_list_duration(const struct aa_ct_seg_list *list)
{
    return list->duration;
}

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
 * Constant joint velocity segment.
 */
struct aa_ct_seg_dq {
    size_t n_q;   ///< Number of configurations

    double *q0;   ///< Waypoint position
    double *dq;   ///< Velocity

    double t0;   ///< Start time
    double t1;   ///< Final time
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
aa_ct_tj_dq_nbrs(struct aa_ct_seg *seg, struct aa_ct_seg_dq **p,
                 struct aa_ct_seg_dq **c, struct aa_ct_seg_dq **n)
{
    *c = (struct aa_ct_seg_dq *) seg->cx;
    *n = (seg->next) ? (struct aa_ct_seg_dq *) seg->next->cx : NULL;
    *p = (seg->prev) ? (struct aa_ct_seg_dq *) seg->prev->cx : NULL;
}

static int aa_ct_seg_dq_eval( struct aa_ct_seg *seg,
                              struct aa_ct_state *state, double t)
{
    struct aa_ct_seg_dq *cx = (struct aa_ct_seg_dq *)seg->cx;
    if( t >= cx->t0 && t <= cx->t1 ) {
        size_t n = AA_MIN( cx->n_q, state->n_q );
        double tt = t - cx->t0;

        if( state->q ) {
            for( size_t i = 0; i < n; i ++ ) {
                state->q[i] = cx->q0[i] + tt * cx->dq[i];
            }
        }
        if( state->dq ) {
            AA_MEM_CPY(state->dq, cx->dq, n);
        }

        return AA_CT_SEG_IN;
    } else {
        return AA_CT_SEG_OUT;
    }
}


static struct aa_ct_seg *
aa_ct_seg_dq_new( struct aa_mem_region *reg,
                  double t0, struct aa_ct_state *state0,
                  struct aa_ct_state *state1,
                  struct aa_ct_limit *limits )
{
    /* Allocate */
    struct aa_ct_seg_dq *cx = AA_MEM_REGION_NEW(reg,struct aa_ct_seg_dq);
    AA_MEM_ZERO(cx,1);
    cx->n_q = AA_MIN(state0->n_q, state1->n_q);
    cx->dq = AA_MEM_REGION_NEW_N( reg, double, cx->n_q );
    cx->q0 = AA_MEM_REGION_NEW_N( reg, double, cx->n_q );
    AA_MEM_CPY(cx->q0, state0->q, cx->n_q);
    cx->t0 = t0;
    cx->t1 = t0;

    struct aa_ct_seg *seg = AA_MEM_REGION_NEW(reg,struct aa_ct_seg);
    AA_MEM_ZERO(seg,1);
    seg->eval = aa_ct_seg_dq_eval;
    seg->cx = cx;
    seg->type = AA_CT_LIN_SEG;

    /* Compute segment time */
    double dt = 0;
    for( size_t i = 0; i < cx->n_q; i ++ ) {
        cx->dq[i] = state1->q[i] - state0->q[i];
        double dti;
        if (cx->dq[i] < 0) {
            dti = cx->dq[i] / limits->min->dq[i];
        } else {
            dti = cx->dq[i] / limits->max->dq[i];
        }
        dt = AA_MAX(dt, dti);

        if( ! std::isfinite(dti) ) {
            fprintf(stderr, "WARNING: time for configuration %lu of linear segment is not normal.\n",
                    i);
        }
    }

    /* Compute velocity */
    for( size_t i = 0; i < cx->n_q; i ++ ) {
        cx->dq[i] /= dt;
    }
    cx->t1 = cx->t0 + dt;

    return seg;
}


struct aa_ct_seg_list *aa_ct_tjq_lin_generate(struct aa_mem_region *reg,
                                              struct aa_ct_pt_list *list,
                                              struct aa_ct_limit *limits)
{
    struct aa_ct_seg_list *segs = new(reg) aa_ct_seg_list(reg);

    auto itr0 = list->list.begin();
    auto itr1 = list->list.begin();
    itr1++;
    segs->n_q = (*itr0)->state.n_q;

    segs->duration = 0;
    for( ; itr1 != list->list.end(); itr0++, itr1++ ) {

        if( segs->n_q != (*itr1)->state.n_q ) {
            fprintf(stderr, "WARNING: mistmactched confiuration count during trajectory generation.\n");
        }

        struct aa_ct_seg *seg = aa_ct_seg_dq_new( reg, segs->duration,
                                                  &(*itr0)->state, &(*itr1)->state, limits );
        aa_ct_seg_list_add(segs, seg);

        struct aa_ct_seg_dq *cx = (struct aa_ct_seg_dq*)seg->cx;
        segs->duration = cx->t1;
    }

    return segs;
}


/**
 * Parabolic blend trajectory segment context.
 * The last segment in a trajectory is a special case: it should always have a
 * velocity of 0 and a dt (duration) of 0.
 */
struct aa_ct_seg_pb_cx {
    size_t n_q;  ///< Number of configurations

    double *q;   ///< Waypoint position
    double *dq;  ///< Velocity of the linear segment
    double *ddq; ///< Acceleration of the blend segment

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
 * @return AA_CT_SEG_OUT if not in segment, AA_CT_SEG_IN if.
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
            state->ddq[i] = c_cx->ddq[i];
        }

    } else if (n_cx && ((c_cx->t + c_cx->b / 2) <= t
                        && t <= (c_cx->t + c_cx->dt - n_cx->b / 2))) {
        // Linear region
        for (size_t i = 0; i < c_cx->n_q; i++) {
            state->q[i] = c_cx->q[i] + c_cx->dq[i] * dt;
            state->dq[i] = c_cx->dq[i];
            state->ddq[i] = 0;
        }

    } else {
        // Off-the-rails region
        return AA_CT_SEG_OUT;
    }

    return AA_CT_SEG_IN;
}

/**
 * Calculates the maximum (a - b) / (max or min) value for a n_q vector, respecting signs.
 *
 * @param min The minumum limit, should be negative.
 * @return The maximum value.
 */
double
aa_ct_tj_pb_limit(double *a, double *b, double *min, double *max, size_t n_q)
{
    double mv = DBL_MIN;
    for (size_t i = 0; i < n_q; i++) {
        double mag = ((a) ? a[i] : 0) - ((b) ? b[i] : 0);
        double v;
        if (mag < 0) {
            v = mag / ((min) ? min[i] : 1);
        } else {
            v = mag / ((max) ? max[i] : 1);
        }
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
                struct aa_ct_limit *limits)
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
    bzero(cx->dq, sizeof(double) * n_q);
    bzero(cx->ddq, sizeof(double) * n_q);

    cx->t = 0;
    cx->dt = DBL_MAX;
    if (pt->next) {
        cx->dt = aa_ct_tj_pb_limit(pt->next->state.q, cx->q, limits->min->dq, limits->max->dq, n_q);
    }
    cx->b = 0;

    seg->cx = (void *) cx;
    seg->type = AA_CT_PB_SEG;
    return seg;
}

/**
 * Update a segment based on new dt.
 *
 * @param seg    Segment to update
 * @param limits Kinematic limits
 */
void
aa_ct_tj_pb_update(struct aa_ct_seg *seg, struct aa_ct_limit *limits)
{
    struct aa_ct_seg_pb_cx *c_cx, *p_cx, *n_cx;
    aa_ct_tj_pb_nbrs(seg, &p_cx, &c_cx, &n_cx);

    // If this isn't the last segment update forward velocities
    if (n_cx)
        for (size_t i = 0; i < c_cx->n_q; i++)
            c_cx->dq[i] = (n_cx->q[i] - c_cx->q[i]) / c_cx->dt;

    // Calculate acceleration based on new velocities
    c_cx->b = aa_ct_tj_pb_limit(c_cx->dq, (p_cx) ? p_cx->dq : NULL,
                                limits->min->ddq, limits->max->ddq, c_cx->n_q);

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
                      struct aa_ct_limit *limits)
{
    struct aa_ct_seg_list *list = new(reg) struct aa_ct_seg_list(reg);

    // Populate segment list with one segment per point
    struct aa_ct_pt *c_pt = pt_list->list.front();
    for (; c_pt != NULL; c_pt = c_pt->next) {
        if (c_pt->next != NULL && aa_veq(c_pt->state.n_q, c_pt->state.q, c_pt->next->state.q, AA_EPSILON)) {
            fprintf(stdout, "Skipping duplicate point\n");
            continue;
        } 
        aa_ct_seg_list_add(list, aa_ct_tj_pb_new(&list->reg, c_pt, limits));
    }

    bool overlap_flag;
    size_t n = pt_list->list.size();

    // Iterate and update segments until no overlap
    do {
        size_t i = 0;
        double f[n];
        overlap_flag = false;

        struct aa_ct_seg *c_seg = list->list.front();
        // Update all segments before checking for overlap.
        for (; c_seg != NULL; c_seg = c_seg->next) {
            aa_ct_tj_pb_update(c_seg, limits);
        }

        c_seg = list->list.front();
        for (; c_seg != NULL; c_seg = c_seg->next, i++) {
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
                overlap_flag = true;
            } else
                f[i] = 1;

            if (aa_feq(f[i], 0.0, 0.00001)) {
                f[i] = 0.00001;
            }
        }

        i = 0;
        for (c_seg = list->list.front(); c_seg != NULL; c_seg = c_seg->next, i++) {
            // Scale each region's time based on calculated constant
            ((struct aa_ct_seg_pb_cx *) c_seg->cx)->dt /= \
                (i < n - 1) ? fmin(f[i + 1], f[i]) : 1;
        }
    } while (overlap_flag);

    // The final duration of a segment needs to be 0, defaults to DBL_MAX.
    ((struct aa_ct_seg_pb_cx *)list->list.back()->cx)->dt = 0;

    list->n_q = ((struct aa_ct_seg_pb_cx *)list->list.front()->cx)->n_q;

    // The full duration of the trajectory needs to be set.
    struct aa_ct_seg *c_seg = list->list.front();
    list->duration = ((struct aa_ct_seg_pb_cx*)c_seg->cx)->b / 2;
    for (; c_seg != NULL; c_seg = c_seg->next) {
        struct aa_ct_seg_pb_cx *seg_cx = (struct aa_ct_seg_pb_cx *)c_seg->cx;
        list->duration += seg_cx->dt;
    }
    list->duration += ((struct aa_ct_seg_pb_cx *) list->list.back()->cx)->b / 2;
    return list;
}


int
aa_ct_tjX_pb_eval(struct aa_ct_seg *seg, struct aa_ct_state *state, double t)
{
    double *tq = state->q;
    double *tdq = state->dq;
    double *tddq = state->ddq;

    double X[6], dX[6], ddX[6];
    state->q = X;
    state->dq = dX;
    state->ddq = ddX;

    int r = aa_ct_tj_pb_eval(seg, state, t);

    if (r) {
        aa_tf_rotvec2quat(&state->q[3], state->X);
        state->X[4] = state->q[0];
        state->X[5] = state->q[1];
        state->X[6] = state->q[2];
    }

    state->q = tq;
    state->dq = tdq;
    state->ddq = tddq;

    return r;
}

// TODO: This is a hack, replace with a real function later.
AA_API struct aa_ct_seg_list *
aa_ct_tjX_pb_generate(struct aa_mem_region *reg, struct aa_ct_pt_list *pt_list,
                      struct aa_ct_limit *limits)
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

    struct aa_ct_state X_lim_max, X_lim_min;
    struct aa_ct_limit Xlimits;
    Xlimits.max = &X_lim_max;
    Xlimits.min = &X_lim_min;
    Xlimits.max->n_q = 6;
    Xlimits.min->n_q = 6;
    Xlimits.max->dq = limits->max->dX;
    Xlimits.max->ddq = limits->max->ddX;
    Xlimits.min->dq = limits->min->dX;
    Xlimits.min->ddq = limits->min->ddX;

    struct aa_ct_seg_list *Xseg_list =
        aa_ct_tjq_pb_generate(reg, Xpt_list, &Xlimits);

    struct aa_ct_seg *c_seg = Xseg_list->list.front();
    for (; c_seg != NULL; c_seg = c_seg->next) {
        c_seg->eval = aa_ct_tjX_pb_eval;
    }

    return Xseg_list;
}

int aa_ct_seg_list_check( struct aa_ct_seg_list * segs, double dt,
                          int (*function)(void *cx, double t, const struct aa_ct_state *state ),
                          void *cx )
{
    struct aa_mem_region *reg = aa_mem_region_local_get();
    size_t n_q = aa_ct_seg_list_n_q(segs);
    struct aa_ct_state *state = aa_ct_state_alloc(reg, n_q, 0);

    int r = 0;
    for( double t = 0; aa_ct_seg_list_eval(segs, state, t); t += dt )
    {
        if( (r = function(cx, t, state)) ) break;
    }

    aa_mem_region_pop(reg, state);

    return r;
}


int aa_ct_seg_list_check_c0( struct aa_ct_seg_list * segs, double dt,
                             double tol, double eps )
{
    // This function checks two aspects of trajectories:
    //   1. The max dist between two steps
    //   2. The max dist between end of one trajectory and beginning of another
    struct aa_mem_region *reg = aa_mem_region_local_get();
    size_t n_q = aa_ct_seg_list_n_q(segs);
    struct aa_ct_state *state0 = aa_ct_state_alloc(reg, n_q, 0);
    struct aa_ct_state *state1 = aa_ct_state_alloc(reg, n_q, 0);
    struct aa_ct_state *temp;

    aa_ct_seg_list_eval(segs, state1, dt);
    for( double t0 = 0, t1 = dt; aa_ct_seg_list_eval(segs, state1, t1); t0 += dt, t1 += dt )
    {
        aa_ct_seg_list_eval(segs, state0, t0);

        double state_diff = 0.0;
        for (size_t i = 0; i < n_q; i++)
        {
            state_diff += fabs(state1->q[i] - state0->q[i]);
        }
        if (state_diff > tol)
        {
            // The max distance between two steps exceeded to given tolerance.
            fprintf(stderr, "WARNING: trajectory is not continious for seg type: %f, %f\n", state_diff, tol);
            fprintf(stderr, "Vels: dq %f, %f ...\n", state0->dq[0], state0->dq[1]);
            fprintf(stderr, "Times: t0: %f, t1: %f\n", t0, t1);
            return 1;
        }

        // Swap state0 to become state1, next loop will overwrite the old state0.
        temp = state0;
        state0 = state1;
        state1 = temp;
    }

    struct aa_ct_seg *c_seg = segs->list.front();
    for (; c_seg->next != NULL; c_seg = c_seg->next)
    {
        if (c_seg->type == AA_CT_LIN_SEG)
        {
            struct aa_ct_seg_dq *p_cx, *c_cx, *n_cx;
            aa_ct_tj_dq_nbrs(c_seg, &p_cx, &c_cx, &n_cx);
            c_seg->eval(c_seg, state0, c_cx->t1);
            c_seg->next->eval(c_seg->next, state1, n_cx->t0);

        }
        else if (c_seg->type == AA_CT_PB_SEG)
        {
            struct aa_ct_seg_pb_cx *p_cx, *c_cx, *n_cx;
            aa_ct_tj_pb_nbrs(c_seg, &p_cx, &c_cx, &n_cx);
            c_seg->eval(c_seg, state0, c_cx->t + c_cx->dt - n_cx->b / 2);
            c_seg->next->eval(c_seg->next, state1, n_cx->t - n_cx->b / 2);
        }
        else
        {
            // Why does the segment have a weird type?
            return 3;
        }
        double state_diff = 0.0;
        for (size_t i = 0; i < n_q; i++)
        {
            state_diff += fabs(state1->q[i] - state0->q[i]);
        }

        if (state_diff > eps)
        {
            // Max distance between two segment endpoints exceed the given epsilon.
            fprintf(stderr, "WARNING: trajectory segment endpoints don't match for seg type %d: "
                    "%f, %f.\n", c_seg->type, state_diff, eps);
            return 2;
        }
    }

    aa_mem_region_pop(reg, state0);
    aa_mem_region_pop(reg, state1);

    // All good!
    return 0;
}


/*********/
/* SLERP */
/*********/

// TODO: non-unit time
struct aa_ct_slerp_seg {
    double *E0;
    double *E1;
    double dx[6];
    double dt;
};

static int aa_ct_seg_slerp_eval( struct aa_ct_seg *seg,
                                 struct aa_ct_state *state, double t)
{
    struct aa_ct_slerp_seg *cx = (struct aa_ct_slerp_seg*) seg->cx;
    if( t <= 0 ) {
        if(state->X) AA_MEM_CPY(state->X, cx->E0, 7);
        if( state->dX ) AA_MEM_ZERO(state->dX,6);
    } else if (t >= cx->dt ) {
        if(state->X) AA_MEM_CPY(state->X, cx->E1, 7);
        if( state->dX ) AA_MEM_ZERO(state->dX,6);
    } else {
        if(state->X) {
            aa_tf_qslerp( t,
                          cx->E0 + AA_TF_QUTR_Q,
                          cx->E1 + AA_TF_QUTR_Q,
                          state->X + AA_TF_QUTR_Q );
            aa_la_linterp( 3,
                           0, cx->E0   + AA_TF_QUTR_T,
                           1, cx->E1   + AA_TF_QUTR_T,
                           t, state->X + AA_TF_QUTR_T );
        }

        if( state->dX ) AA_MEM_CPY( state->dX, cx->dx, 6 );
    }
    if( state->ddX ) AA_MEM_ZERO(state->ddX,6);
    if( state->eff ) AA_MEM_ZERO(state->eff,6);
    return 0;
}

struct aa_ct_seg_list *aa_ct_tjx_slerp_generate(struct aa_mem_region *reg,
                                                struct aa_ct_pt_list *list )
{
    if(2 != list->list.size() ) {
        fprintf(stderr, "Cannot SLERP between %lu point(s)\n.", list->list.size());
        return NULL;
    }

    struct aa_ct_slerp_seg *s = AA_MEM_REGION_NEW(reg, struct aa_ct_slerp_seg);
    {
        // TODO: non-unit time
        s->dt = 1;

        auto itr = list->list.begin();
        s->E0 = (*itr)->state.X;
        itr++;
        s->E1 = (*itr)->state.X;

        // omega
        double qt[4], ql[4];
        aa_tf_qmulc( s->E1+AA_TF_QUTR_Q, s->E0+AA_TF_QUTR_Q, qt);
        aa_tf_qminimize(qt);
        aa_tf_qln(qt, ql);
        for( size_t i = 0; i < 3; i ++ ) {
            s->dx[AA_TF_DX_W+i] = 2 * ql[AA_TF_QUAT_V+i] / s->dt;
        }

        // dv
        for( size_t i = 0; i < 3; i ++ ) {
            s->dx[AA_TF_DX_V+i] = (s->E1[AA_TF_QUTR_T+i] -  s->E0[AA_TF_QUTR_T+i]) / s->dt;
        }

    }

    struct aa_ct_seg_list *segs  = new(reg) aa_ct_seg_list(reg);
    aa_ct_seg_list_add_cx(segs, aa_ct_seg_slerp_eval,  s);
    segs->duration = 1;

    return segs;
}
