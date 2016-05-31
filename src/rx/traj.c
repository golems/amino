#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>

#include <amino/rx/traj.h>
#include <amino/rx/traj_internal.h>

AA_API void
aa_rx_traj_init(struct aa_rx_traj *traj, size_t n_q,
                aa_mem_region_t *reg,
                int (*generate)(struct aa_rx_traj *traj, void *cx))
{
    *traj = (struct aa_rx_traj) {0};
    traj->n_q = n_q;

    traj->reg = reg;
    traj->generate = generate;

    traj->points = aa_mem_rlist_alloc(reg);
    traj->segments = aa_mem_rlist_alloc(reg);
}

AA_API int
aa_rx_traj_generate(struct aa_rx_traj *traj, void *cx)
{
    return traj->generate(traj, cx);
}

int
aa_rx_traj_value_it(struct aa_rx_traj *traj, void *v, double t) {
    // Iterate through segment list until no more segments remain
    do {
        struct aa_rx_trajseg *seg =
            (struct aa_rx_trajseg *) traj->last_seg->data;

        // Evaluate segment at time t. Returns 1 if t is contained.
        if (seg->value(traj, seg, v, t))
            return 1;

        traj->last_seg = traj->last_seg->next;
    } while (traj->last_seg != NULL);

    return 0;
}

AA_API int
aa_rx_traj_value(struct aa_rx_traj *traj, void *v, double t)
{
    if (!traj->last_seg)
        traj->last_seg = traj->segments->head;

    t += traj->t_o;
    int r = aa_rx_traj_value_it(traj, v, t);
    if (r == 0) {
        traj->last_seg = traj->segments->head;
        r = aa_rx_traj_value_it(traj, v, t);
    }

    return r;
}

AA_API void
aa_rx_trajpt_add(struct aa_rx_traj *traj, struct aa_rx_trajpt *pt)
{
    if (traj->tail_pt) {
        traj->tail_pt->next = pt;
        pt->i = traj->tail_pt->i + 1;
    } else
        pt->i = 0;

    pt->prev = traj->tail_pt;
    traj->tail_pt = pt;

    aa_mem_rlist_enqueue_ptr(traj->points, pt);
}

void
aa_rx_trajseg_add(struct aa_rx_traj *traj, struct aa_rx_trajseg *seg)
{
    if (traj->tail_seg) {
        traj->tail_seg->next = seg;
        seg->i = traj->tail_seg->i + 1;
    } else
        seg->i = 0;

    seg->prev = traj->tail_seg;
    traj->tail_seg = seg;

    aa_mem_rlist_enqueue_ptr(traj->segments, seg);
}

double
aa_rx_pb_trajseg_limit(double *a, double *b, double *m, size_t n_q)
{
    double mv = DBL_MIN;
    for (size_t i = 0; i < n_q; i++) {
        double v = fabs(((a) ? a[i] : 0) - ((b) ? b[i] : 0)) / ((m) ? m[i] : 1);
        mv = (mv < v) ? v : mv;
    }

    return mv;
}

int
aa_rx_pb_trajseg_value(struct aa_rx_traj *traj, struct aa_rx_trajseg *seg,
                       void *v, double t)
{
    struct aa_rx_pb_trajval *value = (struct aa_rx_pb_trajval *) v;

    struct aa_rx_pb_trajseg *c = (struct aa_rx_pb_trajseg *) seg;
    struct aa_rx_pb_trajseg *p = (struct aa_rx_pb_trajseg *) seg->prev;
    struct aa_rx_pb_trajseg *n = (struct aa_rx_pb_trajseg *) seg->next;

    double dt = t - c->t_s;
    if ((c->t_s - c->b / 2) <= t && t <= (c->t_s + c->b / 2)) {
        for (size_t i = 0; i < traj->n_q; i++) {
            double p_dq = (p) ? p->dq[i] : 0;
            value->q[i] = c->pt->q[i] + p_dq * dt + \
                c->ddq[i] * pow((dt + c->b / 2), 2) / 2;
            value->dq[i] = p_dq + c->ddq[i] * (dt + c->b / 2);
        }
    } else if (n && (c->t_s + c->b / 2) <= t \
               && t <= (c->t_s + c->dt - n->b / 2)) {
        for (size_t i = 0; i < traj->n_q; i++) {
            value->q[i] = c->pt->q[i] + c->dq[i] * dt;
            value->dq[i] = c->dq[i];
        }
    } else {
        return 0;
    }

    return 1;
}

struct aa_rx_pb_trajseg *
aa_rx_pb_trajseg_create(struct aa_rx_traj *traj)
{
    struct aa_rx_pb_trajseg *seg = AA_MEM_REGION_NEW(traj->reg,
                                                     struct aa_rx_pb_trajseg);
    *seg = (struct aa_rx_pb_trajseg) {
        .value = aa_rx_pb_trajseg_value,
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
aa_rx_pb_trajseg_generate(struct aa_rx_pb_trajseg *seg,
                          double *amax, size_t n_q)
{
    struct aa_rx_pb_trajpt *c = (struct aa_rx_pb_trajpt *) seg->pt;
    struct aa_rx_pb_trajpt *n = (struct aa_rx_pb_trajpt *) c->next;
    
    if (n) 
        for (size_t i = 0; i < n_q; i++)
            seg->dq[i] = (n->q[i] - c->q[i]) / seg->dt;

    struct aa_rx_pb_trajseg *p = (struct aa_rx_pb_trajseg *) seg->prev;
    seg->b = aa_rx_pb_trajseg_limit(seg->dq, (p) ? p->dq : NULL, amax, n_q);

    for (size_t i = 0; i < n_q; i++)
        seg->ddq[i] = (seg->dq[i] - ((p) ? p->dq[i] : 0)) / seg->b;

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
aa_rx_pb_traj_generate(struct aa_rx_traj *traj, void *cx)
{
    struct aa_rx_pb_limits *limits = (struct aa_rx_pb_limits *) cx;

    struct aa_rx_pb_trajpt *c_pt;
    struct aa_rx_pb_trajseg *c_seg;
    for (c_pt = (struct aa_rx_pb_trajpt *) traj->points->head->data;
         c_pt != NULL; c_pt = (struct aa_rx_pb_trajpt *) c_pt->next) {
        c_seg = aa_rx_pb_trajseg_create(traj);
        c_seg->pt = c_pt;
        aa_rx_trajseg_add(traj, (struct aa_rx_trajseg *) c_seg);

        struct aa_rx_pb_trajpt *next = (struct aa_rx_pb_trajpt *) c_pt->next;
        c_seg->dt = (next)
            ? aa_rx_pb_trajseg_limit(next->q, c_pt->q, limits->dqmax, traj->n_q) 
            : DBL_MAX;

        aa_rx_pb_trajseg_generate(c_seg, limits->ddqmax, traj->n_q);
    }

    for (;;) {
        bool flag = false;
        double f[traj->n_q], *fp = f;
        struct aa_rx_pb_trajseg *p_seg = NULL, *n_seg = NULL;
        for (c_seg = (struct aa_rx_pb_trajseg *) traj->segments->head->data;
             c_seg != NULL; c_seg = n_seg, p_seg = c_seg, fp++) {
            n_seg = (struct aa_rx_pb_trajseg *) c_seg->next;

            bool p_overlap =
                (p_seg) ? (p_seg->dt < c_seg->b \
                           && (2 * p_seg->dt - p_seg->b) < c_seg->b) : false;
            bool n_overlap =
                (n_seg) ? (c_seg->dt < c_seg->b \
                           && (2 * c_seg->dt - n_seg->b) < c_seg->b) : false;

            if (p_overlap || n_overlap) {
                *fp = sqrt(fmin((p_seg) ? p_seg->dt : DBL_MAX, c_seg->dt) / \
                           c_seg->b);
                flag = true;
            } else
                *fp = 1;
        }

        if (!flag) {
            c_seg = (struct aa_rx_pb_trajseg *) traj->segments->head->data;
            traj->t_o = -c_seg->b / 2;

            break;
        }

        size_t i = 0;
        for (c_seg = (struct aa_rx_pb_trajseg *) traj->segments->head->data;
             c_seg != NULL; i++) {
            c_seg->dt /= (i < traj->n_q - 1) ? fmin(f[i + 1], f[i]) : f[i];
            aa_rx_pb_trajseg_generate(c_seg, limits->ddqmax, traj->n_q);
        }
    }

    return 0;
}
