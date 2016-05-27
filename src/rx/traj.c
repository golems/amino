#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/traj.h>

void
aa_rx_traj_init(aa_rx_traj_t *traj, struct aa_rx_sg *scenegraph,
                aa_mem_region_t *reg,
                int (*generate)(aa_rx_traj_t *traj, void *cx))
{
    *traj = (aa_rx_traj_t) {0};
    traj->scenegraph = scenegraph;
    traj->n_q = aa_rx_sg_config_count(scenegraph);

    traj->reg = reg;
    traj->generate = generate;

    traj->points = aa_mem_rlist_alloc(reg);
    traj->segments = aa_mem_rlist_alloc(reg);
}

int
aa_rx_traj_generate(aa_rx_traj_t *traj, void *cx)
{
    return traj->generate(traj, cx);
}

int
aa_rx_traj_value_it(aa_rx_traj_t *traj, void *v, double t) {
    do {
        aa_rx_trajseg_t *seg = (aa_rx_trajseg_t *) traj->last_seg->data;

        if (seg->value(traj, seg, v, t))
            return 1;

        traj->last_seg = traj->last_seg->next;
    } while (traj->last_seg != NULL);

    return 0;
}

int
aa_rx_traj_value(aa_rx_traj_t *traj, void *v, double t)
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

void
aa_rx_trajpt_add(aa_rx_traj_t *traj, aa_rx_trajpt_t *pt)
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
aa_rx_trajseg_add(aa_rx_traj_t *traj, aa_rx_trajseg_t *seg)
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
aa_rx_pb_trajseg_value(aa_rx_traj_t *traj, aa_rx_trajseg_t *seg,
                       void *v, double t)
{
    aa_rx_pb_trajval_t *value = (aa_rx_pb_trajval_t *) v;

    aa_rx_pb_trajseg_t *c = (aa_rx_pb_trajseg_t *) seg;
    aa_rx_pb_trajseg_t *p = (aa_rx_pb_trajseg_t *) seg->prev;
    aa_rx_pb_trajseg_t *n = (aa_rx_pb_trajseg_t *) seg->next;

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

aa_rx_pb_trajseg_t *
aa_rx_pb_trajseg_create(aa_rx_traj_t *traj)
{
    aa_rx_pb_trajseg_t *seg = AA_MEM_REGION_NEW(traj->reg, aa_rx_pb_trajseg_t);
    *seg = (aa_rx_pb_trajseg_t) {
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
aa_rx_pb_trajseg_generate(aa_rx_pb_trajseg_t *seg, double *amax, size_t n_q)
{
    aa_rx_pb_trajpt_t *c = (aa_rx_pb_trajpt_t *) seg->pt;
    aa_rx_pb_trajpt_t *n = (aa_rx_pb_trajpt_t *) c->next;
    
    if (n) 
        for (size_t i = 0; i < n_q; i++)
            seg->dq[i] = (n->q[i] - c->q[i]) / seg->dt;

    aa_rx_pb_trajseg_t *p = (aa_rx_pb_trajseg_t *) seg->prev;
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

int
aa_rx_pb_traj_generate(aa_rx_traj_t *traj, void *cx)
{
    double *amax = (double *) cx;
    double *vmin = AA_MEM_REGION_NEW_N(traj->reg, double, traj->n_q);
    double *vmax = AA_MEM_REGION_NEW_N(traj->reg, double, traj->n_q);
    for (size_t i = 0; i < traj->n_q; i++)
        aa_rx_sg_get_limit_vel(traj->scenegraph, (aa_rx_config_id) i,
                               &vmin[i], &vmax[i]);

    aa_rx_pb_trajpt_t *c_pt;
    aa_rx_pb_trajseg_t *c_seg;
    for (c_pt = (aa_rx_pb_trajpt_t *) traj->points->head->data;
         c_pt != NULL; c_pt = (aa_rx_pb_trajpt_t *) c_pt->next) {
        c_seg = aa_rx_pb_trajseg_create(traj);
        c_seg->pt = c_pt;
        aa_rx_trajseg_add(traj, (aa_rx_trajseg_t *) c_seg);

        aa_rx_pb_trajpt_t *next = (aa_rx_pb_trajpt_t *) c_pt->next;
        c_seg->dt = (next) ?
            aa_rx_pb_trajseg_limit(next->q, c_pt->q, vmax, traj->n_q) : DBL_MAX;

        aa_rx_pb_trajseg_generate(c_seg, amax, traj->n_q);
    }

    for (;;) {
        bool flag = false;
        double f[traj->n_q], *fp = f;
        aa_rx_pb_trajseg_t *p_seg = NULL, *n_seg = NULL;
        for (c_seg = (aa_rx_pb_trajseg_t *) traj->segments->head->data;
             c_seg != NULL; c_seg = n_seg, p_seg = c_seg, fp++) {
            n_seg = (aa_rx_pb_trajseg_t *) c_seg->next;

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
            c_seg = (aa_rx_pb_trajseg_t *) traj->segments->head->data;
            traj->t_o = -c_seg->b / 2;

            break;
        }

        size_t i = 0;
        for (c_seg = (aa_rx_pb_trajseg_t *) traj->segments->head->data;
             c_seg != NULL; i++) {
            c_seg->dt /= (i < traj->n_q - 1) ? fmin(f[i + 1], f[i]) : f[i];
            aa_rx_pb_trajseg_generate(c_seg, amax, traj->n_q);
        }
    }

    return 0;
}
