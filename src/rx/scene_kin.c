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
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "amino.h"
#include "amino/rx/rxtype.h"
#include "amino/rx/scenegraph.h"
#include "amino/rx/scene_kin.h"
#include "amino/rx/scene_sub.h"
#include "amino/rx/scene_kin_internal.h"
#include "amino/mat_internal.h"

AA_API void
aa_rx_sg_sub_destroy( struct aa_rx_sg_sub *ssg )
{
    if( ssg->frames ) free( ssg->frames );
    if( ssg->configs ) free( ssg->configs );

    free(ssg);
}


AA_API const struct aa_rx_sg *
aa_rx_sg_sub_sg( const struct aa_rx_sg_sub *sg_sub )
{
    return sg_sub->scenegraph;
}

AA_API size_t
aa_rx_sg_sub_config_count( const struct aa_rx_sg_sub *sg_sub )
{
    return sg_sub->config_count;
}

AA_API size_t
aa_rx_sg_sub_all_config_count( const struct aa_rx_sg_sub *sg_sub )
{
    return aa_rx_sg_config_count( sg_sub->scenegraph );
}

AA_API size_t
aa_rx_sg_sub_frame_count( const struct aa_rx_sg_sub *sg_sub )
{
    return sg_sub->frame_count;
}

AA_API size_t
aa_rx_sg_sub_all_frame_count( const struct aa_rx_sg_sub *sg_sub )
{
    return aa_rx_sg_frame_count( aa_rx_sg_sub_sg(sg_sub) );
}

AA_API aa_rx_config_id
aa_rx_sg_sub_config( const struct aa_rx_sg_sub *sg_sub, size_t i )
{
    return sg_sub->configs[i];
}

AA_API aa_rx_frame_id
aa_rx_sg_sub_frame( const struct aa_rx_sg_sub *sg_sub, size_t i )
{
    return sg_sub->frames[i];
}

AA_API aa_rx_frame_id
aa_rx_sg_sub_frame_ee( const struct aa_rx_sg_sub *sg_sub )
{
    // Only handling chains for now
    return aa_rx_sg_sub_frame(sg_sub, aa_rx_sg_sub_frame_count(sg_sub) - 1 );
}

AA_API aa_rx_config_id*
aa_rx_sg_sub_configs( const struct aa_rx_sg_sub *sg_sub )
{
    return sg_sub->configs;
}

AA_API aa_rx_frame_id*
aa_rx_sg_sub_frames( const struct aa_rx_sg_sub *sg_sub )
{
    return sg_sub->frames;
}


AA_API size_t
aa_rx_sg_chain_frame_count( const struct aa_rx_sg *sg,
                            aa_rx_frame_id root, aa_rx_frame_id tip )
{
    size_t a = 0;
    while( tip != root && tip != AA_RX_FRAME_ROOT ) {
        enum aa_rx_frame_type ft = aa_rx_sg_frame_type( sg, tip );
        switch(ft) {
        case AA_RX_FRAME_FIXED:
            /* break; */
        case AA_RX_FRAME_REVOLUTE:
        case AA_RX_FRAME_PRISMATIC:
            a++;
            break;
        }
        tip = aa_rx_sg_frame_parent(sg, tip);
    }

    return a;
}

AA_API void
aa_rx_sg_chain_frames( const struct aa_rx_sg *sg,
                       aa_rx_frame_id root, aa_rx_frame_id tip,
                       size_t n_frames, aa_rx_frame_id *chain_frames )
{
    aa_rx_frame_id *ptr = chain_frames + n_frames - 1;

    while( tip != root && tip != AA_RX_FRAME_ROOT && ptr >= chain_frames ) {
        enum aa_rx_frame_type ft = aa_rx_sg_frame_type( sg, tip );
        switch(ft) {
        case AA_RX_FRAME_FIXED:
        /*     break; */
        case AA_RX_FRAME_REVOLUTE:
        case AA_RX_FRAME_PRISMATIC:
            *ptr = tip;
            ptr--;
            break;
        }

        tip = aa_rx_sg_frame_parent(sg, tip);
    }
}

AA_API size_t
aa_rx_sg_chain_config_count( const struct aa_rx_sg *sg,
                             size_t n_frames, const aa_rx_frame_id  *chain_frames )
{
    size_t a = 0;
    while( n_frames ) {
        enum aa_rx_frame_type ft = aa_rx_sg_frame_type(sg, chain_frames[--n_frames]);
        switch(ft) {
        case AA_RX_FRAME_FIXED:
            break;
        case AA_RX_FRAME_REVOLUTE:
        case AA_RX_FRAME_PRISMATIC:
            a++;
            break;
        }
    }

    return a;
}


AA_API void
aa_rx_sg_chain_configs( const struct aa_rx_sg *sg,
                        size_t n_frames, const aa_rx_frame_id *chain_frames,
                        size_t n_configs, aa_rx_config_id *chain_configs )
{
    /* TODO: handle duplicate configs */
    for( size_t i_config = 0, i_frame = 0;
         i_frame < n_frames && i_config < n_configs;
         i_frame++ )
    {
        enum aa_rx_frame_type ft = aa_rx_sg_frame_type(sg, chain_frames[i_frame]);

        switch(ft) {
        case AA_RX_FRAME_FIXED:
            break;
        case AA_RX_FRAME_REVOLUTE:
        case AA_RX_FRAME_PRISMATIC:
            chain_configs[i_config++] = aa_rx_sg_frame_config(sg, chain_frames[i_frame]);
            break;
        }
    }
}


AA_API void
aa_rx_sg_sub_config_get(
    const struct aa_rx_sg_sub *ssg,
    size_t n_all, const double *config_all,
    size_t n_subset, double *config_subset )
{
    aa_rx_sg_config_get( ssg->scenegraph,
                         n_all, n_subset,
                         aa_rx_sg_sub_configs(ssg),
                         config_all, config_subset );
}

AA_API void
aa_rx_sg_sub_config_set(
    const struct aa_rx_sg_sub *ssg,
    size_t n_sub, const double *config_subset,
    size_t n_all, double *config_all
    )
{
    aa_rx_sg_config_set( ssg->scenegraph,
                         n_all, n_sub,
                         aa_rx_sg_sub_configs(ssg),
                         config_subset, config_all );

}

AA_API void
aa_rx_sg_sub_config_scatter( const struct aa_rx_sg_sub *ssg,
                             const struct aa_dvec *config_subset,
                             struct aa_dvec *config_all )
{
    if( config_subset->len != ssg->config_count ||
        config_all->len != aa_rx_sg_config_count(ssg->scenegraph)
       ) { aa_lb_err("Mismatched config vector sizes\n"); }

    int inc = (int)config_all->inc;
    for( size_t i = 0, j=0; i < ssg->config_count; i ++, j+=config_subset->inc ) {
        int k =  (int)(ssg->configs[i]) * inc;
        config_all->data[k] = config_subset->data[j];
    }
}


AA_API void
aa_rx_sg_sub_config_gather( const struct aa_rx_sg_sub *ssg,
                            const struct aa_dvec *config_all,
                            struct aa_dvec *config_subset )
{
    if( config_subset->len != ssg->config_count ||
        config_all->len != aa_rx_sg_config_count(ssg->scenegraph)
       ) { aa_lb_err("Mismatched config vector sizes\n"); }

    int inc = (int)config_all->inc;
    for( size_t i = 0, j=0; i < ssg->config_count; i ++, j+=config_subset->inc ) {
        int k =  (int)(ssg->configs[i]) * inc;
        config_subset->data[j] = config_all->data[k];
    }
}


AA_API void
aa_rx_sg_sub_expand_path( const struct aa_rx_sg_sub *ssg, size_t n_pts,
                          const double *q_start,
                          const double *path_sub,
                          double *path_all )
{
    size_t n_all = aa_rx_sg_sub_all_config_count(ssg);
    size_t n_sub = aa_rx_sg_sub_config_count(ssg);


    // Fill start

    if( q_start ) {
        for( size_t i = 0; i < n_pts; i ++ ) {
            AA_MEM_CPY( path_all + i*n_all, q_start, n_all );
        }
    }

    for( size_t i = 0; i < n_pts; i ++ ) {
        aa_rx_sg_sub_config_set( ssg,
                                 n_sub, path_sub + i*n_sub,
                                 n_all, path_all + i*n_all );
    }
}


AA_API struct aa_rx_sg_sub *
aa_rx_sg_chain_create( const struct aa_rx_sg *sg,
                       aa_rx_frame_id root, aa_rx_frame_id tip )
{
    struct aa_rx_sg_sub *ssg = AA_NEW( struct aa_rx_sg_sub );
    ssg->scenegraph = sg;

    ssg->frame_count = aa_rx_sg_chain_frame_count( sg, root, tip);
    ssg->frames =  AA_NEW_AR(aa_rx_frame_id, ssg->frame_count );
    aa_rx_sg_chain_frames( sg, root, tip,
                           ssg->frame_count, ssg->frames );

    ssg->config_count = aa_rx_sg_chain_config_count( sg, ssg->frame_count, ssg->frames );
    ssg->configs = AA_NEW_AR(aa_rx_config_id, ssg->config_count );
    aa_rx_sg_chain_configs( sg, ssg->frame_count, ssg->frames,
                            ssg->config_count, ssg->configs );

    return ssg;
}

AA_API void
aa_rx_sg_sub_jac_twist_fill( const struct aa_rx_sg_sub *ssg,
                             const struct aa_dmat *TF,
                             struct aa_dmat *Jr, struct aa_dmat *Jp )
{
    size_t n_frames = aa_rx_sg_sub_frame_count(ssg);
    size_t n_configs = aa_rx_sg_sub_config_count(ssg);

    aa_lb_check_size(3, Jr->rows);
    aa_lb_check_size(3, Jp->rows);
    aa_lb_check_size(n_configs, Jr->cols);
    aa_lb_check_size(n_configs, Jp->cols);

    aa_rx_frame_id *frames = aa_rx_sg_sub_frames(ssg);
    const struct aa_rx_sg *sg = aa_rx_sg_sub_sg(ssg);

    double *jr = Jr->data, *jp = Jp->data;
    size_t i_frame = 0, i_config = 0;
    for( ;
         i_frame < n_frames && i_config < n_configs;
         i_frame++ )
    {
        aa_rx_frame_id frame = frames[i_frame];
        assert( frame >= 0 );
        assert( (size_t)frame < TF->cols );
        enum aa_rx_frame_type ft = aa_rx_sg_frame_type(sg, frame);

        switch(ft) {
        case AA_RX_FRAME_FIXED:
            break;

        case AA_RX_FRAME_REVOLUTE:
        case AA_RX_FRAME_PRISMATIC: {
            assert( i_config <  n_configs );

            const double *a = aa_rx_sg_frame_axis(sg, frame);
            const double *E = &AA_DMAT_REF(TF,0,(size_t)frame);
            const double *q = E + AA_TF_QUTR_Q;
            const double *t = E + AA_TF_QUTR_T;

            switch(ft)  {
            case AA_RX_FRAME_REVOLUTE: {
                aa_tf_qrot(q,a,jr);
                aa_tf_cross(t, jr, jp);
                break;
            }
            case AA_RX_FRAME_PRISMATIC:
                AA_MEM_ZERO(jr, 3);
                aa_tf_qrot(q,a,jp);
                break;
            default: assert(0);
            }
            jr += Jr->ld;
            jp += Jp->ld;
            i_config++;
            break;
        } /* end joint */
        } /* end switch */
    } /* end for */

    assert( i_frame <= n_frames );
    assert( i_config <= n_configs );
    assert( i_config == n_configs );

}

AA_API struct aa_dmat *
aa_rx_sg_sub_jac_twist_get( const struct aa_rx_sg_sub *ssg, struct aa_mem_region *reg,
                            const struct aa_dmat *TF )
{
    size_t rows, cols;
    aa_rx_sg_sub_jacobian_size(ssg,&rows,&cols);
    struct aa_dmat *J = aa_dmat_alloc(reg,rows,cols);

    struct aa_dmat Jr, Jp;

    aa_dmat_view_block(&Jp, J, AA_TF_DX_V, 0, 3, cols);
    aa_dmat_view_block(&Jr, J, AA_TF_DX_W, 0, 3, cols);

    aa_rx_sg_sub_jac_twist_fill( ssg, TF, &Jr, &Jp );

    return J;
}

AA_API void
aa_rx_sg_chain_jacobian( const struct aa_rx_sg *sg,
                         size_t n_tf, const double *TF_abs, size_t ld_TF,
                         size_t n_frames, aa_rx_frame_id *chain_frames,
                         size_t n_configs, aa_rx_frame_id *chain_configs,
                         double *J, size_t ld_J )
{
    (void)chain_configs; /* Use these for multivariate frame support */

    aa_rx_frame_id frame_ee = chain_frames[n_frames-1];
    const double *pe = &TF_abs[(size_t)frame_ee * ld_TF] + AA_TF_QUTR_T;

    for( size_t i_frame=0, i_config=0;
         i_frame < n_frames && i_config < n_configs;
         i_frame++ )
    {
        double *Jr = J + AA_TF_DX_W; // rotational part
        double *Jt = J + AA_TF_DX_V; // translational part

        aa_rx_frame_id frame = chain_frames[i_frame];
        assert( frame >= 0 );
        assert( (size_t)frame < n_tf );
        enum aa_rx_frame_type ft = aa_rx_sg_frame_type(sg, frame);

        switch(ft) {
        case AA_RX_FRAME_FIXED:
            break;

        case AA_RX_FRAME_REVOLUTE:
        case AA_RX_FRAME_PRISMATIC: {

            const double *a = aa_rx_sg_frame_axis(sg, frame);
            const double *E = TF_abs + (size_t)frame*ld_TF;
            const double *q = E+AA_TF_QUTR_Q;
            const double *t = E+AA_TF_QUTR_T;

            switch(ft)  {
            case AA_RX_FRAME_REVOLUTE: {
                aa_tf_qrot(q,a,Jr);
                double tmp[3];
                for( size_t j = 0; j < 3; j++ ) tmp[j] = pe[j] - t[j];
                aa_tf_cross(Jr, tmp, Jt);
                break;
            }
            case AA_RX_FRAME_PRISMATIC:
                AA_MEM_ZERO(Jr, 3);
                aa_tf_qrot(q,a,Jt);
                break;
            default: assert(0);
            }

            i_config++;
            J += ld_J;
            break;
        } /* end joint */
        } /* end switch */
    } /* end for */
}

AA_API void
aa_rx_sg_sub_jacobian_size( const struct aa_rx_sg_sub *ssg,
                            size_t *rows, size_t *cols )
{
    *rows = 6;
    *cols = ssg->config_count;

}

AA_API void
aa_rx_sg_sub_jacobian( const struct aa_rx_sg_sub *ssg,
                       size_t n_tf, const double *TF_abs, size_t ld_TF,
                       double *J, size_t ld_J )
{
    aa_rx_sg_chain_jacobian( ssg->scenegraph,
                             n_tf, TF_abs, ld_TF,
                             ssg->frame_count, ssg->frames,
                             ssg->config_count, ssg->configs,
                             J, ld_J );
}


AA_API struct aa_dmat *
aa_rx_sg_sub_get_jacobian( const struct aa_rx_sg_sub *ssg,
                           struct aa_mem_region *reg,
                           const struct aa_dmat *TF_abs )
{
    size_t rows, cols;
    aa_rx_sg_sub_jacobian_size(ssg,&rows,&cols);
    struct aa_dmat *J = aa_dmat_alloc(reg,rows,cols);
    aa_rx_sg_sub_jacobian(ssg,
                          TF_abs->cols, TF_abs->data, TF_abs->ld,
                          J->data, J->ld );
    return J;

}

AA_API void
aa_rx_sg_sub_center_configs( const struct aa_rx_sg_sub *ssg,
                             size_t n, double *q )
{
    size_t n_qs = aa_rx_sg_sub_config_count(ssg);
    size_t n_min = AA_MIN(n,n_qs);
    for( size_t i = 0; i < n_min; i ++ ) {
        q[i] = aa_rx_sg_config_center( ssg->scenegraph,
                                       aa_rx_sg_sub_config(ssg, i) );

    }
}

AA_API void
aa_rx_sg_sub_rand_config( const struct aa_rx_sg_sub *ssg, struct aa_dvec *dst )
{
    size_t n = aa_rx_sg_sub_config_count(ssg);
    const struct aa_rx_sg *sg = aa_rx_sg_sub_sg(ssg);
    aa_lb_check_size(dst->len, n);
    for( size_t i = 0; i < n; i ++ ) {
        double min = -M_PI, max = M_PI;
        {
            aa_rx_config_id j = aa_rx_sg_sub_config(ssg, i);
            aa_rx_sg_get_limit_pos(sg, j, & min, &max);
        }
        AA_DVEC_REF(dst,i) = aa_frand_minmax(min,max);
    }
}

AA_API double *
aa_rx_sg_sub_alloc_jacobian( const struct aa_rx_sg_sub *ssg, struct aa_mem_region *region )
{
    return AA_MEM_REGION_NEW_N( region, double, 6*aa_rx_sg_sub_config_count(ssg) );
}

AA_API double *
aa_rx_sg_sub_alloc_config( const struct aa_rx_sg_sub *ssg, struct aa_mem_region *region )
{
    return AA_MEM_REGION_NEW_N( region, double, aa_rx_sg_sub_config_count(ssg) );
}
