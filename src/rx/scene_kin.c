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

AA_API size_t
aa_rx_sg_chain_frame_count( const struct aa_rx_sg *sg,
                            aa_rx_frame_id root, aa_rx_frame_id tip )
{
    size_t a = 0;
    while( tip != root && tip != AA_RX_FRAME_ROOT ) {
        enum aa_rx_frame_type ft = aa_rx_sg_frame_type( sg, tip );
        switch(ft) {
        case AA_RX_FRAME_FIXED:
            break;
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
            break;
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
    (void)chain_frames;
    (void)sg;
    // TODO: handle multi-variate frames, when such frames exist
    return n_frames;
}


AA_API void
aa_rx_sg_chain_configs( const struct aa_rx_sg *sg,
                        size_t n_frames, const aa_rx_frame_id *chain_frames,
                        size_t n_configs, aa_rx_config_id *chain_configs )
{
    assert( n_configs == n_frames );
    for( size_t i = 0, j = 0; i<n_frames && j<n_configs; i++, j++ ) {
        chain_configs[j] = aa_rx_sg_frame_config(sg, chain_frames[i]);
    }
}

AA_API void
aa_rx_sg_chain_jacobian( const struct aa_rx_sg *sg,
                         size_t n_tf, const double *TF_abs, size_t ld_TF,
                         size_t n_frames, aa_rx_frame_id *chain_frames,
                         size_t n_configs, aa_rx_frame_id *chain_configs,
                         double *J, size_t ld_J )
{
    (void)n_configs; (void)chain_configs; /* Use these for multivariate frame support */

    aa_rx_frame_id frame_ee = chain_frames[n_frames-1];
    const double *pe = &TF_abs[(size_t)frame_ee * ld_TF] + AA_TF_QUTR_T;

    for( size_t i = 0; i < n_frames;
         i++, J+= ld_J )
    {
        double *Jr = J + AA_TF_DX_W; // rotational part
        double *Jt = J + AA_TF_DX_V; // translational part

        aa_rx_frame_id frame = chain_frames[i];
        assert( frame >= 0 );
        assert( (size_t)frame < n_tf );
        enum aa_rx_frame_type ft = aa_rx_sg_frame_type(sg, frame);

        switch(ft) {
        case AA_RX_FRAME_FIXED:
            fprintf(stderr,"ERROR: fixed frame found in kinematic chain\n");
            /* I guess we zero this column? */
            AA_MEM_ZERO(J, 6);
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

            break;
        } /* end joint */
        } /* end switch */
    } /* end for */
}
