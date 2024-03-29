/* -*- mode: C++; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University
 * Copyright (c) 2019, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ndantam@mines.edu>
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
#include "amino/rx/scene_fk.h"
#include "amino/rx/scene_ik_internal.h"
#include "amino/rx/scenegraph_internal.h"
#include "amino/rx/scene_sub.h"
#include <set>


// AA_API void
// aa_rx_sg_sub_tf_update( const struct aa_rx_sg_sub *ssg,
//                         const struct aa_dvec *q_sub,
//                         struct aa_dmat *TF_abs )
// {
//     size_t n_all = aa_rx_sg_config_count(ssg->scenegraph);
//     double q_all[n_all];
//     struct aa_dvec vq_all = AA_DVEC_INIT(n_all, q_all, 1);
//     aa_rx_sg_sub_config_scatter(ssg, q_sub, &vq_all);

//     amino::SceneGraph *sg = ssg->scenegraph->sg;

//     for( size_t i_sub = 0; i_sub < ssg->frame_count; i_sub++ ) {
//         double E_rel[7];
//         aa_rx_frame_id i_frame = ssg->frames[i_sub];
//         amino::SceneFrame *f = sg->frames[i_frame];
//         f->tf_rel(q_all, E_rel);

//         double *E_abs = &AA_DMAT_REF(TF_abs, 0, i_frame);
//         if( f->in_global() ) {
//             // TODO: can we somehow get rid of this branch?
//             //       maybe a separate type for global frames
//             AA_MEM_CPY(E_abs, E_rel, 7);
//         } else {
//             double *E_abs_parent = &AA_DMAT_REF(TF_abs, 0, f->parent_id);
//             aa_tf_qutr_mul(E_abs_parent, E_rel, E_abs);
//         }

//     }
// }


AA_API void
aa_rx_fk_sub( struct aa_rx_fk *fk,
              const struct aa_rx_sg_sub *ssg,
              const struct aa_dvec *q_sub )
{
    aa_rx_sg_ensure_clean_frames(ssg->scenegraph);
    size_t n_all = aa_rx_sg_config_count(ssg->scenegraph);
    double q_all[n_all];
    struct aa_dvec vq_all = AA_DVEC_INIT(n_all, q_all, 1);
    aa_rx_sg_sub_config_scatter(ssg, q_sub, &vq_all);

    amino::SceneGraph *sg = ssg->scenegraph->sg;

    for( size_t i_sub = 0; i_sub < ssg->frame_count; i_sub++ ) {
        double E_rel[AA_RX_TF_LEN];
        aa_rx_frame_id i_frame = ssg->frames[i_sub];
        amino::SceneFrame *f = sg->frames[i_frame];
        f->tf_rel(q_all, E_rel);
        aa_rx_fk_set_rel(fk,i_frame,E_rel);
    }

}

AA_API size_t
aa_rx_sg_chain_multiple_frames( const struct aa_rx_sg *sg,
				aa_rx_frame_id root, size_t n_tips,
				aa_rx_frame_id* tips, size_t n_frames,
				aa_rx_frame_id *chain_frames)
{

    std::set<aa_rx_frame_id> frames;

    for(size_t i=0; i < n_tips; i ++){
	aa_rx_frame_id tip = tips[i];
	while( tip !=root && tip != AA_RX_FRAME_ROOT ){
	    if (frames.find(tip) == frames.end()) break;
	    enum aa_rx_frame_type ft = aa_rx_sg_frame_type( sg, tip );
	    switch(ft) {
	    case AA_RX_FRAME_FIXED:
		/* break; */
	    case AA_RX_FRAME_REVOLUTE:
	    case AA_RX_FRAME_PRISMATIC:
		frames.insert(tip);
		break;
	    }
	    tip = aa_rx_sg_frame_parent(sg, tip);
	}
    }

    size_t a=0;
    std::set<aa_rx_frame_id>::iterator it = frames.begin();

    while( it != frames.end()){
	chain_frames[a] = *it;
	a++;
	it++;
    }
    return a;
}
