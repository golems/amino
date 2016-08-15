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

#ifndef AMINO_SCENE_DYN_H
#define AMINO_SCENE_DYN_H

/**
 * @file scene_dyn.h
 * @brief scenegraph dynamics support
 */

/**
 * Set the frame inertial parameters.
 *
 * @param scenegraph   the scenegraph container
 * @param frame        name of frame to set mass parameters for
 * @param mass         mass of the frame
 * @param inertia      the inertia tensor, stored in column major order
 */
AA_API void
aa_rx_sg_frame_set_inertial( struct aa_rx_sg *scenegraph,
                             const char *frame,
                             double mass,
                             const double inertia[9] );
/**
 * Get the frame mass.
 *
 * @return the frame mass or NAN is mass has not been set for frame
 */
AA_API double
aa_rx_sg_frame_get_mass( struct aa_rx_sg *scenegraph,
                         aa_rx_frame_id frame );

/**
 * Get the frame inertia tensor.
 *
 * @return pointer to frame inertia tensor, stored in column major
 *         order, or NULL is mass has not been set for frame.
 */
AA_API const double*
aa_rx_sg_frame_get_inertia( struct aa_rx_sg *scenegraph,
                            aa_rx_frame_id frame );

#endif /*AMINO_SCENE_DYN_H*/
