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

#ifndef AMINO_RX_RXTYPE_INTERNAL_H
#define AMINO_RX_RXTYPE_INTERNAL_H

/* Opaque types shared between different RX modules
 */

/**
 * Container for OpenGL buffers
 */
struct aa_gl_buffers;


/* Cleanup function set in GL module */
AA_EXTERN void
(*aa_gl_buffers_destroy_fun)( struct aa_gl_buffers *bufs );

struct aa_rx_cl_geom;

AA_API void
aa_rx_cl_geom_destroy( struct aa_rx_cl_geom *cl_geom );

/* Cleanup function set in collision module.
 *
 * This separate variable exists for linking reasons.  The variable
 * lives in the geometry library, and the geometry library destroys
 * collision objects by calling the destructor via this function.  The
 * collision library sets this variable to the actual destructor
 * function when aa_rx_cl_init() is called.  This avoid the need to
 * link against both geometry and collision libraries if collision
 * checking is not required.
 */
AA_EXTERN void
(*aa_rx_cl_geom_destroy_fun)( struct aa_rx_cl_geom *cl_geom );

#endif /*AMINO_RX_RXTYPE_INTERNAL_H*/
