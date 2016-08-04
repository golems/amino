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

#ifndef AMINO_RX_RXTYPE_H
#define AMINO_RX_RXTYPE_H

/**
 * @file rxtype.h
 * @brief Scenegraph-related type declarations
 */

/* Opaque types shared between different RX modules
 */

/**
 * @struct aa_rx_sg
 * Opaque type for a scene_graph.
 *
 * A scene graph is a set of frames in SE(3).
 *
 */
struct aa_rx_sg;

/**
 * @struct aa_rx_geom
 * Container for scene geometry
 */
struct aa_rx_geom;

/**
 * Container for collision info
 */
struct aa_rx_cl_geom;


/**
 * @struct aa_rx_win
 * Opaque type for a window
 */
struct aa_rx_win;

/**
 * @struct aa_sdl_display_params
 * Parameters for SDL display function.
 */
struct aa_sdl_display_params;

/**
 * Display handler function to call in SDL loop.
 *
 * @param context A pointer to local context
 * @param updated Whether other parts of these scene are updated
 * @param params Parameters for the display function
 * @return Whether any update has occurred
 */
typedef int (*aa_sdl_display_fun)(
    void *context,
    struct aa_sdl_display_params *params);

/**
 * Display handler function for amino windows to call in SDL loop
 *
 * @param win The window to display in
 * @param context A pointer to the local context for this function
 * @param params Parameters for the display function
 */
typedef int (*aa_sdl_win_display_fun)(
    struct aa_rx_win *win,
    void *context,
    struct aa_sdl_display_params *params);


#endif /*AMINO_RX_RXTYPE_H*/
