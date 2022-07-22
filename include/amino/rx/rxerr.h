/* -*- mode: C; c-basic-offset: 4; -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2015, Rice University,
 *               2020, Colorado School of Mines
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@rice.edu>
 *            Matthew A. Schack <mschack@mines.edu>
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

#ifndef AMINO_RX_RXERR_H
#define AMINO_RX_RXERR_H

/**
 * @file rxerr.h
 * @brief Error codes and functions
 */

/** OK */
#define AA_RX_OK 0
/** Synonym for AA_RX_OK */
AA_EXTERN const int aa_rx_ok;

/** No Solution */
#define AA_RX_NO_SOLUTION  (1<<0)
/** Synonym for AA_RX_NO_SOLUTION */
AA_EXTERN const int aa_rx_no_solution;

/** No Inverse Kinematics Solution */
#define AA_RX_NO_IK  (1<<1)
/** Synonym for AA_RX_NO_IK */
AA_EXTERN const int aa_rx_no_ik;

/** No Motion Plan */
#define AA_RX_NO_MP  (1<<2)
/** Synonym for AA_RX_NO_MP */
AA_EXTERN const int aa_rx_no_mp;

/** Invalid frame */
#define AA_RX_INVALID_FRAME  (1<<3)
/** Synonym for AA_RX_INVALID_FRAME */
AA_EXTERN const int aa_rx_invalid_frame;

/** Invalid Parameter */
#define AA_RX_INVALID_PARAMETER  (1<<4)
/** Synonym for AA_RX_INVALID_PARAMETER */
AA_EXTERN const int aa_rx_invalid_parameter;

/** Invalid State */
#define AA_RX_INVALID_STATE  (1<<5)
/** Synonym for AA_RX_INVALID_STATE */
AA_EXTERN const int aa_rx_invalid_state;

AA_API char *aa_rx_errstr( struct aa_mem_region *reg,
                    int e );

#endif /*AMINO_RX_RXERR_H*/
