/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
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


#ifndef AMINO_UNIT_H
#define AMINO_UNIT_H

// for all constants, MKS_VALUE = XXX_VALUE * AA_UCONV_XXX

// length
#define AA_UCONV_INCH 2.54e-2
#define AA_UCONV_FOOT 3.048e-1
#define AA_UCONV_YARD 9.144e-1
#define AA_UCONV_MILE 1.609344e3
#define AA_UCONV_ANGSTROM 1e-10
#define AA_UCONV_FURLONG 2.0117e2
#define AA_UCONV_FATHOM 1.8288e0

// mass
#define AA_UCONV_POUND_MASS 4.5359e-1
#define AA_UCONV_SLUG 1.4594e1
#define AA_UCONV_TON_SHORT 9.07018474e2
#define AA_UCONV_TON_LONG 1.016004691e3
#define AA_UCONV_TON_METRIC 1e3
#define AA_UCONV_OUNCE 2.83495231e-2
#define AA_UCONV_STONE 6.35029318e0
#define AA_UCONV_GRAIN 6.479891e-5

// time
#define AA_UCONV_HOUR 3.6e3
#define AA_UCONV_FORTNIGHT 1.209600e6

// angle
#define AA_UCONV_DEGREE (M_PI/180.0)

// area
#define AA_UCONV_ACRE  4.0469e3

// volume
#define AA_UCONV_GALLON 3.7854e-3
#define AA_UCONV_BUSHEL 3.5239e-2
#define AA_UCONV_FLUID_OUNCE 2.95735296e-5

// charge
#define AA_UCONV_AMP_HOUR 3.6e3

// force
#define AA_UCONV_POUND_FORCE 4.4482e0

// pressure
#define AA_UCONV_PSI 6.8948e3
#define AA_UCONV_ATM 1.0133e5
#define AA_UCONV_BAR 1e5
#define AA_UCONV_MMHG 1.33322368e2

// energy
#define AA_UCONV_BTU 1.0551e3
#define AA_UCONV_KILOWATT_HOUR 3.6e6

// energy/torque
#define AA_UCONV_FOOT_POUND 1.3558e0

// power
#define AA_UCONV_HORSEPOWER 7.4570e2
#define AA_UCONV_BTU_PER_HOUR 2.9288e-1
#define AA_UCONV_BTU_PER_MINUTE 1.7573e1


#define AA_UCONV_DEGREE (M_PI/180.0)



#endif //AMINO_UNIT_H
