/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "amino.h"

AA_API void
aa_plot_series( size_t m, size_t n, double *t, double *X,
                struct aa_plot_opts *opts ) {

    int r;
    FILE *g = popen("gnuplot -persist", "w");
    //FILE *g = stderr;

    assert(g);
    assert( n > 0 );
    assert( m > 0 );

    if(opts->xlabel) fprintf(g, "set xlabel '%s'\n", opts->xlabel);
    if(opts->ylabel) fprintf(g, "set ylabel '%s'\n", opts->ylabel);
    if(opts->title) fprintf(g, "set title '%s'\n", opts->title);
    if( opts->axis_label ) {
        fprintf(g, "plot '-' with lines title '%s'", opts->axis_label[0]);
        for( size_t i = 1; i < m; i ++ ) {
            fprintf(g, ", '-' with lines title '%s'",opts-> axis_label[i]);
        }
    } else {
        fprintf(g, "plot '-' with lines");
        for( size_t i = 1; i < m; i ++ ) {
            fprintf(g, ", '-' with lines");
        }
    }
    fprintf(g, "\n");

    // print X
    for( size_t s = 0; s < m; s ++ ) {
        for( size_t i = 0; i < n; i++ ) {
            fprintf(g, "%f %f\n", t[i], AA_MATREF(X,m,s,i) );
        }
        fprintf(g, "e\n");
    }
    fflush(g);
    r = pclose(g);
    assert( -1 != r );
}

