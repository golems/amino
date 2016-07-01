/*
 * Copyright (c) 2016, Rice University
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


/*
 * The is a parser for Wavefront OBJ files, written as a Flex Scanner.
 *
 *
 */

%option noyywrap
%option prefix="aa_rx_wf_mtl_"
%option outfile="mtl.c"
%option reentrant
%option yylineno
%option batch
%option never-interactive
%option read

%option stack

%option extra-type="struct wf_mtl_extra*"

%{

#pragma GCC diagnostic warning "-w"

#include <stdio.h>

#include "amino.h"
#include "wavefront_internal.h"

%}

%x EOL
%x NEWMTL

%x NS

%x KA
%x KD
%x KS
%x KE

%x V0
%x V1
%x V2

%x D
%x ILLUM


END [[:blank:]]*("#".*)?"\n"

DIGIT    [[:digit:]]
INT      {DIGIT}+
NUMBER   "-"?{INT}("."{DIGIT}*)?
BLANKS   [[:blank:]]+
MAYBE_BLANKS   [[:blank:]]*

NUMBER_BLANKS {NUMBER}{BLANKS}
NUMBER_END {NUMBER}{MAYBE_BLANKS}

ID [[:graph:]]+


START {MAYBE_BLANKS}

%%

{END} {
    /*fprintf(stderr, "comment: %s\n", yytext);*/
}

<EOL>{END} {
    BEGIN(INITIAL);
}


{START}"newmtl"{BLANKS} {
    BEGIN(NEWMTL);
}

<NEWMTL>{ID} {
    aa_rx_wf_mtl_push( yyextra->mtl, yytext );
    BEGIN(EOL);
}

    /* Specular Weight */
{START}"Ns"{BLANKS} {
    BEGIN(NS);
}

<NS>{NUMBER}{MAYBE_BLANKS} {
    WF_MTL_CURRENT->specular_weight = atof(yytext);
    WF_MTL_CURRENT->has_specular_weight = 1;
    BEGIN(EOL);
}

<V0>{NUMBER}{BLANKS} {
    yyextra->v[0] = atof(yytext);
    BEGIN(V1);
}

<V1>{NUMBER}{BLANKS} {
    yyextra->v[1] = atof(yytext);
    BEGIN(V2);
}

<V2>{NUMBER}{MAYBE_BLANKS} {
    yyextra->v[2] = atof(yytext);
    yy_pop_state(yyscanner);
}


    /* Ambient */
{START}"Ka"{BLANKS} {
    yyextra->v = WF_MTL_CURRENT->ambient;
    BEGIN(KA);
    yy_push_state(V0, yyscanner );
}

<KA>{END} {
    WF_MTL_CURRENT->has_ambient = 1;
    BEGIN(INITIAL);
}


    /* Diffuse */
{START}"Kd"{BLANKS} {
    yyextra->v = WF_MTL_CURRENT->diffuse;
    BEGIN(KD);
    yy_push_state(V0, yyscanner );
}

<KD>{END} {
    WF_MTL_CURRENT->has_diffuse = 1;
    BEGIN(INITIAL);
}

    /* Specular */
{START}"Ks"{BLANKS} {
    yyextra->v = WF_MTL_CURRENT->specular;
    BEGIN(KS);
    yy_push_state(V0, yyscanner );
}

<KS>{END} {
    WF_MTL_CURRENT->has_specular = 1;
    BEGIN(INITIAL);
}


    /* Emission */
{START}"Ke"{BLANKS} {
    yyextra->v = WF_MTL_CURRENT->emission;
    BEGIN(KE);
    yy_push_state(V0, yyscanner );

}

<KE>{END} {
    WF_MTL_CURRENT->has_emission = 1;
    BEGIN(INITIAL);
}


    /* alpha */
{START}"d"{BLANKS} {
    BEGIN(D);
}

<D>{NUMBER}{MAYBE_BLANKS} {
    WF_MTL_CURRENT->alpha = atof(yytext);
    WF_MTL_CURRENT->has_alpha = 1;
    BEGIN(EOL);
}

{START}"illum".*{END} {
    /* TODO */
}

{START}"map_Kd".*{END} {
    /* TODO */
}

    /* Error */
    /* This rule needs to be last */

<*>. {
    fprintf(stderr, "Unrecognized character: %s (line %d)\n", yytext, yylineno);
    return -1;
}

%%

AA_API struct aa_rx_wf_mtl *
aa_rx_wf_mtl_parse(const char *filename)
{
    FILE *in = fopen(filename,"r");
    if( NULL == in ) {
        fprintf(stderr, "Could not open '%s'\n", filename);
        return NULL;
    }

    yyscan_t scanner;
    aa_rx_wf_mtl_lex_init(&scanner);
    aa_rx_wf_mtl_set_in(in, scanner);

    struct wf_mtl_extra e;
    e.mtl = aa_rx_wf_mtl_create();
    aa_rx_wf_set_extra(&e, scanner);

    int r = aa_rx_wf_mtl_lex(scanner);

    aa_rx_wf_mtl_lex_destroy(scanner);
    fclose(in);

    if( r < 0 ) {
        aa_rx_wf_mtl_destroy(e.mtl);
        return NULL;
    } else {
        return e.mtl;
    }
}
