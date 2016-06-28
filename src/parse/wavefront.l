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
%option prefix="aa_rx_wf_"
%option outfile="wavefront.c"
%option reentrant
%option yylineno
%option batch
%option never-interactive
%option read

%option extra-type="struct wf_extra*"

%{

#pragma GCC diagnostic warning "-w"

#include <stdio.h>

#include "amino.h"
#include "wavefront_internal.h"

%}


%x EOL

%x MTLLIB
%x USEMTL
%x OBJECT

%x V0
%x V1
%x V2


%x VN0
%x VN1
%x VN2

%x FV0
%x FT0
%x FN0

%x FV1
%x FT1
%x FN1

%x FV2
%x FT2
%x FN2

%x ERROR

END [[:blank:]]*("#".*)?"\n"

DIGIT    [[:digit:]]
INT      {DIGIT}+
NUMBER   "-"?{INT}("."{DIGIT}*)?
BLANKS   [[:blank:]]+
MAYBE_BLANKS   [[:blank:]]*

NUMBER_BLANKS {NUMBER}{BLANKS}
NUMBER_END {NUMBER}{MAYBE_BLANKS}


START {MAYBE_BLANKS}

%%

{END} {
    /*fprintf(stderr, "comment: %s\n", yytext);*/
}

<EOL>{END} {
    BEGIN(INITIAL);
}

    /* MTLLIB */

{START}"mtllib"{BLANKS} {
    BEGIN(MTLLIB);

}

<MTLLIB>[[:graph:]]+ {
    aa_rx_wf_obj_push_mtl( yyextra->wf_obj, yytext );
    BEGIN(EOL);
}

    /* usemtl */

{START}"usemtl"{BLANKS} {
    BEGIN(USEMTL);
}

<USEMTL>[[:graph:]]+ {
    aa_rx_wf_obj_use_material(yyextra->wf_obj, yytext);
    BEGIN(EOL);
}

    /* OBJECT */

{START}"o"{BLANKS} {
    BEGIN(OBJECT);
}

<OBJECT>[[:graph:]]+ {
    aa_rx_wf_obj_push_object( yyextra->wf_obj, yytext );
    BEGIN(EOL);
}


   /* VECTOR */

{START}"v"{BLANKS} {
    BEGIN(V0);
}

<V0>{NUMBER_BLANKS} {
    aa_rx_wf_obj_push_vertex( yyextra->wf_obj, atof(yytext) );
    BEGIN(V1);
}

<V1>{NUMBER_BLANKS} {
    aa_rx_wf_obj_push_vertex( yyextra->wf_obj, atof(yytext) );
    BEGIN(V2);
}

<V2>{NUMBER_END} {
    aa_rx_wf_obj_push_vertex( yyextra->wf_obj, atof(yytext) );
    BEGIN(EOL);
}

    /* VECTOR-NORMAL */

{START}"vn"{BLANKS} {
    BEGIN(VN0);
}

<VN0>{NUMBER_BLANKS} {
    aa_rx_wf_obj_push_normal( yyextra->wf_obj, atof(yytext) );
    BEGIN(VN1);
}

<VN1>{NUMBER_BLANKS} {
    aa_rx_wf_obj_push_normal( yyextra->wf_obj, atof(yytext) );
    BEGIN(VN2);
}

<VN2>{NUMBER_END} {
    aa_rx_wf_obj_push_normal( yyextra->wf_obj, atof(yytext) );
    BEGIN(EOL);
}


    /* Smooth Shading */

{START}"s"{BLANKS}.* {
    /* fprintf(stderr, "ignoring smooth shading (line %d)\n", yylineno); */
    BEGIN(EOL);
}

    /* Face */
    /* vertex-1/[texture-1]/normal-1... */
    /* TODO: vertex-1 vertex-2 vertex-3 */
    /* TODO: vertex-1/texture-1 vertex-2/texture-2 vertex-3/texture-3 */

{START}"f"{BLANKS} {
    BEGIN(FV0);
}


<FV0>{INT}"/" {
    /* Vertex 0 */
    yyextra->face.v[0] = atol(yytext)-1;
    SET_WF_FACE(v[0]);
    BEGIN(FT0);
}

<FT0>"/" {
    /* Skip Texture 0 */
    SKIP_WF_FACE(t[0]);
    BEGIN(FN0);
}

<FT0>{INT}"/" {
    /* Texture 0 */
    SET_WF_FACE(t[0]);
    BEGIN(FN0);
}

<FN0>{INT}{BLANKS} {
    /* Normal 0 */
    SET_WF_FACE(n[0]);
    BEGIN(FV1);
}

<FV1>{INT}"/" {
    /* Vertex 1 */
    SET_WF_FACE(v[1]);
    BEGIN(FT1);
}

<FT1>"/" {
    /* Skip Texture 1 */
    SKIP_WF_FACE(t[1]);
    BEGIN(FN1);
}

<FT1>{INT}"/" {
    /* Texture 1 */
    SET_WF_FACE(t[1]);
    BEGIN(FN1);
}

<FN1>{INT}{BLANKS} {
    /* Normal 1 */
    SET_WF_FACE(n[1]);
    BEGIN(FV2);
}

<FV2>{INT}"/" {
    /* Vertex 2 */
    SET_WF_FACE(v[2]);
    BEGIN(FT2);
}

<FT2>"/" {
    /* Skip Texture 2 */
    SKIP_WF_FACE(t[2]);
    BEGIN(FN2);
}

<FT2>{INT}"/" {
    /* Texture 2 */
    SET_WF_FACE(t[2]);
    BEGIN(FN2);
}

<FN2>{INT}{END} {
    /* Normal 2 */
    SET_WF_FACE(n[2]);
    aa_rx_wf_obj_push_face(yyextra->wf_obj, &yyextra->face);
    BEGIN(INITIAL);
}


    /* Ignore this thing */
"l"{BLANKS}.* {
}

    /* Error */
    /* This rule needs to be last */

<*>. {
    fprintf(stderr, "Unrecognized character: %s (%d)\n", yytext, yylineno);
    return -1;
}

%%

struct aa_rx_wf_obj *
aa_rx_wf_parse(const char *filename)
{
    FILE *in = fopen(filename,"r");
    if( NULL == in ) {
        fprintf(stderr, "Could not open '%s'\n", filename);
        return NULL;
    }

    yyscan_t scanner;
    aa_rx_wf_lex_init(&scanner);
    aa_rx_wf_set_in(in, scanner);

    struct wf_extra we;
    we.wf_obj = aa_rx_wf_obj_create();

    aa_rx_wf_set_extra(&we, scanner);

    int r = aa_rx_wf_lex(scanner);

    aa_rx_wf_lex_destroy(scanner);
    fclose(in);

    if( r < 0 ) {
        aa_rx_wf_obj_destroy(we.wf_obj);
        return NULL;
    } else {
        return we.wf_obj;
    }
}
