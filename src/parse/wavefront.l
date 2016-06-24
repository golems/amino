%option noyywrap
%option prefix="aa_rx_wf_"
%option outfile="wavefront.c"
%option reentrant
%option yylineno


%{


#pragma GCC diagnostic warning "-w"

#include <stdio.h>
%}

%x MTLLIB
%x USEMTL
%x OBJECT
%x V0
%x V1
%x V2

%x EOL

%x VN0
%x VN1
%x VN2

END [[:blank:]]*("#".*)?"\n"

DIGIT    [[:digit:]]
NUMBER   "-"?{DIGIT}+("."{DIGIT}*)?
BLANKS   [[:blank:]]+
MAYBE_BLANKS   [[:blank:]]*

NUMBER_BLANKS {NUMBER}{BLANKS}
NUMBER_END {NUMBER}{MAYBE_BLANKS}



START ^[[:blank:]]*

%%

^{END} {
    fprintf(stderr, "comment: %s\n", yytext);
}

<EOL>{END} {
    BEGIN(INITIAL);
}

    /* MTLLIB */

{START}"mtllib"{BLANKS}+ {
    BEGIN(MTLLIB);
}

<MTLLIB>.*{END} {
    fprintf(stderr, "mtllib: %s\n", yytext);
    BEGIN(INITIAL);
}

    /* usemtl */

{START}"usemtl"{BLANKS}+ {
    BEGIN(USEMTL);
}

<USEMTL>.*{END} {
    fprintf(stderr, "usemtl: %s\n", yytext);
    BEGIN(INITIAL);
}

    /* OBJECT */

{START}"o"{BLANKS} {
    BEGIN(OBJECT);
}

<OBJECT>.*{END} {
    printf("object: %s\n",yytext);
    BEGIN(INITIAL);
}


    /* VECTOR */

{START}"v"{BLANKS} {
    printf("got v: %s!\n",yytext);
    BEGIN(V0);
}

<V0>{NUMBER_BLANKS} {
    printf("v0: %f (%s)!\n", atof(yytext), yytext);
    BEGIN(V1);
}

<V1>{NUMBER_BLANKS} {
    printf("v1: %f (%s)!\n", atof(yytext), yytext);
    BEGIN(V2);
}

<V2>{NUMBER_END} {
    printf("v2: %f (%s)!\n", atof(yytext), yytext);
    BEGIN(EOL);
}

    /* VECTOR-NORMAL */

{START}"vn"{BLANKS} {
    printf("got vn: %s!\n",yytext);
    BEGIN(VN0);
}

<VN0>{NUMBER_BLANKS} {
    printf("vn0: %f (%s)!\n", atof(yytext), yytext);
    BEGIN(VN1);
}

<VN1>{NUMBER_BLANKS} {
    printf("vn1: %f (%s)!\n", atof(yytext), yytext);
    BEGIN(VN2);
}

<VN2>{NUMBER_END} {
    printf("vn2: %f (%s)!\n", atof(yytext), yytext);
    BEGIN(EOL);
}

<*>. {
    fprintf(stderr, "Unrecognized character: %s (%d)\n", yytext, yylineno);
    exit(EXIT_FAILURE);
}


%%

main()
{
    yyscan_t scanner;
    aa_rx_wf_lex_init(&scanner);
    aa_rx_wf_lex(scanner);
    aa_rx_wf_lex_destroy(scanner);
}
