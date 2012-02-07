#ifndef AA_MANGLE_H
#define AA_MANGLE_H


#define AA_FORT_MANGLE( sym ) sym ## _
#define AA_FORT_MOD_MANGLE( mod, sym )  __ ## mod ## _MOD_ ## sym

#define AA_MANGLE_NAME( type, prefix, name )    \
    aa_ ## prefix ## _ ## type ## _ ## name
#define AA_MANGLE_CLA_NAME( type, name ) aa_cla_ ## type ## name
#define AA_MANGLE_CBLAS_NAME( type, name ) cblas_ ## type ## name
#define AA_MANGLE_LAPACK_NAME( type, name )  type ## name ## _
#define AA_MANGLE_FMOD( type, prefix, name )    \
    aa_ ## prefix ## _mod_ ## type ## _ ## name
#define AA_MANGLE_FMOD_F( type, prefix, name )                  \
    AA_FORT_MOD_MANGLE(amino_ ## la,                            \
                       aa_la_mod_ ## type ##_ ## name ## _c)
#define AA_MANGLE_FMOD_C( type, prefix, name )  \
    AA_MANGLE_FMOD( type, prefix, name ## _c )


#endif
