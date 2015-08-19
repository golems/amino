#include "euler.c"

#ifdef __cplusplus
#define AA_API extern "C"
#define AA_EXTERN extern "C"
#define AA_RESTRICT
#else
/// calling and name mangling convention for functions
#define AA_API
/// name mangling convention external symbols
#define AA_EXTERN extern
/// Defined restrict keyword based on language flavor
#define AA_RESTRICT restrict
#endif //__cplusplus

int aa_tf_qnormalize__( double q[4] )
{
    aa_tf_qnormalize(q);
    return 0;
}

#define AA_TF_DEC_EULER(letters)                                        \
    AA_API void                                                         \
    aa_tf_euler ## letters ## 2rotmat( double e1, double e2, double e3, \
                                       double R[AA_RESTRICT 9] )        \
    {                                                                   \
        aa_tf_euler ## letters ## 2rotmat__( &e1, &e2, &e3, R );        \
    }                                                                   \
    AA_API void                                                         \
    aa_tf_euler ## letters ## 2quat( double e1, double e2, double e3,   \
                                     double q[AA_RESTRICT 4] )          \
    {                                                                   \
        aa_tf_euler ## letters ## 2quat__( &e1, &e2, &e3, q );          \
    }

AA_TF_DEC_EULER( xyz )
AA_TF_DEC_EULER( xzy )

AA_TF_DEC_EULER( yxz )
AA_TF_DEC_EULER( yzx )

AA_TF_DEC_EULER( zyx )
AA_TF_DEC_EULER( zxy )

AA_TF_DEC_EULER( xyx )
AA_TF_DEC_EULER( xzx )

AA_TF_DEC_EULER( yxy )
AA_TF_DEC_EULER( yzy )

AA_TF_DEC_EULER( zxz )
AA_TF_DEC_EULER( zyz )
