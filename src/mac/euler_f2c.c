#include "amino.h"
#include "config.h"

#ifdef USE_FORTRAN
#define SUFFIX(THING) THING ## _
#else
#define SUFFIX(THING) THING ## __
#endif // USE_FORTRAN


int SUFFIX(aa_tf_qnormalize)( double q[4] )
{
    aa_tf_qnormalize(q);
    return 0;
}

#define AA_TF_DEC_EULER(letters)                                        \
    AA_API int                                                          \
    SUFFIX(aa_tf_euler ## letters ## 2rotmat)                           \
    ( double *e1, double *e2, double *e3,                               \
      double R[AA_RESTRICT 9] );                                        \
                                                                        \
    AA_API void                                                         \
    aa_tf_euler ## letters ## 2rotmat( double e1, double e2, double e3, \
                                       double R[AA_RESTRICT 9] )        \
    {                                                                   \
        SUFFIX(aa_tf_euler ## letters ## 2rotmat)( &e1, &e2, &e3, R );  \
    }                                                                   \
                                                                        \
    AA_API int                                                          \
    SUFFIX(aa_tf_euler ## letters ## 2quat)                             \
    ( double *e1, double *e2, double *e3,                               \
      double q[AA_RESTRICT 4] );                                        \
                                                                        \
    AA_API void                                                         \
    aa_tf_euler ## letters ## 2quat( double e1, double e2, double e3,   \
                                     double q[AA_RESTRICT 4] )          \
    {                                                                   \
        SUFFIX(aa_tf_euler ## letters ## 2quat)( &e1, &e2, &e3, q );    \
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
