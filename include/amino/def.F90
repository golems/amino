
#include "def.h"

#ifdef AA_TYPE_DOUBLE
#define TOREAL DBLE
#define AA_FSIZE C_DOUBLE
#define AA_FTYPE real
#endif

#ifdef AA_TYPE_FLOAT
#define AA_FSIZE C_FLOAT
#define AA_FTYPE real
#define TOREAL REAL
#endif

#ifdef AA_TYPE_INT
#define AA_FSIZE 4
#define AA_FTYPE integer
#endif

#ifdef AA_TYPE_LONG
#define AA_FSIZE 8
#define AA_FTYPE integer
#endif


#ifdef AA_TYPE_FLOGICAL1
#define AA_FSIZE 1
#define AA_FTYPE logical
#endif

#ifdef AA_TYPE_FLOGICAL4
#define AA_FSIZE 4
#define AA_FTYPE logical
#endif
