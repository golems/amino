
#include "def.h"

#ifdef AA_LA_TYPE_DOUBLE
#define TOREAL DBLE
#define AA_FSIZE 8
#define AA_FTYPE real
#endif
#ifdef AA_LA_TYPE_FLOAT
#define AA_FSIZE 4
#define AA_FTYPE real
#define TOREAL REAL
#endif

