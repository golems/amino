#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.65])
AC_INIT([amino], [0.20110914], [ntd@gatech.edu])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([include/amino.h])
AC_CONFIG_HEADERS([config.h])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AC_PROG_F77
AC_PROG_LIBTOOL
AC_PROG_CC_C99

# Checks for libraries.

# Checks for header files.
AC_CHECK_HEADERS([inttypes.h stdint.h stdlib.h string.h unistd.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_INT16_T
AC_TYPE_INT32_T
AC_TYPE_INT64_T
AC_C_RESTRICT
AC_TYPE_SIZE_T
AC_TYPE_SSIZE_T
AC_TYPE_UINT16_T
AC_TYPE_UINT32_T
AC_TYPE_UINT64_T
AC_TYPE_UINT8_T
AC_CHECK_TYPES([ptrdiff_t])

# Checks for library functions.
AC_FUNC_ALLOCA
AC_FUNC_MALLOC
AC_FUNC_REALLOC
AC_CHECK_FUNCS([clock_gettime memset pow sqrt])


AC_SEARCH_LIBS([cblas_dgemm],[blas])
AC_SEARCH_LIBS([dgesvd_],[lapack])
AC_SEARCH_LIBS([clock_gettime],[rt])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
