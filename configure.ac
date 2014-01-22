#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.65])
AC_INIT([amino], [0.20120916], [ntd@gatech.edu])

AM_INIT_AUTOMAKE([subdir-objects])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([no])])

AC_CONFIG_SRCDIR([include/amino.h])
AC_CONFIG_HEADERS([config.h])

AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC_C99
AC_PROG_CXX
AM_PROG_CC_C_O
AC_PROG_FC
AC_PROG_LIBTOOL
AC_CHECK_PROGS([MAXIMA], [maxima])

# Checks for libraries.
AC_CHECK_HEADERS([cblas.h], [true], [exit 1])
AC_SEARCH_LIBS([clock_gettime],[rt])
AC_CHECK_LIB([m],[sqrt])
AC_CHECK_FUNCS([sincos])

# Check for thread local storage
AX_TLS

# Doxygen
DX_HTML_FEATURE(ON)
DX_PDF_FEATURE(OFF)
DX_PS_FEATURE(OFF)
DX_INIT_DOXYGEN(amino)

# Enable maximum warnings
AX_CFLAGS_WARN_ALL

dnl APPEND_FLAG( [FLAG] )
dnl Check if FLAG is supported, and if so, add it

m4_ifdef([AX_CHECK_COMPILE_FLAG],
         [AC_DEFUN([APPEND_CFLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CFLAGS="$1 $CFLAGS"])])
          AC_DEFUN([APPEND_CXXFLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [CXXFLAGS="$1 $CXXFLAGS"])])
          AC_DEFUN([APPEND_FCFLAG],
                   [AX_CHECK_COMPILE_FLAG([$1], [FCFLAGS="$1 $FCFLAGS"])])])

m4_ifdef([AX_CHECK_COMPILER_FLAGS],
         [AC_DEFUN([APPEND_CFLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CFLAGS="$1 $CFLAGS"])])
          AC_DEFUN([APPEND_CXXFLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [CXXFLAGS="$1 $CXXFLAGS"])])
          AC_DEFUN([APPEND_FCFLAG],
                   [AX_CHECK_COMPILER_FLAGS([$1], [FCFLAGS="$1 $FCFLAGS"])])])





AC_LANG([C])
APPEND_CFLAG([-Wextra])
APPEND_CFLAG([-Wconversion])
APPEND_CFLAG([-Wpointer-arith])
APPEND_CFLAG([-Wfloat-equal])
APPEND_CFLAG([-Wshadow])
APPEND_CFLAG([-Wwrite-strings])
APPEND_CFLAG([-Wc++-compat])

AC_LANG([C++])
APPEND_CXXFLAG([-Wextra])
APPEND_CXXFLAG([-Wconversion])
APPEND_CXXFLAG([-Wpointer-arith])
APPEND_CXXFLAG([-Wfloat-equal])
APPEND_CXXFLAG([-Wshadow])
APPEND_CXXFLAG([-Wwrite-strings])

AC_LANG([Fortran])
APPEND_FCFLAG([-fimplicit-none])
APPEND_FCFLAG([-ffree-line-length-none])
APPEND_FCFLAG([-Wall])
APPEND_FCFLAG([-pedantic])
APPEND_FCFLAG([-Wextra])
APPEND_FCFLAG([-Wconversion])
APPEND_FCFLAG([-Wconversion-extra])
APPEND_FCFLAG([-Wshadow])
APPEND_FCFLAG([-Wimplicit-interface])
APPEND_FCFLAG([-Wimplicit-procedure])
APPEND_FCFLAG([-Wunderflow])
APPEND_FCFLAG([-Warray-temporaries])
APPEND_FCFLAG([-Wtabs])
APPEND_FCFLAG([-Wunused-parameter])
APPEND_FCFLAG([-Wintrinsic-shadow])

AC_ARG_ENABLE([debug],
        AS_HELP_STRING([--enable-debug], [Build for debugging]))

AS_IF([test "x$enable_debug" = "xyes"],[
    AC_LANG([Fortran])
    APPEND_FCFLAG([-fbounds-check])
    APPEND_FCFLAG([-fstack-check])
    AC_LANG([C])
    APPEND_CFLAG([-fstack-check])
])

AS_IF( [test "x$JAVAPREFIX" = x],
       [
        AS_IF( [test "x$JAVA" = x],  [AC_CHECK_PROGS(JAVA, java kaffe)],  )
        AS_IF( [test "x$JAVAC" = x], [AC_CHECK_PROGS(JAVAC, javac "gcj -C" jikes guavac)] )
        AS_IF( [test "x$JAVAH" = x], [AC_CHECK_PROGS(JAVAH, javah)] )
        AS_IF( [test "x$JAR" = x],   [AC_CHECK_PROGS(JAR, jar)] )
       ],
       [
        AS_IF( [test "x$JAVA" = x],  [AC_CHECK_PROGS(JAVA, java kaffe)],
               [$JAVAPREFIX] )
        AS_IF( [test "x$JAVAC" = x], [AC_CHECK_PROGS(JAVAC, javac "gcj -C" jikes guavac)],
               [$JAVAPREFIX] )
        AS_IF( [test "x$JAVAH" = x], [AC_CHECK_PROGS(JAVAH, javah)],
               [$JAVAPREFIX] )
        AS_IF( [test "x$JAR" = x],   [AC_CHECK_PROGS(JAR, jar)],
               [$JAVAPREFIX] )
       ]
)

AS_IF([test "x$JAVAC" = "x"],
      [BUILD_JAVA=no],
      [
       BUILD_JAVA=yes
       AX_JNI_INCLUDE_DIR
       for JNI_INCLUDE_DIR in $JNI_INCLUDE_DIRS
       do
           JNICPPFLAGS="$JNICPPFLAGS -I$JNI_INCLUDE_DIR"
       done
      ])

AC_SUBST([JNICPPFLAGS])
AM_CONDITIONAL([HAVE_JAVAC],   [test "x$JAVAC" != x])


AC_LANG([C])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT


AC_MSG_NOTICE([=========================])
AC_MSG_NOTICE([= CONFIGURATION SUMMARY =])
AC_MSG_NOTICE([=========================])
AC_MSG_NOTICE([C compiler:        $CC])
AC_MSG_NOTICE([C++ compiler:      $CXX])
AC_MSG_NOTICE([Fortran compiler:  $FC])
AC_MSG_NOTICE([Java compiler:     $JAVAC])
AC_MSG_NOTICE([prefix:            $prefix])
AC_MSG_NOTICE([Build Java Lib:    $BUILD_JAVA])
