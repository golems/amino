#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.65])
AC_INIT([amino], [0.20140818], [ntd@gatech.edu])

AM_INIT_AUTOMAKE([subdir-objects])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([no])])

AC_CONFIG_SRCDIR([include/amino.h])
AC_CONFIG_HEADERS([config.h])

AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC_C99
AC_PROG_CXX
AC_PROG_FC
AM_PROG_CC_C_O
AC_PROG_LIBTOOL
AC_CHECK_PROGS([MAXIMA], [maxima])

# Enable C++ 11
m4_ifdef([AX_CXX_COMPILE_STDCXX_11],
         [AX_CXX_COMPILE_STDCXX_11],
         [m4_ifdef([AX_CXX_COMPILE_STDCXX_0X],
                   [AX_CXX_COMPILE_STDCXX_0X])])


# Checks for libraries.
AC_CHECK_HEADERS([cblas.h], [true], [exit 1])
AC_SEARCH_LIBS([clock_gettime],[rt])
AC_CHECK_LIB([m],[sqrt])

AC_SEARCH_LIBS([dgemm_],[openblas blas])
AC_SEARCH_LIBS([dgesvd_],[lapack])


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



# Fortran Checks
AC_ARG_WITH([fortran],
            AS_HELP_STRING([--without-fortran], [Disable fortran compiler]),
            [],
            [with_fortran=yes])

AM_CONDITIONAL([USE_FORTRAN], [test "x$with_fortran" = xyes])

AS_IF([test "x$with_fortran" = xyes],
[
AC_LANG([Fortran])

AC_SEARCH_LIBS([__iso_c_binding_c_f_pointer_l4],[gfortran])

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

],
[
AC_CHECK_PROGS([F2C], [f2c])
AC_LANG([C])
AC_CHECK_HEADERS([f2c.h])
]
)


# Check for thread local storage
AX_TLS

# Doxygen
m4_ifdef([DX_INIT_DOXYGEN],
         [DX_HTML_FEATURE(ON)
          DX_PDF_FEATURE(OFF)
          DX_MAN_FEATURE(OFF)
          DX_PS_FEATURE(OFF)
          DX_CHM_FEATURE(OFF)
          DX_CHI_FEATURE(OFF)
          DX_MAN_FEATURE(OFF)
          DX_RTF_FEATURE(OFF)
          DX_XML_FEATURE(OFF)
          DX_PDF_FEATURE(OFF)
          DX_INIT_DOXYGEN(amino)],
         [AM_CONDITIONAL([DX_COND_doc],   false)
          AM_CONDITIONAL([DX_COND_html],  false)
          AM_CONDITIONAL([DX_COND_chm],   false)
          AM_CONDITIONAL([DX_COND_chi],   false)
          AM_CONDITIONAL([DX_COND_rtf],   false)
          AM_CONDITIONAL([DX_COND_xml],   false)
          AM_CONDITIONAL([DX_COND_ps],    false)
          AM_CONDITIONAL([DX_COND_pdf],   false)
          AM_CONDITIONAL([DX_COND_latex], false)
          AM_CONDITIONAL([DX_COND_man],   false)
         ])

# Enable maximum warnings
AX_CFLAGS_WARN_ALL

dnl APPEND_FLAG( [FLAG] )
dnl Check if FLAG is supported, and if so, add it





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

AC_ARG_ENABLE([debug],
        AS_HELP_STRING([--enable-debug], [Build for debugging]))



AS_IF([test "x$enable_debug" = "xyes"],[
    AC_LANG([Fortran])
    APPEND_FCFLAG([-fbounds-check])
    APPEND_FCFLAG([-fstack-check])
    AC_LANG([C])
    APPEND_CFLAG([-fstack-check])
])


## Java ##

AC_ARG_WITH([java],
            AS_HELP_STRING([--without-java], [Disable java support]),
            [],
            [with_java=yes])


AS_IF( [test "x$with_java" = xyes],
[
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
])

AC_SUBST([JNICPPFLAGS])
AM_CONDITIONAL([HAVE_JAVAC],   [test "x$JAVAC" != x])


AC_LANG([C])

## GRAPHICS ##

# OpenGL
AC_ARG_WITH([opengl],
            [AS_HELP_STRING([--without-opengl], [Disable OpenGL support])],
            [],
            [with_opengl=yes])

AS_IF([test "x$with_opengl" = xyes], [
  PKG_CHECK_MODULES([GL],
                    [gl],
                    [HAVE_GL=yes])
  dnl PKG_CHECK_MODULES([GLU],
  dnl                   [glu],
  dnl                   [HAVE_GLU=yes])
  ]
)

AC_SUBST([GL_CFLAGS])
AC_SUBST([GL_LIBS])
AM_CONDITIONAL([HAVE_GL], [test "x$HAVE_GL" = xyes])

dnl AC_SUBST([GLU_CFLAGS])
dnl AC_SUBST([GLU_LIBS])
dnl AM_CONDITIONAL([HAVE_GLU], [test "x$HAVE_GLU" = xyes])

# SDL
AC_ARG_WITH([sdl2],
            [AS_HELP_STRING([--without-sdl2], [Disable SDL2 support])],
            [],
            [with_sdl2=$with_opengl])

HAVE_SDL2=disabled
AS_IF([test "x$with_sdl2" = xyes], [
  PKG_CHECK_MODULES([SDL2],
                    [sdl2],
                    [HAVE_SDL2=yes])

  PKG_CHECK_MODULES([GLEW],
                    [glew] )]
)

AM_CONDITIONAL([HAVE_SDL2], [test "x$HAVE_SDL2" = xyes])

AC_SUBST([GLEW_CFLAGS])
AC_SUBST([GLEW_LIBS])
AC_SUBST([SDL2_CFLAGS])
AC_SUBST([SDL2_LIBS])

## SUMMARY ##
AC_MSG_NOTICE([===============================])
AC_MSG_NOTICE([= AMINO CONFIGURATION SUMMARY =])
AC_MSG_NOTICE([===============================])
AC_MSG_NOTICE([prefix:            $prefix])
AC_MSG_NOTICE([C compiler:        $CC])
AC_MSG_NOTICE([C++ compiler:      $CXX])
AC_MSG_NOTICE([Use Fortran        $with_fortran])
AS_IF([test "x$with_fortran" = xyes],[
AC_MSG_NOTICE([Fortran compiler:  $FC])]
)
AC_MSG_NOTICE([Use Java           $with_java])
AS_IF([test "x$with_java" = xyes], [
AC_MSG_NOTICE([Java compiler:     $JAVAC])]
AC_MSG_NOTICE([Build Java Lib:    $BUILD_JAVA])
)
AC_MSG_NOTICE([OpenGL:            $with_opengl])
AC_MSG_NOTICE([SDL2:              $HAVE_SDL2])


AS_IF([test "x$CC" = "x"],
      [AC_MSG_ERROR([No C compiler detected])])
AS_IF([test "x$CXX" = "x"],
      [AC_MSG_ERROR([No C++ compiler detected])])
AS_IF([test "x$FC" = "x"],
      [AC_MSG_ERROR([No Fortran compiler detected])])


AC_CONFIG_FILES([Makefile])
AC_OUTPUT
