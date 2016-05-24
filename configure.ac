#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.65])
AC_INIT([amino], [0.20140818], [ntd@gatech.edu])

AM_INIT_AUTOMAKE([subdir-objects])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

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
AM_PATH_LISPDIR

# Enable C++ 11
m4_ifdef([AX_CXX_COMPILE_STDCXX_11],
         [AX_CXX_COMPILE_STDCXX_11],
         [m4_ifdef([AX_CXX_COMPILE_STDCXX_0X],
                   [AX_CXX_COMPILE_STDCXX_0X])])


# Checks for libraries.
AC_CHECK_HEADERS([cblas.h], [true], [exit 1])
AC_CHECK_HEADERS([stdatomic.h])

AC_SEARCH_LIBS([clock_gettime], [rt])
AC_SEARCH_LIBS([sqrt], [m])
AC_SEARCH_LIBS([dlopen], [dl])
AC_SEARCH_LIBS([pthread_once], [pthread])


AC_SEARCH_LIBS([dgemm_],[blas])
AC_SEARCH_LIBS([dgesvd_],[lapack])


# Check for cblas_blas enum
AC_MSG_CHECKING([CBLAS_LAYOUT])
AC_TRY_COMPILE([#include <cblas.h>
               ],
               [CBLAS_LAYOUT test_var;],
               [AC_MSG_RESULT([yes])
                aa_cblas_order_type=CBLAS_LAYOUT],
               [AC_MSG_RESULT([no])])

AS_IF([test x="x$aa_cblas_order_type"],
[
AC_MSG_CHECKING([enum CBLAS_ORDER])
AC_TRY_COMPILE([#include <cblas.h>
               ],
               [enum CBLAS_ORDER test_var;],
               [AC_MSG_RESULT([yes])
                aa_cblas_order_type="enum CBLAS_ORDER"],
               [AC_MSG_RESULT([no])])
])

AC_SUBST([aa_cblas_order_type])

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
    APPEND_CFLAG([-g])
    AC_LANG([C])
    APPEND_CFLAG([-fstack-check])
    APPEND_CFLAG([-g])
    AC_LANG([C++])
    APPEND_CXXFLAG([-g])
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


#######################
## OPTIONAL PACKAGES ##
#######################

## OpenGL ##
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

## SDL ##
AC_ARG_WITH([sdl2],
            [AS_HELP_STRING([--without-sdl2], [Disable SDL2 support])],
            [],
            [with_sdl2="$with_opengl"])

HAVE_SDL2=disabled
AS_IF([test "x$with_sdl2" = xyes], [
  AS_IF([test "x$HAVE_GL" != xyes], [
    AC_MSG_ERROR([SDL support requires OpenGL, but OpenGL not found])
  ])
  PKG_CHECK_MODULES([SDL2],
                    [sdl2],
                    [HAVE_SDL2=yes])

  PKG_CHECK_MODULES([GLEW],
                    [glew] )]
)

AC_SUBST([GLEW_CFLAGS])
AC_SUBST([GLEW_LIBS])
AC_SUBST([SDL2_CFLAGS])
AC_SUBST([SDL2_LIBS])
AM_CONDITIONAL([HAVE_SDL2], [test "x$HAVE_SDL2" = xyes])

## FCL ##
AC_ARG_WITH([fcl],
            [AS_HELP_STRING([--without-fcl], [Disable support for the Flexible Collision Library (FCL)])],
            [],
            [with_fcl=yes])

HAVE_FCL=disabled
AS_IF([test "x$with_fcl" = xyes], [
  PKG_CHECK_MODULES([FCL],
                    [fcl],
                    [HAVE_FCL=yes])
  # TODO: smartly check with we also need boost_system
  FCL_LIBS="$FCL_LIBS -lboost_system"
])
AC_SUBST([FCL_CFLAGS])
AC_SUBST([FCL_LIBS])
AM_CONDITIONAL([HAVE_FCL], [test "x$HAVE_FCL" = xyes])

## OMPL ##
AC_ARG_WITH([ompl],
            [AS_HELP_STRING([--without-ompl], [Disable support for the Open Motion Planning Library (OMPL)])],
            [],
            [with_ompl="$with_fcl"])

HAVE_OMPL=disabled
AS_IF([test "x$with_ompl" = xyes], [
  AS_IF([test "x$HAVE_FCL" != xyes], [
    AC_MSG_ERROR([OMPL support requires FCL, but FCL not found])
  ])
  PKG_CHECK_MODULES([OMPL],
                    [ompl],
                    [HAVE_OMPL=yes])
  # TODO: smartly check with we also need boost_system
  OMPL_LIBS="$OMPL_LIBS = -lboost_date_time -lboost_system -lboost_chrono -lboost_thread -lboost_serialization"
])

AC_SUBST([OMPL_CFLAGS])
AC_SUBST([OMPL_LIBS])
AM_CONDITIONAL([HAVE_OMPL], [test "x$HAVE_OMPL" = xyes])

## Spacenav ##
AC_ARG_WITH([spacenav],
            [AS_HELP_STRING([--without-spacenav], [Disable support for 3D Connexion space navigator])],
            [],
            [with_spacenav=$with_sdl])

HAVE_SPACENAV=disabled
AS_IF([test "x$with_spacenave" != xno], [
  AC_CHECK_HEADERS([spnav.h], [HAVE_SPACENAV=yes], [HAVE_SPACENAV=no])
  AC_SEARCH_LIBS([spnav_open],[spnav])
])

AM_CONDITIONAL([HAVE_SPACENAV], [test "x$HAVE_SPACENAV" = xyes])

## LP_SOLVE ##
AC_LANG([C])
AC_ARG_WITH([lpsolve],
            [AS_HELP_STRING([--without-lpsolve], [Disable support for LP_Solve])],
            [],
            [with_lpsolve=auto])
# check header
AS_IF([test "x$with_lpsolve" = xauto], [
  AC_CHECK_HEADERS([lpsolve/lp_lib.h], [HAVE_LPSOLVE=yes], [HAVE_LPSOLVE=no])
],[
  HAVE_LPSOLVE="$with_lpsolve"
])

# check library
AS_IF([test "x$HAVE_LPSOLVE" = xyes], [
  AC_SEARCH_LIBS([make_lp],[lpsolve55],
                 [HAVE_LPSOLVE=yes],
                 [HAVE_LPSOLVE=no])
])

AH_TEMPLATE([HAVE_LPSOLVE],
            [Define presense of LP-solve library])
AM_CONDITIONAL([HAVE_LPSOLVE], [test "x$HAVE_LPSOLVE" = xyes])
AS_IF([test "x$HAVE_LPSOLVE" = xyes], [AC_DEFINE([HAVE_LPSOLVE])])


## GLPK ##
AC_LANG([C])
AC_ARG_WITH([glpk],
            [AS_HELP_STRING([--without-glpk], [Disable support for GLPK])],
            [],
            [with_glpk=auto])
# check header
AS_IF([test "x$with_glpk" = xauto], [
  AC_CHECK_HEADERS([glpk.h], [HAVE_GLPK=yes], [HAVE_GLPK=no])
],[
  HAVE_GLPK="$with_glpk"
])

# check library
AS_IF([test "x$HAVE_GLPK" = xyes], [
  AC_SEARCH_LIBS([glp_create_prob],[glpk],
                 [HAVE_GLPK=yes],
                 [HAVE_GLPK=no])
])

AM_CONDITIONAL([HAVE_GLPK], [test "x$HAVE_GLPK" = xyes])
AH_TEMPLATE([HAVE_GLPK],
            [Define presense of GLPK library])
AS_IF([test "x$HAVE_GLPK" = xyes], [AC_DEFINE([HAVE_GLPK])])


## CLP ##
AC_LANG([C++])
AC_ARG_WITH([clp],
            [AS_HELP_STRING([--without-clp], [Disable support for CLP])],
            [],
            [with_clp=auto])

AS_IF([test "x$with_clp" = xauto], [
  AC_CHECK_HEADERS([coin/ClpModel.hpp], [HAVE_CLP=yes], [HAVE_CLP=no])
],[
  HAVE_CLP="$with_clp"
])

AM_CONDITIONAL([HAVE_CLP], [test "x$HAVE_CLP" = xyes])
AH_TEMPLATE([HAVE_CLP],
            [Define presense of CLP library])
AS_IF([test "x$HAVE_CLP" = xyes], [AC_DEFINE([HAVE_CLP])])


## CGAL ##
AC_LANG([C++])
AC_ARG_WITH([cgal],
            [AS_HELP_STRING([--without-cgal], [Disable support for CGAL])],
            [],
            [with_cgal=auto])

AS_IF([test "x$with_cgal" = xauto], [
  AC_CHECK_HEADERS([CGAL/basic.h], [HAVE_CGAL=yes], [HAVE_CGAL=no])
],[
  HAVE_CGAL="$with_cgal"
])

AM_CONDITIONAL([HAVE_CGAL], [test "x$HAVE_CGAL" = xyes])

## Lisp ##
AC_ARG_WITH([common-lisp],
            [AS_HELP_STRING([--with-common-lisp=LISP], [Set the lisp system to use, or set to 'no' to disable])],
            [],
            [with_common_lisp="$with_ompl"])
AS_IF([test "x$COMMON_LISP" = x -a "x$with_common_lisp" != xno], [
AC_CHECK_PROGS([COMMON_LISP], [sbcl])

AC_DEFUN([CL_CHECK_SYSTEM],
         [AC_MSG_CHECKING([for Common Lisp system $1])
          AS_IF(["$srcdir/share/check-clpkg.sh" "$COMMON_LISP" "$srcdir" ":$1"],
                [AC_MSG_RESULT([yes])
                 $2],
                [AC_MSG_RESULT([no])
                 $3])])

AC_DEFUN([CL_CHECK_SYSTEM_ERROR],
         [CL_CHECK_SYSTEM([$1],[],[AC_MSG_ERROR([$1 not found])])])

CL_CHECK_SYSTEM([quicklisp],[],
                [AC_MSG_WARN([Quicklisp not found: Common Lisp dependencies may not load])])
CL_CHECK_SYSTEM_ERROR([alexandria])
CL_CHECK_SYSTEM_ERROR([cffi])
CL_CHECK_SYSTEM_ERROR([cffi-grovel])
CL_CHECK_SYSTEM_ERROR([cxml])
CL_CHECK_SYSTEM_ERROR([cl-ppcre])
CL_CHECK_SYSTEM_ERROR([sycamore])
])

AC_SUBST([COMMON_LISP])
AM_CONDITIONAL([HAVE_COMMON_LISP],[test "x$COMMON_LISP" != "x"])


#############
## SUMMARY ##
#############
AC_MSG_NOTICE([===============================])
AC_MSG_NOTICE([= AMINO CONFIGURATION SUMMARY =])
AC_MSG_NOTICE([===============================])
AC_MSG_NOTICE([prefix:            $prefix])
AC_MSG_NOTICE([C compiler:        $CC])
AC_MSG_NOTICE([C++ compiler:      $CXX])
AC_MSG_NOTICE([Use Fortran        $with_fortran])
AC_MSG_NOTICE([Use Common Lisp:   $with_common_lisp])
AC_MSG_NOTICE([Common Lisp:       $COMMON_LISP])
AS_IF([test "x$with_fortran" = xyes],[
AC_MSG_NOTICE([Fortran compiler:  $FC])]
)
AC_MSG_NOTICE([Use Java           $with_java])
AS_IF([test "x$with_java" = xyes], [
AC_MSG_NOTICE([Java compiler:     $JAVAC])]
AC_MSG_NOTICE([Build Java Lib:    $BUILD_JAVA])
)
AC_MSG_NOTICE([Use OpenGL:        $with_opengl])
AC_MSG_NOTICE([Use SDL2:          $HAVE_SDL2])
AC_MSG_NOTICE([Use Spacenav:      $HAVE_SPACENAV])
AC_MSG_NOTICE([Use FCL:           $HAVE_FCL])
AC_MSG_NOTICE([Use OMPL:          $HAVE_OMPL])
AC_MSG_NOTICE([Use LP_SOLVE:      $HAVE_LPSOLVE])
AC_MSG_NOTICE([Use CLP:           $HAVE_CLP])
AC_MSG_NOTICE([Use GLPK:          $HAVE_GLPK])
AC_MSG_NOTICE([Use CGAL:          $HAVE_CGAL])


AS_IF([test "x$CC" = "x"],
      [AC_MSG_ERROR([No C compiler detected])])
AS_IF([test "x$CXX" = "x"],
      [AC_MSG_ERROR([No C++ compiler detected])])
AS_IF([test "x$FC" = "x"],
      [AC_MSG_ERROR([No Fortran compiler detected])])


AC_CONFIG_FILES([Makefile include/amino/cblas_order.h lisp/Makefile share/amino.pc share/amino_gl.pc share/amino_collision.pc share/amino_planning.pc])
AC_OUTPUT
