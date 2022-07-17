#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.65])

AC_INIT([amino], [1.0-beta2], [ndantam@mines.edu])

HEADER_VERSION=1.0
includedir=${includedir}/amino-$HEADER_VERSION
AC_SUBST([includedir])
AC_SUBST([HEADER_VERSION])

AM_INIT_AUTOMAKE([subdir-objects -Wall -Wno-portability])
m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])

AC_CONFIG_SRCDIR([include/amino.h])
AC_CONFIG_HEADERS([config.h])

AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_USE_SYSTEM_EXTENSIONS
AC_PROG_CC_C99
AC_PROG_CXX
AC_PROG_FC
AC_PROG_F77
AM_PROG_CC_C_O
AM_PROG_AR
AC_PROG_LIBTOOL
AM_PATH_LISPDIR

# Check for Maxima
AC_ARG_WITH([maxima],
            AS_HELP_STRING([--without-maxima], [Do not run maxima]),
            [],
            [with_maxima=yes])
AS_IF([test "x$with_maxima" = xyes],
      [AC_CHECK_PROGS([MAXIMA], [maxima])])
AM_CONDITIONAL([HAVE_MAXIMA], [test "x$MAXIMA" != x])

# Used for testing
AC_CHECK_PROGS([VALGRIND], [valgrind])
AM_CONDITIONAL([HAVE_VALGRIND], [test "x$VALGRIND" != x])


# Enable C++ 11
m4_ifdef([AX_CXX_COMPILE_STDCXX_11],
         [AX_CXX_COMPILE_STDCXX_11],
         [m4_ifdef([AX_CXX_COMPILE_STDCXX_0X],
                   [AX_CXX_COMPILE_STDCXX_0X])])



# Check the OS
UNAME=`uname`

# Use Accelerate Framework by default on MacOSX
AC_ARG_WITH([accelerate-framework],
            AS_HELP_STRING([--without-accelerate-framework], [Disable accelerate framework]),
            [],
            [with_accelerate_framework=auto])

AS_IF([test "x$with_accelerate_framework" = xauto -a "x$UNAME" = xDarwin],
      [AC_MSG_NOTICE([Automatically enabling accelerate framework on Darwin])
       with_accelerate_framework=yes])

AS_IF([test "x$with_accelerate_framework" = xyes],
      [CFLAGS="$CFLAGS -framework Accelerate"
       LDFLAGS="$LDFLAGS -framework Accelerate"])

# Checks for libraries.

AC_SEARCH_LIBS([dgemm_],[blas],[true],
                 [AC_MSG_ERROR([BLAS is required.])])
AC_SEARCH_LIBS([dgesvd_],[lapack],[true],
                 [AC_MSG_ERROR([LAPACK is required.])])

AC_CHECK_HEADERS([stdatomic.h])

AC_SEARCH_LIBS([sqrt], [m])
AC_SEARCH_LIBS([dlopen], [dl])
AC_SEARCH_LIBS([pthread_once], [pthread])

HAVE_TIME=no
AH_TEMPLATE([HAVE_CLOCK_GETTIME],
            [Define presense of clock_gettime() call])
AH_TEMPLATE([HAVE_MACH_ABSOLUTE_TIME],
            [Define presense of mach_absolute_time() call])

AS_IF([test xyes != "x$HAVE_TIME"],
      [AC_SEARCH_LIBS([clock_gettime], [rt],
                     [AC_DEFINE([HAVE_CLOCK_GETTIME])
                      HAVE_TIME=yes])])

AS_IF([test xyes != "x$HAVE_TIME"],
      [AC_SEARCH_LIBS([mach_absolute_time], [],
                     [AC_DEFINE([HAVE_MACH_ABSOLUTE_TIME])
                      HAVE_TIME=yes])])

AS_IF([test xyes != "x$HAVE_TIME"],
      [AC_MSG_ERROR([Could not find suitable time function.])])

# Check for cblas.h
AC_CHECK_HEADERS([cblas.h], [INCLUDE_CBLAS_H='#include <cblas.h>'])
AS_IF([test x = "x$INCLUDE_CBLAS_H"],
      [AC_CHECK_HEADERS([cblas_openblas.h], [INCLUDE_CBLAS_H='#include <cblas_openblas.h>'])])
AS_IF([test x = "x$INCLUDE_CBLAS_H"],
      [AC_MSG_ERROR([cblas.h is required.])])
AC_SUBST([INCLUDE_CBLAS_H])

# Check for cblas_blas enum
AC_MSG_CHECKING([CBLAS_LAYOUT])
AC_TRY_COMPILE([$INCLUDE_CBLAS_H
               ],
               [CBLAS_LAYOUT test_var;],
               [AC_MSG_RESULT([yes])
                aa_cblas_order_type=CBLAS_LAYOUT],
               [AC_MSG_RESULT([no])])

AS_IF([test x = "x$aa_cblas_order_type"],
[
AC_MSG_CHECKING([enum CBLAS_ORDER])
AC_TRY_COMPILE([$INCLUDE_CBLAS_H
               ],
               [enum CBLAS_ORDER test_var;],
               [AC_MSG_RESULT([yes])
                aa_cblas_order_type="enum CBLAS_ORDER"],
               [AC_MSG_RESULT([no])])
])

AC_SUBST([aa_cblas_order_type])


AC_MSG_CHECKING([CBLAS_TRANSPOSE])
AC_TRY_COMPILE([$INCLUDE_CBLAS_H
               ],
               [CBLAS_TRANSPOSE test_var;],
               [AC_MSG_RESULT([yes])
                aa_cblas_transpose_type=CBLAS_TRANSPOSE],
               [AC_MSG_RESULT([no])])

AS_IF([test x = "x$aa_cblas_transpose_type"],
[
AC_MSG_CHECKING([enum CBLAS_TRANSPOSE])
AC_TRY_COMPILE([$INCLUDE_CBLAS_H
               ],
               [enum CBLAS_TRANSPOSE test_var;],
               [AC_MSG_RESULT([yes])
                aa_cblas_transpose_type="enum CBLAS_TRANSPOSE"],
               [AC_MSG_RESULT([no])])
])
AC_SUBST([aa_cblas_transpose_type])


m4_ifdef([AX_CHECK_COMPILE_FLAG],
         [AC_DEFUN([CHECK_COMPILE_FLAG],
                   [AX_CHECK_COMPILE_FLAG([$1],[$2],[$3])])])
m4_ifdef([AX_CHECK_COMPILER_FLAG],
         [AC_DEFUN([CHECK_COMPILE_FLAG],
                   [AX_CHECK_COMPILER_FLAG([$1],[$2],[$3])])])

m4_ifdef([CHECK_COMPILE_FLAG], [],
         [AC_MSG_WARN([Unable to test compiler flags])
          AC_DEFUN([CHECK_COMPILE_FLAG],
                   [$3])])

AC_DEFUN([APPEND_CFLAG],
          [CHECK_COMPILE_FLAG([$1], [AM_CFLAGS="$1 $AM_CFLAGS"])])
AC_DEFUN([APPEND_CXXFLAG],
         [CHECK_COMPILE_FLAG([$1], [AM_CXXFLAGS="$1 $AM_CXXFLAGS"])])
AC_DEFUN([APPEND_FCFLAG],
         [CHECK_COMPILE_FLAG([$1], [AM_FCFLAGS="$1 $AM_FCFLAGS"])])


dnl m4_ifdef([APPEND_FCFLAG],[],
dnl          [AC_DEFUN([APPEND_FCFLAG],[])])
dnl m4_ifdef([APPEND_CFLAG],[],
dnl          [AC_DEFUN([APPEND_CFLAG],[])])
dnl m4_ifdef([APPEND_CXXFLAG],[],
dnl          [AC_DEFUN([APPEND_CXXFLAG],[])])


# Fortran Checks
# Use fortran compiler if it exists.  Otherwise fallback on f2c.

AC_ARG_WITH([fortran],
            AS_HELP_STRING([--without-fortran], [Disable fortran compiler]),
            [],
            [with_fortran=auto])


AS_IF([test "x$with_fortran" != xno -a "x$FC" != x],
[
AC_DEFINE([USE_FORTRAN],[1],[Use the fortran compiler])
AC_LANG([Fortran])

AC_SEARCH_LIBS([__iso_c_binding_c_f_pointer_l4],[gfortran])

AC_LANG([Fortran])
APPEND_FCFLAG([-fimplicit-none])
APPEND_FCFLAG([-ffree-line-length-none])
APPEND_FCFLAG([-Wall])
APPEND_FCFLAG([-pedantic])
APPEND_FCFLAG([-Wextra])
dnl APPEND_FCFLAG([-Wconversion])
dnl APPEND_FCFLAG([-Wconversion-extra])
APPEND_FCFLAG([-Wshadow])
APPEND_FCFLAG([-Wimplicit-interface])
APPEND_FCFLAG([-Wimplicit-procedure])
APPEND_FCFLAG([-Wunderflow])
APPEND_FCFLAG([-Warray-temporaries])
APPEND_FCFLAG([-Wtabs])
APPEND_FCFLAG([-Wunused-parameter])
APPEND_FCFLAG([-Wintrinsic-shadow])

])

AC_CHECK_PROGS([F2C], [f2c])
AC_LANG([C])
AC_CHECK_HEADERS([f2c.h])

AS_IF([test "x$with_fortran" = xyes -a "x$FC" = x],
      [AC_MSG_ERROR([No Fortran compiler detected])])
AS_IF([test "x$with_fortran" = xno -a "x$F2C" = x],
      [AC_MSG_ERROR([No F2C detected])])
AS_IF([test "x$FC" = x -a "x$F2C" = x],
      [AC_MSG_ERROR([No Fortran compiler or F2C detected])])

AM_CONDITIONAL([HAVE_F2C],   [test "x$F2C" != x])
AM_CONDITIONAL([USE_FORTRAN], [test "x$with_fortran" != xno -a "x$FC" != x])

# Doxygen
AC_ARG_WITH([doxygen],
            AS_HELP_STRING([--without-doxygen], [Disable doxygen support]),
            [],
            [with_doxygen=auto])
AS_IF([test "x$with_doxygen" != xno],
      [AC_CHECK_PROGS([DOXYGEN],[doxygen])])
AS_IF([test "x$with_doxygen" = xyes -a "x$DOXYGEN" = x],
      [AC_MSG_ERROR([Doxygen not found])])
AM_CONDITIONAL([HAVE_DOXYGEN], [test "x$DOXYGEN" != x])



AC_ARG_ENABLE([man],
              [AS_HELP_STRING([--disable-man], [Do not build man pages])],
              [],
              [enable_man="yes"])

AS_IF([test "x$enable_man" = xyes -a "x$build" = "x$host"],
      [AC_CHECK_PROGS([HELP2MAN], [help2man])
       AC_CHECK_PROGS([GROFF], [groff])])

AM_CONDITIONAL([HAVE_HELP2MAN], [test x$HELP2MAN != x])
AM_CONDITIONAL([HAVE_GROFF], [test x$GROFF != x])

AC_SUBST([BASE_CFLAGS],[$CFLAGS])

# Enable maximum warnings
m4_ifdef([AX_CFLAGS_WARN_ALL],
         [AX_CFLAGS_WARN_ALL])

dnl APPEND_FLAG( [FLAG] )
dnl Check if FLAG is supported, and if so, add it




# Check for C flags
AC_LANG([C])
APPEND_CFLAG([-Wextra])
dnl APPEND_CFLAG([-Wpedantic])
APPEND_CFLAG([-Wconversion])
APPEND_CFLAG([-Wpointer-arith])
APPEND_CFLAG([-Wfloat-equal])
APPEND_CFLAG([-Wshadow])
APPEND_CFLAG([-Wwrite-strings])
APPEND_CFLAG([-Wc++-compat])
CHECK_COMPILE_FLAG([-Wno-unused-function],
                   [HAVE_WNO_UNUSED_FUNCTION=yes],
                   [HAVE_WNO_UNUSED_FUNCTION=no])
CHECK_COMPILE_FLAG([-Wno-deprecated-declarations],
                   [HAVE_WNO_NO_DEPRECATED_DECLARATIONS_C=yes],
                   [HAVE_WNO_NO_DEPRECATED_DECLARATIONS_C=no])

# Check for C++ flags
AC_LANG([C++])
APPEND_CXXFLAG([-Wextra])
dnl APPEND_CXXFLAG([-Wpedantic])
APPEND_CXXFLAG([-Wconversion])
APPEND_CXXFLAG([-Wpointer-arith])
APPEND_CXXFLAG([-Wfloat-equal])
APPEND_CXXFLAG([-Wshadow])
APPEND_CXXFLAG([-Wwrite-strings])
CHECK_COMPILE_FLAG([-Wno-conversion],
                   [HAVE_WNO_CONVERSION=yes],
                   [HAVE_WNO_CONVERSION=no])
CHECK_COMPILE_FLAG([-Wno-deprecated-declarations],
                   [HAVE_WNO_NO_DEPRECATED_DECLARATIONS_CXX=yes],
                   [HAVE_WNO_NO_DEPRECATED_DECLARATIONS_CXX=no])

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


AS_IF([test "x$HAVE_WNO_NO_DEPRECATED_DECLARATIONS_C" = xyes],
      [TEST_CFLAGS_APPEND="$TEST_CFLAGS_APPEND -Wno-deprecated-declarations"])
AC_SUBST([TEST_CFLAGS_APPEND])

AS_IF([test "x$HAVE_WNO_NO_DEPRECATED_DECLARATIONS_CXX" = xyes],
      [TEST_CXXFLAGS_APPEND="$TEST_CXXFLAGS_APPEND -Wno-deprecated-declarations"])
AC_SUBST([TEST_CXXFLAGS_APPEND])


## Java ##

AC_ARG_WITH([java],
            AS_HELP_STRING([--with-java], [Enable java support]),
            [],
            [with_java=no])


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
       m4_ifdef([AX_JNI_INCLUDE_DIR],
                [AX_JNI_INCLUDE_DIR],
                [JAVAC=""])
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


## Lex/Flex ##
AC_ARG_WITH([lex],
            AS_HELP_STRING([--without-lex], [Do not run lex/flex]),
            [],
            [with_lex=yes])
AS_IF([test "x$with_lex" != xyes],
      [AM_PROG_LEX])

AS_IF([test "x$LEX" != x -a "x$with_lex" = xyes],
      [HAVE_LEX=yes],
      [HAVE_LEX=no])

AM_CONDITIONAL([HAVE_LEX], [test "x$HAVE_LEX" = xyes])

AS_IF([test "x$HAVE_WNO_UNUSED_FUNCTION" = xyes],
      [LEX_CFLAGS_APPEND="$LEX_CFLAGS_APPEND -Wno-unused-function"])
AC_SUBST([LEX_CFLAGS_APPEND])


## Blender ##

AC_PATH_PROGS([BLENDER], [blender])

AS_IF([test "x$BLENDER" = "x"],
  [],
  [
      # MacOSX is unable to execute blender via a symbolic link.
      # Resolve the symbolic link.
      AC_CHECK_PROGS([READLINK], [greadlink readlink])
      AC_MSG_CHECKING([blender truename])
      AS_IF([test "x$READLINK" != x],
            [BLENDER=`$READLINK -f "$BLENDER"`
             AC_MSG_RESULT([$BLENDER])],
            [AC_MSG_RESULT([unknown])
             AC_MSG_WARN([Could not resolve potential blender symlinks.  Blender may not run.])])

      # Try to load a collada object
      AC_MSG_CHECKING([blender COLLADA support])
      rm -rf colladatest.obj
      "$BLENDER" -b -P $srcdir/share/exec/meshconv -- $srcdir/colladatest.dae -o colladatest.obj > /dev/null 2>/dev/null
      AS_IF([test -e colladatest.obj],
            [AC_MSG_RESULT([yes])
             HAVE_BLENDER_COLLADA=yes
            ],
            [AC_MSG_RESULT([no])])
  ])


## OpenGL ##
AC_ARG_WITH([opengl],
            [AS_HELP_STRING([--without-opengl], [Disable OpenGL support])],
            [],
            [with_opengl=yes])


AS_IF([test "x$UNAME" = xDarwin],
[
  AS_IF([test "x$GL_CFLAGS" = x],
        [export GL_CFLAGS="-framework OpenGL"
         AC_MSG_NOTICE([Guessing GL_CFLAGS for Darwin: $GL_CFLAGS])
        ])
  AS_IF([test "x$GL_LIBS" = x],
        [export GL_LIBS="-framework OpenGL"
         AC_MSG_NOTICE([Guessing GL_LIBS for Darwin: $GL_LIBS])
        ])
])

AS_IF([test "x$with_opengl" = xyes], [
  PKG_CHECK_MODULES([GL],
                    [gl],
                    [HAVE_GL=yes])
  ]
)

AC_SUBST([GL_CFLAGS])
AC_SUBST([GL_LIBS])

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
  ]
)

AC_SUBST([SDL2_CFLAGS])
AC_SUBST([SDL2_LIBS])

## AMINO GL ##
AS_IF([test "x$HAVE_SDL2" = xyes -a "x$HAVE_GL" = xyes],
      [AMINO_GL=yes],
      [AMINO_GL=no])
AC_SUBST([AMINO_GL])
AM_CONDITIONAL([AMINO_GL], [test "x$AMINO_GL" = xyes])

## Eigen3 ###
PKG_CHECK_MODULES([EIGEN],
                  [eigen3],
                  [HAVE_EIGEN=yes],
                  [HAVE_EIGEN=no])
AC_SUBST([EIGEN_CFLAGS])
AC_SUBST([EIGEN_LIBS])

AS_IF([test "x$HAVE_EIGEN" = xyes -a "x$HAVE_WNO_CONVERSION" = xyes],
      [EIGEN_CFLAGS_APPEND="$EIGEN_CFLAGS_APPEND -Wno-conversion"])
AC_SUBST([EIGEN_CFLAGS_APPEND])

AM_CONDITIONAL([HAVE_EIGEN], [test "x$HAVE_EIGEN" = xyes])

## FCL ##
AC_ARG_WITH([fcl],
            [AS_HELP_STRING([--without-fcl], [Disable support for the Flexible Collision Library (FCL)])],
            [],
            [with_fcl=auto])

HAVE_FCL=disabled
AS_IF([test "x$with_fcl" != xno], [
  PKG_CHECK_MODULES([FCL],
                    [fcl >= 0.6.0],
                    [HAVE_FCL=yes],
                    [HAVE_FCL=no])
])

AS_IF([test "x$with_fcl" = xyes -a "x$HAVE_FCL" != xyes],
      [AC_MSG_ERROR([FCL not found])])

# FCL pointer type
# std::shared_ptr
dnl AS_IF([test "x$HAVE_FCL" = xyes],
dnl [
dnl CXXFLAGS_SAVED="$CXXFLAGS"
dnl CXXFLAGS="$CXXFLAGS $FCL_CFLAGS"

dnl AC_MSG_CHECKING([FCL uses std::shared_ptr ])
dnl AC_LANG([C++])
dnl AC_TRY_COMPILE([#include <fcl/fcl.h>
dnl                ],
dnl                [std::shared_ptr<fcl::CollisionGeometryf> a( new fcl::Boxf(1,1,1) );
dnl                fcl::CollisionObjectf* obj = new fcl::CollisionObjectf(a, fcl::Transform3f());
dnl                ],
dnl                [AA_FCL_SHARED_PTR="std::shared_ptr"
dnl                AC_MSG_RESULT(["yes"])],
dnl                [AC_MSG_RESULT([no"])]
dnl                )

dnl # boost::shared_ptr
dnl AS_IF([test "x$AA_FCL_SHARED_PTR" = x],
dnl       [AC_MSG_CHECKING([FCL uses boost::shared_ptr ])
dnl        AC_LANG([C++])
dnl        AC_TRY_COMPILE([#include <fcl/fcl.h>
dnl                       ],
dnl                       [boost::shared_ptr<fcl::CollisionGeometryf> a( new fcl::Boxf(1,1,1) );
dnl                        fcl::CollisionObject* obj = new fcl::CollisionObject(a, fcl::Transform3f());
dnl                        ],
dnl                [AA_FCL_SHARED_PTR="boost::shared_ptr"
dnl                 AC_MSG_RESULT(["yes"])],
dnl                [AC_MSG_RESULT(["no"])]
dnl                )
dnl       ])

dnl CXXFLAGS="$CXXFLAGS_SAVED"

dnl AS_IF([test "x$AA_FCL_SHARED_PTR" = x],
dnl       [AC_MSG_ERROR([Could not determine FCL shared_ptr type ])])


dnl AC_DEFINE_UNQUOTED([AA_FCL_SHARED_PTR],[$AA_FCL_SHARED_PTR],[Type for FCL shared pointers.])

dnl ]) # end HAVE_FCL


dnl # FCL extra linking
dnl # boost_system is sometimes required
dnl AC_DEFUN([FCL_LINK],
dnl          [
dnl            AS_IF([test "x$FCL_LINKED" != xyes],
dnl            [
dnl              AC_LANG([C++])
dnl              AC_MSG_CHECKING([FCL extra libs "$1"])
dnl              CXXFLAGS_SAVED="$CXXFLAGS"
dnl              LIBS_SAVED="$LIBS"

dnl              LIBS="$1 $FCL_LIBS $LIBS"
dnl              CXXFLAGS="$CXXFLAGS $FCL_CFLAGS"

dnl              AC_LINK_IFELSE(
dnl                [AC_LANG_PROGRAM([
dnl                                   #include <fcl/broadphase/broadphase.h>
dnl                                   #include <fcl/BVH/BVH_model.h>
dnl                                 ],
dnl                                 [fcl::BroadPhaseCollisionManager *manager =
dnl                                  new fcl::DynamicAABBTreeCollisionManager();])],
dnl                [AC_MSG_RESULT([yes])
dnl                 FCL_LINKED=yes
dnl                 FCL_LIBS="$FCL_LIBS $1"
dnl                ],
dnl                [AC_MSG_RESULT([no])])

dnl              CXXFLAGS="$CXXFLAGS_SAVED"
dnl              LIBS="$LIBS_SAVED"
dnl            ])
dnl          ])

dnl AS_IF([test "x$HAVE_FCL" = xyes],
dnl [
dnl FCL_LINK([])
dnl FCL_LINK([-lboost_system])
dnl FCL_LINK([-lboost_system-mt])

dnl AS_IF([test "x$FCL_LINKED" != xyes],
dnl       [AC_MSG_ERROR([Could not link against FCL])])

dnl ])

AC_SUBST([FCL_CFLAGS])
AC_SUBST([FCL_LIBS])
AC_SUBST([HAVE_FCL])
AM_CONDITIONAL([HAVE_FCL], [test "x$HAVE_FCL" = xyes])

## OCTOMAP ##
AC_LANG([C++])
AC_ARG_WITH([octomap],
            [AS_HELP_STRING([--without-octomap], [Disable support for Octrees])],
            [],
            [with_octomap="auto"])

HAVE_OCTOMAP=disabled
AS_IF([test "x$with_octomap" != xno], [
  PKG_CHECK_MODULES([OCTOMAP],
                    [octomap],
                    [HAVE_OCTOMAP=yes],
                    [HAVE_OCTOMAP=no])

 AC_SUBST([OCTOMAP_CFLAGS])
 AC_SUBST([OCTOMAP_LIBS])
 AC_SUBST([OCTOMAP_IO])

 AM_CONDITIONAL([HAVE_OCTOMAP], [test "x$HAVE_OCTOMAP" = xyes])
])


## OMPL ##
AC_LANG([C++])
AC_ARG_WITH([ompl],
            [AS_HELP_STRING([--without-ompl], [Disable support for the Open Motion Planning Library (OMPL)])],
            [],
            [with_ompl="auto"])

HAVE_OMPL=disabled
AS_IF([test "x$with_ompl" != xno], [
  PKG_CHECK_MODULES([OMPL],
                    [ompl],
                    [HAVE_OMPL=yes],
                    [HAVE_OMPL=no])
])

AS_IF([test "x$with_ompl" = xyes -a "x$HAVE_OMPL" != xyes],
      [AC_MSG_ERROR([OMPL not found])])

# check that we also have FCL
AS_IF([test "x$HAVE_OMPL" = xyes -a "x$HAVE_FCL" != xyes],
[
  AS_IF([test "x$with_ompl" = xyes],
        [AC_MSG_ERROR([OMPL support requires FCL])],
        [AC_MSG_WARN([OMPL support requires FCL, disabling])
         HAVE_OMPL=disabled])
])

# check CFLAGS for OMPL
AS_IF([test "x$HAVE_OMPL" = xyes -a "x$HAVE_WNO_CONVERSION" = xyes],
      [OMPL_CFLAGS_APPEND="$OMPL_CFLAGS_APPEND -Wno-conversion"])
AC_SUBST([OMPL_CFLAGS_APPEND])

# check for OMPL type templates
AS_IF([test "x$HAVE_OMPL" = xyes],
[
  AC_LANG([C++])
  CPPFLAGS_SAVED="$CPPFLAGS"
  CPPFLAGS="$CPPFLAGS $OMPL_CFLAGS $CXXFLAGS"


  AC_MSG_CHECKING([bundled TypedSpaceInformation.h])
  AC_COMPILE_IFELSE([AC_LANG_SOURCE([
                               #include <ompl/base/TypedSpaceInformation.h>
                               int main() { return 0; }
                      ])
                    ],
                    [
                      AC_MSG_RESULT([yes])
                      ompl_typed_header=yes
                    ],
                    [
                      AC_MSG_RESULT([no])
                    ])

  # Check boost::shared_ptr compat header
  AS_IF([test "x$ompl_typed_header" != xyes],
        [
          AC_MSG_CHECKING([compat TypedSpaceInformation.h using boost::shared_ptr])
          CPPFLAGS="$CPPFLAGS_SAVED $OMPL_CFLAGS $CXXFLAGS -I$srcdir/include/amino/ompl-compat"
          AC_COMPILE_IFELSE([AC_LANG_SOURCE([
#include <ompl/base/TypedSpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
using namespace ::ompl::base;
typedef TypedSpaceInformation<RealVectorStateSpace> RVSpaceInformation;
int main() {
    RVSpaceInformation::SpacePtr ss(new RealVectorStateSpace(1));
    TypedSpaceInformation<RealVectorStateSpace> x(ss);
    return 0;
}
                               ])
                             ],
                            [
                              AC_MSG_RESULT([yes])
                              ompl_typed_header=yes
                              OMPL_CFLAGS="$OMPL_CFLAGS -I$srcdir/include/amino/ompl-compat"
                            ],
                            [AC_MSG_RESULT([no])])
       ])


  # Check std::shared_ptr compat header
  AS_IF([test "x$ompl_typed_header" != xyes],
        [
          AC_MSG_CHECKING([compat TypedSpaceInformation.h using std::shared_ptr])
          CPPFLAGS="$CPPFLAGS_SAVED $OMPL_CFLAGS $CXXFLAGS -I$srcdir/include/amino/ompl-compat2"
          AC_COMPILE_IFELSE([AC_LANG_SOURCE([
#include <ompl/base/TypedSpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
using namespace ::ompl::base;
typedef TypedSpaceInformation<RealVectorStateSpace> RVSpaceInformation;
int main() {
    RVSpaceInformation::SpacePtr ss(new RealVectorStateSpace(1));
    TypedSpaceInformation<RealVectorStateSpace> x(ss);
    return 0;
}
                              ])
                            ],
                            [
                              AC_MSG_RESULT([yes])
                              ompl_typed_header=yes
                              OMPL_CFLAGS="$OMPL_CFLAGS -I$srcdir/include/amino/ompl-compat2"
                            ],
                            [AC_MSG_RESULT([no])])
       ])


  AS_IF([test "x$ompl_typed_header" != xyes],
        [AC_MSG_ERROR([Could not compile with OMPL TypedSpaceInformation])])


  dnl AC_CHECK_HEADERS([ompl/base/TypedSpaceInformation.h],
  dnl                  [],
  dnl                  [
  dnl                  AC_MSG_NOTICE([Including OMPL compatibility headers])
  dnl                  OMPL_CFLAGS="$OMPL_CFLAGS -I$srcdir/include/amino/ompl-compat2"])

  CPPFLAGS="$CPPFLAGS_SAVED"
])


# Workaround bug in ompl.pc
# https://github.com/ompl/ompl/pull/886
OMPL_LIBS=$(echo $OMPL_LIBS | sed -e 's/-L_link_dirs-NOTFOUND//')

AC_SUBST([OMPL_CFLAGS])
AC_SUBST([OMPL_LIBS])
AC_SUBST([HAVE_OMPL])
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

AC_SUBST([HAVE_LPSOLVE])
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

AC_SUBST([HAVE_GLPK])
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

AC_SUBST([HAVE_CLP])
AM_CONDITIONAL([HAVE_CLP], [test "x$HAVE_CLP" = xyes])
AH_TEMPLATE([HAVE_CLP],
            [Define presense of CLP library])
AS_IF([test "x$HAVE_CLP" = xyes], [AC_DEFINE([HAVE_CLP])])


## NLOPT ##
AC_LANG([C])
# check header
AC_CHECK_HEADERS([nlopt.h], [HAVE_NLOPT=yes], [HAVE_NLOPT=no])
# check library
AS_IF([test "x$HAVE_NLOPT" = xyes], [
  AC_SEARCH_LIBS([nlopt_create],[nlopt],
                 [HAVE_NLOPT=yes],
                 [HAVE_NLOPT=no])
])
# Result
AC_SUBST([HAVE_NLOPT])
AM_CONDITIONAL([HAVE_NLOPT], [test "x$HAVE_NLOPT" = xyes])
AH_TEMPLATE([HAVE_NLOPT],
            [Define presense of NLOPT library])
AS_IF([test "x$HAVE_NLOPT" = xyes],
      [AC_DEFINE([HAVE_NLOPT])],
      [AC_MSG_ERROR([NLopt is required.])])

## CGAL ##
AC_ARG_WITH([cgal],
            [AS_HELP_STRING([--enable-cgal], [Disable support for CGAL])],
            [],
            [with_cgal=no])

dnl AC_LANG([C++])
dnl AS_IF([test "x$with_cgal" = xauto], [
dnl   AC_CHECK_HEADERS([CGAL/basic.h], [HAVE_CGAL=yes], [HAVE_CGAL=no])
dnl ],[
dnl   HAVE_CGAL="$with_cgal"
dnl ])

AC_SUBST([HAVE_CGAL])
AM_CONDITIONAL([HAVE_CGAL], [test "x$with_cgal" = xyes])

## Lisp ##
AC_ARG_WITH([common-lisp],
            [AS_HELP_STRING([--with-common-lisp=LISP], [Set the lisp system to use, or set to 'no' to disable])],
            [],
            [with_common_lisp="auto"])

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

CL_CHECK_SYSTEM([quicklisp],
                [HAVE_QUICKLISP=yes],
                [AC_MSG_WARN([Quicklisp not found: Common Lisp dependencies may not load])])

AS_IF([test "x$HAVE_QUICKLISP" != xyes -a "x$with_common_lisp=auto"],
      [COMMON_LISP=""
       AC_MSG_NOTICE([Disabling common lisp support.])])

])

AS_IF([test "x$COMMON_LISP" != x],
      [CL_CHECK_SYSTEM_ERROR([cffi-grovel])])

dnl CL_CHECK_SYSTEM_ERROR([alexandria])
dnl CL_CHECK_SYSTEM_ERROR([cffi])
dnl CL_CHECK_SYSTEM_ERROR([cxml])
dnl CL_CHECK_SYSTEM_ERROR([cl-ppcre])
dnl CL_CHECK_SYSTEM_ERROR([sycamore])


AC_SUBST([COMMON_LISP])
AM_CONDITIONAL([HAVE_COMMON_LISP],[test "x$COMMON_LISP" != "x"])

dnl AS_IF([test "x$COMMON_LISP" != "x" -a "x$SBCL_HOME" = x],
dnl [
dnl AC_MSG_CHECKING([SBCL_HOME])
dnl SBCL_HOME=`$COMMON_LISP --noinform --non-interactive --eval '(format t "~A" (sb-posix:getenv "SBCL_HOME"))'`
dnl AC_MSG_RESULT(["$SBCL_HOME"])
dnl AC_SUBST([SBCL_HOME])
dnl ])

## PYTHON ##


AC_ARG_WITH([python],
            [AS_HELP_STRING([--with-python],
                            [Enable/disable python suport])],
            [],
            [with_python="auto"])

AS_IF([test "x$with_python" != xno], [
  AM_PATH_PYTHON([3.7],
                 ,
                 [AS_IF([test "x$with_python" = xyes],
                       [AC_MSG_ERROR([No python detected])])
                 ])
])

AM_CONDITIONAL([HAVE_PYTHON], [test "x$PYTHON" != "x" -a "x$PYTHON" != "x:"])

AC_CHECK_PROGS([PYLINT],[pylint3])
AM_CONDITIONAL([HAVE_PYLINT], [test "x$PYLINT" != x])

###########
## Demos ##
###########

AC_ARG_ENABLE([demos],
              [AS_HELP_STRING([--enable-demos], [Build demos])],
              [],
              [enable_demos=auto])

# Auto-configure demo builds
AS_IF([test "x$enable_demos" = xauto],
[

AS_IF(
      # default to disabled demos on Darwin due to linking issues
      [test "x$UNAME" = xDarwin],
      [AC_MSG_NOTICE([Disabling demos on Darwin / Mac OS X])
       enable_demos=no],

      # Enable demos when required dependencies exist
      [test "x$COMMON_LISP" != "x" -a "x$HAVE_GL" = xyes -a "x$HAVE_SDL2" = xyes],
      [AC_MSG_NOTICE([Automatically enabling demos])
       enable_demos=yes],

      # Disable demos
      [AC_MSG_NOTICE([Disabling demos due to missing dependencies])
       enable_demos=no]
     )
])

AM_CONDITIONAL([BUILD_DEMOS], [test "x$enable_demos" = "xyes"])

######################
## URDF Robot Demos ##
######################

ENABLED_DEMOS=""

AC_DEFUN([DEF_DEMO],
[
AC_ARG_ENABLE([demo-$1],
              [AS_HELP_STRING([--enable-demo-$1], [Build the $1 demo])],
              [],
              [])

AS_IF([test "x$enable_demo_$1" = "xyes"],
      [ENABLED_DEMOS="$1 $ENABLED_DEMOS"])

AM_CONDITIONAL([BUILD_DEMO_$2], [test "x$enable_demo_$1" = "xyes"])

])

AC_ARG_ENABLE([demo-all-urdf],
              [AS_HELP_STRING([--enable-demo-all-urdf], [Build all the URDF demos])],
              [
                enable_demo_baxter="$enable_demo_all_urdf"
                enable_demo_ur10="$enable_demo_all_urdf"
                enable_demo_schunk="$enable_demo_all_urdf"
                enable_demo_mico="$enable_demo_all_urdf"
                enable_demo_jaco="$enable_demo_all_urdf"
                enable_demo_fetch="$enable_demo_all_urdf"
              ])

DEF_DEMO([baxter],[BAXTER])
DEF_DEMO([ur10],[UR10])
DEF_DEMO([schunk],[SCHUNK])
DEF_DEMO([mico],[MICO])
DEF_DEMO([jaco],[JACO])
DEF_DEMO([fetch],[FETCH])




AS_IF([test "x$CC" = "x"],
      [AC_MSG_ERROR([No C compiler detected])])
AS_IF([test "x$CXX" = "x"],
      [AC_MSG_ERROR([No C++ compiler detected])])



AC_CONFIG_FILES([ Makefile
                  submodules/sycamore/Makefile
                  submodules/cl-ntdoc/Makefile
                  include/amino/cblas_order.h
                  share/amino.pc
                  share/amino-gl.pc
                  share/amino-collision.pc
                  share/amino-planning.pc
                  lisp/config.lisp
                  Doxyfile])

AC_DEFUN([CONFIG_SCRIPT],
         [AC_CONFIG_FILES([$1],
                          [chmod +x $1])])

CONFIG_SCRIPT([demo/urdf/jaco/jaco-view])
CONFIG_SCRIPT([demo/urdf/mico/mico-view])
CONFIG_SCRIPT([demo/urdf/ur/ur10-view])
CONFIG_SCRIPT([demo/urdf/schunk/lwa4d-view])
CONFIG_SCRIPT([demo/urdf/baxter/baxter-view])
CONFIG_SCRIPT([demo/urdf/fetch/fetch-view])

CONFIG_SCRIPT([python/test/mattest.sh])
CONFIG_SCRIPT([python/test/tftest.sh])
CONFIG_SCRIPT([python/test/py_tutorial_test.sh])
CONFIG_SCRIPT([python/test/pylint.sh])

CONFIG_SCRIPT([aarxc])

AC_SUBST([AM_CFLAGS])
AC_SUBST([AM_CXXFLAGS])
AC_SUBST([AM_FCFLAGS])

AC_OUTPUT


#############
## SUMMARY ##
#############

AS_BOX([AMINO CONFIGURATION SUMMARY])

AC_MSG_NOTICE([prefix:            $prefix])
AC_MSG_NOTICE([C compiler:        $CC])
AC_MSG_NOTICE([CFLAGS:            $CFLAGS])
AC_MSG_NOTICE([C++ compiler:      $CXX])
AC_MSG_NOTICE([CXX:               $CXXFLAGS])
AC_MSG_NOTICE([Use Fortran        $with_fortran])
AS_IF([test "x$with_fortran" != xno],[
  AC_MSG_NOTICE([Fortran compiler:  $FC])
  AC_MSG_NOTICE([FCFLAGS:           $FCFLAGS])
])
AC_MSG_NOTICE([Use Java           $with_java])
AS_IF([test "x$with_java" != xno], [
  AC_MSG_NOTICE([Java compiler:     $JAVAC])
  AC_MSG_NOTICE([Build Java Lib:    $BUILD_JAVA])
])
AC_MSG_NOTICE([Common Lisp:       $COMMON_LISP])
AS_IF([test "x$with_common_lisp" != xno], [
  AC_MSG_NOTICE([Use Common Lisp:   $with_common_lisp])
])
AC_MSG_NOTICE([Use Python:        $with_python])
AS_IF([test "x$with_python" != xno], [
AC_MSG_NOTICE([Python:            $PYTHON])
])
AC_MSG_NOTICE([Use SDL2/OpenGL:   $AMINO_GL])
AC_MSG_NOTICE([Use Spacenav:      $HAVE_SPACENAV])
AC_MSG_NOTICE([Use FCL:           $HAVE_FCL])
AC_MSG_NOTICE([Use OMPL:          $HAVE_OMPL])
AC_MSG_NOTICE([Use LP_SOLVE:      $HAVE_LPSOLVE])
AC_MSG_NOTICE([Use CLP:           $HAVE_CLP])
AC_MSG_NOTICE([Use GLPK:          $HAVE_GLPK])
AC_MSG_NOTICE([Use CGAL:          $HAVE_CGAL])
AC_MSG_NOTICE([Use OCTOMAP:       $HAVE_OCTOMAP])
AC_MSG_NOTICE([Build Demos:       $enable_demos])
AC_MSG_NOTICE([URDF Demos:        $ENABLED_DEMOS])

AS_IF([test '(' "x$enable_demo_baxter" = "xyes" -o "x$enable_demo_ur10" = xyes ')' -a "x$ROS_PACKAGE_PATH" = x],
      [AC_MSG_WARN([URDF demo enabled, but no ROS_PACKAGE_PATH.  May not find URDF.])])


AS_IF([test "x$BLENDER" = x],
      [AC_MSG_WARN([Blender not found, only Wavefront OBJ meshes supported])],
      [test "x$HAVE_BLENDER_COLLADA" != xyes],
      [AC_MSG_WARN([Blender lacks COLLADA support, cannot load DAE meshes])])

AS_BOX([END SUMMARY])
