SUBDIRS = . lisp

ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I$(top_srcdir)/include -I$(top_srcdir)/src/mac
EXTRA_DIST = src/mac/gen.mac src/mac/euler.mac   \
             src/la_impl.c                       \
             src/la_mod.F90 src/la_implf.F90     \
             src/mem_mod.F90 src/mem_impl.F90    \
             src/tff.F90                         \
             src/mac/tf.mac                      \
             src/mac/aa_tf_euler.f90             \
             src/mac/ls.mac src/mac/aa_ls_se3.f90

dist_pkgdata_DATA = src/mac/amino.mac

include_HEADERS = include/amino.h include/amino.hpp
noinst_PROGRAMS =

bin_PROGRAMS =
bin_SCRIPTS = share/aarxc

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = share/amino.pc \
	share/amino_gl.pc \
	share/amino_collision.pc \
	share/amino_planning.pc

pkginclude_HEADERS = 	                \
	include/amino/mem.hpp           \
	include/amino/rx.h              \
	include/amino/tf.hpp            \
	include/amino/debug.h           \
	include/amino/test.h            \
	include/amino/endconv.h         \
	include/amino/io.h              \
	include/amino/kin.h             \
	include/amino/lapack.h          \
	include/amino/lapack_impl.h     \
	include/amino/clapack.h         \
	include/amino/clapack_impl.h    \
	include/amino/math.h            \
	include/amino/la.h              \
	include/amino/la_impl.h         \
	include/amino/mem.h             \
	include/amino/opt/opt.h         \
	include/amino/opt/qp.h          \
	include/amino/opt/lp.h          \
	include/amino/plot.h            \
	include/amino/tf.h              \
	include/amino/time.h            \
	include/amino/type.h            \
	include/amino/unit.h            \
	include/amino/cv.h              \
	include/amino/validate.h        \
	include/amino/const.h           \
	include/amino/def.h             \
	include/amino/undef.h           \
	include/amino/heap.h            \
	include/amino/mangle.h          \
	include/amino/def.F90           \
	include/amino/undef.F90

rxincludedir = $(pkgincludedir)/rx

rxinclude_HEADERS = 	                \
	include/amino/rx/rxtype.h       \
	include/amino/rx/scenegraph.h   \
	include/amino/rx/scene_geom.h   \
	include/amino/rx/scene_gl.h     \
	include/amino/rx/scene_kin.h    \
	include/amino/rx/scene_dyn.h    \
	include/amino/rx/scene_ompl.h   \
	include/amino/rx/scene_collision.h \
	include/amino/rx/scene_sdl.h    \
	include/amino/rx/scene_win.h


lib_LTLIBRARIES = libamino.la libamino_xerbla_nop.la



libamino_la_SOURCES =                  \
	src/mem.c                      \
	src/la.c                       \
	src/la2.c                      \
	src/tf.c                       \
	src/tf/tfmatrix.c              \
	src/tf/quat.c                  \
	src/tf/conv.c                  \
	src/rx/errstr.c 	       \
	src/rx/scenegraph.cpp 	       \
	src/rx/sg_api.cpp 	       \
	src/rx/sg_capi.c 	       \
	src/rx/scene_geom.c 	       \
	src/rx/geom_opt.c              \
	src/rx/scene_kin.c             \
	src/rx/ik_opt.c                \
	src/rx/ik_jacobian.c           \
	src/rx/plugin.c                \
	src/rx/mp_seq.cpp              \
	src/math.c                     \
	src/plot.c                     \
	src/debug.c                    \
	src/kin.c                      \
	src/validate.c                 \
	src/time.c                     \
	src/io.c                       \
	src/are.c                      \
	src/sigsys.c                   \
	src/heap.c                     \
	src/cv.c


libamino_xerbla_nop_la_SOURCES = src/xerbla_nop.c

src/mac/mac.f: src/mac/amino.mac src/mac/gen.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && $(MAXIMA) --very-quiet -b gen.mac
	@echo [maxima end]

BUILT_SOURCES = src/mac/aa_tf_euler.f90 src/mac/aa_ls_se3.f90 src/mac/euler.f

src/mac/aa_tf_euler.f90: src/mac/amino.mac src/mac/euler.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && $(MAXIMA) --very-quiet -b euler.mac
	@echo [maxima end]


src/mac/euler.f: src/mac/amino.mac src/mac/euler77.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && $(MAXIMA) --very-quiet -b euler77.mac
	@echo [maxima end]

src/mac/aa_ls_se3.f90: src/mac/amino.mac src/mac/ls.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && $(MAXIMA) --very-quiet -b ls.mac
	@echo [maxima end]

src/mac/mac_tf.f90: src/mac/amino.mac src/mac/tf.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && $(MAXIMA) --very-quiet -b tf.mac
	@echo [maxima end]

distclean-local:
	-rm -rf doxygen-doc

#vecbench_SOURCES = src/test/vecbench.c
#vecbench_LDADD = libamino.la -lblas -llapack

# use some convenience libraries to force fortran module dependencies
noinst_LTLIBRARIES =                   \
	libamino_mem.la                \
	libamino_la.la                 \
	libamino_tf.la

## FORTRAN ##

if USE_FORTRAN

# each convenience library depends on the fortran source for that module
libamino_mem_la_SOURCES = mem_mod.f90
libamino_la_la_SOURCES = la_mod.f90
libamino_tf_la_SOURCES = tf_mod.f90

# each module file depends on the object files for its conveneince library
amino_mem.mod: $(libamino_mem_la_OBJECTS)
amino_la.mod: $(libamino_la_la_OBJECTS)
amino_tf.mod: $(libamino_tf_la_OBJECTS)

# dependent objects dependent on the appropriate module
$(libamino_la_la_OBJECTS): amino_mem.mod
$(libamino_tf_la_OBJECTS): amino_la.mod

libamino_la_LIBADD =                   \
	-lgfortran                     \
	libamino_mem.la                \
	libamino_la.la                 \
	libamino_tf.la

nodist_include_HEADERS = amino_la.mod amino_mem.mod amino_tf.mod

else

# No fortran compiler

BUILT_SOURCES += src/mac/euler.c
src/mac/euler.c: src/mac/euler.f
	cd src/mac && $(F2C) euler.f

libamino_la_SOURCES += src/mac/euler_f2c.c


endif



## JAVA ##
JSOURCES =                                        \
	$(top_srcdir)/java/Lib.java

EXTRA_DIST += $(JSOURCES) doc/javadoc

if HAVE_JAVAC

BUILT_SOURCES += org_golems_amino_Lib.h

JCLASSES =                                        \
	org/golems/amino/Lib.class

org/golems/amino/Lib.class: $(JSOURCES)
	$(JAVAC) -d $(top_builddir) $(JSOURCES)

org_golems_amino_Lib.h: $(top_builddir)/org/golems/amino/Lib.class
	$(JAVAH) org.golems.amino.Lib
	touch $@

org.golems.amino.jar: $(JCLASSES)
	$(JAR) cf $@ $(JCLASSES)


# install the jar file somewhere reasonable
javadir = $(datadir)/java
java_DATA = org.golems.amino.jar

lib_LTLIBRARIES += libaminoj.la
libaminoj_la_CPPFLAGS = $(AM_CPPFLAGS) $(CPPFLAGS) $(JNICPPFLAGS)
libaminoj_la_SOURCES = java/aminoj.c
libaminoj_la_LIBADD = libamino.la

# Javadocs
doc/.javadoc.stamp: $(JSOURCES)
	javadoc -d doc/javadoc $(JSOURCES)
	touch $@

doc/javadoc: doc/.javadoc.stamp

javadoc: doc/.javadoc.stamp

endif

## OpenGL support
if HAVE_GL
lib_LTLIBRARIES += libamino_gl.la
libamino_gl_la_SOURCES = src/amino_gl.c src/rx/geodesic.cpp

libamino_gl_la_CFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
libamino_gl_la_CXXFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
libamino_gl_la_LIBADD = libamino.la $(GL_LIBS)


## SDL ##
if HAVE_SDL2

libamino_gl_la_SOURCES += \
	src/amino_sdl.c \
	src/rx/sdl_win.c

noinst_PROGRAMS += sdl_test
sdl_test_SOURCES = src/rx/sdl_test.c
sdl_test_CFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
sdl_test_LDADD = libamino.la $(SDL2_LIBS)

sdl_test_LDADD += libamino_gl.la $(GL_LIBS) $(GLEW_LIBS)  -lglut

endif # SDL2

endif # GL


## TESTS ##
noinst_LTLIBRARIES += libtestutil.la
libtestutil_la_SOURCES = src/test/testutil.c

TESTS = aa_test
noinst_PROGRAMS += aa_test
aa_test_SOURCES = src/test/aa_test.c
aa_test_LDADD = libamino.la libtestutil.la

TESTS += q_test
noinst_PROGRAMS += q_test
q_test_SOURCES = src/test/q_test.c
q_test_LDADD = libamino.la libtestutil.la

TESTS += aa_regress
noinst_PROGRAMS += aa_regress
aa_regress_SOURCES = src/test/aa_regress.c
aa_regress_LDADD = libamino.la libtestutil.la

TESTS += sg_test
noinst_PROGRAMS += sg_test
sg_test_SOURCES = src/test/sg_test.c
sg_test_LDADD = libamino.la libtestutil.la

if USE_FORTRAN
TESTS += la_test
noinst_PROGRAMS += la_test
la_test_SOURCES = src/test/la_test.F90
la_test_LDADD = libamino.la libtestutil.la
endif

$(la_test_OBJECTS): amino_la.mod # manually specify fortran module dependency

foo:
	echo $(la_test_OBJECTS)

TESTS += aa_fuzz
noinst_PROGRAMS += aa_fuzz
aa_fuzz_SOURCES = src/test/aa_fuzz.c
aa_fuzz_LDADD = libamino.la  libtestutil.la

TESTS += aa_fuzz_tf
noinst_PROGRAMS += aa_fuzz_tf
aa_fuzz_tf_SOURCES = src/test/aa_fuzz_tf.c
aa_fuzz_tf_LDADD = libamino.la libtestutil.la

TESTS += testcpp
noinst_PROGRAMS += testcpp
testcpp_SOURCES = src/test/testcpp.cpp
testcpp_LDADD = libamino.la libtestutil.la

TESTS += la_ctest
noinst_PROGRAMS += la_ctest
la_ctest_SOURCES = src/test/la_ctest.c
la_ctest_LDADD = libamino.la libtestutil.la


if HAVE_FCL


lib_LTLIBRARIES += libamino_cl.la
libamino_cl_la_SOURCES = \
	src/rx/amino_fcl.cpp \
	src/rx/collision_set.cpp

libamino_cl_la_CFLAGS = $(FCL_CFLAGS)
libamino_cl_la_CXXFLAGS = $(FCL_CFLAGS)
libamino_cl_la_LIBADD = libamino.la $(FCL_LIBS)


TESTS += fcl_test
noinst_PROGRAMS += fcl_test
fcl_test_SOURCES = src/test/aa_test_fcl.cpp
fcl_test_CFLAGS = $(FCL_CFLAGS)
fcl_test_LDADD = libamino.la libamino_cl.la $(FCL_LIBS)

TESTS += cl_test
noinst_PROGRAMS += cl_test
cl_test_SOURCES = src/test/test_collision.c
cl_test_LDADD = libamino.la libamino_cl.la

endif # HAVE_FCL

if HAVE_OMPL

lib_LTLIBRARIES += libamino_planning.la
libamino_planning_la_SOURCES = \
	src/rx/scene_ompl.cpp
libamino_planning_la_CFLAGS = $(OMPL_CFLAGS)
libamino_planning_la_CXXFLAGS = $(OMPL_CFLAGS)
libamino_planning_la_LIBADD = libamino.la libamino_cl.la $(OMPL_LIBS)

endif # HAVE_OMPL

if HAVE_LPSOLVE

lib_LTLIBRARIES += libamino_opt_lpsolve.la
libamino_opt_lpsolve_la_SOURCES = \
	src/opt/opt_lpsolve.c
#libamino_opt_cgal_la_CFLAGS = $(OMPL_CFLAGS)
#libamino_opt_cgal_la_CXXFLAGS = $(OMPL_CFLAGS)
#libamino_opt_cgal_la_LIBADD = libamino.la -lCGAL -lgmp

noinst_PROGRAMS += test_lpsolve
test_lpsolve_SOURCES = src/test/test_lpsolve.c
test_lpsolve_LDADD = libamino_opt_lpsolve.la libamino.la libtestutil.la
TESTS += test_lpsolve

endif

if HAVE_CLP

lib_LTLIBRARIES += libamino_opt_clp.la
libamino_opt_clp_la_SOURCES = \
	src/opt/opt_clp.cpp
#libamino_opt_cgal_la_CFLAGS = $(OMPL_CFLAGS)
#libamino_opt_cgal_la_CXXFLAGS = $(OMPL_CFLAGS)
libamino_opt_clp_la_LIBADD = libamino.la -lClp

noinst_PROGRAMS += test_clp
test_clp_SOURCES = src/test/test_clp.c
test_clp_LDADD = libamino_opt_clp.la libamino.la libtestutil.la
TESTS += test_clp

endif



if HAVE_CGAL

lib_LTLIBRARIES += libamino_opt_cgal.la
libamino_opt_cgal_la_SOURCES = \
	src/opt/opt_cgal.cpp
#libamino_opt_cgal_la_CFLAGS = $(OMPL_CFLAGS)
#libamino_opt_cgal_la_CXXFLAGS = $(OMPL_CFLAGS)
libamino_opt_cgal_la_LIBADD = libamino.la -lCGAL -lgmp

noinst_PROGRAMS += opt_test
opt_test_SOURCES = src/test/opt_test.c
opt_test_LDADD = libamino_opt_cgal.la libamino.la libtestutil.la

endif




# noinst_PROGRAMS += test_vec
# test_vec_SOURCES = src/test/test_vec.c
# test_vec_LDADD = libamino.la -lblas -llapack libtestutil.la
# test_vec_CFLAGS = $(CFLAGS) -march=native

## PREPROCESSING ##

clapack_doc.c: doc/clapack_doc.c include/amino/clapack.h include/amino/clapack_impl.h
	$(CPP) $(AM_CPPFLAGS) -CC $< -o $@
lapack_doc.c: doc/lapack_doc.c include/amino/lapack.h include/amino/lapack_impl.h
	$(CPP) $(AM_CPPFLAGS) -CC $< -o $@
la_doc.c: doc/la_doc.c include/amino/la.h include/amino/la_impl.h
	$(CPP) $(AM_CPPFLAGS) -CC $< -o $@

la_mod.f90: src/la_mod.F90 src/la_implf.F90 src/mac/aa_ls_se3.f90 include/amino/def.h include/amino/undef.h include/amino/def.F90 include/amino/undef.F90 include/amino/mangle.h
	$(CPP) $(AM_CPPFLAGS) -x c  $< -o $@

mem_mod.f90: src/mem_mod.F90 src/mem_impl.F90 include/amino/def.h include/amino/undef.h include/amino/def.F90 include/amino/undef.F90 include/amino/mangle.h
	$(CPP) $(AM_CPPFLAGS) -x c  $< -o $@

tf_mod.f90: src/tff.F90 # src/mac/aa_tf_euler.f90
	$(CPP) $(AM_CPPFLAGS) -x c  $< -o $@


EXTRA_DX = clapack_doc.c lapack_doc.c la_doc.c la_mod.f90


doc/manual/scenegraph.html: doc/manual/scenegraph.org org-compile.el
	emacs --batch --quick \
	--load org-compile.el       \
	--eval '(golems-org-emit "$<" "scenegraph.html")'

clean-local:
	-rm -rf debian/tmp                  \
		debian/files                \
		debian/libamino?            \
		debian/libamino-dev         \
		debian/*.log                \
		debian/*.substvars          \
		debian/*.debhelper          \
		debian/usr/*                \
		debian/DEBIAN/*             \
		texput.log                  \
		$(EXTRA_DX)                 \
		*.mod

include aminclude.am
