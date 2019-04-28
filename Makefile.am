bin_PROGRAMS =
noinst_PROGRAMS =
noinst_LTLIBRARIES =
BUILT_SOURCES =
lib_LTLIBRARIES =
EXTRA_DIST =
CLEAN_LOCAL_FILES =
dist_bin_SCRIPTS =
TESTS =

INSTALL_EXEC_LOCAL =
ALL_LOCAL =
UNINSTALL_LOCAL =
CLEAN_LOCAL =

DIST_SUBDIRS = submodules/sycamore submodules/cl-ntdoc .

AM_CPPFLAGS = -I$(top_srcdir)/include -I$(top_builddir)/include -I$(top_srcdir)/src/mac
ACLOCAL_AMFLAGS = -I m4

AM_DISTCHECK_CONFIGURE_FLAGS=--without-lex --without-maxima

include Makefile-lisp.am
include Makefile-demo.am

EXTRA_DIST += \
	src/la_impl.c                       \
	src/la_mod.F90 src/la_implf.F90     \
	src/mem_mod.F90 src/mem_impl.F90    \
	src/tff.F90                         \
	src/mac/euler.f                     \
	euler.c                             \
	src/mac/aa_ls_se3.f90               \
	share/check-clpkg.sh                \
	share/load-quicklisp.lisp           \
	share/make-aarx.sh                  \
	share/make-aarx.lisp                \
	share/robray-mode.el                \
	share/exec/meshconv                 \
	share/exec/povremote

noinst_HEADERS = \
             src/opt/opt_internal.h \
             src/tf/quat_ord.c \
             src/tf/quat_duqu.c \
             src/tf/quat_qutr.c \
             src/rx/sg_convenience.h \
             include/wavefront_internal.h \
             include/amino/mat_internal.h \
             include/amino/ct/traj_internal.hpp \
             include/amino/rx/scene_kin_internal.h \
             include/amino/rx/scene_wk_internal.h \
             include/amino/rx/scenegraph_internal.h \
             include/amino/rx/scene_geom_internal.h \
             include/amino/rx/scene_collision_internal.h \
             include/amino/rx/scene_gl_internal.h \
             include/amino/rx/scene_sdl_internal.h \
             include/amino/rx/rxtype_internal.h \
             include/amino/rx/scene_fcl.h \
             include/amino/rx/ompl/scene_ompl_internal.h \
             include/amino_internal.h

dist_pkgdata_DATA = src/mac/amino.mac

include_HEADERS = include/amino.h include/amino.hpp


pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = \
	share/amino.pc \
	share/amino-gl.pc \
	share/amino-collision.pc \
	share/amino-planning.pc \
	share/amino-gl-$(HEADER_VERSION).pc \
	share/amino-collision-$(HEADER_VERSION).pc \
	share/amino-planning-$(HEADER_VERSION).pc

share/amino-gl-$(HEADER_VERSION).pc:
	ln -s amino-gl.pc share/amino-gl-$(HEADER_VERSION).pc

share/amino-collision-$(HEADER_VERSION).pc:
	ln -s amino-collision.pc share/amino-collision-$(HEADER_VERSION).pc

share/amino-planning-$(HEADER_VERSION).pc:
	ln -s amino-planning.pc share/amino-planning-$(HEADER_VERSION).pc

share/amino-$(HEADER_VERSION).pc:
	ln -s amino.pc share/amino-$(HEADER_VERSION).pc


nodist_pkginclude_HEADERS = \
	include/amino/cblas_order.h

pkginclude_HEADERS = 	                \
	include/amino/mem.hpp           \
	include/amino/rx.h              \
	include/amino/mat.h             \
	include/amino/mat.hpp           \
	include/amino/diffeq.h          \
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
	include/amino/refcount.h        \
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
	include/amino/getset.h          \
	include/amino/def.F90           \
	include/amino/undef.F90

rxincludedir = $(pkgincludedir)/rx

rxinclude_HEADERS = 	                \
	include/amino/rx/rxtype.h       \
	include/amino/rx/rxerr.h        \
	include/amino/rx/scenegraph.h   \
	include/amino/rx/scene_geom.h   \
	include/amino/rx/scene_gl.h     \
	include/amino/rx/scene_sub.h    \
	include/amino/rx/scene_kin.h    \
	include/amino/rx/scene_dyn.h    \
	include/amino/rx/scene_collision.h \
	include/amino/rx/scene_planning.h \
	include/amino/rx/scene_sdl.h    \
	include/amino/rx/scene_win.h    \
	include/amino/rx/scene_plugin.h \
	include/amino/rx/mp_seq.h \
	include/amino/rx/spnav_button.h \
	include/amino/rx/rx_ct.h \
	include/amino/rx/scene_wk.h \
	include/amino/rx/wavefront.h

rxomplincludedir = $(rxincludedir)/ompl
rxomplinclude_HEADERS = \
	include/amino/rx/ompl/scene_state_space.h \
	include/amino/rx/ompl/scene_state_validity_checker.h \
	include/amino/rx/ompl/scene_workspace_goal.h \
	include/amino/rx/ompl/scene_ompl.h

omplcompatincludedir = $(pkgincludedir)/ompl-compat/ompl/base
omplcompatinclude_HEADERS = \
	include/amino/ompl-compat/ompl/base/TypedStateValidityChecker.h \
	include/amino/ompl-compat/ompl/base/TypedSpaceInformation.h

omplcompat2includedir = $(pkgincludedir)/ompl-compat2/ompl/base
omplcompat2include_HEADERS = \
	include/amino/ompl-compat2/ompl/base/TypedStateValidityChecker.h \
	include/amino/ompl-compat2/ompl/base/TypedSpaceInformation.h

ctincludedir = $(pkgincludedir)/ct

ctinclude_HEADERS = \
	include/amino/ct/traj.h \
	include/amino/ct/state.h


noinst_PROGRAMS += mtl_test
mtl_test_SOURCES = src/test/mtl_test.c
mtl_test_LDADD = libamino.la

noinst_PROGRAMS += obj_test
obj_test_SOURCES = src/test/wavefront_test.c
obj_test_LDADD = libamino.la

lib_LTLIBRARIES += libamino.la libamino-xerbla-nop.la

libamino_la_SOURCES =                  \
	src/mem.c                      \
	src/refcount.c                 \
	src/mat.c                      \
	src/la.c                       \
	src/la2.c                      \
	src/opt/opt.c                  \
	src/tf.c                       \
	src/tf/tfmatrix.c              \
	src/tf/quat.c                  \
	src/diffeq.c                   \
	src/tf/conv.c                  \
	src/mac/euler_f2c.c            \
	src/rx/errstr.c                \
	src/rx/scenegraph.cpp          \
	src/rx/sg_api.cpp              \
	src/rx/sg_capi.c               \
	src/rx/scene_geom.c            \
	src/rx/geom_opt.c              \
	src/rx/scene_kin.c             \
	src/rx/ik_opt.c                \
	src/rx/scene_ik.c              \
	src/rx/ik_jacobian.c           \
	src/rx/plugin.c                \
	src/rx/rx_ct.c                 \
	src/rx/scene_wk.c              \
	src/rx/lc3.c                   \
	src/rx/mp_seq.cpp              \
	src/ct/traj.cpp                \
	src/ct/state.c                 \
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
	src/cv.c                       \
	src/wavefront/wavefront.c      \
	src/wavefront/mtl.c            \
	src/wavefront/wf_obj.c         \
	src/wavefront/wf_mtl.c


libamino_la_LIBADD =

libamino_xerbla_nop_la_SOURCES = src/xerbla_nop.c

EXTRA_DIST += src/mac/amino.mac src/mac/euler.mac src/mac/euler77.mac src/mac/ls.mac


############
## MAXIMA ##
############
if HAVE_MAXIMA

# EXTRA_DIST += src/mac/aa_tf_euler.f90
# BUILT_SOURCES += src/mac/aa_tf_euler.f90
# src/mac/aa_tf_euler.f90: src/mac/amino.mac src/mac/euler.mac
#	@echo [maxima start]
#	cd $(top_srcdir)/src/mac && $(MAXIMA) --very-quiet -b euler.mac
#	@echo [maxima end]


EXTRA_DIST += src/mac/euler.f
BUILT_SOURCES += src/mac/euler.f
src/mac/euler.f: src/mac/amino.mac src/mac/euler77.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && $(MAXIMA) --very-quiet -b euler77.mac
	@echo [maxima end]

EXTRA_DIST += src/mac/aa_ls_se3.f90
BUILT_SOURCES += src/mac/aa_ls_se3.f90
src/mac/aa_ls_se3.f90: src/mac/amino.mac src/mac/ls.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && $(MAXIMA) --very-quiet -b ls.mac
	@echo [maxima end]

endif


#vecbench_SOURCES = src/test/vecbench.c
#vecbench_LDADD = libamino.la -lblas -llapack

#############
## FORTRAN ##
#############

if HAVE_F2C

BUILT_SOURCES += euler.c
EXTRA_DIST += euler.c
euler.c: src/mac/euler.f
	$(F2C) src/mac/euler.f

endif

if USE_FORTRAN

# use some convenience libraries to force fortran module dependencies
noinst_LTLIBRARIES +=                  \
	libamino_mem.la                \
	libamino_la.la                 \
	libamino_tf.la

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

libamino_la_LIBADD +=                  \
	libamino_mem.la                \
	libamino_la.la                 \
	libamino_tf.la

nodist_include_HEADERS = amino_la.mod amino_mem.mod amino_tf.mod

libamino_la_SOURCES += src/mac/euler.f

else

libamino_la_SOURCES += euler.c
# do nothing, euler.c in included in src/mac/euler_f2c.c

endif

###############
## WAVEFRONT ##
###############
BUILT_SOURCES += src/wavefront/wavefront.c src/wavefront/mtl.c
EXTRA_DIST += src/wavefront/wavefront.l src/wavefront/mtl.l

src/wavefront/wavefront.c: src/wavefront/wavefront.l
	$(LEX) -t $(top_srcdir)/src/wavefront/wavefront.l > src/wavefront/wavefront.c

src/wavefront/mtl.c: src/wavefront/mtl.l
	$(LEX) -t $(top_srcdir)/src/wavefront/mtl.l > src/wavefront/mtl.c

###########
## EMACS ##
###########
lisp_LISP = share/robray-mode.el

#########
## VIM ##
#########
vimdir         = $(pkgdatadir)/vim
vimsyntaxdir   = $(vimdir)/syntax
vimftdetectdir = $(vimdir)/ftdetect

dist_vimsyntax_DATA = share/vim/syntax/robray.vim
dist_vimftdetect_DATA = share/vim/ftdetect/robray.vim

############
## PYTHON ##
############

if HAVE_PYTHON

TESTS += python/test/mattest.sh
TESTS += python/test/tftest.sh

ALL_LOCAL += all-python
all-python:
	cd "$(srcdir)/python" && $(PYTHON) setup.py build \
		--build-base $(shell readlink -f $(builddir))/python/build \
		--verbose

INSTALL_EXEC_LOCAL += install-exec-python
install-exec-python:
	cd "$(srcdir)/python" && $(PYTHON) setup.py build \
		--build-base $(shell readlink -f $(builddir))/python/build \
		install \
		--prefix "$(DESTDIR)$(prefix)" \
		--record "$(DESTDIR)$(pkgpythondir)/install_files.txt" \
		--verbose

UNINSTALL_LOCAL += uninstall-python
uninstall-python:
	if test -f "$(DESTDIR)$(pkgpythondir)/install_files.txt" ; then \
		cat "$(DESTDIR)$(pkgpythondir)/install_files.txt" | \
			xargs rm -rf; \
		rm "$(DESTDIR)$(pkgpythondir)/install_files.txt"; \
		rmdir  "$(DESTDIR)$(pkgpythondir)"; \
	fi

CLEAN_LOCAL += clean-python
clean-python:
	-rm -rf "$(builddir)/python/build"

endif

# package
EXTRA_DIST += \
	python/setup.py \
	python/amino/mat.py \
	python/amino/tf.py \
	python/amino/mixin.py \
	python/amino/scenegraph.py \
	python/amino/scenewin.py \
	python/amino/lib.py \
	python/amino/__init__.py


# tests
EXTRA_DIST += \
	python/test/tftest.py \
	python/test/mattest.py


##########
## JAVA ##
##########

JSOURCES =                                        \
	$(top_srcdir)/java/Lib.java

EXTRA_DIST += $(JSOURCES)
# EXTRA_DIST += doc/javadoc

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

## SDL2/OpenGL ##
if AMINO_GL

# OpenGL support
lib_LTLIBRARIES += libamino-gl.la
libamino_gl_la_SOURCES = src/amino_gl.c

libamino_gl_la_CFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
libamino_gl_la_CXXFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
libamino_gl_la_LIBADD = $(GL_LIBS) $(SDL2_LIBS) libamino.la


# SDL

libamino_gl_la_SOURCES += \
	src/amino_sdl.c \
	src/sdl_handler.c \
	src/sdl_ui.c \
	src/rx/sdl_win.c

noinst_PROGRAMS += sdl_test
sdl_test_SOURCES = src/rx/sdl_test.c
sdl_test_CFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
sdl_test_LDADD = libamino.la $(SDL2_LIBS)

sdl_test_LDADD += libamino-gl.la $(GL_LIBS)

bin_PROGRAMS += aarx-view

aarx_view_SOURCES = src/rx/aarx-view.c
aarx_view_CFLAGS = $(SDL2_CFLAGS) $(GL_CFLAGS)
aarx_view_LDADD = libamino-gl.la libamino.la $(SDL2_LIBS)


endif # SDL2/OpenGL


## TESTS ##
noinst_LTLIBRARIES += libtestutil.la
libtestutil_la_SOURCES = src/test/testutil.c


noinst_PROGRAMS += memregion
memregion_SOURCES = src/test/memregion.c
memregion_LDADD = libamino.la libtestutil.la

TESTS += aa_test
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

TESTS += ct_traj
noinst_PROGRAMS += ct_traj
ct_traj_SOURCES = src/test/ct_traj.c
ct_traj_LDADD = libamino.la libtestutil.la
ct_traj_LDFLAGS = -no-install


TESTS += mat_test
noinst_PROGRAMS += mat_test
mat_test_SOURCES = src/test/mat_test.c
mat_test_LDADD = libamino.la libtestutil.la

TESTS += matpp_test
noinst_PROGRAMS += matpp_test
matpp_test_SOURCES = src/test/matpp_test.cpp
matpp_test_LDADD = libamino.la libtestutil.la

if USE_FORTRAN
TESTS += la_test
noinst_PROGRAMS += la_test
la_test_SOURCES = src/test/la_test.F90
la_test_LDADD = libamino.la libtestutil.la
endif

$(la_test_OBJECTS): amino_la.mod # manually specify fortran module dependency

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


lib_LTLIBRARIES += libamino-collision.la
libamino_collision_la_SOURCES = \
	src/rx/amino_fcl.cpp \
	src/rx/collision_set.cpp

libamino_collision_la_CFLAGS = $(FCL_CFLAGS)
libamino_collision_la_CXXFLAGS = $(FCL_CFLAGS)
libamino_collision_la_LIBADD = $(FCL_LIBS)


TESTS += fcl_test
noinst_PROGRAMS += fcl_test
fcl_test_SOURCES = src/test/aa_test_fcl.cpp
fcl_test_CFLAGS = $(FCL_CFLAGS)
fcl_test_LDADD = libamino.la libamino-collision.la $(FCL_LIBS)

TESTS += cl_test
noinst_PROGRAMS += cl_test
cl_test_SOURCES = src/test/test_collision.c
cl_test_LDADD = libamino.la libamino-collision.la

endif # HAVE_FCL

if HAVE_OMPL

lib_LTLIBRARIES += libamino-planning.la
libamino_planning_la_SOURCES = \
	src/rx/mp/scene_ompl.cpp \
	src/rx/mp/scene_state_validity_checker.cpp \
	src/rx/mp/scene_state_space.cpp \
	src/rx/mp/workspace_goal.cpp \
	src/rx/mp/ompl_rrt.cpp \
	src/rx/mp/ompl_sbl.cpp \
	src/rx/mp/ompl_kpiece.cpp
libamino_planning_la_CFLAGS = $(OMPL_CFLAGS)
libamino_planning_la_CXXFLAGS = $(OMPL_CFLAGS)
libamino_planning_la_LIBADD = $(OMPL_LIBS)

endif # HAVE_OMPL


noinst_PROGRAMS += test_lpsolve
test_lpsolve_SOURCES = src/test/test_lpsolve.c
test_lpsolve_LDADD =
TESTS += test_lpsolve

if HAVE_LPSOLVE
libamino_la_SOURCES += src/opt/opt_lpsolve.c
endif

if HAVE_GLPK
libamino_la_SOURCES += src/opt/opt_glpk.c
endif # HAVE_GLPK

if HAVE_CLP
libamino_la_SOURCES += src/opt/opt_clp.cpp
libamino_la_LIBADD += -lClp
endif # HAVE_CLP

test_lpsolve_LDADD += libamino.la libtestutil.la



## NLOPT
if HAVE_NLOPT
libamino_la_SOURCES += src/rx/ik_nlopt.c
endif

if HAVE_CGAL

lib_LTLIBRARIES += libamino-opt-cgal.la
libamino_opt_cgal_la_SOURCES = src/opt/opt_cgal.cpp
libamino_opt_cgal_la_LIBADD = libamino.la -lCGAL -lgmp

noinst_PROGRAMS += opt_test
opt_test_SOURCES = src/test/opt_test.c
opt_test_LDADD = libamino-opt-cgal.la libamino.la libtestutil.la

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

clean-local: $(CLEAN_LOCAL)
	-rm -rf debian/tmp                  \
		$(CLEAN_LOCAL_FILES)        \
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
		*.mod                       \
                doc/javadoc                 \
                doc/.javadoc.stamp          \
                org.golems.amino.jar        \
                org_golems_amino_Lib.h      \
                org/golems/amino/Lib.class  \
                doc/md/copying.md

distclean-local:
	-rm -rf doc/manual doc/aarxc.1 doc/doxygen_sqlite3.db
	-rm -rf lisp/libdir.lisp
	-rm -rf lisp-amino.md lisp-rx.md aminopy.md
	if test "x$(top_srcdir)" != "x$(top_builddir)"; then \
		rm -rf $(top_builddir)/lisp; \
	fi


install-exec-local: $(INSTALL_EXEC_LOCAL)

uninstall-local: $(UNINSTALL_LOCAL)

all-local: $(ALL_LOCAL)

sync-doc:
	rsync -r doc/manual/ code.golems.org:/srv/www/amino


doc/md/copying.md: $(top_srcdir)/COPYING $(top_srcdir)/doc/md/copying.md.header
	$(MKDIR_P) doc/md/
	cat $(top_srcdir)/doc/md/copying.md.header  \
	    $(top_srcdir)/COPYING \
	    > doc/md/copying.md


doc/md/acknowledgements.md: $(top_srcdir)/AUTHORS $(top_srcdir)/doc/md/acknowledgments.md.header
	$(MKDIR_P) doc/md/
	cat $(top_srcdir)/doc/md/acknowledgments.md.header  \
	    $(top_srcdir)/AUTHORS \
	    > doc/md/acknowledgments.md


EXTRA_DIST += \
	doc/md/copying.md.header \
	doc/md/faq.md \
	doc/md/download.md \
	doc/md/acknowledgments.md.header \
	doc/md/faq.md \
	doc/md/Installation.md \
	doc/md/mainpage.md \
	doc/md/scenecompiler.md \
	doc/md/gui.md \
	doc/md/scenefile.md \
	doc/footer.html \
	doc/carousel.html \
	doc/logo.png \
        share/doc.lisp



EXTRA_DIST += lisp-amino.md lisp-rx.md aminopy.md

if HAVE_COMMON_LISP

lisp-amino.md: aarx.core share/doc.lisp
	LD_LIBRARY_PATH=.libs:$(LD_LIBRARY_PATH) \
	./aarx.core \
		--eval "(load \"$(top_srcdir)/share/doc.lisp\")" \
		--eval '(sb-ext:quit)'

lisp-rx.md: lisp-amino.md
aminopy.md: lisp-amino.md

else

lisp-amino.md:
	touch lisp-amino.md

lisp-rx.md:
	touch lisp-rx.md

endif # HAVE_COMMON_LISP




if HAVE_DOXYGEN



doxygen: doc/md/copying.md doc/md/mainpage.md lisp-rx.md lisp-amino.md
	$(DOXYGEN)

doc/manual: doxygen

MANUAL_SVGS = \
	doc/manual/planerot.svg \
	doc/manual/spacerot.svg \
	doc/manual/axang.svg \
	doc/manual/qplane.svg \
	doc/manual/rotmat2d.svg \
	doc/manual/eulerrot.svg \
	doc/manual/localframe.svg \
	doc/manual/localpoint.svg \
	doc/manual/tfpoint.svg \
	doc/manual/tfchain.svg \
	doc/manual/robotframe.svg \
	doc/manual/cplane.svg

doc: doxygen $(MANUAL_SVGS)
	$(MKDIR_P) doc/manual/doc
	cp $(top_srcdir)/doc/logo-medium.png doc/manual/doc

doc/md/tutorial/fig/%.pdf: doc/md/tutorial/fig/%.tex
	$(MKDIR_P) $(dir $@)
	TEXINPUTS="$(dir $<):$(TEXINPUTS)" pdflatex  -output-directory $(dir $@) $<

doc/manual/%.svg: doc/md/tutorial/fig/%.pdf
	$(MKDIR_P) $(dir $@)
	pdftocairo -svg $< $@


else

doc/manual:
	$(MKDIR_P) doc/manual

endif




EXTRA_DIST += doc/manual
