ACLOCAL_AMFLAGS = -I m4
INCLUDES = -I$(top_srcdir)/include -I$(top_srcdir)/src/mac
EXTRA_DIST = src/mac/gen.mac src/mac/euler.mac   \
             src/la_impl.c                       \
             src/la_mod.F90 src/la_implf.F90     \
             src/mem_mod.F90 src/mem_impl.F90    \
             src/tff.F90                         \
             src/mac/tf.mac                      \
             src/mac/aa_tf_euler.f90

dist_pkgdata_DATA = src/mac/amino.mac

include_HEADERS = include/amino.h include/amino.hpp

nodist_include_HEADERS = amino_la.mod amino_mem.mod amino_tf.mod

pkginclude_HEADERS = 	                \
	include/amino/mem.hpp           \
	include/amino/tf.hpp           \
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
	include/amino/mangle.h          \
	include/amino/def.F90           \
	include/amino/undef.F90


lib_LTLIBRARIES = libamino.la libamino_xerbla_nop.la



libamino_la_SOURCES =                  \
	src/mem.c                      \
	src/la.c                       \
	src/la2.c                      \
	src/tf.c                       \
	src/math.c                     \
	src/plot.c                     \
	src/debug.c                    \
	src/kin.c                      \
	src/validate.c                 \
	src/time.c                     \
	src/io.c                       \
	src/are.c                      \
	src/sigsys.c                   \
	src/cv.c

libamino_la_LIBADD =                   \
	libamino_mem.la                \
	libamino_la.la                 \
	libamino_tf.la

libamino_xerbla_nop_la_SOURCES = src/xerbla_nop.c

src/mac/mac.f: src/mac/amino.mac src/mac/gen.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && maxima --very-quiet -b gen.mac
	@echo [maxima end]

BUILT_SOURCES = src/mac/aa_tf_euler.f90

src/mac/aa_tf_euler.f90: src/mac/amino.mac src/mac/euler.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && maxima --very-quiet -b euler.mac
	@echo [maxima end]

src/mac/mac_tf.f90: src/mac/amino.mac src/mac/tf.mac
	@echo [maxima start]
	cd $(top_srcdir)/src/mac && maxima --very-quiet -b tf.mac
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
$(la_test_OBJECTS): amino_la.mod # manually specify fortran module dependency

foo:
	echo $(la_test_OBJECTS)


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
libaminoj_la_CPPFLAGS = $(CPPFLAGS) $(JNICPPFLAGS)
libaminoj_la_SOURCES = java/aminoj.c
libaminoj_la_LIBADD = libamino.la

# Javadocs
doc/.javadoc.stamp: $(JSOURCES)
	javadoc -d doc/javadoc $(JSOURCES)
	touch $@

doc/javadoc: doc/.javadoc.stamp

javadoc: doc/.javadoc.stamp

endif


## TESTS ##

TESTS = aa_test la_test aa_regress q_test aa_fuzz testcpp
noinst_PROGRAMS = aa_test la_test aa_fuzz aa_regress q_test testcpp
noinst_LTLIBRARIES += libtestutil.la

libtestutil_la_SOURCES = src/test/testutil.c

aa_test_SOURCES = src/test/aa_test.c
aa_test_LDADD = libamino.la -lblas -llapack libtestutil.la

q_test_SOURCES = src/test/q_test.c
q_test_LDADD = libamino.la -lblas -llapack libtestutil.la

aa_regress_SOURCES = src/test/aa_regress.c
aa_regress_LDADD = libamino.la -lblas -llapack libtestutil.la

la_test_SOURCES = src/test/la_test.F90
la_test_LDADD = libamino.la -lblas -llapack libtestutil.la

aa_fuzz_SOURCES = src/test/aa_fuzz.c
aa_fuzz_LDADD = libamino.la -lblas -llapack libtestutil.la

testcpp_SOURCES = src/test/testcpp.cpp
testcpp_LDADD = libamino.la -lblas -llapack libtestutil.la

## PREPROCESSING ##

clapack_doc.c: doc/clapack_doc.c include/amino/clapack.h include/amino/clapack_impl.h
	$(CPP) $(INCLUDES) -CC $< -o $@
lapack_doc.c: doc/lapack_doc.c include/amino/lapack.h include/amino/lapack_impl.h
	$(CPP) $(INCLUDES) -CC $< -o $@
la_doc.c: doc/la_doc.c include/amino/la.h include/amino/la_impl.h
	$(CPP) $(INCLUDES) -CC $< -o $@

la_mod.f90: src/la_mod.F90 src/la_implf.F90 include/amino/def.h include/amino/undef.h include/amino/def.F90 include/amino/undef.F90 include/amino/mangle.h
	$(CPP) $(INCLUDES) -x c  $< -o $@

mem_mod.f90: src/mem_mod.F90 src/mem_impl.F90 include/amino/def.h include/amino/undef.h include/amino/def.F90 include/amino/undef.F90 include/amino/mangle.h
	$(CPP) $(INCLUDES) -x c  $< -o $@

tf_mod.f90: src/tff.F90 src/mac/aa_tf_euler.f90 src/tf/qv.f90
	$(CPP) $(INCLUDES) -x c  $< -o $@


EXTRA_DX = clapack_doc.c lapack_doc.c la_doc.c la_mod.f90

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
