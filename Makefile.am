ACLOCAL_AMFLAGS = -I m4
INCLUDES = -I$(top_srcdir)/include
EXTRA_DIST = src/mac/gen.mac src/mac/mac.f       \
             src/la_impl.c                       \
             src/la_mod.F90 src/la_implf.F90     \
             src/mem_mod.F90 src/mem_impl.F90

dist_pkgdata_DATA = src/mac/amino.mac

include_HEADERS = include/amino.h

nodist_include_HEADERS = amino_la.mod amino_mem.mod

pkginclude_HEADERS = 	                \
	include/amino/debug.h           \
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
	include/amino/def.h             \
	include/amino/undef.h           \
	include/amino/mangle.h          \
	include/amino/def.F90           \
	include/amino/undef.F90

TESTS = aa_test la_test

lib_LTLIBRARIES = libamino.la libamino_xerbla_nop.la

noinst_PROGRAMS = aa_test la_test

libamino_la_SOURCES =                  \
	src/mem.c                      \
	src/la.c                       \
	src/la2.c                      \
	src/tf.c                       \
	src/math.c                     \
	src/plot.c                     \
	src/debug.c                    \
	src/kin.c                      \
	src/mac/mac.f                  \
	src/validate.c                 \
	src/time.c                     \
	src/io.c                       \
	src/are.c                      \
	src/sigsys.c                   \
	src/cv.c                       \
	mem_mod.f90                    \
	la_mod.f90

libamino_xerbla_nop_la_SOURCES = src/xerbla_nop.f

src/mac/mac.f: src/mac/amino.mac src/mac/gen.mac
	@echo [maxima start]
	cd src/mac && maxima --very-quiet -b gen.mac
	@echo [maxima end]


distclean-local:
	-rm -rf doxygen-doc

aa_test_SOURCES = src/test/aa_test.c
aa_test_LDADD = libamino.la -lblas -llapack

la_test_SOURCES = src/test/la_test.F90
la_test_LDADD = libamino.la -lblas -llapack

$(la_test_OBJECTS): amino_la.mod # manually specify fortran module dependency

clapack_doc.c: doc/clapack_doc.c include/amino/clapack.h include/amino/clapack_impl.h
	$(CPP) $(INCLUDES) -CC $< -o $@
lapack_doc.c: doc/lapack_doc.c include/amino/lapack.h include/amino/lapack_impl.h
	$(CPP) $(INCLUDES) -CC $< -o $@
la_doc.c: doc/la_doc.c include/amino/la.h include/amino/la_impl.h
	$(CPP) $(INCLUDES) -CC $< -o $@

la_mod.f90: src/la_mod.F90 src/la_implf.F90 include/amino/def.h include/amino/undef.h include/amino/def.F90 include/amino/undef.F90 include/amino/mangle.h
	$(CPP) $(INCLUDES) -x c -CC $< -o $@
amino_la.mod: la_mod.$(OBJEXT)

mem_mod.f90: src/mem_mod.F90 src/mem_impl.F90 include/amino/def.h include/amino/undef.h include/amino/def.F90 include/amino/undef.F90 include/amino/mangle.h
	$(CPP) $(INCLUDES) -x c -CC $< -o $@
amino_mem.mod: mem_mod.$(OBJEXT)

la_mod.$(OBJEXT): amino_mem.mod

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
