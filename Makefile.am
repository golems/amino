INCLUDES = -I$(top_srcdir)/include
EXTRA_DIST = src/mac/amino.mac src/mac/gen.mac src/la_impl.c       \
             src/la_mod.F90 src/la_float.F90

include_HEADERS = include/amino.h

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
	include/amino/undef.h

TESTS = aa_test

lib_LTLIBRARIES = libamino.la

bin_PROGRAMS = aa_test

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
	la_mod.f90

src/mac/mac.f: src/mac/amino.mac src/mac/gen.mac
	@echo [maxima start]
	cd src/mac && maxima --very-quiet -b gen.mac
	@echo [maxima end]


distclean-local:
	-rm -rf doxygen-doc

aa_test_SOURCES = src/aa_test.c
aa_test_LDADD = libamino.la

clapack-doc.c: doc/clapack-doc.c
	$(CPP) $(INCLUDES) -CC $< -o $@
lapack-doc.c: doc/lapack-doc.c
	$(CPP) $(INCLUDES) -CC $< -o $@
la-doc.c: doc/la-doc.c
	$(CPP) $(INCLUDES) -CC $< -o $@

la_mod.f90: src/la_mod.F90 src/la_float.F90 include/amino/def.h include/amino/undef.h
	$(CPP) $(INCLUDES) -x c -CC $< -o $@

EXTRA_DX = clapack-doc.c lapack-doc.c la-doc.c

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
