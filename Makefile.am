INCLUDES = -I$(top_srcdir)/include
EXTRA_DIST = src/mac/amino.mac src/mac/gen.mac src/la_impl.c

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
	include/amino/validate.h

TESTS = aa_test

lib_LTLIBRARIES = libamino.la

bin_PROGRAMS = aa_test

libamino_la_SOURCES = src/mem.c src/la.c src/la2.c src/tf.c src/math.c src/plot.c src/debug.c src/kin.c src/mac/mac.f src/validate.c src/time.c src/io.c src/are.c src/sigsys.c src/cv.c

src/mac/mac.f: src/mac/amino.mac src/mac/gen.mac
	@echo [maxima start]
	cd src/mac && maxima --very-quiet -b gen.mac
	@echo [maxima end]

clean-local:
	-rm -rf debian/tmp debian/files debian/libamino? debian/libamino-dev debian/*.log debian/*.substvars texput.log debian/*.debhelper debian/usr/* debian/DEBIAN/*

distclean-local:
	-rm -rf doxygen-doc

aa_test_SOURCES = src/aa_test.c
aa_test_LDADD = libamino.la

include aminclude.am
