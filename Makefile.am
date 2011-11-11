INCLUDES = -I$(top_srcdir)/include
EXTRA_DIST = src/mac/amino.mac src/mac/gen.mac

include_HEADERS = 	                \
	include/amino/debug.h           \
	include/amino/endconv.h         \
	include/amino.h                 \
	include/amino/io.h              \
	include/amino/kin.h             \
	include/amino/lapack.h          \
	include/amino/math.h            \
	include/amino/mem.h             \
	include/amino/plot.h            \
	include/amino/tf.h              \
	include/amino/time.h            \
	include/amino/type.h            \
	include/amino/unit.h            \
	include/amino/validate.h

TESTS = aa_test

lib_LTLIBRARIES = libamino.la

bin_PROGRAMS = aa_test

libamino_la_SOURCES = src/mem.c src/la.c src/tf.c src/math.c src/plot.c src/debug.c src/kin.c src/mac/mac.f src/validate.c src/time.c src/io.c src/are.c src/sigsys.c

src/mac/mac.f: src/mac/amino.mac src/mac/gen.mac
	@echo [maxima start]
	cd src/mac && maxima --very-quiet -b gen.mac
	@echo [maxima end]

clean-local:
	-rm -rf debian/tmp debian/files debian/libamino? debian/libamino-dev debian/*.log debian/*.substvars texput.log debian/*.debhelper

distclean-local:
	-rm -rf doxygen-doc

aa_test_SOURCES = src/aa_test.c
aa_test_LDADD = libamino.la

include aminclude.am
