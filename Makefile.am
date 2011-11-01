INCLUDES = -I$(top_srcdir)/include

include_HEADERS = include

TESTS = aa_test

lib_LTLIBRARIES = libamino.la

bin_PROGRAMS = aa_test

libamino_la_SOURCES = src/mem.c src/la.c src/tf.c src/math.c src/plot.c src/debug.c src/kin.c src/mac/mac.f src/validate.c src/time.c src/io.c

src/mac/mac.f: src/mac/amino.mac src/mac/gen.mac
	@echo [maxima start]
	cd src/mac && maxima --very-quiet -b gen.mac
	@echo [maxima end]



aa_test_SOURCES = src/aa_test.c
aa_test_LDADD = libamino.la
