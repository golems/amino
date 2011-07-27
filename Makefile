# Project Name
PROJECT := amino

# Project Version
VERSION := 20110719

# Binary Files
#BINFILES :=

# Library files
SHAREDLIBS := amino

all: default

include /usr/share/make-common/common.1.mk

#CFLAGS += -O0 -Wno-conversion
CFLAGS += --std=gnu99 -O2
FFLAGS += -I/usr/include -O2

default: $(LIBFILES) $(BINFILES) test

test: build/aa_test build/aa_testpp
	./build/aa_test
	./build/aa_testpp

OBJS :=  mem.o la.o tf.o math.o plot.o debug.o kin.o mac/mac.o validate.o time.o io.o
BOBJS := $(addprefix build/, $(OBJS))
# lapack should also link (c)blas and gfortran if needed
LIBS := m lapack rt


# maxima code generation

$(SRCDIR)/mac/mac.f: $(SRCDIR)/mac/amino.mac $(SRCDIR)/mac/gen.mac
	@echo [maxima start]
	cd $(SRCDIR)/mac && maxima --very-quiet -b gen.mac
	@echo [maxima end]

$(call LINKLIB, amino, $(OBJS), $(LIBS))
$(call LINKBIN, aa_test, aa_test.o $(OBJS), $(LIBS))
$(call LINKBIN, aa_testpp, aa_testpp.o $(OBJS), $(LIBS) stdc++)

.PHONY: default clean doc test valgrind

valgrind: $(BUILDDIR)/aa_test
	valgrind --leak-check=full --track-origins=yes ./build/aa_test

doc:
	doxygen

clean:
	rm -fr $(BINFILES) $(LIBFILES) src/mac/*.f $(BUILDDIR)/*.o $(BUILDDIR)/mac/*.o .deps debian *.deb


