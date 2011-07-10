# Project Name
PROJECT := amino

# Project Version
VERSION := 20110710

# Binary Files
#BINFILES :=

# Library files
SHAREDLIBS := amino

all: default

include /usr/share/make-common/common.1.mk

#CFLAGS += -O0 -Wno-conversion
CFLAGS += --std=gnu99 -O0
FFLAGS += -I/usr/include

default: $(LIBFILES) $(BINFILES) test

test: build/aa_test build/aa_testpp
	./build/aa_test
	./build/aa_testpp

OBJS :=  mem.o la.o tf.o math.o plot.o debug.o kin.o mac/mac.o validate.o
BOBJS := $(addprefix build/, $(OBJS))
LIBS := m lapack-3 blas-3 rt


# maxima code generation

$(SRCDIR)/mac/mac.f: $(SRCDIR)/mac/amino.mac $(SRCDIR)/mac/gen.mac
	@echo [maxima start]
	cd $(SRCDIR)/mac && maxima --very-quiet -b gen.mac
	@echo [maxima end]

$(call LINKLIB, amino, $(OBJS), $(LIBS))
$(call LINKBIN, aa_test, aa_test.o $(OBJS), $(LIBS))
$(call LINKBIN, aa_testpp, aa_testpp.o $(OBJS), $(LIBS) stdc++)

.PHONY: default clean doc test valgrind

valgrind: test
	valgrind ./build/aa_test

doc:
	doxygen

clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) src/mac/*.f $(BUILDDIR)/*.o .deps debian *.deb *.lzma


