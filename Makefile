# Project Name
PROJECT := amino

# Project Version
VERSION := 20100727

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

test: build/aa_test
	./build/aa_test

OBJS :=  mem.o la.o tf.o math.o plot.o debug.o mac/mac.o
BOBJS := $(addprefix build/, $(OBJS))

$(SRCDIR)/mac/mac.f: $(SRCDIR)/mac/amino.mac $(SRCDIR)/mac/gen.mac
	cd $(SRCDIR)/mac && maxima --very-quiet -b gen.mac

# Link against ATLAS blas/lapack
$(call LINKLIB, amino, $(OBJS), m lapack-3 blas-3 rt)

build/aa_test: build/aa_test.o $(BOBJS)
	gcc -o $@ $< $(BOBJS)  -lm -llapack-3 -lblas-3 -lrt

.PHONY: default clean doc

doc:
	doxygen

clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) src/mac/*.f $(BUILDDIR)/*.o .dep debian *.deb *.lzma


