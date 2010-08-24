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
	LD_LIBRARY_PATH=./build ./build/aa_test

# Link against ATLAS blas/lapack
$(call LINKLIB, amino, mem.o la.o tf.o math.o plot.o debug.o, m lapack-3 blas-3 rt)

build/aa_test: build/aa_test.o build/libamino.so
	gcc -o $@ build/aa_test.o -Lbuild -lamino

.PHONY: default clean doc

doc:
	doxygen

clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) $(BUILDDIR)/*.o .dep debian *.deb *.lzma


