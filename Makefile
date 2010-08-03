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

$(call LINKLIB, amino, mem.o la.o tf.o math.o)
$(call LINKBIN, aa_test, aa_test.o mem.o la.o tf.o math.o, m blas lapack)

.PHONY: default clean doc

doc:
	doxygen

clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) $(BUILDDIR)/*.o .dep debian *.deb *.lzma


