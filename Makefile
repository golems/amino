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
CFLAGS += --std=gnu99
FFLAGS += -I/usr/include

default: $(LIBFILES) $(BINFILES)

$(call LINKLIB, amino, mem.o la.o tf.o)

.PHONY: default clean doc

doc:
	doxygen

clean:
	rm -fr *.o $(BINFILES) $(LIBFILES) $(BUILDDIR)/*.o .dep debian *.deb *.lzma


