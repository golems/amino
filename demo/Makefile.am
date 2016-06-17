AM_CPPFLAGS = -I$(top_srcdir)/include

if BUILD_DEMOS


BUILT_SOURCES =

## Compilation and Linking flags
#
# the GL_LIBS, GL_CFLAGS, SDL_LIBS, etc. are determined by pkg-config
# during configuration

DEMO_LIBS = $(top_builddir)/libamino.la $(top_builddir)/libamino_gl.la $(GL_LIBS) $(SDL2_LIBS) $(GLEW_LIBS)
DEMO_CFLAGS =  $(SDL2_CFLAGS) $(GL_CFLAGS)

# call aarxc with paths set to look in the build and source trees
AARXC = LD_LIBRARY_PATH=$(top_builddir)/.libs:$(LD_LIBRARY_PATH) PATH=$(top_srcdir)/lisp:$(PATH) $(top_srcdir)/share/aarxc

###############
## SIMPLE RX ##
###############

simple-rx/scene.c: simple-rx/scene.robray
	mkdir -p simple-rx
		$(AARXC) $(srcdir)/simple-rx/scene.robray -o simple-rx/scene.c

noinst_PROGRAMS = simple-rx/simple-rx

simple_rx_simple_rx_SOURCES = simple-rx/scene.c simple-rx/simple-rx.c
simple_rx_simple_rx_CFLAGS = $(DEMO_CFLAGS)
simple_rx_simple_rx_LDADD = $(DEMO_LIBS)

clean-local:
	-rm -f simple-rx/scene.c simple-rx/scene.c.h

endif # BUILD_DEMOS
