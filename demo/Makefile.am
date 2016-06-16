if BUILD_DEMOS

BUILT_SOURCES =

DEMO_LIBS = $(top_builddir)/libamino.la $(top_builddir)/libamino_gl.la $(GL_LIBS) $(SDL2_LIBS) $(GLEW_LIBS)
DEMO_CFLAGS =  $(SDL2_CFLAGS) $(GL_CFLAGS)

###############
## SIMPLE RX ##
###############

# BUILT_SOURCES += simple-rx/scene.c

simple-rx/scene.c: simple-rx/scene.robray
	mkdir -p simple-rx
	LD_LIBRARY_PATH=$(top_builddir)/.libs:$(LD_LIBRARY_PATH) PATH=$(top_srcdir)/share:$(top_srcdir)/lisp:$(PATH) \
		aarxc $(srcdir)/simple-rx/scene.robray -o simple-rx/scene.c

noinst_PROGRAMS = simple-rx/simple-rx

simple_rx_simple_rx_SOURCES = simple-rx/scene.c simple-rx/simple-rx.c
simple_rx_simple_rx_CFLAGS = $(DEMO_CFLAGS)
simple_rx_simple_rx_LDADD = $(DEMO_LIBS)

endif # BUILD_DEMOS
