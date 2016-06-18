AM_CPPFLAGS = -I$(top_srcdir)/include

if BUILD_DEMOS


BUILT_SOURCES =
noinst_PROGRAMS =
noinst_LTLIBRARIES =

## Compilation and Linking flags
#
# the GL_LIBS, GL_CFLAGS, SDL_LIBS, etc. are determined by pkg-config
# during configuration

DEMO_LIBS = $(top_builddir)/libamino.la $(top_builddir)/libamino_gl.la $(GL_LIBS) $(SDL2_LIBS) $(GLEW_LIBS)
DEMO_CFLAGS =  $(SDL2_CFLAGS) $(GL_CFLAGS)

PLANNING_LIBS = $(top_builddir)/libamino_planning.la $(top_builddir)/libamino_cl.la

# call aarxc with paths set to look in the build and source trees
AARXC = LD_LIBRARY_PATH=$(top_builddir)/.libs:$(LD_LIBRARY_PATH) PATH=$(top_srcdir)/lisp:$(PATH) $(top_srcdir)/share/aarxc

###############
## SIMPLE RX ##
###############

simple-rx/scene.c: simple-rx/scene.robray
	mkdir -p simple-rx
		$(AARXC) $(srcdir)/simple-rx/scene.robray -o simple-rx/scene.c

noinst_PROGRAMS += simple-rx/simple-scenefile

simple_rx_simple_scenefile_SOURCES = simple-rx/scene.c simple-rx/simple-scenefile.c
simple_rx_simple_scenefile_CFLAGS = $(DEMO_CFLAGS)
simple_rx_simple_scenefile_LDADD = $(DEMO_LIBS)


noinst_PROGRAMS += simple-rx/simple-procedural

simple_rx_simple_procedural_SOURCES = simple-rx/scene.c simple-rx/simple-procedural.c
simple_rx_simple_procedural_CFLAGS = $(DEMO_CFLAGS)
simple_rx_simple_procedural_LDADD = $(DEMO_LIBS)

clean-local:
	-rm -f simple-rx/scene.c simple-rx/scene.c.h


############
## Baxter ##
############

if BUILD_DEMO_BAXTER

urdf/baxter/baxter-model.c:
	$(AARXC) 'package://baxter_description/urdf/baxter.urdf' \
		-o urdf/baxter/baxter-model.c -n "baxter"

noinst_LTLIBRARIES += urdf/baxter/libbaxter.la
urdf_baxter_libbaxter_la_SOURCES = urdf/baxter/baxter-model.c
# TODO: fix code generation to emit single floats when appropriate
urdf_baxter_libbaxter_la_CFLAGS = -Wno-float-conversion

noinst_PROGRAMS += urdf/baxter/baxter-simple
urdf_baxter_baxter_simple_SOURCES = urdf/baxter/baxter-simple.c urdf/baxter/baxter-demo.c
urdf_baxter_baxter_simple_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_simple_LDADD = $(DEMO_LIBS) urdf/baxter/libbaxter.la

noinst_PROGRAMS += urdf/baxter/baxter-ik
urdf_baxter_baxter_ik_SOURCES = urdf/baxter/baxter-ik.c urdf/baxter/baxter-demo.c
urdf_baxter_baxter_ik_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_ik_LDADD = $(DEMO_LIBS) urdf/baxter/libbaxter.la

noinst_PROGRAMS += urdf/baxter/baxter-wksp
urdf_baxter_baxter_wksp_SOURCES = urdf/baxter/baxter-wksp.c urdf/baxter/baxter-demo.c
urdf_baxter_baxter_wksp_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_wksp_LDADD = $(DEMO_LIBS) urdf/baxter/libbaxter.la

noinst_PROGRAMS += urdf/baxter/baxter-collision
urdf_baxter_baxter_collision_SOURCES = urdf/baxter/baxter-collision.c urdf/baxter/baxter-demo.c
urdf_baxter_baxter_collision_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_collision_LDADD = $(PLANNING_LIBS) $(DEMO_LIBS) urdf/baxter/libbaxter.la

noinst_PROGRAMS += urdf/baxter/baxter-ompl
urdf_baxter_baxter_ompl_SOURCES = urdf/baxter/baxter-ompl.c urdf/baxter/baxter-demo.c
urdf_baxter_baxter_ompl_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_ompl_LDADD = $(PLANNING_LIBS) $(DEMO_LIBS) urdf/baxter/libbaxter.la

noinst_PROGRAMS += urdf/baxter/baxter-ompl-workspace
urdf_baxter_baxter_ompl_workspace_SOURCES = urdf/baxter/baxter-ompl-workspace.c urdf/baxter/baxter-demo.c
urdf_baxter_baxter_ompl_workspace_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_ompl_workspace_LDADD = $(PLANNING_LIBS) $(DEMO_LIBS) urdf/baxter/libbaxter.la

noinst_PROGRAMS += urdf/baxter/baxter-ompl-sequence
urdf_baxter_baxter_ompl_sequence_SOURCES = urdf/baxter/baxter-ompl-sequence.c urdf/baxter/baxter-demo.c
urdf_baxter_baxter_ompl_sequence_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_ompl_sequence_LDADD = $(PLANNING_LIBS) $(DEMO_LIBS) urdf/baxter/libbaxter.la

#noinst_PROGRAMS += urdf/baxter/baxter-plugin
#urdf_baxter_baxter_plugin_SOURCES = urdf/baxter/baxter-plugin.c urdf/baxter/baxter-demo.c
#urdf_baxter_baxter_plugin_CFLAGS = $(DEMO_CFLAGS)
#urdf_baxter_baxter_plugin_LDADD = $(DEMO_LIBS) urdf/baxter/libbaxter.la

endif # BUILD_DEMO_BAXTER

endif # BUILD_DEMOS
