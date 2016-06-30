AM_CPPFLAGS = -I$(top_srcdir)/include

if BUILD_DEMOS


noinst_PROGRAMS =
noinst_LTLIBRARIES =
lib_LTLIBRARIES=

BUILT_SCENES =

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
	$(AARXC) $(srcdir)/simple-rx/scene.robray -o simple-rx/scene.c

BUILT_SCENES += simple-rx/scene.c simple-rx/scene.c.h

noinst_PROGRAMS += simple-rx/simple-scenefile

simple_rx_simple_scenefile_SOURCES = simple-rx/scene.c simple-rx/simple-scenefile.c
simple_rx_simple_scenefile_CFLAGS = $(DEMO_CFLAGS)
simple_rx_simple_scenefile_LDADD = $(DEMO_LIBS)


noinst_PROGRAMS += simple-rx/simple-procedural

simple_rx_simple_procedural_SOURCES = simple-rx/scene.c simple-rx/simple-procedural.c
simple_rx_simple_procedural_CFLAGS = $(DEMO_CFLAGS)
simple_rx_simple_procedural_LDADD = $(DEMO_LIBS)


############
## Baxter ##
############

if BUILD_DEMO_BAXTER

urdf/baxter/baxter-model.c:
	$(AARXC) 'package://baxter_description/urdf/baxter.urdf' \
		-o urdf/baxter/baxter-model.c -n "baxter"

urdf/baxter/baxter-model.c.h: urdf/baxter/baxter-model.c

BUILT_SCENES += urdf/baxter/baxter-model.c urdf/baxter/baxter-model.c.h

noinst_LTLIBRARIES += urdf/baxter/libbaxter.la
urdf_baxter_libbaxter_la_SOURCES = urdf/baxter/baxter-model.c

noinst_LTLIBRARIES += urdf/baxter/libbaxter_demo.la
urdf_baxter_libbaxter_demo_la_SOURCES = urdf/baxter/baxter-demo.c
urdf_baxter_libbaxter_demo_la_CFLAGS = $(DEMO_CFLAGS)
urdf/baxter/libbaxter_demo.$(OBJEXT): urdf/baxter/baxter-model.c.h

lib_LTLIBRARIES += urdf/baxter/libamino_baxter.la
urdf_baxter_libamino_baxter_la_SOURCES =
urdf_baxter_libamino_baxter_la_LIBADD = urdf/baxter/libbaxter.la
urdf_baxter_libamino_baxter_la_LDFLAGS = -avoid-version -module -shared -export-dynamic

noinst_PROGRAMS += urdf/baxter/baxter-simple
urdf_baxter_baxter_simple_SOURCES = urdf/baxter/baxter-simple.c
urdf_baxter_baxter_simple_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_simple_LDADD = urdf/baxter/libbaxter_demo.la $(DEMO_LIBS) urdf/baxter/libbaxter.la
urdf/baxter/baxter-simple.$(OBJEXT): urdf/baxter/baxter-model.c.h

noinst_PROGRAMS += urdf/baxter/baxter-ik
urdf_baxter_baxter_ik_SOURCES = urdf/baxter/baxter-ik.c
urdf_baxter_baxter_ik_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_ik_LDADD = urdf/baxter/libbaxter_demo.la $(DEMO_LIBS) urdf/baxter/libbaxter.la
urdf/baxter/baxter-ik.$(OBJEXT): urdf/baxter/baxter-model.c.h

noinst_PROGRAMS += urdf/baxter/baxter-wksp
urdf_baxter_baxter_wksp_SOURCES = urdf/baxter/baxter-wksp.c
urdf_baxter_baxter_wksp_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_wksp_LDADD = urdf/baxter/libbaxter_demo.la $(DEMO_LIBS) urdf/baxter/libbaxter.la
urdf/baxter/baxter-wksp.$(OBJEXT): urdf/baxter/baxter-model.c.h

noinst_PROGRAMS += urdf/baxter/baxter-collision
urdf_baxter_baxter_collision_SOURCES = urdf/baxter/baxter-collision.c
urdf_baxter_baxter_collision_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_collision_LDADD = urdf/baxter/libbaxter_demo.la $(PLANNING_LIBS) $(DEMO_LIBS) urdf/baxter/libbaxter.la
urdf/baxter/baxter-collision.$(OBJEXT): urdf/baxter/baxter-model.c.h

noinst_PROGRAMS += urdf/baxter/baxter-ompl
urdf_baxter_baxter_ompl_SOURCES = urdf/baxter/baxter-ompl.c urdf/baxter/baxter-demo.c
urdf_baxter_baxter_ompl_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_ompl_LDADD = urdf/baxter/libbaxter_demo.la $(PLANNING_LIBS) $(DEMO_LIBS) urdf/baxter/libbaxter.la
urdf/baxter/baxter-ompl.$(OBJEXT): urdf/baxter/baxter-model.c.h

noinst_PROGRAMS += urdf/baxter/baxter-ompl-workspace
urdf_baxter_baxter_ompl_workspace_SOURCES = urdf/baxter/baxter-ompl-workspace.c
urdf_baxter_baxter_ompl_workspace_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_ompl_workspace_LDADD = urdf/baxter/libbaxter_demo.la $(PLANNING_LIBS) $(DEMO_LIBS) urdf/baxter/libbaxter.la
urdf/baxter/baxter-ompl-workspace.$(OBJEXT): urdf/baxter/baxter-model.c.h

noinst_PROGRAMS += urdf/baxter/baxter-ompl-sequence
urdf_baxter_baxter_ompl_sequence_SOURCES = urdf/baxter/baxter-ompl-sequence.c
urdf_baxter_baxter_ompl_sequence_CFLAGS = $(DEMO_CFLAGS)
urdf_baxter_baxter_ompl_sequence_LDADD = urdf/baxter/libbaxter_demo.la $(PLANNING_LIBS) $(DEMO_LIBS) urdf/baxter/libbaxter.la
urdf/baxter/baxter-ompl-sequence.$(OBJEXT): urdf/baxter/baxter-model.c.h

#noinst_PROGRAMS += urdf/baxter/baxter-plugin
#urdf_baxter_baxter_plugin_SOURCES = urdf/baxter/baxter-plugin.c urdf/baxter/baxter-demo.c
#urdf_baxter_baxter_plugin_CFLAGS = $(DEMO_CFLAGS)
#urdf_baxter_baxter_plugin_LDADD = $(DEMO_LIBS) urdf/baxter/libbaxter.la

endif # BUILD_DEMO_BAXTER


if BUILD_DEMO_UR10

urdf/ur/ur10_robot.urdf:
	rosrun xacro xacro.py \
		`rospack find ur_description`/urdf/ur10_robot.urdf.xacro \
		-o urdf/ur/ur10_robot.urdf


urdf/ur/ur10-model.c: urdf/ur/ur10_robot.urdf
	$(AARXC) urdf/ur/ur10_robot.urdf -o urdf/ur/ur10-model.c -n "ur10"

BUILT_SCENES += urdf/ur/ur10-model.c urdf/ur/ur10-model.c

lib_LTLIBRARIES += urdf/ur/libamino_ur.la
urdf_ur_libamino_ur_la_SOURCES = urdf/ur/ur10-model.c

endif # BUILD_DEMO_UR10


if BUILD_DEMO_SCHUNK

urdf/schunk/lwa4d.urdf:
	rosrun xacro xacro.py urdf/schunk/lwa4d_robot.urdf.xacro \
		-o urdf/schunk/lwa4d.urdf

urdf/schunk/lwa4d-model.c: urdf/schunk/lwa4d.urdf
	$(AARXC) urdf/schunk/lwa4d.urdf \
		-o urdf/schunk/lwa4d-model.c -n "lwa4d"

urdf/schunk/lwa4d-model.c.h: urdf/schunk/lwa4d-model.c

BUILT_SCENES += urdf/schunk/lwa4d-model.c.h urdf/schunk/lwa4d-model.c

lib_LTLIBRARIES += urdf/schunk/libamino_schunk.la
urdf_schunk_libamino_schunk_la_SOURCES = urdf/schunk/lwa4d-model.c
urdf_schunk_libamino_schunk_la_LDFLAGS = -avoid-version -module -shared -export-dynamic

endif # BUILD_DEMO_SCHUNK

if BUILD_DEMO_MICO

urdf/mico/mico_robot.urdf:
	rosrun xacro xacro.py urdf/mico/mico_robot.urdf.xacro \
		-o urdf/mico/mico_robot.urdf

urdf/mico/mico-model.c: urdf/mico/mico_robot.urdf
	$(AARXC) urdf/mico/mico_robot.urdf \
		-o urdf/mico/mico-model.c -n "mico"

BUILT_SCENES += urdf/mico/mico-model.c.h urdf/mico/mico-model.c


lib_LTLIBRARIES += urdf/mico/libamino_mico.la
urdf_mico_libamino_mico_la_SOURCES = urdf/mico/mico-model.c
urdf_mico_libamino_mico_la_LDFLAGS = -avoid-version -module -shared -export-dynamic

endif # BUILD_DEMO_MICO

if BUILD_DEMO_JACO

urdf/jaco/jaco_robot.urdf:
	rosrun xacro xacro.py urdf/jaco/jaco_robot.urdf.xacro \
		-o urdf/jaco/jaco_robot.urdf

urdf/jaco/jaco-model.c: urdf/jaco/jaco_robot.urdf
	$(AARXC) urdf/jaco/jaco_robot.urdf \
		-o urdf/jaco/jaco-model.c -n "jaco"

urdf/jaco/jaco-model.c.h: urdf/jaco/jaco-model.c

BUILT_SCENES += urdf/jaco/jaco-model.c urdf/jaco/jaco-model.c

noinst_LTLIBRARIES += urdf/jaco/libjaco.la
urdf_jaco_libjaco_la_SOURCES = urdf/jaco/jaco-model.c

lib_LTLIBRARIES += urdf/jaco/libamino_jaco.la
urdf_jaco_libamino_jaco_la_SOURCES =
urdf_jaco_libamino_jaco_la_LIBADD = urdf/jaco/libjaco.la
urdf_jaco_libamino_jaco_la_LDFLAGS = -avoid-version -module -shared -export-dynamic

noinst_PROGRAMS += urdf/jaco/jaco-ompl
urdf_jaco_jaco_ompl_SOURCES = urdf/jaco/jaco-ompl.c
urdf_jaco_jaco_ompl_CFLAGS = $(DEMO_CFLAGS)
urdf_jaco_jaco_ompl_LDADD = $(PLANNING_LIBS) $(DEMO_LIBS) urdf/jaco/libjaco.la
urdf/jaco/jaco-ompl.$(OBJEXT): urdf/jaco/jaco-model.c.h

endif # BUILD_DEMO_JACO


## Extra ##

clean-local:
	-rm -f $(BUILT_SCENES)


endif # BUILD_DEMOS
