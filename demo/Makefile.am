AM_CPPFLAGS = -I$(top_srcdir)/include

if BUILD_DEMOS


BUILT_SOURCES =
noinst_PROGRAMS =
noinst_LTLIBRARIES =

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
	mkdir -p simple-rx
		$(AARXC) $(srcdir)/simple-rx/scene.robray -o simple-rx/scene.c

BUILT_SCENES += urdf/simple-rx/scene.c urdf/simple-rx/scene.c.h

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


BUILT_SCENES += urdf/baxter/baxter-model.c urdf/baxter/baxter-model.c.h

noinst_LTLIBRARIES += urdf/baxter/libbaxter.la
urdf_baxter_libbaxter_la_SOURCES = urdf/baxter/baxter-model.c

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


if BUILD_DEMO_UR10

urdf/ur/ur10_robot.urdf:
	rosrun xacro xacro.py \
		`rospack find ur_description`/urdf/ur10_robot.urdf.xacro \
		-o urdf/ur/ur10_robot.urdf


urdf/ur/ur10-model.c: urdf/ur/ur10_robot.urdf
	$(AARXC) urdf/ur/ur10_robot.urdf -o urdf/ur/ur10-model.c -n "ur"

BUILT_SCENES += urdf/ur/ur10-model.c urdf/ur/ur10-model.c

noinst_LTLIBRARIES += urdf/ur/libur10.la
urdf_ur_libur10_la_SOURCES = urdf/ur/ur10-model.c

noinst_PROGRAMS += urdf/ur/ur-view
urdf_ur_ur_view_SOURCES = urdf/ur/ur-view.c
urdf_ur_ur_view_CFLAGS = $(DEMO_CFLAGS)
urdf_ur_ur_view_LDADD = $(DEMO_LIBS) urdf/ur/libur10.la

endif # BUILD_DEMO_UR10


if BUILD_DEMO_SCHUNK

urdf/schunk/lwa4d.urdf:
	rosrun xacro xacro.py urdf/schunk/lwa4d_robot.urdf.xacro \
		-o urdf/schunk/lwa4d.urdf


endif # BUILD_DEMO_SCHUNK

if BUILD_DEMO_MICO

urdf/mico/mico_robot.urdf:
	rosrun xacro xacro.py urdf/mico/mico_robot.urdf.xacro \
		-o urdf/mico/mico_robot.urdf


endif # BUILD_DEMO_MICO

if BUILD_DEMO_JACO

urdf/jaco/jaco_robot.urdf:
	rosrun xacro xacro.py urdf/jaco/jaco_robot.urdf.xacro \
		-o urdf/jaco/jaco_robot.urdf


urdf/jaco/jaco-model.c: urdf/jaco/jaco_robot.urdf
	$(AARXC) urdf/jaco/jaco_robot.urdf \
		-o urdf/jaco/jaco-model.c -n "jaco"

BUILT_SCENES += urdf/jaco/jaco-model.c urdf/jaco/jaco-model.c

noinst_LTLIBRARIES += urdf/jaco/libjaco.la
urdf_jaco_libjaco_la_SOURCES = urdf/jaco/jaco-model.c

noinst_PROGRAMS += urdf/jaco/jaco-view
urdf_jaco_jaco_view_SOURCES = urdf/jaco/jaco-view.c
urdf_jaco_jaco_view_CFLAGS = $(DEMO_CFLAGS)
urdf_jaco_jaco_view_LDADD = $(DEMO_LIBS) urdf/jaco/libjaco.la

endif # BUILD_DEMO_JACO


## Extra ##

clean-local:
	-rm -f $(BUILT_SCENES)


endif # BUILD_DEMOS
