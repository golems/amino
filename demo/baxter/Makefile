CC := gcc --std=gnu99
CXX := g++ --std=gnu++11

## Use pkg-config to determine the approprate compilation flags
PKG_CONFIG_MODULES := amino amino_gl amino_collision sdl2 glew
CFLAGS_ALL := $(CFLAGS) -g `pkg-config --cflags $(PKG_CONFIG_MODULES)` -I../../include/
LDFLAGS_ALL := $(LDLAGS) -g `pkg-config --libs $(PKG_CONFIG_MODULES)` -L../../.libs

OMPL_MODULES := ompl amino_planning
OMPL_LIBS := -lboost_date_time -lboost_system -lboost_chrono -lboost_thread -lboost_serialization
CFLAGS_OMPL := $(CFLAGS_ALL) -g `pkg-config --cflags $(OMPL_MODULES)`
LDFLAGS_OMPL :=  `pkg-config --libs $(OMPL_MODULES)` $(LDFLAGS_ALL) $(OMPL_LIBS)

PROGS := baxter-simple \
	baxter-collision \
	baxter-ompl \
	baxter-ompl-workspace \
	baxter-ompl-sequence \
	baxter-wksp \
	baxter-ik \
	baxter-plugin

OTHER_DEPS := baxter-model.o baxter-demo.c
ALL: $(PROGS)


baxter-ik: baxter-ik.c $(OTHER_DEPS) baxter-demo.h
	$(CC) $(OTHER_DEPS) baxter-ik.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o baxter-ik

baxter-wksp: baxter-wksp.c $(OTHER_DEPS) baxter-demo.h
	$(CC) $(OTHER_DEPS) baxter-wksp.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -lm -lopenblas -o baxter-wksp

baxter-simple: baxter-simple.c $(OTHER_DEPS) baxter-demo.h
	$(CC) $(OTHER_DEPS) baxter-simple.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o baxter-simple

baxter-plugin: baxter-plugin.c $(OTHER_DEPS) baxter-demo.h
	$(CC) $(OTHER_DEPS) baxter-plugin.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o baxter-plugin -ldl

baxter-collision: baxter-collision.c $(OTHER_DEPS) baxter-demo.h
	$(CC) $(OTHER_DEPS) baxter-collision.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o baxter-collision

baxter-ompl: baxter-ompl.c $(OTHER_DEPS) baxter-demo.h
	$(CC) $(OTHER_DEPS) baxter-ompl.c $(CFLAGS_OMPL) $(LDFLAGS_OMPL) -o baxter-ompl

baxter-ompl-workspace: baxter-ompl-workspace.c $(OTHER_DEPS) baxter-demo.h
	$(CC) $(OTHER_DEPS) baxter-ompl-workspace.c $(CFLAGS_OMPL) $(LDFLAGS_OMPL) -o baxter-ompl-workspace

baxter-ompl-sequence: baxter-ompl-sequence.c $(OTHER_DEPS) baxter-demo.h
	$(CC) $(OTHER_DEPS) baxter-ompl-sequence.c $(CFLAGS_OMPL) $(LDFLAGS_OMPL) -o baxter-ompl-sequence

baxter-model.o: baxter-model.c
	$(CC) -c baxter-model.c $(CFLAGS_ALL) -fPIC

baxter-model.c:
	aarxc 'package://baxter_description/urdf/baxter.urdf' -o baxter-model.c -f "baxter_demo_load_baxter"

baxter.so: scene.o
	gcc -shared -o baxter.so scene.o

clean:
	rm -f $(PROGS)

realclean: clean
	rm -f baxter-model.c scene.c *.o
