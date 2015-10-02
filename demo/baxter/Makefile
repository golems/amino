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

PROGS := baxter-simple baxter-collision baxter-ompl baxter-wksp

ALL: $(PROGS)

baxter-wksp: scene.o baxter-wksp.c
	$(CC) scene.o baxter-wksp.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -lm -lopenblas -o baxter-wksp

baxter-simple: scene.o baxter-simple.c
	$(CC) scene.o baxter-simple.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o baxter-simple

baxter-collision: scene.o baxter-collision.c
	$(CC) scene.o baxter-collision.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o baxter-collision

baxter-ompl: scene.o baxter-ompl.c
	$(CC) scene.o baxter-ompl.c $(CFLAGS_OMPL) $(LDFLAGS_OMPL) -o baxter-ompl

scene.o: scene.c
	$(CC) -c scene.c $(CFLAGS_ALL)

scene.c: scene.robray
	arxc 'package://baxter_description/urdf/baxter.urdf' -o scene.c

clean:
	rm -f $(PROGS)

realclean: clean
	rm -f scene.c *.o
