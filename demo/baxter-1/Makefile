CC := gcc --std=gnu99

## Use pkg-config to determine the approprate compilation flags
PKG_CONFIG_MODULES := amino amino_gl amino_collision sdl2 glew
CFLAGS_ALL := $(CFLAGS) `pkg-config --cflags $(PKG_CONFIG_MODULES)`
LDFLAGS_ALL := $(LDLAGS) `pkg-config --libs $(PKG_CONFIG_MODULES)`

ALL: baxter-rx baxter-cl

baxter-rx: scene.o baxter-rx.c
	$(CC) scene.o baxter-rx.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o baxter-rx

baxter-cl: scene.o baxter-cl.c
	$(CC) scene.o baxter-cl.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o baxter-cl

scene.o: scene.c
	$(CC) -c scene.c $(CFLAGS_ALL)

scene.c: scene.robray
	arxc 'package://baxter_description/urdf/baxter.urdf' -o scene.c

clean:
	rm -f scene.c baxter-rx
