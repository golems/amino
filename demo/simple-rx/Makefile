CC := gcc --std=gnu99

## Use pkg-config to determine the approprate compilation flags
PKG_CONFIG_MODULES := amino amino_gl sdl2 glew
CFLAGS_ALL := $(CFLAGS) `pkg-config --cflags $(PKG_CONFIG_MODULES)`
LDFLAGS_ALL := $(LDLAGS) `pkg-config --libs $(PKG_CONFIG_MODULES)`

default: simple-rx sync

simple-rx: scene.o simple-rx.c
	$(CC) scene.o simple-rx.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o simple-rx


sync: scene.o sync.c
	$(CC) scene.o sync.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o sync

scene.o: scene.c
	$(CC) -c scene.c $(CFLAGS_ALL)

scene.c: scene.robray
	aarxc scene.robray -o scene.c

clean:
	rm -f scene.c simple-rx
