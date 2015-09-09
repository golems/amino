CC := gcc --std=gnu99

## Use pkg-config to determine the approprate compilation flags
PKG_CONFIG_MODULES := amino amino_gl sdl2 glew
CFLAGS_ALL := $(CFLAGS) `pkg-config --cflags $(PKG_CONFIG_MODULES)`
LDFLAGS_ALL := $(LDLAGS) `pkg-config --libs $(PKG_CONFIG_MODULES)`

simple-rx: scene.c simple-rx.c
	$(CC) scene.c simple-rx.c $(CFLAGS_ALL) $(LDFLAGS_ALL) -o simple-rx
