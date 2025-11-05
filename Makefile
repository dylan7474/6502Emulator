# Makefile.linux - Compiled for Linux (requires SDL2, SDL2_mixer, libcurl, SDL2_ttf)

CC = gcc
TARGET = 6502
SRCS = 6502.c
# CFLAGS uses sdl2-config to find include paths and adds optimization/warnings
CFLAGS = -Wall -O2 `sdl2-config --cflags`

# LDFLAGS uses sdl2-config to find libraries and links to necessary SDL and external libraries
# ADDED: -lSDL2_ttf for text rendering
LDFLAGS = `sdl2-config --libs` -lSDL2_mixer -lSDL2_ttf -lcurl -lm

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
