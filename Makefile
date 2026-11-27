# Makefile - Linux build (requires SDL2 only)

CC = gcc
TARGET = 6502
SRCS = 6502.c
# CFLAGS uses sdl2-config to find include paths and adds optimization/warnings
CFLAGS = -Wall -O2 `sdl2-config --cflags`

# LDFLAGS uses sdl2-config to find libraries and links to necessary SDL and external libraries
LDFLAGS = `sdl2-config --libs` -lm

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)
