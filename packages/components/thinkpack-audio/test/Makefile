# Makefile for thinkpack-audio host-based unit tests.
#
# Compiles thinkpack_audio.c on Linux/macOS with gcc and runs the
# Unity-compat host tests.  Reuses the shared stubs and unity_compat.h
# from packages/components/thinkpack-behaviors/test.
#
# Usage:
#   make test
#   make clean

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra \
         -I../include \
         -I../../thinkpack-behaviors/test \
         -I../../thinkpack-behaviors/test/stubs

SRCS   = test_audio.c \
         ../thinkpack_audio.c

TARGET = test_runner

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)
