# Makefile for thinkpack-audio host-based unit tests.
#
# Compiles thinkpack_audio.c on Linux/macOS with gcc and runs the
# Unity-compat host tests.  unity_compat.h and the ESP-IDF stubs come
# from packages/components/host-test.  `just components::test` runs
# every component suite.
#
# Usage:
#   make test
#   make clean

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra \
         -I../include \
         -I../../host-test

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
