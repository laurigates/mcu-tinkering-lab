# Makefile for thinkpack-behaviors host-based unit tests
#
# Compiles command_dispatcher.c and command_executor.c for Linux/macOS using
# gcc and runs Unity-compatible tests without ESP-IDF or hardware.
# Suitable for CI and local development.
#
# Usage:
#   make test     — build and run tests (default)
#   make clean    — remove build artefacts

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra \
         -Wno-unused-function \
         -I../include \
         -I../../thinkpack-protocol/include \
         -Istubs \
         -I.

SRCS   = test_commands.c \
         ../command_dispatcher.c \
         ../command_executor.c \
         ../../thinkpack-protocol/thinkpack_protocol.c

TARGET = test_runner

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS) unity_compat.h stubs/esp_log.h stubs/esp_err.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)
