# Makefile for robocar-i2c-protocol host-based unit tests
#
# Compiles i2c_protocol.c for Linux/macOS using gcc and runs Unity-compatible
# tests without ESP-IDF or hardware. Suitable for CI and local development.
#
# Usage:
#   make test     — build and run tests (default)
#   make clean    — remove build artefacts

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra \
         -Wno-unused-function \
         -I../include \
         -Istubs \
         -I.

SRCS   = test_i2c_protocol.c ../i2c_protocol.c
TARGET = test_runner

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS) unity_compat.h stubs/esp_log.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)
