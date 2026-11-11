# Makefile for thinkpack-protocol host-based unit tests
#
# Compiles thinkpack_protocol.c for Linux/macOS using gcc and runs Unity-compatible
# tests without ESP-IDF or hardware. unity_compat.h and the ESP-IDF stubs
# come from packages/components/host-test. `just components::test` runs
# every component suite.
#
# Usage:
#   make test     — build and run tests (default)
#   make clean    — remove build artefacts

CC     = gcc
HOST_TEST = ../../host-test
CFLAGS = -std=c11 -Wall -Wextra \
         -Wno-unused-function \
         -I../include \
         -I$(HOST_TEST)

SRCS   = test_thinkpack_protocol.c ../thinkpack_protocol.c
TARGET = test_runner

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS) $(HOST_TEST)/unity_compat.h $(HOST_TEST)/esp_log.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)
