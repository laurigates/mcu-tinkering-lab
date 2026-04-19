# Makefile for thinkpack-mesh fragment reassembler host-based unit tests
#
# Compiles fragment_reassembler.c + thinkpack_protocol.c under plain gcc
# (no ESP-IDF, no hardware). Stubs supply any missing headers.
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

SRCS   = test_fragment_reassembler.c \
         ../fragment_reassembler.c \
         ../../thinkpack-protocol/thinkpack_protocol.c

TARGET = test_runner

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS) unity_compat.h stubs/esp_log.h
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)
