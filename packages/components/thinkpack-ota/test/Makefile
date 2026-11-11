# Makefile for thinkpack-ota host-based unit tests.
#
# Compiles the pure-logic sources on the host with gcc and runs the
# Unity-compat tests. unity_compat.h and the ESP-IDF stubs come from
# packages/components/host-test. `just components::test` runs every
# component suite.
#
# The apply.c and ota_receiver.c sources are ESP-only (guarded by
# #ifdef ESP_PLATFORM) so they are omitted from the host build.
#
# Usage:
#   make test
#   make clean

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra \
         -I../include \
         -I../../thinkpack-protocol/include \
         -I../../host-test

COMMON_SRCS = ../chunk_state.c \
              ../manifest_parser.c \
              ../sha256_verify.c

TARGETS = test_chunk_state test_manifest_parser test_sha256_verify

.PHONY: all test clean

all: test

test: $(TARGETS)
	@set -e; for t in $(TARGETS); do \
	    echo "==> $$t"; ./$$t; \
	done

test_chunk_state: test_chunk_state.c $(COMMON_SRCS)
	$(CC) $(CFLAGS) -o $@ $^

test_manifest_parser: test_manifest_parser.c $(COMMON_SRCS)
	$(CC) $(CFLAGS) -o $@ $^

test_sha256_verify: test_sha256_verify.c $(COMMON_SRCS)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGETS)
