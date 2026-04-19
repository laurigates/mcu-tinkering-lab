# Makefile for thinkpack-behaviors host-based unit tests
#
# Compiles command_dispatcher.c, command_executor.c, echo_chamber.c,
# hot_cold.c and nfc_story_sounds.c for Linux/macOS using gcc and runs
# Unity-compatible tests without ESP-IDF or hardware.  Suitable for CI
# and local development.
#
# Usage:
#   make test         — build and run all test binaries (default)
#   make test_runner  — build/run the commands test binary
#   make echo_chamber_test_runner — build/run the echo-chamber test binary
#   make hot_cold_test_runner     — build/run the hot-cold test binary
#   make nfc_story_sounds_test_runner — build/run the story-sounds test binary
#   make clean        — remove build artefacts

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra \
         -Wno-unused-function \
         -I../include \
         -I../../thinkpack-protocol/include \
         -Istubs \
         -I.

COMMON_SRCS   = ../../thinkpack-protocol/thinkpack_protocol.c

COMMANDS_SRCS = test_commands.c \
                ../command_dispatcher.c \
                ../command_executor.c \
                $(COMMON_SRCS)

ECHO_SRCS     = echo_chamber_test.c \
                ../echo_chamber.c \
                $(COMMON_SRCS)

HOT_COLD_SRCS = test_hot_cold.c \
                ../hot_cold.c

STORY_SRCS    = test_nfc_story_sounds.c \
                ../nfc_story_sounds.c

TARGETS = test_runner echo_chamber_test_runner \
          hot_cold_test_runner nfc_story_sounds_test_runner

.PHONY: all test clean

all: test

test: $(TARGETS)
	./test_runner
	./echo_chamber_test_runner
	./hot_cold_test_runner
	./nfc_story_sounds_test_runner

test_runner: $(COMMANDS_SRCS) unity_compat.h stubs/esp_log.h stubs/esp_err.h
	$(CC) $(CFLAGS) -o $@ $(COMMANDS_SRCS)

echo_chamber_test_runner: $(ECHO_SRCS) unity_compat.h
	$(CC) $(CFLAGS) -o $@ $(ECHO_SRCS)

hot_cold_test_runner: $(HOT_COLD_SRCS) unity_compat.h
	$(CC) $(CFLAGS) -o $@ $(HOT_COLD_SRCS)

nfc_story_sounds_test_runner: $(STORY_SRCS) unity_compat.h
	$(CC) $(CFLAGS) -o $@ $(STORY_SRCS)

clean:
	rm -f $(TARGETS)
