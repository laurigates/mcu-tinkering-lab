# Makefile for thinkpack-behaviors host-based unit tests
#
# Compiles command_dispatcher.c, command_executor.c, echo_chamber.c,
# hot_cold.c and nfc_story_sounds.c for Linux/macOS using gcc and runs
# Unity-compatible tests without ESP-IDF or hardware.  unity_compat.h and
# the ESP-IDF stubs come from packages/components/host-test, which the
# other component suites share.  `just components::test` runs them all.
#
# Usage:
#   make test         — build and run all test binaries (default)
#   make test_runner  — build/run the commands test binary
#   make echo_chamber_test_runner — build/run the echo-chamber test binary
#   make hot_cold_test_runner     — build/run the hot-cold test binary
#   make nfc_story_sounds_test_runner — build/run the story-sounds test binary
#   make clean        — remove build artefacts

CC     = gcc
HOST_TEST = ../../host-test
CFLAGS = -std=c11 -Wall -Wextra \
         -Wno-unused-function \
         -I../include \
         -I../../thinkpack-protocol/include \
         -I$(HOST_TEST)

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

test_runner: $(COMMANDS_SRCS) $(HOST_TEST)/unity_compat.h \
             $(HOST_TEST)/esp_log.h $(HOST_TEST)/esp_err.h
	$(CC) $(CFLAGS) -o $@ $(COMMANDS_SRCS)

echo_chamber_test_runner: $(ECHO_SRCS) $(HOST_TEST)/unity_compat.h
	$(CC) $(CFLAGS) -o $@ $(ECHO_SRCS)

hot_cold_test_runner: $(HOT_COLD_SRCS) $(HOST_TEST)/unity_compat.h
	$(CC) $(CFLAGS) -o $@ $(HOT_COLD_SRCS)

nfc_story_sounds_test_runner: $(STORY_SRCS) $(HOST_TEST)/unity_compat.h
	$(CC) $(CFLAGS) -o $@ $(STORY_SRCS)

clean:
	rm -f $(TARGETS)
