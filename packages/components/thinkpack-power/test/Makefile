# Makefile for thinkpack-power host-based unit tests.
#
# Compiles the pure-logic classifier, adaptive beacon mapping, and
# LED pulse animation on Linux/macOS with gcc and runs the Unity-compat
# host tests. Reuses the shared unity_compat.h from thinkpack-behaviors.
#
# power_mgr.c is included but its ESP-IDF code is #ifdef'd out on host
# builds, leaving only the trivial stubs.
#
# Usage:
#   make test
#   make clean

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra \
         -I../include \
         -I../../thinkpack-behaviors/test

SRCS   = test_runner.c \
         ../classify.c \
         ../led_pulse.c \
         ../power_mgr.c

# test_classify.c, test_adaptive_beacon.c and test_led_pulse.c are #included
# from test_runner.c so Unity's static counters stay in a single TU.

TARGET = test_runner

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)
