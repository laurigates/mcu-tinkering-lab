# Makefile for status_led host-based unit tests
#
# Compiles status_led.c for Linux/macOS using gcc with mocked hardware
# drivers. No ESP-IDF or hardware required. unity_compat.h, esp_log.h and
# esp_err.h come from packages/components/host-test; stubs/ holds the
# drivers only this suite mocks. `just components::test` runs every
# component suite.
#
# Usage:
#   make test     — build and run tests (default)
#   make clean    — remove build artefacts

CC     = gcc
HOST_TEST = ../../../../../components/host-test
CFLAGS = -std=c11 -Wall -Wextra \
         -Wno-unused-function \
         -I../include \
         -Istubs \
         -Istubs/freertos \
         -I$(HOST_TEST)

SRCS   = test_status_led.c mocks.c ../status_led.c
TARGET = test_runner

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS) $(HOST_TEST)/unity_compat.h \
           $(HOST_TEST)/esp_err.h $(HOST_TEST)/esp_log.h stubs/esp_timer.h \
           stubs/led_strip.h stubs/freertos/FreeRTOS.h stubs/freertos/task.h \
           ../status_led.c
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)
