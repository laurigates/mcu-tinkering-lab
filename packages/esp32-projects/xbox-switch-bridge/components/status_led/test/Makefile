# Makefile for status_led host-based unit tests
#
# Compiles status_led.c for Linux/macOS using gcc with mocked hardware
# drivers. No ESP-IDF or hardware required.
#
# Usage:
#   make test     — build and run tests (default)
#   make clean    — remove build artefacts

CC     = gcc
CFLAGS = -std=c11 -Wall -Wextra \
         -Wno-unused-function \
         -I../include \
         -Istubs \
         -Istubs/freertos \
         -I.

SRCS   = test_status_led.c mocks.c ../status_led.c
TARGET = test_runner

.PHONY: all test clean

all: test

test: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS) unity_compat.h \
           stubs/esp_err.h stubs/esp_log.h stubs/esp_timer.h \
           stubs/led_strip.h stubs/freertos/FreeRTOS.h stubs/freertos/task.h \
           ../status_led.c
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(TARGET)
