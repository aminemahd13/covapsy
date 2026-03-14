# Build libreactive_core.so for COVAPSY reactive driving
#
# Usage:
#   make            — build shared library
#   make clean      — remove build artifacts
#   make install    — copy .so alongside Python package
#   make test       — compile and run a quick smoke test

CC       ?= gcc
CFLAGS   := -shared -fPIC -O2 -march=native -Wall -Wextra -Wno-unused-parameter
LDFLAGS  := -lm

SRC      := reactive_core.c
HDR      := reactive_core.h
TARGET   := libreactive_core.so
INSTALL  := ../covapsy_nav/$(TARGET)

.PHONY: all clean install test

all: $(TARGET)

$(TARGET): $(SRC) $(HDR)
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDFLAGS)

install: $(TARGET)
	cp $(TARGET) $(INSTALL)
	@echo "Installed $(INSTALL)"

clean:
	rm -f $(TARGET) $(INSTALL) test_core

test: $(SRC) $(HDR)
	$(CC) -O2 -Wall -o test_core $(SRC) -lm -DTEST_MAIN
	./test_core
	@echo "Smoke test passed."
