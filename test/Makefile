# Host golden test for cas_core -- plain C11, no Zephyr needed
# (HANDOFF.md 7.5-4; the core is platform-independent by NFR).

CC      ?= cc
CFLAGS  ?= -O2 -std=c11 -Wall -Wextra
INC     := -I../include
SRC     := ../src/cas/cas_core.c
HDR     := ../include/cas/cas_core.h ../include/cas/cas_preset.h

.PHONY: test clean

test: test_golden
	./test_golden

test_golden: test_golden.c $(SRC) $(HDR)
	$(CC) $(CFLAGS) $(INC) -o $@ test_golden.c $(SRC) -lm

clean:
	rm -f test_golden
