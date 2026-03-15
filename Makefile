VERSION = 1.3.2
CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -pedantic -std=gnu99 -DVERSION=\"$(VERSION)\"
LDFLAGS = -lm

# Cross-platform
ifeq ($(OS),Windows_NT)
    TARGET = bin/chal.exe
    RM     = powershell -Command "Remove-Item -Path $(TARGET) -ErrorAction SilentlyContinue"
    MKDIR  = powershell -Command "if(-not(Test-Path bin)){New-Item -ItemType Directory -Path bin}"
else
    TARGET = bin/chal
    RM     = rm -f
    MKDIR  = mkdir -p bin
endif

.PHONY: all debug clean perft

all: $(TARGET)

debug: CFLAGS = -g -O0 -Wall -Wextra -pedantic -std=gnu99 -DVERSION=\"$(VERSION)\"
debug: $(TARGET)

$(TARGET): src/chal.c
	$(MKDIR)
	$(CC) $(CFLAGS) src/chal.c -o $(TARGET) $(LDFLAGS)

perft: $(TARGET)
	$(TARGET) perft 6

clean:
	$(RM) $(TARGET)
