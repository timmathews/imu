CC := gcc
CFLAGS := -g3
LIBS := -lm
OBJDIR := obj

ifeq ($(PREFIX),)
  PREFIX = /usr
endif

LIB_INSTALL_DIR = $(PREFIX)/bin

TOP =	../

VERSION = 0.1

SOURCES =	dcm.c \
		lmath.c \
		main.c \
		lsm9ds0/lsm9ds0.c

OBJECTS = $(SOURCES:.c=.o)

SOURCE_DIST = Makefile

.SUFFIXES: .c .o
.PHONY: all clean install dist

imu: $(OBJECTS)
	$(CC) -o $(TOP)$@ $^ $(CFLAGS) $(LIBS)

all: imu

install: all
	mkdir -p $(LIB_INSTALL_DIR)
	cp $(OBJECTS) $(LIB_INSTALL_DIR)

dist:
	mkdir -p $(TOP)$(SOURCES:.c=)-$(VERSION)/src
	cp $(SOURCE_DIST) $(SOURCES) $(TOP)$(SOURCES)-$(VERSION)/src

clean:
	rm -rf *.o
