TARGET = imu

CC := gcc
CFLAGS := -g3 -std=gnu99 -Wall -I./include
LIBS := -lm
OBJDIR = obj
VERSION = 0.1

ifeq ($(PREFIX),)
  PREFIX = /usr
endif

INSTALL_DIR = $(PREFIX)/bin

SRC = src

VPATH =		src:src/lsm9ds0

SOURCES =	dcm.c \
		lmath.c \
		main.c \
		lsm9ds0.c

OBJS = $(patsubst %,$(OBJDIR)/%,$(SOURCES:.c=.o))

BIN_DIST =	ChangeLog \
		INSTALL \
		LICENSE

SOURCE_DIST = 	Makefile

all: $(OBJS)
	$(CC) -o $(TARGET) $^ $(CFLAGS) $(LIBS)

$(OBJDIR)/%.o: %.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR):
	mkdir -p $(OBJDIR)

.PHONY: install dist clean

install:
	install $(TARGET) $(INSTALL_DIR)

dist:
	mkdir -p $(TARGET)-$(VERSION)
	cp $(BIN_DIST) $(SOURCE_DIST) $(TARGET)-$(VERSION)
	tar -czf $(TARGET)-$(VERSION).tar.gz $(TARGET)-$(VERSION)
	rm -rf $(TARGET)-$(VERSION)

clean:
	rm -rf $(TARGET) *.tar.gz obj/
