SRC =		./src

BIN_DIST =	ChangeLog \
		INSTALL \
		LICENSE

SOURCE_DIST = 	Makefile

TARGET = imu

VERSION = 0.1

all:
	$(MAKE) -C $(SRC) -f Makefile

install: all
	$(MAKE) -C $(SRC) -f Makefile install

dist:
	mkdir -p $(TARGET)-$(VERSION)
	$(MAKE) -C $(SRC) -f Makefile dist
	cp $(BIN_DIST) $(SOURCE_DIST) $(TARGET)-$(VERSION)
	tar -czf $(TARGET)-$(VERSION).tar.gz $(TARGET)-$(VERSION)
	rm -rf $(TARGET)-$(VERSION)

clean:
	$(MAKE) -C $(SRC) -f Makefile clean
	rm -f *.tar.gz
	rm $(TARGET)
