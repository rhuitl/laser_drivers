all: installed
TARBALL = build/743981833845e9ffbd0041bf5a2c04a277454cc1
TARBALL_URL = https://github.com/rhuitl/sicktoolbox/tarball/743981833845e9ffbd0041bf5a2c04a277454cc1
SOURCE_DIR = build/rhuitl-sicktoolbox-7439818/trunk
MD5SUM_FILE = 743981833845e9ffbd0041bf5a2c04a277454cc1.md5sum
UNPACK_CMD = tar xzf
#TARBALL_PATCH = stdlib_include.patch

include $(shell rospack find mk)/download_unpack_build.mk

# override the extraction routing in download_unpack_build.mk
$(SOURCE_DIR)/unpacked: $(TARBALL_PATCH)
	-mkdir -p build
ifneq ($(strip $(MD5SUM_FILE)),)
	if [ ! -f $(MD5SUM_FILE) ]; then echo "Error: Couldn't find md5sum file $(MD5SUM_FILE)" && false; fi
	`rospack find rosbuild`/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL) `awk {'print $$1'} $(MD5SUM_FILE)`
else
	`rospack find rosbuild`/bin/download_checkmd5.py $(TARBALL_URL) $(TARBALL)
endif
	touch -c $(TARBALL)
	rm -rf $(SOURCE_DIR) $(INITIAL_DIR)
ifneq ($(strip $(UNPACK_CMD)),)
	cd build; $(UNPACK_CMD) ../$(TARBALL)
else
	cd build; tar xzf ../$(TARBALL)
endif
ifneq ($(strip $(INITIAL_DIR)),)
	mv $(INITIAL_DIR) $(SOURCE_DIR)
endif
ifneq ($(strip $(TARBALL_PATCH)),)
	cd $(SOURCE_DIR) && $(foreach patch,$(TARBALL_PATCH), patch -p0 < ../../../$(patch);)
endif
	touch $(SOURCE_DIR)/unpacked


installed: wiped $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && autoreconf -i && ./configure --prefix=`rospack find sicktoolbox2`/sicktoolbox
	cd $(SOURCE_DIR) && make && make install
	touch installed

wiped: Makefile
	make wipe
	touch wiped

clean:
	-rm -rf sicktoolbox-1.0 sicktoolbox build/rhuitl-sicktoolbox-9a43f4f
	rm -f *~ installed

.PHONY : clean wipe

wipe: clean
	rm -f build/9a43f4ffc2aee65f669430ae05b7e132ec37888d
