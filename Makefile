# Usage:
#  from this upper directory, above projects run make
#  to compile:
#  	make PROJECT=foo
#  to compile and upload to arduino:  
#  	make all PROJECT=foo
# Ctags will just come for the ride

PROJECT ?= gps_base

# BOARD_TYPE ?= SparkFun:samd:RedBoard_Turbo
BOARD_TYPE  ?= SparkFun:samd:samd21_proRF
SERIAL_PORT ?= /dev/ttyACM0

ARDUINO_CLI = /usr/bin/arduino-cli
CTAGS       = /usr/bin/ctags

# it'd be nice to send this as a variable into compile but I don't see the option
# so we write it to git-version.h in the project
GIT_VERSION := $(shell git describe --tags --always --dirty --abbrev=4 2> /dev/null)

compile: ctags
		@echo "#define GIT_VERSION \"$(GIT_VERSION)\"" > $(PWD)/$(PROJECT)/git-version.h
		$(ARDUINO_CLI) compile --fqbn $(BOARD_TYPE) $(PROJECT)

upload:
		$(ARDUINO_CLI) upload -p $(SERIAL_PORT) --fqbn $(BOARD_TYPE) $(PROJECT)

ctags:
		@$(CTAGS) -f $(PWD)/$(PROJECT)/tags -R $(PWD)/$(PROJECT)

all: compile upload

clean:
		@rm -rf $(PWD)/$(PROJECT)/build

.PHONY: compile upload all clean ctags
