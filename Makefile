# Usage:
#  run make from this upper directory, above the projects 
#  specify the project with PROJECT=x
#  PROJECT must be in upper case with no dash
#
#  to compile:
#  	make PROJECT=foo
#  to compile and upload to arduino:  
#  	make all PROJECT=foo
#
# The git version is calculated and made into an include file
# to be displayed at runtime.
#
# Ctags and gitversion calculation just come for the ride

PROJECT ?= gps_base

# BOARD_TYPE ?= SparkFun:samd:RedBoard_Turbo
BOARD_TYPE  ?= SparkFun:samd:samd21_proRF
SERIAL_PORT ?= /dev/ttyACM0

ARDUINO_CLI = /usr/bin/arduino-cli
CTAGS       = /usr/bin/ctags

# it'd be nice to send this as a variable into compile but I don't see the option
# so we write it to git_version.h in the project
GIT_VERSION := $(shell git describe --tags --always --dirty 2> /dev/null)

compile: ctags gitversion
		@echo "#define GIT_VERSION \"$(GIT_VERSION)\"" > $(PWD)/$(PROJECT)/git_version.h
		$(ARDUINO_CLI) compile --fqbn $(BOARD_TYPE) $(PROJECT)

upload:
		$(ARDUINO_CLI) upload -p $(SERIAL_PORT) --fqbn $(BOARD_TYPE) $(PROJECT)

ctags:
		@$(CTAGS) -f $(PWD)/$(PROJECT)/tags -R $(PWD)/$(PROJECT)

gitversion:
		@echo $(GIT_VERSION)

all: compile upload

clean:
		@rm -rf $(PWD)/$(PROJECT)/build

.PHONY: compile upload all clean ctags gitversion
