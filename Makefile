# Usage:
#  from this upper directory, above projects run make
#  to compile:
#  	make PROJECT=foo
#  to compile and upload to arduino:  
#  	make all PROJECT=foo
# Ctags will just come for the ride

PROJECT ?= Blink

# BOARD_TYPE ?= SparkFun:samd:RedBoard_Turbo
BOARD_TYPE  ?= SparkFun:samd:samd21_proRF
SERIAL_PORT ?= /dev/ttyACM0

ARDUINO_CLI = /usr/bin/arduino-cli
CTAGS       = /usr/bin/ctags

compile: ctags
		$(ARDUINO_CLI) compile --fqbn $(BOARD_TYPE) $(PROJECT)
		@rm $(PWD)/$(PROJECT)/build/*/*.hex
		@rm $(PWD)/$(PROJECT)/build/*/*.map

upload:
		$(ARDUINO_CLI) upload -p $(SERIAL_PORT) --fqbn $(BOARD_TYPE) $(PROJECT)

all: compile upload

ctags:
	@$(CTAGS) -f $(PWD)/$(PROJECT)/tags -R $(PWD)/$(PROJECT)

clean:
		@rm -rf $(PWD)/$(PROJECT)/build

.PHONY: compile upload all clean ctags
