#*************************************************************************
#Title:    
#Authors:  Michael Petersen
#          
#File:     
#
#*************************************************************************

BASE_NAME = snd-player

GITREV := $(shell git rev-parse --short=6 HEAD)
REV    := $(shell ./git-revision.sh)

REV    = 0.0.0

BOARD     = esp32:esp32:esp32s2:CDCOnBoot=cdc,UploadMode=cdc,PSRAM=enabled
OPTIONS   = --build-property upload.maximum_size=4063232 --build-property compiler.cpp.extra_flags='-DGIT_REV=0x$(GITREV)L -DVERSION_STRING="$(REV)"' --build-property compiler.optimization_flags=-O2
PORT      = /dev/ttyACM0

ARDUINO   = arduino-cli
BUILD_DIR = ./build

COMPILE   = $(ARDUINO) compile -b $(BOARD) --build-path $(BUILD_DIR) $(OPTIONS) $(BASE_NAME) --warnings all

help:
	@echo "make compile.... Build $(BASE_NAME)"
	@echo "make upload..... Flash the firmware"
	@echo "make terminal... Open uart terminal"

# https://arduino.github.io/arduino-cli/0.32/commands/arduino-cli_compile/

compile:
	$(COMPILE)

compile-verbose:
	$(COMPILE) -v

upload:
	# Need to provide board option to reliably upload
	$(ARDUINO) upload -v --input-dir $(BUILD_DIR)  -b $(BOARD) -p $(PORT) $(BASE_NAME)

terminal:
	miniterm.py $(PORT) 115200

clean:
	rm -r $(BUILD_DIR)
