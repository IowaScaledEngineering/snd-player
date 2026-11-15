#*************************************************************************
#Title:    
#Authors:  Michael Petersen
#          
#File:     
#
#*************************************************************************

BASE_NAME = snd-player

ARD_ESP32_VER = 3.3.12

GITREV := $(shell git rev-parse --short=6 HEAD)
REV    := $(shell ./git-revision.sh)

BOARD     = esp32:esp32:esp32s2:CDCOnBoot=cdc,UploadMode=cdc,PSRAM=enabled
OPTIONS   = --build-property upload.maximum_size=4063232 --build-property compiler.cpp.extra_flags='-DGIT_REV=0x$(GITREV)L -DVERSION_STRING="$(REV)"' --build-property compiler.optimization_flags=-O2
PORT      = /dev/ttyACM0

ARDUINO   = arduino-cli
BUILD_DIR = ./build

COMPILE   = $(ARDUINO) compile -b $(BOARD) --build-path $(BUILD_DIR) $(OPTIONS) $(BASE_NAME) --warnings all

define ESP_TOUCH_RESET
	@echo "Putting ESP32-S2 into bootloader mode..."
	@python3 -c "import serial; s = serial.Serial('$(PORT)', 1200); s.close()"
	@bash -c 'sleep 0.2'
	@echo "Waiting for port $(PORT) to reappear..."
	@bash -c 'for i in {1..10}; do [ -e $(PORT) ] && exit 0; sleep 0.3; done; echo "Port $(PORT) did not reappear!"; exit 1'
	@bash -c 'sleep 0.5; echo "Done!"'
endef

.PHONY: help compile compile-verbose upload upload-fresh release flash terminal clean

help:
	@echo "make compile....... Build $(BASE_NAME)"
	@echo "make upload........ Flash the application firmware"
	@echo "make upload-fresh.. Flash all the firmware"
	@echo "make release....... Create merged .bin file"
	@echo "make flash......... Flash merged .bin file, provide file as BIN variable"
	@echo "make terminal...... Open uart terminal"

# https://arduino.github.io/arduino-cli/0.32/commands/arduino-cli_compile/

compile:
	$(COMPILE)

compile-verbose:
	$(COMPILE) -v

# Routine upload: Only flashes the sketch binary (preserves preferences/NVS)
upload:
	$(ESP_TOUCH_RESET)
	esptool --chip esp32s2 --port $(PORT) --baud 921600 --before default-reset --after hard-reset write-flash \
		0x10000 $(BUILD_DIR)/$(BASE_NAME).ino.bin

# Fresh board upload: Flashes bootloader, partition table, boot app, and sketch
upload-fresh:
	$(ESP_TOUCH_RESET)
	esptool --chip esp32s2 --port $(PORT) --baud 921600 --before default-reset --after hard-reset write-flash \
		0x1000 $(BUILD_DIR)/$(BASE_NAME).ino.bootloader.bin \
		0x8000 $(BUILD_DIR)/$(BASE_NAME).ino.partitions.bin \
		0xe000 $(BUILD_DIR)/boot_app0.bin \
		0x10000 $(BUILD_DIR)/$(BASE_NAME).ino.bin

release:
	esptool --chip esp32s2 merge-bin -o releases/$(BASE_NAME)-$(REV)-$(GITREV).bin --flash-mode keep --flash-freq keep --flash-size keep --format hex 0x1000 $(BUILD_DIR)/$(BASE_NAME).ino.bootloader.bin 0x8000 $(BUILD_DIR)/$(BASE_NAME).ino.partitions.bin 0xe000 $(BUILD_DIR)/boot_app0.bin 0x10000 $(BUILD_DIR)/$(BASE_NAME).ino.bin

flash:
	$(ESP_TOUCH_RESET)
	esptool --chip esp32s2 --port $(PORT) --baud 921600 --before default-reset --after hard-reset write-flash 0x0 $(BIN)
	
terminal:
	miniterm.py $(PORT) 115200

clean:
	rm -r $(BUILD_DIR)
