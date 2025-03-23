/*************************************************************************
Title:    Sound Player
Authors:  Michael Petersen <railfan@drgw.net>
File:     snd-player.ino
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2024 Michael Petersen

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <SPI.h>
#include <SD.h>
#include <Preferences.h>
#include <vector>
#include <strings.h>
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"

#include "sound.h"

#include "squeal/flangeClip01.h"
#include "squeal/flangeClip02.h"
#include "squeal/flangeClip03.h"

// 3 sec watchdog 
#define TWDT_TIMEOUT_MS    3000

// Samples for each buffer
#define AUDIO_BUFFER_SIZE   512
#define AUDIO_BUFFER_NUM      4

i2s_chan_handle_t i2s_tx_handle;

// Pins
#define EN1         9
#define EN2        10
#define EN3         3
#define EN4        21
#define LEDA       11
#define LEDB       12
#define VOLDN      13
#define VOLUP      14
#define AUX1       15
#define AUX2       16
#define AUX3       17
#define AUX4       18
#define AUX5        8
#define I2S_SD      4
#define I2S_DOUT    GPIO_NUM_5
#define I2S_BCLK    GPIO_NUM_6
#define I2S_LRCLK   GPIO_NUM_7
#define SDCLK      36
#define SDMOSI     35
#define SDMISO     37
#define SDCS       34
#define SDDET      33

// Bit positions for inputs
#define VOL_UP_BUTTON   0x01
#define VOL_DN_BUTTON   0x02
#define EN1_INPUT       0x10
#define EN2_INPUT       0x20
#define EN3_INPUT       0x40
#define EN4_INPUT       0x80

// Volume
#define VOL_STEP_MAX   30
#define VOL_STEP_NOM   20

uint8_t volumeStep = 0;
uint16_t volume = 0;
uint8_t volumeUpCoef = 0;
uint8_t volumeDownCoef = 0;

uint16_t volumeLevels[] = {
		0,      // 0
		100,
		200,
		300,
		400,
		500,
		600,
		700,
		800,
		900,
		1000,   // 10
		1900,
		2800,
		3700,
		4600,
		5500,
		6400,
		7300,
		8200,
		9100,
		10000,  // 20
		11000,
		12000,
		13000,
		14000,
		15000,
		16000,
		17000,
		18000,
		19000,
		20000,  // 30
};

bool restart = false;

bool unmute = false;
uint8_t silenceDecisecsMax = 0;
uint8_t silenceDecisecsMin = 0;

Preferences preferences;

Sound *wavSoundNext;
Sound *wavSound;

uint8_t debounce(uint8_t debouncedState, uint8_t newInputs)
{
	static uint8_t clock_A = 0, clock_B = 0;
	uint8_t delta = newInputs ^ debouncedState; // Find all of the changes
	uint8_t changes;

	clock_A ^= clock_B; //Increment the counters
	clock_B  = ~clock_B;

	clock_A &= delta; //Reset the counters if no changes
	clock_B &= delta; //were detected.

	changes = ~((~delta) | clock_A | clock_B);
	debouncedState ^= changes;
	return(debouncedState);
}

char* rtrim(char* in)
{
	char* endPtr = in + strlen(in) - 1;
	while (endPtr >= in && isspace(*endPtr))
		*endPtr-- = 0;

	return in;
}

char* ltrim(char* in)
{
	char* startPtr = in;
	uint32_t bytesToMove = strlen(in);
	while(isspace(*startPtr))
		startPtr++;
	bytesToMove -= (startPtr - in);
	memmove(in, startPtr, bytesToMove);
	in[bytesToMove] = 0;
	return in;
}

bool configKeyValueSplit(char* key, uint32_t keySz, char* value, uint32_t valueSz, const char* configLine)
{
	char lineBuffer[256];
	char* separatorPtr = NULL;
	char* lineBufferPtr = NULL;
	uint32_t bytesToCopy;

	separatorPtr = strchr(configLine, '=');
	if (NULL == separatorPtr)
		return false;

	memset(key, 0, keySz);
	memset(value, 0, valueSz);

	// Copy the part that's eligible to be a key into the line buffer
	bytesToCopy = separatorPtr - configLine;
	if (bytesToCopy > sizeof(lineBuffer)-1)
		bytesToCopy = sizeof(lineBuffer);
	memset(lineBuffer, 0, sizeof(lineBuffer));
	strncpy(lineBuffer, configLine, bytesToCopy);

	lineBufferPtr = ltrim(rtrim(lineBuffer));
	if (0 == strlen(lineBufferPtr) || '#' == lineBufferPtr[0])
		return false;

	strncpy(key, lineBufferPtr, keySz);

//	bytesToCopy = strlen(separatorPtr+1);
//	if (bytesToCopy > sizeof(lineBuffer)-1)
//		bytesToCopy = sizeof(lineBuffer);
	memset(lineBuffer, 0, sizeof(lineBuffer));
	// Changed to sizeof(lineBuffer)-1 below instead of bytesToCopy due to -Werror=stringop-overflow and -Werror=stringop-truncation
	strncpy(lineBuffer, separatorPtr+1, sizeof(lineBuffer)-1);
	lineBufferPtr = ltrim(rtrim(lineBuffer));
	if (0 == strlen(lineBufferPtr))
	{
		memset(key, 0, keySz);
		return false;
	}
	strncpy(value, lineBufferPtr, valueSz);
	return true;
}

hw_timer_t * timer = NULL;
volatile bool timerTick = false;

void IRAM_ATTR processVolume(void)
{
	timerTick = true;
}

void setup()
{
	// Open serial communications and wait for port to open:
	Serial.begin(115200);

	pinMode(VOLDN, INPUT_PULLUP);
	pinMode(VOLUP, INPUT_PULLUP);
	pinMode(I2S_SD, OUTPUT);
	digitalWrite(I2S_SD, 0);	// Disable amplifier

	pinMode(LEDA, OUTPUT);
	pinMode(LEDB, OUTPUT);
	digitalWrite(LEDA, 0);
	digitalWrite(LEDB, 0);

	pinMode(EN1, INPUT_PULLUP);
	pinMode(EN2, INPUT_PULLUP);
	pinMode(EN3, INPUT_PULLUP);
	pinMode(EN4, INPUT_PULLUP);

	pinMode(AUX1, OUTPUT);
	pinMode(AUX2, OUTPUT);
	pinMode(AUX3, OUTPUT);
	pinMode(AUX4, OUTPUT);
	pinMode(AUX5, OUTPUT);

	esp_task_wdt_config_t twdt_config = {
		.timeout_ms = TWDT_TIMEOUT_MS,
		.idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1,    // Bitmask of all cores
		.trigger_panic = false,
	};
	esp_task_wdt_init(&twdt_config);
    	esp_task_wdt_add(NULL); //add current thread to WDT watch
	esp_task_wdt_reset();

	timer = timerBegin(1000000);                  // 1MHz = 1us
	timerAttachInterrupt(timer, &processVolume);
}

typedef enum
{
	PLAYER_IDLE,
	PLAYER_PLAY,
	PLAYER_RETRY,
	PLAYER_FLUSH,
} PlayerState;

PlayerState playerState;

void playerInit(void)
{
	playerState = PLAYER_IDLE;
}

void play(i2s_chan_handle_t i2s_handle)
{
	size_t i;
	int16_t sampleValue;
	static uint32_t outputValue;  // Static so the value persists between _PLAY and _RETRY states
	size_t bytesWritten;

	esp_task_wdt_reset();

	switch(playerState)
	{
		case PLAYER_IDLE:
			if(NULL != wavSoundNext)
			{
				wavSound = wavSoundNext;
				wavSoundNext = NULL;
				wavSound->open();
				playerState = PLAYER_PLAY;
			}
			break;
		case PLAYER_PLAY:
			if(wavSound->available())
			{
				esp_task_wdt_reset();
digitalWrite(AUX2, 1);
				sampleValue = wavSound->getNextSample();
digitalWrite(AUX2, 0);
				int32_t adjustedValue = sampleValue * volume / volumeLevels[VOL_STEP_NOM];
				if(adjustedValue > 32767)
					sampleValue = 32767;
				else if(adjustedValue < -32768)
					sampleValue = -32768;
				else
					sampleValue = adjustedValue;
				// Combine into 32 bit word (left & right)
				outputValue = (sampleValue<<16) | (sampleValue & 0xffff);
digitalWrite(AUX3, 1);
				i2s_channel_write(i2s_handle, &outputValue, 4, &bytesWritten, 1);
				if(0 == bytesWritten)
					playerState = PLAYER_RETRY;
digitalWrite(AUX3, 0);
			}
			else
			{
				wavSound->close();
				if(NULL == wavSoundNext)
				{
					// No file queued, so flush
					playerState = PLAYER_FLUSH;
				}
				else
				{
					// Another file is already queued, start playing immediately so it's seamless
					playerState = PLAYER_IDLE;
				}
			}
			break;
		case PLAYER_RETRY:
digitalWrite(AUX4, 1);
			i2s_channel_write(i2s_handle, &outputValue, 4, &bytesWritten, 1);
			if(0 == bytesWritten)
				playerState = PLAYER_RETRY;
			else
				playerState = PLAYER_PLAY;
digitalWrite(AUX4, 0);
			break;
		case PLAYER_FLUSH:
			// Make sure current buffer is full to flush any partial data in it to the I2S engine
			outputValue = 0;
			for(i=0; i<AUDIO_BUFFER_SIZE; i++)
			{
				i2s_channel_write(i2s_handle, &outputValue, 4, &bytesWritten, 1);
				// If it times out, then the buffer is full
			}
			break;
	}
}

void loop()
{
	bool usingSdSounds = false;
	size_t fileNameLength;
	File rootDir;
	File wavFile;
	const char *fileName;
	uint16_t channels = 0;
	uint32_t sampleRate = 0;
	uint16_t bitsPerSample = 0;
	uint32_t wavDataSize = 0;
	uint8_t lastSampleNum = 255;

	uint8_t buttonsPressed = 0, oldButtonsPressed = 0;
	unsigned long pressTime = 0;
	uint8_t inputStatus = 0;

	std::vector<Sound *> squealSounds;
	wavSoundNext = NULL;

	esp_task_wdt_reset();
	
	playerInit();

	Serial.println("ISE Sound Player");

	Serial.print("Version: ");
	Serial.println(VERSION_STRING);

	Serial.print("Git Rev: ");
	Serial.println(GIT_REV, HEX);

	// Read NVM configuration
	preferences.begin("squeal", false);
	volumeStep = preferences.getUChar("volume", VOL_STEP_NOM);
	silenceDecisecsMax = preferences.getUChar("silenceMax", 50);
	silenceDecisecsMin = preferences.getUChar("silenceMin", 0);

	volumeUpCoef = preferences.getUChar("volumeUp", 10);
	if(0 == volumeUpCoef)
	{
		volumeUpCoef = 1;
		preferences.putUChar("volumeUp", volumeUpCoef);
	}

	volumeDownCoef = preferences.getUChar("volumeDown", 8);
	if(0 == volumeDownCoef)
	{
		volumeDownCoef = 1;
		preferences.putUChar("volumeDown", volumeDownCoef);
	}

	esp_task_wdt_reset();

	// Check SD card
	if(SD.begin())
	{
		// Check for and read config file
		File f = SD.open("/config.txt");
		if (f)
		{
			while(f.available())
			{
				char keyStr[128];
				char valueStr[128];
				bool kvFound = configKeyValueSplit(keyStr, sizeof(keyStr), valueStr, sizeof(valueStr), f.readStringUntil('\n').c_str());
				if (!kvFound)
					continue;

				// Okay, looks like we have a valid key/value pair, see if it's something we care about
				if (0 == strcmp(keyStr, "silenceMax"))
				{
					silenceDecisecsMax = atoi(valueStr);
				}
				else if (0 == strcmp(keyStr, "silenceMin"))
				{
					silenceDecisecsMin = atoi(valueStr);
				}
				else if (0 == strcmp(keyStr, "volumeUp"))
				{
					volumeUpCoef = atoi(valueStr);
				}
				else if (0 == strcmp(keyStr, "volumeDown"))
				{
					volumeDownCoef = atoi(valueStr);
				}
			}
		}
		f.close();

		// Find WAV files
		rootDir = SD.open("/");
		while(true)
		{
			esp_task_wdt_reset();
			wavFile = rootDir.openNextFile();

			if (!wavFile)
			{
				break;	// No more files
			}
			if(wavFile.isDirectory())
			{
				Serial.print("	Skipping directory: ");
				Serial.println(wavFile.name());
			}
			else
			{
				fileName = wavFile.name();
				fileNameLength = strlen(fileName);
				if(fileNameLength < 5)
					continue;  // Filename too short (x.wav = min 5 chars)
				const char *extension = &fileName[strlen(fileName)-4];
				if(strcasecmp(extension, ".wav"))
				{
					Serial.print("	Ignoring: ");
					Serial.println(fileName);
					continue;  // Not a wav file (by extension anyway)
				}
				
				if(!wavFile.find("fmt "))  // Includes trailing space
				{
					Serial.print("! No fmt section: ");
					Serial.println(fileName);
					continue;
				}

				wavFile.seek(wavFile.position() + 6);  // Seek to number of channels
				wavFile.read((uint8_t*)&channels, 2);  // Read channels - WAV is little endian, only works if uC is also little endian

				if(channels > 1)
				{
					Serial.print("! Not mono: ");
					Serial.println(fileName);
					continue;
				}

				wavFile.read((uint8_t*)&sampleRate, 4);  // Read sample rate - WAV is little endian, only works if uC is also little endian

				if((8000 != sampleRate) && (16000 != sampleRate) && (32000 != sampleRate) && (44100 != sampleRate))
				{
					Serial.print("! Incorrect sample rate: ");
					Serial.println(fileName);
					continue;
				}

				wavFile.seek(wavFile.position() + 6);  // Seek to bits per sample
				wavFile.read((uint8_t*)&bitsPerSample, 2);	// Read bits per sample - WAV is little endian, only works if uC is also little endian

				if(16 != bitsPerSample)
				{
					Serial.print("! Not 16-bit: ");
					Serial.println(fileName);
					continue;
				}

				if(!wavFile.find("data"))
				{
					Serial.print("! No data section: ");
					Serial.println(fileName);
					continue;
				}

				// If we got here, then it looks like a valid wav file
				// Get data length and offset

				wavFile.read((uint8_t*)&wavDataSize, 4);	// Read data size - WAV is little endian, only works if uC is also little endian
				// Offset is now the current position

				Serial.print("+ Adding ");
				Serial.print(fileName);
				Serial.print(" (");
				Serial.print(sampleRate);
				Serial.print(",");
				Serial.print(wavDataSize);
				Serial.print(",");
				Serial.print(wavFile.position());
				Serial.println(")");

				squealSounds.push_back(new SdSound(fileName, wavDataSize, wavFile.position(), sampleRate));
				usingSdSounds = true;
			}
			wavFile.close();
		}
		rootDir.close();
	}

	// Print configuration values
	Serial.print("Volume: ");
	Serial.println(volumeStep);
	Serial.print("Silence Max: ");
	Serial.print(silenceDecisecsMax/10.0, 1);
	Serial.println("s");
	Serial.print("Silence Min: ");
	Serial.print(silenceDecisecsMin/10.0, 1);
	Serial.println("s");
	Serial.print("Volume Up Coef: ");
	Serial.println(volumeUpCoef);
	Serial.print("Volume Down Coef: ");
	Serial.println(volumeDownCoef);

	Serial.println("");

	esp_task_wdt_reset();

	if(usingSdSounds)
	{
		Serial.print("Using SD card sounds (");
		Serial.print(squealSounds.size());
		Serial.println(")");
		// Quadruple blink blue
		digitalWrite(LEDA, 1); delay(250); digitalWrite(LEDA, 0); delay(250);
		esp_task_wdt_reset();
		digitalWrite(LEDA, 1); delay(250); digitalWrite(LEDA, 0); delay(250);
		esp_task_wdt_reset();
		digitalWrite(LEDA, 1); delay(250); digitalWrite(LEDA, 0); delay(250);
		esp_task_wdt_reset();
		digitalWrite(LEDA, 1); delay(250); digitalWrite(LEDA, 0); delay(250);
		esp_task_wdt_reset();
	}
	else
	{
		squealSounds.push_back(new MemSound(1, flangeClip01_wav, flangeClip01_wav_len, 16000));
		squealSounds.push_back(new MemSound(2, flangeClip02_wav, flangeClip02_wav_len, 16000));
		squealSounds.push_back(new MemSound(3, flangeClip03_wav, flangeClip03_wav_len, 16000));

		Serial.print("Using built-in sounds (");
		Serial.print(squealSounds.size());
		Serial.println(")");
		// Double blink blue
		digitalWrite(LEDA, 1); delay(250); digitalWrite(LEDA, 0); delay(250);
		esp_task_wdt_reset();
		digitalWrite(LEDA, 1); delay(250); digitalWrite(LEDA, 0); delay(250);
		esp_task_wdt_reset();
	}

	timerAlarm(timer, 10000, true, 0);            // 1us * 10000 = 10ms, autoreload, unlimited reloads

	// Default dma_frame_num = 240, dma_desc_num = 6 (i2s_common.h)
	//    Total DMA size = 240 * 6 * 2 * 16 / 8 = 5760 bytes
	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	i2s_new_channel(&chan_cfg, &i2s_tx_handle, NULL);

	i2s_std_config_t std_cfg = {
		.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
		.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
		.gpio_cfg = {
			.mclk = I2S_GPIO_UNUSED,
			.bclk = I2S_BCLK,
			.ws = I2S_LRCLK,
			.dout = I2S_DOUT,
			.din = I2S_GPIO_UNUSED,
			.invert_flags = {
				.mclk_inv = false,
				.bclk_inv = false,
				.ws_inv = false,
			},
		},
	};

	i2s_channel_init_std_mode(i2s_tx_handle, &std_cfg);
	i2s_channel_enable(i2s_tx_handle);

	i2s_chan_info_t chan_info;
	i2s_channel_get_info(i2s_tx_handle, &chan_info);
	Serial.print("DMA buffer size: ");
	Serial.println(chan_info.total_dma_buf_size);

	digitalWrite(I2S_SD, 1);	// Enable amplifier

	while(1)
	{
		esp_task_wdt_reset();

		// Read inputs
		if(digitalRead(VOLUP))
			inputStatus &= ~VOL_UP_BUTTON;
		else
			inputStatus |= VOL_UP_BUTTON;

		if(digitalRead(VOLDN))
			inputStatus &= ~VOL_DN_BUTTON;
		else
			inputStatus |= VOL_DN_BUTTON;

		if(digitalRead(EN1))
			inputStatus &= ~EN1_INPUT;
		else
			inputStatus |= EN1_INPUT;

		if(digitalRead(EN2))
			inputStatus &= ~EN2_INPUT;
		else
			inputStatus |= EN2_INPUT;

		if(digitalRead(EN3))
			inputStatus &= ~EN3_INPUT;
		else
			inputStatus |= EN3_INPUT;

		if(digitalRead(EN4))
			inputStatus &= ~EN4_INPUT;
		else
			inputStatus |= EN4_INPUT;

		// Do things on 10ms interval
		if(timerTick)
		{
			timerTick = false;

			// Debounce
			buttonsPressed = debounce(buttonsPressed, inputStatus);

			// Process volume
			uint16_t deltaVolume;
			uint16_t volumeTarget;
			volumeTarget = volumeLevels[volumeStep] * unmute;

			if(volume < volumeTarget)
			{
				deltaVolume = (volumeTarget - volume);
				if((deltaVolume > 0) && (deltaVolume < volumeUpCoef))
					deltaVolume = volumeUpCoef;  // Make sure it goes all the way to min or max
				volume += deltaVolume / volumeUpCoef;
			}
			else if(volume > volumeTarget)
			{
				deltaVolume = (volume - volumeTarget);
				if((deltaVolume > 0) && (deltaVolume < volumeDownCoef))
					deltaVolume = volumeDownCoef;  // Make sure it goes all the way to min or max
				volume -= deltaVolume / volumeDownCoef;
			}
		}

		// Turn off volume LED
		uint16_t ledHoldTime = (VOL_STEP_NOM == volumeStep) ? 1000 : 100;
		if((millis() - pressTime) > ledHoldTime)
		{
			digitalWrite(LEDB, 0);
		}

		// Find rising edge of volume up button
		if((buttonsPressed ^ oldButtonsPressed) & (buttonsPressed & VOL_UP_BUTTON))
		{
			pressTime = millis();
			if(volumeStep < VOL_STEP_MAX)
			{
				volumeStep++;
				preferences.putUChar("volume", volumeStep);
			}
			Serial.print("Vol Up: ");
			Serial.println(volumeStep);
			digitalWrite(LEDB, 1);
		}

		// Find rising edge of volume down button
		if((buttonsPressed ^ oldButtonsPressed) & (buttonsPressed & VOL_DN_BUTTON))
		{
			pressTime = millis();
			if(volumeStep > 0)
			{
				volumeStep--;
				preferences.putUChar("volume", volumeStep);
			}
			Serial.print("Vol Dn: ");
			Serial.println(volumeStep);
			digitalWrite(LEDB, 1);
		}

		oldButtonsPressed = buttonsPressed;

		// Check for serial input
		if(Serial.available() > 0)
		{
			uint8_t serialChar = Serial.read();
			switch(serialChar)
			{
				case 'a':
					if(silenceDecisecsMax < 255)
					{
						silenceDecisecsMax++;
						preferences.putUChar("silenceMax", silenceDecisecsMax);
					}
					Serial.print("Silence Max: ");
					Serial.print(silenceDecisecsMax/10.0, 1);
					Serial.println("s");
					break;
				case 'z':
					if(silenceDecisecsMax > silenceDecisecsMin)
					{
						silenceDecisecsMax--;
						preferences.putUChar("silenceMax", silenceDecisecsMax);
					}
					Serial.print("Silence Max: ");
					Serial.print(silenceDecisecsMax/10.0, 1);
					Serial.println("s");
					break;

				case 's':
					if(silenceDecisecsMin < silenceDecisecsMax)
					{
						silenceDecisecsMin++;
						preferences.putUChar("silenceMin", silenceDecisecsMin);
					}
					Serial.print("Silence Min: ");
					Serial.print(silenceDecisecsMin/10.0, 1);
					Serial.println("s");
					break;
				case 'x':
					if(silenceDecisecsMin > 0)
					{
						silenceDecisecsMin--;
						preferences.putUChar("silenceMin", silenceDecisecsMin);
					}
					Serial.print("Silence Min: ");
					Serial.print(silenceDecisecsMin/10.0, 1);
					Serial.println("s");
					break;

				case 'd':
					if(volumeUpCoef < 255)
					{
						volumeUpCoef++;
						preferences.putUChar("volumeUp", volumeUpCoef);
					}
					Serial.print("Volume Up Coef: ");
					Serial.println(volumeUpCoef);
					break;
				case 'c':
					if(volumeUpCoef > 1)
					{
						volumeUpCoef--;
						preferences.putUChar("volumeUp", volumeUpCoef);
					}
					Serial.print("Volume Up Coef: ");
					Serial.println(volumeUpCoef);
					break;

				case 'f':
					if(volumeDownCoef < 255)
					{
						volumeDownCoef++;
						preferences.putUChar("volumeDown", volumeDownCoef);
					}
					Serial.print("Volume Down Coef: ");
					Serial.println(volumeDownCoef);
					break;
				case 'v':
					if(volumeDownCoef > 1)
					{
						volumeDownCoef--;
						preferences.putUChar("volumeDown", volumeDownCoef);
					}
					Serial.print("Volume Down Coef: ");
					Serial.println(volumeDownCoef);
					break;

				case 'q':
					restart = true;
					break;
			}
		}

		// Figure out if any enable inputs are pressed and light LED
		bool enable = (buttonsPressed & (EN1_INPUT | EN2_INPUT | EN3_INPUT | EN4_INPUT)) ? true : false;

		if(enable)
		{
			digitalWrite(LEDA, 1);
		}
		else
		{
			digitalWrite(LEDA, 0);
		}


		// FIXME: Temporary code to unmute when any input enabled
		if(enable)
			unmute = true;
		else
			unmute = false;




		if(NULL == wavSoundNext)
		{
			Serial.print("Heap free: ");
			Serial.println(esp_get_free_heap_size());

			// Don't play the same sample twice in a row
			// Have to initialize to something, so will never play sample 255 first
			// Can't be zero since it would never play anything with a single sample
			uint8_t sampleNum;
			sampleNum = random(0, squealSounds.size());
			if(squealSounds.size() > 2)
			{
				// With three or more sounds, don't repeat the last one
				while(sampleNum == lastSampleNum)
				{
					esp_task_wdt_reset();
					sampleNum = random(0, squealSounds.size());
					Serial.println("*");
				}
			}
			Serial.print("Queueing... ");
			Serial.println(sampleNum);
			wavSoundNext = squealSounds[sampleNum];
			lastSampleNum = sampleNum;
		}


digitalWrite(AUX1, 1);
		// Pump the sound player
		play(i2s_tx_handle);
digitalWrite(AUX1, 0);


		if(restart)
		{
			restart = false;
			Serial.print("\n*** Restarting ***\n\n");
			squealSounds.clear();
			break;	// Restart the loop() function
		}

	}

	// Never get here but this is what we would do to clean up
	digitalWrite(I2S_SD, 0);	// Disable amplifier
	i2s_channel_disable(i2s_tx_handle);
	i2s_del_channel(i2s_tx_handle);
}
