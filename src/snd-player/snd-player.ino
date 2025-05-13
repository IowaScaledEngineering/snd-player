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

void IRAM_ATTR tickTimer(void)
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
	timerAttachInterrupt(timer, &tickTimer);
	timerAlarm(timer, 10000, true, 0);            // 1us * 10000 = 10ms, autoreload, unlimited reloads
}

typedef enum
{
	PLAYER_IDLE,
	PLAYER_INIT,
	PLAYER_RECONFIGURE,
	PLAYER_PLAY,
	PLAYER_RETRY,
	PLAYER_FLUSH,
	PLAYER_FLUSHING,
	PLAYER_RESET,
} PlayerState;

PlayerState playerState;
bool stopPlayer;

struct WavSound {
	Sound *wav;
	bool seamlessPlay;
};

struct WavSound wavSoundNext;
uint32_t dmaBufferSize;

void playerInit(void)
{
	playerState = PLAYER_RESET;
}

void play(i2s_chan_handle_t i2s_handle)
{
	int16_t sampleValue;
	size_t bytesWritten;
	i2s_std_clk_config_t clk_cfg;
	// Static so the value persists between calls to play()
	static uint32_t outputValue;  // needs to persist for PLAYER_RETRY
	static Sound *wavSound;
	static bool seamlessPlay;
	static uint32_t oldSampleRate;
	static uint32_t flushCount;

	esp_task_wdt_reset();

	switch(playerState)
	{
		case PLAYER_IDLE:
			if(NULL != wavSoundNext.wav)
			{
				// Queue not empty
				playerState = PLAYER_INIT;
			}
			break;

		case PLAYER_INIT:
			wavSound = wavSoundNext.wav;  // Read the queue
			seamlessPlay = wavSoundNext.seamlessPlay;
			wavSoundNext.wav = NULL;  // Clear the queue
			wavSound->open();         // Open the sound
			if(wavSound->getSampleRate() == oldSampleRate)
				playerState = PLAYER_PLAY;
			else
				playerState = PLAYER_RECONFIGURE;
			break;

		case PLAYER_RECONFIGURE:
			clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(wavSound->getSampleRate());
			digitalWrite(I2S_SD, 0);             // Disable amplifier
			i2s_channel_disable(i2s_tx_handle);  // Disable I2S
			i2s_channel_reconfig_std_clock(i2s_tx_handle, &clk_cfg);  // Reset sample rate
			i2s_channel_enable(i2s_tx_handle);  // Enable I2S
			digitalWrite(I2S_SD, 1);             // Enable amplifier
			playerState = PLAYER_PLAY;
			break;

		case PLAYER_PLAY:
			if(stopPlayer)
			{
				playerState = PLAYER_FLUSH;
			}
			else if(wavSound->available())
			{
				// Sound not done, more samples available
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
				{
					// Sample rejected, DMA buffer full
					playerState = PLAYER_RETRY;
				}
digitalWrite(AUX3, 0);
			}
			else
			{
				// Sound done, no samples available
				wavSound->close();
				if((NULL != wavSoundNext.wav) && (seamlessPlay))
				{
					// Queue not empty and seamless playing, so grab next
					playerState = PLAYER_INIT;
				}
				else
				{
					// Otherwise, flush
					playerState = PLAYER_FLUSH;
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
			flushCount = 0;
			#pragma GCC diagnostic push
			#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
			playerState = PLAYER_FLUSHING;
			// Intentional fall through so we attempt one flush
		case PLAYER_FLUSHING:
			#pragma GCC diagnostic pop
			outputValue = 0;
			i2s_channel_write(i2s_handle, &outputValue, 4, &bytesWritten, 1);
			if(0 != bytesWritten)
			{
				// Success
				flushCount++;
			}
			if(flushCount >= dmaBufferSize)
			{
				if(NULL != wavSoundNext.wav)
				{
					// Queue not empty
					playerState = PLAYER_INIT;
				}
				else
				{
					// Queue empty
					playerState = PLAYER_RESET;
				}
			}
			break;
		
		case PLAYER_RESET:
			digitalWrite(I2S_SD, 0);             // Disable amplifier
			i2s_channel_disable(i2s_tx_handle);  // Disable I2S
			oldSampleRate = 0;
			stopPlayer = false;
			playerState = PLAYER_IDLE;
			break;
	}
}


struct WavData {
	uint32_t sampleRate;
	uint32_t wavDataSize;
	size_t dataStartPosition;
};


bool validateWavFile(File *wavFile, struct WavData *wavData)
{
	const char *fileName;
	size_t fileNameLength;
	uint16_t channels;
	uint16_t bitsPerSample;
	uint32_t sampleRate;
	uint32_t wavDataSize;

	fileName = wavFile->name();
	fileNameLength = strlen(fileName);
	if(fileNameLength < 5)
		return false;  // Filename too short (x.wav = min 5 chars)
	const char *extension = &fileName[strlen(fileName)-4];
	if(strcasecmp(extension, ".wav"))
	{
		Serial.print("	Ignoring: ");
		Serial.println(fileName);
		return false;  // Not a wav file (by extension anyway)
	}
	
	if(!wavFile->find("fmt "))  // Includes trailing space
	{
		Serial.print("! No fmt section: ");
		Serial.println(fileName);
		return false;
	}

	wavFile->seek(wavFile->position() + 6);  // Seek to number of channels
	wavFile->read((uint8_t*)&channels, 2);  // Read channels - WAV is little endian, only works if uC is also little endian

	if(channels > 1)
	{
		Serial.print("! Not mono: ");
		Serial.println(fileName);
		return false;
	}

	wavFile->read((uint8_t*)&sampleRate, 4);  // Read sample rate - WAV is little endian, only works if uC is also little endian
	wavData->sampleRate = sampleRate;

	if((8000 != sampleRate) && (16000 != sampleRate) && (32000 != sampleRate) && (44100 != sampleRate))
	{
		Serial.print("! Incorrect sample rate: ");
		Serial.println(fileName);
		return false;
	}

	wavFile->seek(wavFile->position() + 6);  // Seek to bits per sample
	wavFile->read((uint8_t*)&bitsPerSample, 2);	// Read bits per sample - WAV is little endian, only works if uC is also little endian

	if(16 != bitsPerSample)
	{
		Serial.print("! Not 16-bit: ");
		Serial.println(fileName);
		return false;
	}

	if(!wavFile->find("data"))
	{
		Serial.print("! No data section: ");
		Serial.println(fileName);
		return false;
	}

	wavFile->read((uint8_t*)&wavDataSize, 4);	// Read data size - WAV is little endian, only works if uC is also little endian
	wavData->wavDataSize = wavDataSize;
	// Actual data is now the current position
	
	wavData->dataStartPosition = wavFile->position();
	return true;
}

void findWavFiles(File *rootDir, std::vector<Sound *> *soundsVector)
{
	File wavFile;
	WavData wavData;

	while(true)
	{
		esp_task_wdt_reset();
		wavFile = rootDir->openNextFile();

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
			if(validateWavFile(&wavFile, &wavData))
			{
				// If we got here, then it looks like a valid wav file
				Serial.print("+ Adding ");
				Serial.print(wavFile.name());
				Serial.print(" (");
				Serial.print(wavData.sampleRate);
				Serial.print(",");
				Serial.print(wavData.wavDataSize);
				Serial.print(",");
				Serial.print(wavData.dataStartPosition);
				Serial.println(")");

				String fullFileName = String("ambient/") + wavFile.name();

				soundsVector->push_back(new SdSound(fullFileName.c_str(), wavData.wavDataSize, wavData.dataStartPosition, wavData.sampleRate));
			}
		}
		wavFile.close();
	}
}


void loop()
{
	File rootDir;

	uint8_t lastSampleNum = 255;   // Have to initialize to something, so will never play sample 255 first, Can't be zero since it would never play anything with a single sample

	uint32_t silenceDecisecs;
	unsigned long silenceStart;

	bool ambientMode = false;

	uint8_t buttonsPressed = 0, oldButtonsPressed = 0;
	unsigned long pressTime = 0;
	uint8_t inputStatus = 0;

	std::vector<Sound *> ambientSounds;
	std::vector<Sound *> event1Sounds;
	std::vector<Sound *> event2Sounds;
	std::vector<Sound *> event3Sounds;
	std::vector<Sound *> event4Sounds;
	wavSoundNext.wav = NULL;
	wavSoundNext.seamlessPlay = false;

	typedef enum
	{
		SOUNDPLAYER_IDLE,
		SOUNDPLAYER_AMBIENT_INIT,
		SOUNDPLAYER_AMBIENT_QUEUE,
		SOUNDPLAYER_AMBIENT_PLAY,
		SOUNDPLAYER_AMBIENT_SILENCE,
	} SoundplayerState;

	SoundplayerState state = SOUNDPLAYER_IDLE;

	esp_task_wdt_reset();
	
	playerInit();

	Serial.println("ISE Sound Player");

	Serial.print("Version: ");
	Serial.println(VERSION_STRING);

	Serial.print("Git Rev: ");
	Serial.println(GIT_REV, HEX);

	// Read NVM configuration
	preferences.begin("soundplayer", false);
	volumeStep = preferences.getUChar("volume", VOL_STEP_NOM);
	
	// Set defaults
	silenceDecisecsMax = 0;
	silenceDecisecsMin = 0;
	volumeUpCoef = 10;
	volumeDownCoef = 8;

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

		if((rootDir = SD.open("/ambient")))
		{
			// Ambient mode, find WAV files
			Serial.println("\nFound ambient directory");
			findWavFiles(&rootDir, &ambientSounds);
			rootDir.close();
			if(ambientSounds.size() > 0)
			{
				// Only set Ambient mode if sounds are found
				ambientMode = true;
			}
		}
		else
		{
			// Not ambient mode
			ambientMode = false;

			// FIXME: do stuff
		}
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

	if(ambientMode || (event1Sounds.size() > 0) || (event2Sounds.size() > 0) || (event3Sounds.size() > 0) || (event4Sounds.size() > 0))
	{
		Serial.print("Using SD card sounds (");
		Serial.print(ambientSounds.size() + event1Sounds.size() + event2Sounds.size() + event3Sounds.size() + event4Sounds.size());
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
		Serial.print("No valid sounds on SD card!");
		// Blink blue / orange
		while(1)
		{
			digitalWrite(LEDA, 1); delay(100); digitalWrite(LEDA, 0);
			esp_task_wdt_reset();
			digitalWrite(LEDB, 1); delay(100); digitalWrite(LEDB, 0);
			esp_task_wdt_reset();
		}
	}

	// Default dma_frame_num = 240, dma_desc_num = 6 (i2s_common.h)
	//    Total DMA size = 240 * 6 * 2 * 16 / 8 = 5760 bytes
	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	i2s_new_channel(&chan_cfg, &i2s_tx_handle, NULL);

	i2s_std_config_t std_cfg = {
		.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
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
	dmaBufferSize = chan_info.total_dma_buf_size;
	Serial.print("DMA buffer size: ");
	Serial.println(dmaBufferSize);

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
				case 'q':
					restart = true;
					break;
				case '~':
					Serial.print("Clearing preferences...");
					preferences.clear();
					break;
			}
		}

		// Figure out if any enable inputs are pressed and light LED
		bool enable1 = (buttonsPressed & (EN1_INPUT)) ? true : false;
		bool enable2 = (buttonsPressed & (EN2_INPUT)) ? true : false;
		bool enable3 = (buttonsPressed & (EN3_INPUT)) ? true : false;
		bool enable4 = (buttonsPressed & (EN4_INPUT)) ? true : false;
		bool enable = enable1 || enable2 || enable3 || enable4;

		if(enable)
		{
			digitalWrite(LEDA, 1);
		}
		else
		{
			digitalWrite(LEDA, 0);
		}

		switch(state)
		{
			case SOUNDPLAYER_IDLE:
				if(ambientMode)
				{
					state = SOUNDPLAYER_AMBIENT_INIT;
				}
				else
				{
					// Not ambient mode, wait for inputs
				}
				break;
			case SOUNDPLAYER_AMBIENT_INIT:
				Serial.println("\nAmbient Mode");
				unmute = false;
				state = SOUNDPLAYER_AMBIENT_QUEUE;
				break;
			case SOUNDPLAYER_AMBIENT_QUEUE:
				Serial.print("Heap free: ");
				Serial.println(esp_get_free_heap_size());

				uint8_t sampleNum;
				sampleNum = random(0, ambientSounds.size());
				if(ambientSounds.size() > 2)
				{
					// With three or more sounds, don't repeat the last one
					while(sampleNum == lastSampleNum)
					{
						esp_task_wdt_reset();
						sampleNum = random(0, ambientSounds.size());
						Serial.println("*");
					}
				}
				Serial.print("Queueing... ");
				Serial.println(sampleNum);
				wavSoundNext.wav = ambientSounds[sampleNum];
				lastSampleNum = sampleNum;
				state = SOUNDPLAYER_AMBIENT_PLAY;
				break;
			case SOUNDPLAYER_AMBIENT_PLAY:
				if(enable)
					unmute = true;
				else
					unmute = false;
				if(PLAYER_IDLE == playerState)
				{
					// Done playing, add some silence
					silenceDecisecs = random(silenceDecisecsMin, silenceDecisecsMax);
					Serial.print("Silence... ");
					Serial.print(silenceDecisecs/10.0, 1);
					Serial.println("s");
					silenceStart = millis();
					state = SOUNDPLAYER_AMBIENT_SILENCE;
				}
				break;
			case SOUNDPLAYER_AMBIENT_SILENCE:
				if(millis() >= (silenceStart + 100 * silenceDecisecs))
				{
					state = SOUNDPLAYER_AMBIENT_QUEUE;
				}
				break;
		}


digitalWrite(AUX1, 1);
		// Pump the sound player
		play(i2s_tx_handle);
digitalWrite(AUX1, 0);


		if(restart)
		{
			restart = false;
			Serial.print("\n*** Restarting ***\n\n");
			ambientSounds.clear();
			event1Sounds.clear();
			event2Sounds.clear();
			event3Sounds.clear();
			event4Sounds.clear();
			break;	// Restart the loop() function
		}

	}

	// Never get here but this is what we would do to clean up
	digitalWrite(I2S_SD, 0);	// Disable amplifier
	i2s_channel_disable(i2s_tx_handle);
	i2s_del_channel(i2s_tx_handle);
}
