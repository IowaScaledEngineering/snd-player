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
#define EN1        GPIO_NUM_9
#define EN2        GPIO_NUM_10
#define EN3        GPIO_NUM_3
#define EN4        GPIO_NUM_21
#define LEDA       GPIO_NUM_11
#define LEDB       GPIO_NUM_12
#define VOLDN      GPIO_NUM_13
#define VOLUP      GPIO_NUM_14
#define AUX1       GPIO_NUM_15
#define AUX2       GPIO_NUM_16
#define AUX3       GPIO_NUM_17
#define AUX4       GPIO_NUM_18
#define AUX5       GPIO_NUM_8
#define I2S_SD     GPIO_NUM_4
#define I2S_DOUT   GPIO_NUM_5
#define I2S_BCLK   GPIO_NUM_6
#define I2S_LRCLK  GPIO_NUM_7
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

typedef enum
{
	MODE_ONESHOT,
	MODE_CONTINUOUS,
	MODE_BME,
} ConfigMode;

struct EventConfig {
	ConfigMode mode;
	bool shuffle;
	bool level;
	int32_t beginIndex;
	int32_t endIndex;
};



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
	gpio_set_level(I2S_SD, 0);	// Disable amplifier

	pinMode(LEDA, OUTPUT);
	pinMode(LEDB, OUTPUT);
	gpio_set_level(LEDA, 0);
	gpio_set_level(LEDB, 0);

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
//gpio_set_level(AUX5, 1);
			wavSound->open();         // Open the sound
//gpio_set_level(AUX5, 0);
			if(wavSound->getSampleRate() == oldSampleRate)
				playerState = PLAYER_PLAY;
			else
				playerState = PLAYER_RECONFIGURE;
			break;

		case PLAYER_RECONFIGURE:
			clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(wavSound->getSampleRate());
			gpio_set_level(I2S_SD, 0);             // Disable amplifier
			i2s_channel_disable(i2s_tx_handle);  // Disable I2S
			i2s_channel_reconfig_std_clock(i2s_tx_handle, &clk_cfg);  // Reset sample rate
			i2s_channel_enable(i2s_tx_handle);  // Enable I2S
			gpio_set_level(I2S_SD, 1);             // Enable amplifier
			oldSampleRate = wavSound->getSampleRate();
			playerState = PLAYER_PLAY;
			break;

		case PLAYER_PLAY:
			if(stopPlayer)
			{
				stopPlayer = false;
				playerState = PLAYER_FLUSH;
			}
			else if(wavSound->available())
			{
				// Sound not done, more samples available
//gpio_set_level(AUX2, 1);
				sampleValue = wavSound->getNextSample();
//gpio_set_level(AUX2, 0);
				int32_t adjustedValue = sampleValue * volume / volumeLevels[VOL_STEP_NOM];
				if(adjustedValue > 32767)
					sampleValue = 32767;
				else if(adjustedValue < -32768)
					sampleValue = -32768;
				else
					sampleValue = adjustedValue;
				// Combine into 32 bit word (left & right)
				outputValue = (sampleValue<<16) | (sampleValue & 0xffff);
//gpio_set_level(AUX3, 1);
				i2s_channel_write(i2s_handle, &outputValue, 4, &bytesWritten, 1);
				if(0 == bytesWritten)
				{
					// Sample rejected, DMA buffer full
					playerState = PLAYER_RETRY;
				}
//gpio_set_level(AUX3, 0);
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
//gpio_set_level(AUX4, 1);
			i2s_channel_write(i2s_handle, &outputValue, 4, &bytesWritten, 1);
			if(0 == bytesWritten)
				playerState = PLAYER_RETRY;
			else
				playerState = PLAYER_PLAY;
//gpio_set_level(AUX4, 0);
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
			gpio_set_level(I2S_SD, 0);             // Disable amplifier
			i2s_channel_disable(i2s_tx_handle);  // Disable I2S
			oldSampleRate = 0;
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

void findWavFiles(File *rootDir, String dirName, std::vector<Sound *> *soundsVector, EventConfig *config)
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
// One Shot Mode: No option file
//
// Beginning-Middle-End Mode: bme.opt
//
// Continuous Mode: continuous.opt
//    Shuffle after each WAV: shuffle.opt
//    Level sensitive (end immediately): level.opt
			if(0 == strcmp(wavFile.name(), "bme.opt"))
			{
				config->mode = MODE_BME;
			}
			else if(0 == strcmp(wavFile.name(), "continuous.opt"))
			{
				config->mode = MODE_CONTINUOUS;
			}
			else if(0 == strcmp(wavFile.name(), "shuffle.opt"))
			{
				config->shuffle = true;
			}
			else if(0 == strcmp(wavFile.name(), "level.opt"))
			{
				config->level = true;
			}
			else if(validateWavFile(&wavFile, &wavData))
			{
				// If we got here, then it looks like a valid wav file
				String fullFileName = dirName + wavFile.name();

				Serial.print("+ Adding ");
				Serial.print(fullFileName);
				Serial.print(" (");
				Serial.print(wavData.sampleRate);
				Serial.print(",");
				Serial.print(wavData.wavDataSize);
				Serial.print(",");
				Serial.print(wavData.dataStartPosition);
				Serial.print(")");

				if(!strcmp(wavFile.name(), "begin.wav"))
				{
					config->beginIndex = soundsVector->size();   // Current size will be index of the newly pushed object below
					Serial.print(" - BME BEGIN");
				}
				else if(!strcmp(wavFile.name(), "end.wav"))
				{
					config->endIndex = soundsVector->size();   // Current size will be index of the newly pushed object below
					Serial.print(" - BME END");
				}

				Serial.println("");

				soundsVector->push_back(new SdSound(fullFileName.c_str(), wavData.wavDataSize, wavData.dataStartPosition, wavData.sampleRate));
			}
		}
		wavFile.close();
	}
}


void loop()
{
	File rootDir;

	uint32_t lastSampleNum = UINT32_MAX;   // Have to initialize to something, so will never play UINT32_MAX samples first, can't be zero since it would never play anything with a single sample

	uint32_t silenceDecisecs;
	unsigned long silenceStart;

	bool ambientMode = false;

	uint8_t buttonsPressed = 0, oldButtonsPressed = 0;
	unsigned long pressTime = 0;
	uint8_t inputStatus = 0;

	uint32_t i;
	uint32_t activeEvent;

	uint32_t sampleNum;

	std::vector<Sound *> ambientSounds;
	std::vector<Sound *> eventSounds[4];
	wavSoundNext.wav = NULL;
	wavSoundNext.seamlessPlay = false;

	EventConfig ambientConfig;  // not used, but needed for symmetry with other events
	EventConfig eventConfig[4];
	for(i=0; i<4; i++)
	{
		eventConfig[i].mode = MODE_ONESHOT;
		eventConfig[i].shuffle = false;
		eventConfig[i].level = false;
		eventConfig[i].beginIndex = -1;
		eventConfig[i].endIndex = -1;
	}

	typedef enum
	{
		SOUNDPLAYER_IDLE,
		SOUNDPLAYER_AMBIENT_INIT,
		SOUNDPLAYER_AMBIENT_QUEUE,
		SOUNDPLAYER_AMBIENT_PLAY,
		SOUNDPLAYER_AMBIENT_SILENCE,
		SOUNDPLAYER_ONESHOT_QUEUE,
		SOUNDPLAYER_WAIT_FOR_END,
		SOUNDPLAYER_CONTINUOUS_INIT,
		SOUNDPLAYER_CONTINUOUS_RANDOM,
		SOUNDPLAYER_CONTINUOUS_SAME,
		SOUNDPLAYER_CONTINUOUS_WAIT,
		SOUNDPLAYER_CONTINUOUS_MUTE,
		SOUNDPLAYER_BME_BEGIN,
		SOUNDPLAYER_BME_WAIT1,
		SOUNDPLAYER_BME_MIDDLE,
		SOUNDPLAYER_BME_WAIT2,
		SOUNDPLAYER_BME_END,
		SOUNDPLAYER_BME_WAIT3,
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

	bool enableInput[4];
	bool oldEnableInput[4] = {0};
	bool risingInput[4];
	bool enable;

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
			findWavFiles(&rootDir, "ambient/", &ambientSounds, &ambientConfig);
			rootDir.close();
			if(ambientSounds.size() > 0)
			{
				// Only set Ambient mode if sounds are found
				ambientMode = true;
			}
		}

		if(!ambientMode)
		{
			// Not ambient mode, check event directories
			ambientMode = false;

			for(i=0; i<4; i++)
			{
				char rootDirectory[8] = "/event_";  // SD.open need preceding slash
				rootDirectory[6] = '1' + i;
				char directoryName[8] = "event_/";  // filename needs trailing slash
				directoryName[5] = '1' + i;
				if((rootDir = SD.open(rootDirectory)))
				{
					findWavFiles(&rootDir, directoryName, &eventSounds[i], &eventConfig[i]);
				}
			}
		}
	}

	Serial.println("");

	for(i=0; i<4; i++)
	{
		Serial.print("Event ");
		Serial.print(i);
		Serial.print(" Mode: ");
		switch(eventConfig[i].mode)
		{
			case MODE_ONESHOT:
				Serial.println("One Shot");
				break;
			case MODE_CONTINUOUS:
				Serial.print("Continuous");
				if(eventConfig[i].level)
					Serial.print(" Level");
				if(eventConfig[i].shuffle)
					Serial.print(" Shuffle");
				Serial.println("");
				break;
			case MODE_BME:
				Serial.println("Beginning-Middle-End");
				break;
		}
	}

	Serial.println("");

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

	if(ambientMode || (eventSounds[0].size() > 0) || (eventSounds[1].size() > 0) || (eventSounds[2].size() > 0) || (eventSounds[3].size() > 0))
	{
		Serial.print("Using SD card sounds (");
		Serial.print(ambientSounds.size() + eventSounds[0].size() + eventSounds[1].size() + eventSounds[2].size() + eventSounds[3].size());
		Serial.println(")");
		// Quadruple blink blue
		gpio_set_level(LEDA, 1); delay(250); gpio_set_level(LEDA, 0); delay(250);
		esp_task_wdt_reset();
		gpio_set_level(LEDA, 1); delay(250); gpio_set_level(LEDA, 0); delay(250);
		esp_task_wdt_reset();
		gpio_set_level(LEDA, 1); delay(250); gpio_set_level(LEDA, 0); delay(250);
		esp_task_wdt_reset();
		gpio_set_level(LEDA, 1); delay(250); gpio_set_level(LEDA, 0); delay(250);
		esp_task_wdt_reset();
	}
	else
	{
		Serial.print("No valid sounds on SD card!");
		// Blink blue / orange
		while(1)
		{
			gpio_set_level(LEDA, 1); delay(100); gpio_set_level(LEDA, 0);
			esp_task_wdt_reset();
			gpio_set_level(LEDB, 1); delay(100); gpio_set_level(LEDB, 0);
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

	gpio_set_level(I2S_SD, 1);	// Enable amplifier

	while(1)
	{
		esp_task_wdt_reset();

		// Read inputs
		if(gpio_get_level(VOLUP))
			inputStatus &= ~VOL_UP_BUTTON;
		else
			inputStatus |= VOL_UP_BUTTON;

		if(gpio_get_level(VOLDN))
			inputStatus &= ~VOL_DN_BUTTON;
		else
			inputStatus |= VOL_DN_BUTTON;

		if(gpio_get_level(EN1))
			inputStatus &= ~EN1_INPUT;
		else
			inputStatus |= EN1_INPUT;

		if(gpio_get_level(EN2))
			inputStatus &= ~EN2_INPUT;
		else
			inputStatus |= EN2_INPUT;

		if(gpio_get_level(EN3))
			inputStatus &= ~EN3_INPUT;
		else
			inputStatus |= EN3_INPUT;

		if(gpio_get_level(EN4))
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
gpio_set_level(AUX2, 1);
gpio_set_level(AUX2, 0);
		uint16_t ledHoldTime = (VOL_STEP_NOM == volumeStep) ? 1000 : 100;
		if((millis() - pressTime) > ledHoldTime)
		{
			gpio_set_level(LEDB, 0);
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
			gpio_set_level(LEDB, 1);
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
			gpio_set_level(LEDB, 1);
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
		enableInput[0] = (buttonsPressed & (EN1_INPUT)) ? true : false;
		enableInput[1] = (buttonsPressed & (EN2_INPUT)) ? true : false;
		enableInput[2] = (buttonsPressed & (EN3_INPUT)) ? true : false;
		enableInput[3] = (buttonsPressed & (EN4_INPUT)) ? true : false;
		enable = enableInput[0] || enableInput[1] || enableInput[2] || enableInput[3];

		for(i=0; i<4; i++)
		{
			if( (1 == enableInput[i]) && (0 == oldEnableInput[i]) )
				risingInput[i] = true;
			else
				risingInput[i] = false;
			oldEnableInput[i] = enableInput[i];
		}

		if(enable)
		{
			gpio_set_level(LEDA, 1);
		}
		else
		{
			gpio_set_level(LEDA, 0);
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
					if(enable)
					{
						for(i=0; i<4; i++)
						{
							// Find first input activated
							if(enableInput[i] && (eventSounds[i].size() > 0))
							{
								if( (risingInput[i]) && (MODE_ONESHOT == eventConfig[i].mode))
								{
									activeEvent = i;
									state = SOUNDPLAYER_ONESHOT_QUEUE;
								}
								else if(MODE_CONTINUOUS == eventConfig[i].mode)
								{
									activeEvent = i;
									state = SOUNDPLAYER_CONTINUOUS_INIT;
								}
								else if(MODE_BME == eventConfig[i].mode)
								{
									activeEvent = i;
									state = SOUNDPLAYER_BME_BEGIN;
								}
								break;  // Exit for loop
							}
						}
					}
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
				wavSoundNext.seamlessPlay = false;
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



			case SOUNDPLAYER_ONESHOT_QUEUE:
				unmute = true;
				Serial.print("\nEvent ");
				Serial.print(activeEvent+1);
				Serial.println(": One Shot Mode");
				Serial.print("Heap free: ");
				Serial.println(esp_get_free_heap_size());

				sampleNum = random(0, eventSounds[activeEvent].size());
				Serial.print("Queueing... ");
				Serial.println(sampleNum);
				wavSoundNext.wav = eventSounds[activeEvent][sampleNum];
				wavSoundNext.seamlessPlay = false;
				state = SOUNDPLAYER_WAIT_FOR_END;
				break;
				


			case SOUNDPLAYER_WAIT_FOR_END:
				if(PLAYER_IDLE == playerState)
				{
					unmute = true;
					state = SOUNDPLAYER_IDLE;
				}
				break;



			case SOUNDPLAYER_CONTINUOUS_INIT:
				unmute = true;
				Serial.print("\nEvent ");
				Serial.print(activeEvent+1);
				Serial.println(": Continuous Mode");
				Serial.print("Heap free: ");
				Serial.println(esp_get_free_heap_size());
				lastSampleNum = UINT32_MAX;
				state = SOUNDPLAYER_CONTINUOUS_RANDOM;
				break;

			case SOUNDPLAYER_CONTINUOUS_RANDOM:
				sampleNum = random(0, eventSounds[activeEvent].size());
				if(eventSounds[activeEvent].size() > 2)
				{
					// With three or more sounds, don't repeat the last one
					while(sampleNum == lastSampleNum)
					{
						esp_task_wdt_reset();
						sampleNum = random(0, eventSounds[activeEvent].size());
						Serial.println("*");
					}
				}
				Serial.print("Queueing... ");
				Serial.println(sampleNum);
				wavSoundNext.wav = eventSounds[activeEvent][sampleNum];
				wavSoundNext.seamlessPlay = false;
				lastSampleNum = sampleNum;
				state = SOUNDPLAYER_CONTINUOUS_WAIT;
				break;

			case SOUNDPLAYER_CONTINUOUS_SAME:
				Serial.print("Re-Queueing... ");
				Serial.println(sampleNum);
				wavSoundNext.wav = eventSounds[activeEvent][sampleNum];
				wavSoundNext.seamlessPlay = false;
				state = SOUNDPLAYER_CONTINUOUS_WAIT;
				break;

			case SOUNDPLAYER_CONTINUOUS_WAIT:
				if(enableInput[activeEvent])
				{
					// Enable still active
					if(NULL == wavSoundNext.wav)
					{
						// Queue empty
						if(eventConfig[activeEvent].shuffle)
							state = SOUNDPLAYER_CONTINUOUS_RANDOM;
						else
							state = SOUNDPLAYER_CONTINUOUS_SAME;
					}
				}
				else
				{
					// Enable not active
					wavSoundNext.wav = NULL;  // Remove anything queued so it doesn't play
					lastSampleNum = UINT32_MAX;
					if(eventConfig[activeEvent].level)
						state = SOUNDPLAYER_CONTINUOUS_MUTE;
					else
						state = SOUNDPLAYER_WAIT_FOR_END;
				}
				break;

			case SOUNDPLAYER_CONTINUOUS_MUTE:
				unmute = false;
				if(0 == volume)
				{
					stopPlayer = true;
					state = SOUNDPLAYER_WAIT_FOR_END;
				}
				break;



			case SOUNDPLAYER_BME_BEGIN:
				unmute = true;
				Serial.print("\nEvent ");
				Serial.print(activeEvent+1);
				Serial.println(": BME Mode");
				Serial.print("Heap free: ");
				Serial.println(esp_get_free_heap_size());

				// Queue begin file if one exists
				if(eventConfig[activeEvent].beginIndex >= 0)
				{
					wavSoundNext.wav = eventSounds[activeEvent][eventConfig[activeEvent].beginIndex];
					wavSoundNext.seamlessPlay = true;
				}
				state = SOUNDPLAYER_BME_WAIT1;
				break;
			case SOUNDPLAYER_BME_WAIT1:
				if(NULL == wavSoundNext.wav)
					state = SOUNDPLAYER_BME_MIDDLE;
				break;
			case SOUNDPLAYER_BME_MIDDLE:
				// Queue middle file.
				i = eventSounds[activeEvent].size();
				Serial.println(i);
				if(eventConfig[activeEvent].beginIndex >= 0)
				{
					// Valid begin sound
					i--;
				}
				if(eventConfig[activeEvent].endIndex >= 0)
				{
					// Valid end sound
					i--;
				}
				Serial.println(i);
				
				sampleNum = random(0, i);
				Serial.println(sampleNum);
				
				if((eventConfig[activeEvent].beginIndex >= 0) && (sampleNum >= eventConfig[activeEvent].beginIndex))
				{
					// Skip begin sound
					sampleNum++;
					// Check that it isn't now the end sound
					if(sampleNum >= eventConfig[activeEvent].endIndex)
						sampleNum++;
				}
				else if((eventConfig[activeEvent].endIndex >= 0) && (sampleNum >= eventConfig[activeEvent].endIndex))
				{
					// Skip end sound
					sampleNum++;
					// Check that it isn't now the begin sound
					if(sampleNum >= eventConfig[activeEvent].beginIndex)
						sampleNum++;
				}
				Serial.println(sampleNum);
				
				wavSoundNext.wav = eventSounds[activeEvent][sampleNum];
				wavSoundNext.seamlessPlay = true;
				state = SOUNDPLAYER_BME_WAIT2;
				break;
			case SOUNDPLAYER_BME_WAIT2:
				if((enableInput[activeEvent]) && (NULL == wavSoundNext.wav))
					state = SOUNDPLAYER_BME_MIDDLE;
				else if(!enableInput[activeEvent])
					state = SOUNDPLAYER_BME_END;
				break;
			case SOUNDPLAYER_BME_END:
				// Queue end file if it exists.  It's OK if this overwrites a queued middle since we now want it to end.
				if(eventConfig[activeEvent].endIndex >= 0)
				{
					wavSoundNext.wav = eventSounds[activeEvent][eventConfig[activeEvent].endIndex];
					wavSoundNext.seamlessPlay = true;
				}
				state = SOUNDPLAYER_BME_WAIT3;
				break;
			case SOUNDPLAYER_BME_WAIT3:
				// Wait until the end is playing
				if(NULL == wavSoundNext.wav)
				{
					state = SOUNDPLAYER_WAIT_FOR_END;
				}
				break;
		}


gpio_set_level(AUX1, 1);
		// Pump the sound player
		play(i2s_tx_handle);
gpio_set_level(AUX1, 0);


		if(restart)
		{
			restart = false;
			Serial.print("\n*** Restarting ***\n\n");
			ambientSounds.clear();
			eventSounds[0].clear();
			eventSounds[1].clear();
			eventSounds[2].clear();
			eventSounds[3].clear();
			break;	// Restart the loop() function
		}

	}

	// Never get here but this is what we would do to clean up
	gpio_set_level(I2S_SD, 0);	// Disable amplifier
	i2s_channel_disable(i2s_tx_handle);
	i2s_del_channel(i2s_tx_handle);
}
