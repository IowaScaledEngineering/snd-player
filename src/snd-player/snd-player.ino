/*************************************************************************
Title:    Sound Player
Authors:  Michael Petersen <railfan@drgw.net>
File:     snd-player.ino
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2025 Michael Petersen

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

#include "io.h"
#include "sound.h"
#include "audio.h"

// 3 sec watchdog 
#define TWDT_TIMEOUT_MS    3000

// Bit positions for inputs
#define VOL_UP_BUTTON   0x01
#define VOL_DN_BUTTON   0x02
#define EN1_INPUT       0x10
#define EN2_INPUT       0x20
#define EN3_INPUT       0x40
#define EN4_INPUT       0x80

uint8_t volumeStep = VOL_STEP_NOM;

extern struct WavSound wavSoundNext;

bool restart = false;

uint32_t silenceDecisecsMax = 0;
uint32_t silenceDecisecsMin = 0;

Preferences preferences;

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

struct WavData {
	uint32_t sampleRate;
	uint32_t wavDataSize;
	size_t dataStartPosition;
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


void printMemoryUsage(void)
{
	Serial.printf("\n[SYS]: stack: %u heap: %u\n\n", uxTaskGetStackHighWaterMark(NULL), xPortGetFreeHeapSize());
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
	Serial.begin();

	pinMode(SDDET, INPUT_PULLUP);

	pinMode(VOLDN, INPUT_PULLUP);
	pinMode(VOLUP, INPUT_PULLUP);

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
	unsigned long silenceTick;

	bool ambientMode = false;

	uint8_t buttonsPressed = 0, oldButtonsPressed = 0;
	unsigned long pressTime = 0;
	uint32_t blinkCount = UINT32_MAX;  // Prevent blink on startup when at min/max volume
	uint8_t inputStatus = 0;

	unsigned long sdDetectTime = 0;

	uint32_t i, j;
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
		SOUNDPLAYER_AMBIENT_PLAY_WAIT,
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
	audioSetVolumeUpCoef(10);
	audioSetVolumeDownCoef(8);

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
					audioSetVolumeUpCoef(atoi(valueStr));
				}
				else if (0 == strcmp(keyStr, "volumeDown"))
				{
					audioSetVolumeDownCoef(atoi(valueStr));
				}
			}
		}
		f.close();

		esp_task_wdt_reset();
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

		esp_task_wdt_reset();
		if(!ambientMode)
		{
			// Not ambient mode, check event directories
			ambientMode = false;

			for(i=0; i<4; i++)
			{
				esp_task_wdt_reset();
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

	if(!ambientMode)
	{
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
	}

	Serial.print("\n");

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
	Serial.println(audioGetVolumeUpCoef());
	Serial.print("Volume Down Coef: ");
	Serial.println(audioGetVolumeDownCoef());

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
		Serial.println("No valid sounds on SD card!");
		// Blink blue / orange
		while(1)
		{
			gpio_set_level(LEDA, 1); delay(250); gpio_set_level(LEDA, 0);
			esp_task_wdt_reset();
			gpio_set_level(LEDB, 1); delay(250); gpio_set_level(LEDB, 0);
			esp_task_wdt_reset();
			if(0 == gpio_get_level(SDDET))
			{
				// Card inserted
				if(millis() > sdDetectTime + 500)  //  Need 500ms of continuous insertion
				{
					Serial.println("SD Card Inserted");
					restart = true;
					break;
				}
			}
			else
			{
				sdDetectTime = millis();
			}
		}
	}

	audioInit();
	
	printMemoryUsage();

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
			
			audioProcessVolume();
		}

		// Turn off volume LED
		uint16_t ledHoldTime = (VOL_STEP_NOM == volumeStep) ? 1000 : 100;
		if(millis() > pressTime + ledHoldTime)
		{
			gpio_set_level(LEDB, 0);
		}

		if( (0 == volumeStep) || (VOL_STEP_MAX == volumeStep) )
		{
			if((millis() > pressTime + 2*ledHoldTime) && (blinkCount < 2))
			{
				pressTime = millis();  // Trigger another blink
				gpio_set_level(LEDB, 1);
				blinkCount++;
			}
		}

		// Find rising edge of volume up button
		if((buttonsPressed ^ oldButtonsPressed) & (buttonsPressed & VOL_UP_BUTTON))
		{
			pressTime = millis();
			if(volumeStep < VOL_STEP_MAX)
			{
				volumeStep++;
				audioSetVolumeStep(volumeStep);
				preferences.putUChar("volume", volumeStep);
			}
			Serial.print("Vol Up: ");
			Serial.println(volumeStep);
			gpio_set_level(LEDB, 1);
			blinkCount = 0;
		}

		// Find rising edge of volume down button
		if((buttonsPressed ^ oldButtonsPressed) & (buttonsPressed & VOL_DN_BUTTON))
		{
			pressTime = millis();
			if(volumeStep > 0)
			{
				volumeStep--;
				audioSetVolumeStep(volumeStep);
				preferences.putUChar("volume", volumeStep);
			}
			Serial.print("Vol Dn: ");
			Serial.println(volumeStep);
			gpio_set_level(LEDB, 1);
			blinkCount = 0;
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
				Serial.println("Ambient Mode");
				audioMute();
				state = SOUNDPLAYER_AMBIENT_QUEUE;
				break;
			case SOUNDPLAYER_AMBIENT_QUEUE:
				if(NULL == wavSoundNext.wav)
				{
					printMemoryUsage();

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
					state = SOUNDPLAYER_AMBIENT_PLAY_WAIT;
				}
				break;
			case SOUNDPLAYER_AMBIENT_PLAY_WAIT:
				// Wait for the player to start playing
				if(audioIsPlaying())
					state = SOUNDPLAYER_AMBIENT_PLAY;
				break;
			case SOUNDPLAYER_AMBIENT_PLAY:
				if(enable)
					audioUnmute();
				else
					audioMute();
				if(!audioIsPlaying())
				{
					// Done playing, add some silence
					silenceDecisecs = random(silenceDecisecsMin, silenceDecisecsMax);
					Serial.print("Silence... ");
					Serial.print(silenceDecisecs/10.0, 1);
					Serial.println("s");
					silenceStart = millis();
					silenceTick = silenceStart;
					state = SOUNDPLAYER_AMBIENT_SILENCE;
				}
				break;
			case SOUNDPLAYER_AMBIENT_SILENCE:
				if(millis() >= silenceTick + 1000)
				{
					silenceTick += 1000;
					Serial.print("-");
				}
				if(millis() >= (silenceStart + 100 * silenceDecisecs))
				{
					if(silenceTick != silenceStart)
						Serial.print("\n");   // We printed some ticks above, so send a newline
					state = SOUNDPLAYER_AMBIENT_QUEUE;
				}
				break;



			case SOUNDPLAYER_ONESHOT_QUEUE:
				audioUnmute();
				printMemoryUsage();
				Serial.print("Event ");
				Serial.print(activeEvent+1);
				Serial.println(": One Shot Mode");

				sampleNum = random(0, eventSounds[activeEvent].size());
				Serial.print("Queueing... ");
				Serial.println(sampleNum);
				wavSoundNext.wav = eventSounds[activeEvent][sampleNum];
				wavSoundNext.seamlessPlay = false;
				state = SOUNDPLAYER_WAIT_FOR_END;
				break;
				


			case SOUNDPLAYER_WAIT_FOR_END:
				if(!audioIsPlaying())
				{
					audioUnmute();
					state = SOUNDPLAYER_IDLE;
				}
				break;



			case SOUNDPLAYER_CONTINUOUS_INIT:
				audioUnmute();
				printMemoryUsage();
				Serial.print("Event ");
				Serial.print(activeEvent+1);
				Serial.println(": Continuous Mode");
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
				audioMute();
				if(audioIsMuted())
				{
					audioStopPlaying();
					state = SOUNDPLAYER_WAIT_FOR_END;
				}
				break;



			case SOUNDPLAYER_BME_BEGIN:
				audioUnmute();
				printMemoryUsage();
				Serial.print("Event ");
				Serial.print(activeEvent+1);
				Serial.println(": BME Mode");

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
//				Serial.println(i);
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
//				Serial.println(i);
				
				sampleNum = random(0, i);
//				Serial.println(sampleNum);
				
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
//				Serial.println(sampleNum);
				
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

		gpio_set_level(AUX1, 1);  ///////////////////////////////////////////////////////////////////////////////////////////
		if(1 == gpio_get_level(SDDET))
		{
			// Card removed
			if(millis() > sdDetectTime + 500)  //  Need 500ms of continuous removal
			{
				Serial.println("SD Card Removed");
				restart = true;
			}
		}
		else
		{
			sdDetectTime = millis();
		}
		gpio_set_level(AUX1, 0);  ///////////////////////////////////////////////////////////////////////////////////////////


		if(restart)
		{
			restart = false;
			Serial.print("\n*** Restarting ***\n\n");

			audioTerminate();

			for(i=0; i<ambientSounds.size(); i++)
			{
				delete ambientSounds[i];
			}
			
			for(j=0; j<4; j++)
			{
				for(i=0; i<eventSounds[j].size(); i++)
				{
					delete eventSounds[j][i];
				}
			}

			ambientSounds.clear();
			eventSounds[0].clear();
			eventSounds[1].clear();
			eventSounds[2].clear();
			eventSounds[3].clear();

			SD.end();

			break;	// Restart the loop() function
		}

	}

}
