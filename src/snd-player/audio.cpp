/*************************************************************************
Title:    I2S Audio Functions
Authors:  Michael Petersen <railfan@drgw.net>
File:     audio.c
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
#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "io.h"
#include "sound.h"
#include "audio.h"

// Frames per I2S DMA buffer
#define I2S_NFRAMES   240
// Number of I2S DMA buffers
#define I2S_NBUFFERS    6

i2s_chan_handle_t i2s_tx_handle;

typedef enum
{
	PLAYER_IDLE,
	PLAYER_INIT,
	PLAYER_RECONFIGURE,
	PLAYER_PLAY,
	PLAYER_FLUSH,
	PLAYER_FLUSHING,
	PLAYER_RESET,
} PlayerState;

PlayerState playerState;
bool stopPlayer;
bool killPlayer = false;

bool unmute;
uint8_t audioVolumeStep = VOL_STEP_NOM;
uint16_t audioVolume = 0;
uint8_t audioVolumeUpCoef = 0;
uint8_t audioVolumeDownCoef = 0;
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

struct WavSound wavSoundNext;

uint32_t dmaBufferSize;

TaskHandle_t audioPumpHandle = NULL;


void audioStopPlaying(void)
{
	stopPlayer = true;
}


bool audioIsPlaying(void)
{
	return (PLAYER_IDLE != playerState);
}


void audioSetVolumeStep(uint8_t newVolumeStep)
{
	audioVolumeStep = newVolumeStep;
}


void audioSetVolumeUpCoef(uint8_t value)
{
	audioVolumeUpCoef = value;
}


uint8_t audioGetVolumeUpCoef(void)
{
	return(audioVolumeUpCoef);
}


void audioSetVolumeDownCoef(uint8_t value)
{
	audioVolumeDownCoef = value;
}


uint8_t audioGetVolumeDownCoef(void)
{
	return(audioVolumeDownCoef);
}


void audioMute(void)
{
	unmute = false;
}


void audioUnmute(void)
{
	unmute = true;
}


bool audioIsMuted(void)
{
	return( (unmute == false) && (0 == audioVolume) );
}


void audioProcessVolume(void)
{
	uint16_t deltaVolume;
	uint16_t volumeTarget;
	volumeTarget = volumeLevels[audioVolumeStep] * unmute;

	if(audioVolume < volumeTarget)
	{
		deltaVolume = (volumeTarget - audioVolume);
		if((deltaVolume > 0) && (deltaVolume < audioVolumeUpCoef))
			deltaVolume = audioVolumeUpCoef;  // Make sure it goes all the way to min or max
		audioVolume += deltaVolume / audioVolumeUpCoef;
	}
	else if(audioVolume > volumeTarget)
	{
		deltaVolume = (audioVolume - volumeTarget);
		if((deltaVolume > 0) && (deltaVolume < audioVolumeDownCoef))
			deltaVolume = audioVolumeDownCoef;  // Make sure it goes all the way to min or max
		audioVolume -= deltaVolume / audioVolumeDownCoef;
	}
}


static void audioPump(void *args)
{
	int16_t sampleValue;
	size_t bytesWritten;
	i2s_std_clk_config_t clk_cfg;
	uint32_t outputValue;
	Sound *wavSound = NULL;
	bool seamlessPlay = false;
	uint32_t oldSampleRate = 0;
	uint32_t flushCount = 0;
	unsigned long timeoutTimer;

	playerState = PLAYER_RESET;

	while(1)
	{
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
				stopPlayer = false;  // Needed here in case we run out of samples before muting is complete
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
					gpio_set_level(AUX2, 1);  ///////////////////////////////////////////////////////////////////////////////////////////
					timeoutTimer = millis();
					sampleValue = wavSound->getNextSample();
					if(millis() > timeoutTimer + 150)
					{
						// Getting the next sample took longer than 150ms (5760 bytes @ 44.1kHz = 130ms) so something is wrong
						// Delay task for 1s to allow main loop to clean up
						vTaskDelay(1000 / portTICK_PERIOD_MS);
					}
					gpio_set_level(AUX2, 0);  ///////////////////////////////////////////////////////////////////////////////////////////
					int32_t adjustedValue = sampleValue * audioVolume / volumeLevels[VOL_STEP_NOM];
					if(adjustedValue > 32767)
						sampleValue = 32767;
					else if(adjustedValue < -32768)
						sampleValue = -32768;
					else
						sampleValue = adjustedValue;
					// Combine into 32 bit word (left & right)
					outputValue = (sampleValue<<16) | (sampleValue & 0xffff);
					gpio_set_level(AUX3, 1);  ///////////////////////////////////////////////////////////////////////////////////////////
					i2s_channel_write(i2s_tx_handle, &outputValue, 4, &bytesWritten, portMAX_DELAY);
					gpio_set_level(AUX3, 0);  ///////////////////////////////////////////////////////////////////////////////////////////
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

			case PLAYER_FLUSH:
				flushCount = 0;
				#pragma GCC diagnostic push
				#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
				playerState = PLAYER_FLUSHING;
				// Intentional fall through so we attempt one flush
			case PLAYER_FLUSHING:
				#pragma GCC diagnostic pop
				outputValue = 0;
				while(flushCount < dmaBufferSize)
				{
					i2s_channel_write(i2s_tx_handle, &outputValue, 4, &bytesWritten, portMAX_DELAY);
					flushCount++;
				}
				playerState = PLAYER_RESET;
				break;
			
			case PLAYER_RESET:
				gpio_set_level(I2S_SD, 0);             // Disable amplifier
				i2s_channel_disable(i2s_tx_handle);  // Disable I2S
				oldSampleRate = 0;
				playerState = PLAYER_IDLE;
				break;
		}

		if(killPlayer)
		{
			if(NULL != wavSound)
				wavSound->close();
			killPlayer = false;
			break;
		}

		if(PLAYER_IDLE == playerState)
		{
			vTaskDelay(10 / portTICK_PERIOD_MS);  // Block execution of this task for 10ms since we're not doing anything useful at the moment
		}
		
	}
	vTaskDelete(NULL);
}


void audioInit(void)
{
	pinMode(I2S_SD, OUTPUT);
	gpio_set_level(I2S_SD, 0);	// Disable amplifier

	wavSoundNext.wav = NULL;
	wavSoundNext.seamlessPlay = false;

	// Default dma_frame_num = 240, dma_desc_num = 6 (i2s_common.h)
	//    Total DMA size = 240 * 6 * 2 * 16 / 8 = 5760 bytes
	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	chan_cfg.dma_frame_num = I2S_NFRAMES;
	chan_cfg.dma_desc_num = I2S_NBUFFERS;
	Serial.print("dma_frame_num: ");
	Serial.println(chan_cfg.dma_frame_num);
	Serial.print("dma_desc_num: ");
	Serial.println(chan_cfg.dma_desc_num);
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

	xTaskCreate(audioPump, "audioPump", 8192, NULL, 5, &audioPumpHandle);
}


void audioTerminate(void)
{
	gpio_set_level(I2S_SD, 0);	// Disable amplifier
	killPlayer = true;
	while(killPlayer)
	{
		delay(10);  // Wait for the task to terminate
	}
	i2s_channel_disable(i2s_tx_handle);
	i2s_del_channel(i2s_tx_handle);
}
