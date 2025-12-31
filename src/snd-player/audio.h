/*************************************************************************
Title:    I2S Audio Functions
Authors:  Michael Petersen <railfan@drgw.net>
File:     audio.h
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

#pragma once

#define VOL_STEP_MAX   30
#define VOL_STEP_NOM   20

struct WavSound {
	Sound *wav;
	bool seamlessPlay;
};

void audioStopPlaying(void);
bool audioIsPlaying(void);

void audioSetVolumeStep(uint8_t);
void audioSetVolumeUpCoef(uint8_t);
uint8_t audioGetVolumeUpCoef(void);
void audioSetVolumeDownCoef(uint8_t);
uint8_t audioGetVolumeDownCoef(void);

void audioMute(void);
void audioUnmute(void);
bool audioIsMuted(void);

void audioProcessVolume(void);

void audioInit(void);
void audioTerminate(void);
