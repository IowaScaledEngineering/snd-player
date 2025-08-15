/*************************************************************************
Title:    Sound Player
Authors:  Michael Petersen <railfan@drgw.net>
File:     sound.h
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

#pragma once

#define FILE_BUFFER_SIZE 2048

class Sound
{
	protected:
		size_t dataSize;
		size_t byteCount;
		uint32_t sampleRate;

	public:
		virtual void open(void);
		virtual size_t available(void)
		{
			if(dataSize > byteCount)
				return(dataSize - byteCount);
			else
				return 0;
		}
		virtual size_t read(uint8_t *buffer, size_t numBytes);
		virtual int16_t getNextSample(void);
		virtual void close(void);
		uint32_t getSampleRate(void)
		{
			return sampleRate;
		}
};

class SdSound : public Sound
{
	char *fileName;
	size_t dataOffset;
	File wavFile;
	uint8_t fileBuffer[FILE_BUFFER_SIZE];
	size_t fileBufferLength;  // might be less than actual buffer size
	size_t fileBufferPosition;
	int16_t sampleValue;

	public:
		SdSound(const char *fname, size_t numBytes, size_t offset, uint16_t sr)
		{
			fileName = strdup(fname);
			dataOffset = offset;
			dataSize = numBytes;
			sampleRate = sr;
		}
		~SdSound()
		{
			free(fileName);
		}
		void open(void)
		{
			wavFile = SD.open(String("/") + fileName);
			wavFile.seek(dataOffset);
			Serial.print("Open: ");
			Serial.println(fileName);
			byteCount = 0;
			fileBufferLength = 0;
			fileBufferPosition = 0;
		}
		size_t fileBufferAvailable(void)
		{
			if(fileBufferLength > fileBufferPosition)
				return(fileBufferLength - fileBufferPosition);
			else
				return 0;
		}
		int16_t getNextSample(void)
		{
			size_t bytesToRead, bytesRead;

			if( (fileBufferAvailable() < 2) && available() )
			{
				// We're out of bytes (or had an odd number for some strange reason)
				// and there's more of the file to grab, so grab it
				if(available() < FILE_BUFFER_SIZE)
				{
					bytesToRead = available();
				}
				else
				{
					bytesToRead = FILE_BUFFER_SIZE;
				}
				bytesRead = wavFile.read(fileBuffer, bytesToRead);
				fileBufferLength = bytesRead;
				fileBufferPosition = 0;
			}

			if(fileBufferAvailable() >= 2)
			{
				// We have at least 2 bytes in the local buffer
				sampleValue = *((int16_t *)(fileBuffer+fileBufferPosition));
				fileBufferPosition += 2;
				byteCount += 2;
				return sampleValue;
			}
			else
			{
				// We got called even though there was nothing to send
				return 0;
			}
		}
		size_t read(uint8_t *buffer, size_t numBytes)
		{
			size_t bytesToRead, bytesRead;
			if(available() < numBytes)
				bytesToRead = available();
			else
				bytesToRead = numBytes;

			bytesRead = wavFile.read(buffer, bytesToRead);
			byteCount += bytesRead;
			return bytesRead;
		}
		void close(void)
		{
			wavFile.close();
		}
};

class MemSound : public Sound
{
	const uint8_t *dataPtr;
	uint8_t soundNum;
	int16_t sampleValue;

	public:
		MemSound(uint8_t num, const uint8_t *sound, size_t numBytes, uint16_t sr)
		{
			soundNum = num;
			dataPtr = sound;
			dataSize = numBytes;
			sampleRate = sr;
		}
		~MemSound()
		{
			// No need to free dataPtr since it is const
		}
		void open(void)
		{
			Serial.print("Open: Internal ");
			Serial.println(soundNum);
			byteCount = 0;
		}
		int16_t getNextSample(void)
		{
			if(available() >= 2)
			{
				sampleValue = *((int16_t *)(dataPtr+byteCount));
				byteCount += 2;
				return sampleValue;
			}
			else
			{
				return 0;
			}
		}
		size_t read(uint8_t *buffer, size_t numBytes)
		{
			size_t bytesToRead;
			if(available() < numBytes)
				bytesToRead = available();
			else
				bytesToRead = numBytes;
			memcpy(buffer, dataPtr+byteCount, bytesToRead);
			byteCount += bytesToRead;
			return bytesToRead;
		}
		void close(void)
		{
			return;
		}
};
