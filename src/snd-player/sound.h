/*************************************************************************
Title:    Sound Functions
Authors:  Michael Petersen <railfan@drgw.net>
File:     sound.h
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

#include <SD.h>

extern "C" {
#include "mp3common.h"
}

#define FILE_BUFFER_SIZE     2048
#define MP3_FILE_BUFFER_SIZE 2048
#define MP3_MAX_PCM_SAMPLES  (MAX_NGRAN * MAX_NSAMP * MAX_NCHAN)

class Sound
{
	protected:
		size_t dataSize;
		size_t byteCount;
		uint32_t sampleRate;

	public:
		virtual ~Sound()
		{
		}
		virtual void open(void);
		virtual size_t available(void)
		{
			if(dataSize > byteCount)
				return(dataSize - byteCount);
			else
				return 0;
		}
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
				gpio_set_level(AUX5, 1);  ///////////////////////////////////////////////////////////////////////////////////////////
				bytesRead = wavFile.read(fileBuffer, bytesToRead);
				gpio_set_level(AUX5, 0);  ///////////////////////////////////////////////////////////////////////////////////////////
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
		void close(void)
		{
			wavFile.close();
		}
};

class SdMp3Sound : public Sound
{
	typedef enum
	{
		MP3_STEREO = 0x00,
		MP3_JOINT  = 0x01,
		MP3_DUAL   = 0x02,
		MP3_MONO   = 0x03
	} Mp3StereoMode;

	// Matches Helix internal FrameHeader layout (56 bytes)
	struct Mp3FrameHeader
	{
		MPEGVersion ver;
		int layer;
		int crc;
		int brIdx;
		int srIdx;
		int paddingBit;
		int privateBit;
		Mp3StereoMode sMode;
		int modeExt;
		int copyFlag;
		int origFlag;
		int emphasis;
		int CRCWord;
		const SFBandTable *sfBand;
	};

	char *fileName;
	size_t dataOffset;
	File mp3File;
	uint8_t fileBuffer[MP3_FILE_BUFFER_SIZE];
	size_t fileBufferLength;
	size_t fileBufferPosition;
	MP3DecInfo *mp3DecInfo;
	int16_t *pcmBuffer;
	size_t pcmLength;
	size_t pcmPosition;

	size_t fileBufferAvailable(void)
	{
		if(fileBufferLength > fileBufferPosition)
			return(fileBufferLength - fileBufferPosition);
		else
			return 0;
	}

	size_t pcmSamplesAvailable(void)
	{
		if(pcmLength > pcmPosition)
			return(pcmLength - pcmPosition);
		else
			return 0;
	}

	static int findSyncWord(const uint8_t *buf, int nBytes)
	{
		for(int i = 0; i < nBytes - 1; i++)
		{
			if((0xFF == buf[i]) && (0xE0 == (buf[i + 1] & 0xE0)))
				return i;
		}
		return -1;
	}

	static int unpackFrameHeader(MP3DecInfo *decInfo, const uint8_t *buf, int bytesLeft)
	{
		if((NULL == decInfo) || (NULL == decInfo->FrameHeaderPS) || (bytesLeft < 4))
			return -1;

		uint32_t sr = 0, frameSize = 0, sideInfoSize = 0;
		MPEGVersion ver = MPEG1;
		if(!parseHeader(buf, &sr, &frameSize, &sideInfoSize, &ver))
			return -1;

		int layer = 4 - ((buf[1] >> 1) & 0x03);
		int crc = (~buf[1]) & 0x01;
		if(crc && (bytesLeft < 6))
			return -1;

		int brIdx = (buf[2] >> 4) & 0x0F;
		int srIdx = (buf[2] >> 2) & 0x03;
		int paddingBit = (buf[2] >> 1) & 0x01;
		int privateBit = buf[2] & 0x01;
		Mp3StereoMode sMode = (Mp3StereoMode)((buf[3] >> 6) & 0x03);
		int modeExt = (MP3_JOINT == sMode) ? ((buf[3] >> 4) & 0x03) : 0;
		int copyFlag = (buf[3] >> 3) & 0x01;
		int origFlag = (buf[3] >> 2) & 0x01;
		int emphasis = buf[3] & 0x03;

		int fhBytes = crc ? 6 : 4;
		int nSlots = (int)frameSize - (int)sideInfoSize - fhBytes;
		if((nSlots < 0) || (nSlots > MAINBUF_SIZE))
			return -1;

		Mp3FrameHeader *fh = (Mp3FrameHeader *)decInfo->FrameHeaderPS;
		fh->ver = ver;
		fh->layer = layer;
		fh->crc = crc;
		fh->brIdx = brIdx;
		fh->srIdx = srIdx;
		fh->paddingBit = paddingBit;
		fh->privateBit = privateBit;
		fh->sMode = sMode;
		fh->modeExt = modeExt;
		fh->copyFlag = copyFlag;
		fh->origFlag = origFlag;
		fh->emphasis = emphasis;
		fh->CRCWord = crc ? (((int)buf[4] << 8) | (int)buf[5]) : 0;
		fh->sfBand = &sfBandTable[ver][srIdx];

		decInfo->nChans = (MP3_MONO == sMode) ? 1 : 2;
		decInfo->samprate = (int)sr;
		decInfo->nGrans = (MPEG1 == ver) ? NGRANS_MPEG1 : NGRANS_MPEG2;
		decInfo->nGranSamps = ((int)samplesPerFrameTab[ver][layer - 1]) / decInfo->nGrans;
		decInfo->layer = layer;
		decInfo->version = ver;
		decInfo->bitrate = ((int)bitrateTab[ver][layer - 1][brIdx]) * 1000;
		decInfo->nSlots = nSlots;

		return fhBytes;
	}

	int decodeFrame(MP3DecInfo *decInfo, uint8_t **inbuf, int *bytesLeft, int16_t *outbuf)
	{
		int fhBytes = unpackFrameHeader(decInfo, *inbuf, *bytesLeft);
		if(fhBytes < 0)
			return ERR_MP3_INVALID_FRAMEHEADER;

		if((uint32_t)decInfo->samprate != sampleRate)
			return ERR_MP3_INVALID_FRAMEHEADER;

		int sideBytes = sideBytesTab[decInfo->version][(1 == decInfo->nChans) ? 0 : 1];
		int totalFrameBytes = fhBytes + sideBytes + decInfo->nSlots;
		if(*bytesLeft < totalFrameBytes)
			return ERR_MP3_INDATA_UNDERFLOW;

		int siBytes = UnpackSideInfo(decInfo, *inbuf + fhBytes);
		if(siBytes < 0)
			return ERR_MP3_INVALID_SIDEINFO;

		*inbuf += fhBytes + siBytes;
		*bytesLeft -= (fhBytes + siBytes);

		if((decInfo->mainDataBegin < 0) || (decInfo->mainDataBegin + decInfo->nSlots > MAINBUF_SIZE))
		{
			*inbuf += decInfo->nSlots;
			*bytesLeft -= decInfo->nSlots;
			decInfo->mainDataBytes = 0;
			return ERR_MP3_INVALID_SIDEINFO;
		}

		if(decInfo->mainDataBytes < decInfo->mainDataBegin)
		{
			// Bit reservoir does not have enough bytes yet; append and move to next frame
			if(decInfo->mainDataBytes + decInfo->nSlots <= MAINBUF_SIZE)
			{
				memcpy(decInfo->mainBuf + decInfo->mainDataBytes, *inbuf, decInfo->nSlots);
				decInfo->mainDataBytes += decInfo->nSlots;
			}
			else
			{
				int keep = MAINBUF_SIZE - decInfo->nSlots;
				if(keep > 0)
				{
					memmove(decInfo->mainBuf, decInfo->mainBuf + decInfo->mainDataBytes - keep, keep);
					memcpy(decInfo->mainBuf + keep, *inbuf, decInfo->nSlots);
					decInfo->mainDataBytes = MAINBUF_SIZE;
				}
				else
				{
					decInfo->mainDataBytes = 0;
				}
			}
			*inbuf += decInfo->nSlots;
			*bytesLeft -= decInfo->nSlots;
			return ERR_MP3_MAINDATA_UNDERFLOW;
		}

		if(decInfo->mainDataBegin > 0)
		{
			memmove(decInfo->mainBuf, decInfo->mainBuf + decInfo->mainDataBytes - decInfo->mainDataBegin, decInfo->mainDataBegin);
		}
		memcpy(decInfo->mainBuf + decInfo->mainDataBegin, *inbuf, decInfo->nSlots);
		decInfo->mainDataBytes = decInfo->mainDataBegin + decInfo->nSlots;
		*inbuf += decInfo->nSlots;
		*bytesLeft -= decInfo->nSlots;

		unsigned char *mainPtr = decInfo->mainBuf;
		int bitOffset = 0;
		int availBits = decInfo->mainDataBytes << 3;

		for(int gr = 0; gr < decInfo->nGrans; gr++)
		{
			for(int ch = 0; ch < decInfo->nChans; ch++)
			{
				int prevBitOffset = bitOffset;
				int nBytes = UnpackScaleFactors(decInfo, mainPtr, &bitOffset, decInfo->part23Length[gr][ch], gr, ch);
				int sfBits = 8 * nBytes - prevBitOffset + bitOffset;
				int huffBlockBits = decInfo->part23Length[gr][ch] - sfBits;
				mainPtr += nBytes;
				availBits -= sfBits;
				if((nBytes < 0) || (availBits < huffBlockBits))
					return ERR_MP3_INVALID_SCALEFACT;

				prevBitOffset = bitOffset;
				nBytes = DecodeHuffman(decInfo, mainPtr, &bitOffset, huffBlockBits, gr, ch);
				if(nBytes < 0)
					return ERR_MP3_INVALID_HUFFCODES;
				mainPtr += nBytes;
				availBits -= (8 * nBytes - prevBitOffset + bitOffset);
			}
			if(Dequantize(decInfo, gr) < 0)
				return ERR_MP3_INVALID_DEQUANTIZE;
			for(int ch = 0; ch < decInfo->nChans; ch++)
			{
				if(IMDCT(decInfo, gr, ch) < 0)
					return ERR_MP3_INVALID_IMDCT;
			}
			if(Subband(decInfo, outbuf + gr * decInfo->nGranSamps * decInfo->nChans) < 0)
				return ERR_MP3_INVALID_SUBBAND;
		}

		return ERR_MP3_NONE;
	}

	void refillFileBuffer(void)
	{
		size_t avail = fileBufferAvailable();
		if((avail < 1600) && (byteCount < dataSize))
		{
			if((avail > 0) && (fileBufferPosition > 0))
			{
				memmove(fileBuffer, fileBuffer + fileBufferPosition, avail);
			}
			fileBufferLength = avail;
			fileBufferPosition = 0;

			size_t space = MP3_FILE_BUFFER_SIZE - fileBufferLength;
			size_t remainingInFile = dataSize - byteCount;
			size_t bytesToRead = (remainingInFile < space) ? remainingInFile : space;

			gpio_set_level(AUX5, 1);  ///////////////////////////////////////////////////////////////////////////////////////////
			size_t bytesRead = mp3File.read(fileBuffer + fileBufferLength, bytesToRead);
			gpio_set_level(AUX5, 0);  ///////////////////////////////////////////////////////////////////////////////////////////

			if(0 == bytesRead)
			{
				byteCount = dataSize;
			}
			else
			{
				fileBufferLength += bytesRead;
				byteCount += bytesRead;
			}
		}
	}

	bool decodeNextFrame(void)
	{
		if((NULL == mp3DecInfo) || (NULL == pcmBuffer))
			return false;

		for(int attempts = 0; attempts < 128; attempts++)
		{
			refillFileBuffer();

			size_t avail = fileBufferAvailable();
			if(avail < 4)
			{
				fileBufferPosition = 0;
				fileBufferLength = 0;
				byteCount = dataSize;
				return false;
			}

			int syncOffset = findSyncWord(fileBuffer + fileBufferPosition, (int)avail);
			if(syncOffset < 0)
			{
				size_t keep = (0xFF == fileBuffer[fileBufferLength - 1]) ? 1 : 0;
				fileBufferPosition = fileBufferLength - keep;
				if(byteCount >= dataSize)
				{
					fileBufferPosition = 0;
					fileBufferLength = 0;
					return false;
				}
				continue;
			}

			if(syncOffset > 0)
			{
				fileBufferPosition += syncOffset;
				if((fileBufferAvailable() < 1600) && (byteCount < dataSize))
				{
					continue;
				}
			}

			uint8_t *inPtr = fileBuffer + fileBufferPosition;
			int bytesLeft = (int)fileBufferAvailable();
			int err = decodeFrame(mp3DecInfo, &inPtr, &bytesLeft, pcmBuffer);

			if(ERR_MP3_NONE == err)
			{
				fileBufferPosition = (size_t)(inPtr - fileBuffer);
				size_t monoSamples = (size_t)(mp3DecInfo->nGrans * mp3DecInfo->nGranSamps);
				if(2 == mp3DecInfo->nChans)
				{
					// Downmix interleaved stereo to mono in-place
					for(size_t i = 0; i < monoSamples; i++)
					{
						int32_t left = pcmBuffer[2 * i];
						int32_t right = pcmBuffer[2 * i + 1];
						pcmBuffer[i] = (int16_t)((left + right) / 2);
					}
				}
				pcmLength = monoSamples;
				pcmPosition = 0;
				return true;
			}
			else if(ERR_MP3_MAINDATA_UNDERFLOW == err)
			{
				fileBufferPosition = (size_t)(inPtr - fileBuffer);
				continue;
			}
			else if(ERR_MP3_INDATA_UNDERFLOW == err)
			{
				if(byteCount >= dataSize)
				{
					// Truncated frame at end of file
					fileBufferPosition = 0;
					fileBufferLength = 0;
					return false;
				}
				// Otherwise advance 1 byte if buffer was already full to avoid getting stuck
				if(fileBufferAvailable() >= 1600)
				{
					fileBufferPosition++;
				}
				continue;
			}
			else
			{
				// Invalid header or corrupt frame: advance past the byte or consumed frame
				size_t consumed = (size_t)(inPtr - (fileBuffer + fileBufferPosition));
				fileBufferPosition += (consumed > 0) ? consumed : 1;
			}
		}

		// Exceeded retry limit on corrupt data
		fileBufferPosition = 0;
		fileBufferLength = 0;
		byteCount = dataSize;
		return false;
	}

	public:
		static bool parseHeader(const uint8_t *hdr, uint32_t *sr, uint32_t *frameSize, uint32_t *sideInfoSize, MPEGVersion *verOut)
		{
			if((0xFF != hdr[0]) || (0xE0 != (hdr[1] & 0xE0)))
				return false;

			uint8_t verBits = (hdr[1] >> 3) & 0x03;
			if(1 == verBits)
				return false;  // Reserved MPEG version
			MPEGVersion ver = (0 == verBits) ? MPEG25 : ((3 == verBits) ? MPEG1 : MPEG2);

			int layer = 4 - ((hdr[1] >> 1) & 0x03);
			if(3 != layer)
				return false;  // Must be Layer III

			int brIdx = (hdr[2] >> 4) & 0x0F;
			if((0 == brIdx) || (15 == brIdx))
				return false;

			int srIdx = (hdr[2] >> 2) & 0x03;
			if(3 == srIdx)
				return false;

			int emphasis = hdr[3] & 0x03;
			if(2 == emphasis)
				return false;

			int paddingBit = (hdr[2] >> 1) & 0x01;
			Mp3StereoMode sMode = (Mp3StereoMode)((hdr[3] >> 6) & 0x03);
			int sideBytes = sideBytesTab[ver][(MP3_MONO == sMode) ? 0 : 1];
			int totalBytes = slotTab[ver][srIdx][brIdx] + paddingBit;
			int fhBytes = ((~hdr[1]) & 0x01) ? 6 : 4;

			if(totalBytes - sideBytes - fhBytes < 0)
				return false;

			if(sr)
				*sr = (uint32_t)samplerateTab[ver][srIdx];
			if(frameSize)
				*frameSize = (uint32_t)totalBytes;
			if(sideInfoSize)
				*sideInfoSize = (uint32_t)sideBytes;
			if(verOut)
				*verOut = ver;
			return true;
		}

		SdMp3Sound(const char *fname, size_t numBytes, size_t offset, uint32_t sr)
		{
			fileName = strdup(fname);
			dataOffset = offset;
			dataSize = numBytes;
			sampleRate = sr;
			mp3DecInfo = NULL;
			pcmBuffer = NULL;
			pcmLength = 0;
			pcmPosition = 0;
			fileBufferLength = 0;
			fileBufferPosition = 0;
			byteCount = 0;
		}
		~SdMp3Sound()
		{
			close();
			free(fileName);
		}
		void open(void)
		{
			close();
			mp3File = SD.open(String("/") + fileName);
			mp3File.seek(dataOffset);
			Serial.print("Open: ");
			Serial.println(fileName);
			byteCount = 0;
			fileBufferLength = 0;
			fileBufferPosition = 0;
			pcmLength = 0;
			pcmPosition = 0;
			mp3DecInfo = AllocateBuffers();
			pcmBuffer = (int16_t *)malloc(MP3_MAX_PCM_SAMPLES * sizeof(int16_t));
		}
		size_t available(void)
		{
			size_t pcmAvailBytes = pcmSamplesAvailable() * 2;
			size_t unreadFileBytes = (dataSize > byteCount) ? (dataSize - byteCount) : 0;
			size_t inBufAvail = fileBufferAvailable();
			if(pcmAvailBytes > 0)
				return(pcmAvailBytes + inBufAvail + unreadFileBytes);
			if(inBufAvail + unreadFileBytes >= 8)
				return(inBufAvail + unreadFileBytes);
			return 0;
		}
		int16_t getNextSample(void)
		{
			if(0 == pcmSamplesAvailable())
			{
				decodeNextFrame();
			}

			if(pcmSamplesAvailable() > 0)
			{
				return pcmBuffer[pcmPosition++];
			}
			else
			{
				return 0;
			}
		}
		void close(void)
		{
			if(mp3File)
			{
				mp3File.close();
			}
			if(NULL != mp3DecInfo)
			{
				FreeBuffers(mp3DecInfo);
				mp3DecInfo = NULL;
			}
			if(NULL != pcmBuffer)
			{
				free(pcmBuffer);
				pcmBuffer = NULL;
			}
			pcmLength = 0;
			pcmPosition = 0;
			fileBufferLength = 0;
			fileBufferPosition = 0;
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
		void close(void)
		{
			return;
		}
};
