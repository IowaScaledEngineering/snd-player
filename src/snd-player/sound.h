#pragma once

class Sound
{
  protected:
    size_t dataSize;
    size_t byteCount;
    uint16_t sampleRate;

  public:
    virtual void open(void);
    virtual size_t available(void)
    {
      return(dataSize - byteCount);
    }
    virtual size_t read(uint8_t *buffer, size_t numBytes);
    virtual void close(void);
    uint16_t getSampleRate(void)
    {
      return sampleRate;
    }
};

class SdSound : public Sound
{
  char *fileName;
  size_t dataOffset;
  File wavFile;

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
