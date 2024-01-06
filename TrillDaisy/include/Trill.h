#pragma once

#include "daisy_seed.h"
#include <array>

namespace TrillDaisy {

class Trill {
public:
  using Logger = void(*)(const char*);
  using TouchType = uint16_t;
  using I2cAddress = uint8_t;

  enum class Mode : int8_t {
    AUTO = -1,
    CENTROID = 0,
    RAW = 1,
    BASELINE = 2,
    DIFF = 3
  };

  enum class TrasmitOffset{
    COMMAND = 0,
    DATA = 4
  };

  constexpr static std::size_t NUM_DEVICE_TYPES = 4U;

  enum class DeviceType {
    TRILL_NONE = -1,
    TRILL_UNKNOWN = 0,
    TRILL_BAR = 1,
    TRILL_SQUARE = 2,
  };

  struct TrillDefaults 
  {
    Trill::DeviceType device;
    Trill::Mode mode;
    I2cAddress address;
  };

  enum class MaxNumberOfTouches{
    oneDim = 5,
    twoDim = 4
  };

  void init(DeviceType deviceType, Logger loggerCallback);
  bool prepareForDataRead();
  void scanSensor();
  uint8_t getNumDetectedVerticalTouches();
  uint8_t getNumDetectedHorizontalTouches();
  float getAverageX();
  float getAverageY();
  float getAverageSize();
  float getTouchLocation(uint8_t touchNum);
  float getTouchHorizontalLocation(uint8_t touchNum);
  float getTouchSize(uint8_t touchNum);
  float getTouchHorizontalSize(uint8_t touchNum);

  struct __attribute__ ((packed)) ScanSettings{
    uint8_t speed;
    uint8_t numberOfBits;
  };
 
private:
  enum class TRILL_COMMAND {
    None = 0,
    Mode = 1,
    ScanSettings = 2,
    Prescaler = 3,
    NoiseThreshold = 4,
    Idac = 5,
    BaselineUpdate = 6,
    MinimumSize = 7,
    AutoScanInterval = 16,
    Identify = 255
  };


  bool identifyType();
  bool sendCommand(TRILL_COMMAND cmd);
  template<typename DataType>
  bool sendCommand(TRILL_COMMAND cmd, const DataType& data);
  void updateRescale();
  bool setScanSettings();
  void updateNumberOfTouches();
  bool readData(uint8_t *data, uint16_t size);

  static constexpr uint8_t _interCommandDelay_ms = 15;
  static constexpr uint8_t _numberOfCentroids = 32;
  DeviceType _deviceType = DeviceType::TRILL_NONE;
  I2cAddress _deviceAddr;
  Mode _mode;
  uint8_t _firmwareVersion;
  daisy::I2CHandle::Config _i2Config;
  daisy::I2CHandle _i2cHandle;
  Logger _loggerCallback = nullptr;
  uint8_t _detectedVerticalTouches = 0U;
  uint8_t _detectedHorizontalTouches = 0U;
  float _rescaleVertical;
  float _rescaleHorizontal;
  float _rescaleSize;
  ScanSettings _scanSettings{0,12};

  std::array<uint8_t,_numberOfCentroids*2*sizeof(uint16_t)> _rawBuffer;

static constexpr uint8_t getMaxNumberOfTouches(DeviceType device){
  switch (device)
  {
  case DeviceType::TRILL_BAR:
    return 5; 
    break;
  case DeviceType::TRILL_SQUARE:
    return 4;
    break;
  default:
    return 0;
    break;
  }
} 


};

template<typename DataType>
bool Trill::sendCommand(TRILL_COMMAND cmd, const DataType& data)
{
  std::array<uint8_t, sizeof(DataType)+2U> buffer;
  buffer[0] = static_cast<uint8_t>(TrasmitOffset::COMMAND);
  buffer[1] = static_cast<uint8_t>(cmd);
  std::memcpy(&buffer[2], &data, sizeof(data));
  return daisy::I2CHandle::Result::OK == _i2cHandle.TransmitBlocking(_deviceAddr, buffer.data(), buffer.size(), 3000);
}



} // namespace TrillDaisy
