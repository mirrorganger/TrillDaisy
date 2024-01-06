#include "Trill.h"

namespace TrillDaisy {



struct RescaleFactorT{
	float posV;
	float posH;
	float size;
};

static constexpr std::array<RescaleFactorT,7U> trillRescaleFactors = {
	RescaleFactorT{1, 0,  1}, // UNKNOWN = 0,
	RescaleFactorT{3200, 0,  4566}, // BAR = 1,
	RescaleFactorT{1792, 1792,  3780}, // SQUARE = 2,
	RescaleFactorT{4096, 0,  1}, // CRAFT = 3,
	RescaleFactorT{3584,  0,  5000}, // RING = 4,
	RescaleFactorT{1920, 1664, 4000}, // HEX = 5,
	RescaleFactorT{3712, 0,  1200}, // FLEX = 6,
};




void Trill::init(DeviceType deviceType, Logger loggerCallback) {

  _loggerCallback = loggerCallback;

  static constexpr std::array<TrillDefaults, NUM_DEVICE_TYPES> trillDefaults = {
      TrillDefaults{DeviceType::TRILL_NONE, Mode::AUTO, 0xFF},
      TrillDefaults{DeviceType::TRILL_UNKNOWN, Mode::AUTO, 0xFF},
      TrillDefaults{DeviceType::TRILL_BAR, Mode::CENTROID, 0x20},
      TrillDefaults{DeviceType::TRILL_SQUARE, Mode::CENTROID, 0x28}};

  _deviceAddr = trillDefaults[static_cast<size_t>(deviceType) + 1U].address;
  _i2Config.address = _deviceAddr;
  _i2Config.speed = daisy::I2CHandle::Config::Speed::I2C_100KHZ;
  _i2Config.periph = daisy::I2CHandle::Config::Peripheral::I2C_1;
  _i2Config.mode = daisy::I2CHandle::Config::Mode::I2C_MASTER;
  _i2Config.pin_config.scl = {DSY_GPIOB, 8};
  _i2Config.pin_config.sda = {DSY_GPIOB, 9};
  daisy::I2CHandle::Result res = _i2cHandle.Init(_i2Config);

  if (res == daisy::I2CHandle::Result::ERR) {
    return;
  }
  _loggerCallback("I2C initialized");
  identifyType();
  if ((_deviceType != deviceType) &&
      (_deviceType == DeviceType::TRILL_UNKNOWN)) {
    return;
  }
  switch (_deviceType) {
  case DeviceType::TRILL_BAR:
    _loggerCallback("TRILL BAR found");
    break;
  case DeviceType::TRILL_SQUARE:
    _loggerCallback("TRILL SQUARE  found");
    break;
  default:
    break;
  }
  daisy::System::Delay(_interCommandDelay_ms);
  _mode = trillDefaults[static_cast<size_t>(_deviceType) + 1U].mode;
  if (!sendCommand(TRILL_COMMAND::Mode, _mode)) {
    _loggerCallback("Error setting mode");
    return;
  }
  daisy::System::Delay(_interCommandDelay_ms);
  if (!sendCommand(TRILL_COMMAND::BaselineUpdate)) {
    _loggerCallback("Error setting baseline update");
    return;
  }
  daisy::System::Delay(_interCommandDelay_ms);
  if (!setScanSettings()) {
    _loggerCallback("Error setting scan settings update");
    return;
  }
  updateRescale();
}

bool Trill::identifyType() {
  if (sendCommand(TRILL_COMMAND::Identify)) {
    daisy::System::Delay(_interCommandDelay_ms);
    std::array<uint8_t, 4U> receiveBuffer;
    if (readData(receiveBuffer.data(), receiveBuffer.size())) {
      _deviceType = static_cast<DeviceType>(receiveBuffer[1]);
      _firmwareVersion = receiveBuffer[2];
      return true;
    }
  }
  return false;
}

void Trill::updateRescale()
{
	float scale = 1 << (12 - _scanSettings.numberOfBits);
	_rescaleVertical = 1.f / trillRescaleFactors[static_cast<uint8_t>(_deviceType)].posV;
	_rescaleHorizontal = 1.f / trillRescaleFactors[static_cast<uint8_t>(_deviceType)].posH;
	_rescaleSize = scale / trillRescaleFactors[static_cast<uint8_t>(_deviceType)].size;
}

bool Trill::setScanSettings(){
  if (_scanSettings.speed > 3)
    _scanSettings.speed = 3;
  if (_scanSettings.numberOfBits < 9)
    _scanSettings.numberOfBits = 9;
  if (_scanSettings.numberOfBits > 16)
    _scanSettings.numberOfBits = 16;

  return sendCommand(TRILL_COMMAND::ScanSettings, _scanSettings);
}

bool Trill::sendCommand(TRILL_COMMAND cmd) {
  std::array<uint8_t, 2U> buf = {static_cast<uint8_t>(TrasmitOffset::COMMAND),
                                 static_cast<uint8_t>(cmd)};
  return daisy::I2CHandle::Result::OK ==
         _i2cHandle.TransmitBlocking(_deviceAddr, buf.data(), buf.size(), 1000);
};

bool Trill::prepareForDataRead() {
  std::array<uint8_t, 1U> buf = {static_cast<uint8_t>(TrasmitOffset::DATA)};
  return daisy::I2CHandle::Result::OK ==
         _i2cHandle.TransmitBlocking(_deviceAddr, buf.data(), buf.size(), 1000);
};

bool Trill::readData(uint8_t *data, uint16_t size) {
  return daisy::I2CHandle::Result::OK ==
         _i2cHandle.ReceiveBlocking(_deviceAddr, data, size, 1000);
}

void Trill::scanSensor() {
  readData(_rawBuffer.data(), _rawBuffer.size());
  updateNumberOfTouches();
}

uint8_t Trill::getNumDetectedVerticalTouches() {
  return _detectedVerticalTouches;
}

uint8_t Trill::getNumDetectedHorizontalTouches() {
  return _detectedHorizontalTouches;
}

float Trill::getAverageX(){

  if(_detectedVerticalTouches==0) return 0.0;

  float average = 0.0;
  float totalSize = 0.0;
  for (size_t i = 0; i < _detectedVerticalTouches; i++)
  {
    average += getTouchLocation(i) * getTouchSize(i); 
    totalSize += getTouchSize(i);
  }
  return average/totalSize;
}

float Trill::getAverageY(){
  
  if(_detectedHorizontalTouches==0) return 0.0;
  
  float average = 0.0;
  float totalSize = 0.0;
  for (size_t i = 0; i < _detectedHorizontalTouches; i++)
  {
    average += getTouchHorizontalLocation(i) * getTouchHorizontalSize(i); 
    totalSize += getTouchHorizontalSize(i);
  }
  return average/totalSize;
}

float Trill::getAverageSize(){

  if(_detectedVerticalTouches==0) return 0.0;

  float average = 0.0;
  for (size_t i = 0; i < _detectedVerticalTouches; i++)
  {
    average += getTouchSize(i); 
  }
  return average/_detectedVerticalTouches;
}



float Trill::getTouchLocation(uint8_t touchNum)
{
    if(touchNum > getMaxNumberOfTouches(_deviceType)) return 0;
    TouchType location = (_rawBuffer[sizeof(TouchType)*touchNum] << 8) + _rawBuffer[sizeof(TouchType)*touchNum + 1];
    return static_cast<float>(location) * _rescaleVertical;
}

float Trill::getTouchSize(uint8_t touchNum)
{
    if(touchNum > getMaxNumberOfTouches(_deviceType)) return 0;
    uint8_t offset = sizeof(TouchType) * getMaxNumberOfTouches(_deviceType);
    TouchType touchSize = (_rawBuffer[offset + sizeof(TouchType)*touchNum] << 8) + _rawBuffer[offset + sizeof(TouchType)*touchNum + 1];
    return static_cast<float>(touchSize)*  _rescaleSize;
}

float Trill::getTouchHorizontalLocation(uint8_t touchNum)
{
  if(touchNum > getMaxNumberOfTouches(_deviceType)) return 0;
  uint8_t offset = 2 * sizeof(TouchType) * getMaxNumberOfTouches(_deviceType);
  TouchType location = (_rawBuffer[offset + sizeof(TouchType)*touchNum] << 8) + _rawBuffer[offset + sizeof(TouchType)*touchNum + 1];
  return static_cast<float>(location) * _rescaleHorizontal;
}

float Trill::getTouchHorizontalSize(uint8_t touchNum){
  
  if(touchNum > getMaxNumberOfTouches(_deviceType)) return 0;
  
  uint8_t offset = 3 * sizeof(TouchType) * getMaxNumberOfTouches(_deviceType);
  TouchType touchSize = (_rawBuffer[offset + sizeof(TouchType)*touchNum] << 8) + _rawBuffer[offset + sizeof(TouchType)*touchNum + 1];
  return static_cast<float>(touchSize) * _rescaleSize;
}


void Trill::updateNumberOfTouches() {

  _detectedHorizontalTouches = 0U;
  _detectedVerticalTouches = 0U;

  auto maxNumberOfTouches = getMaxNumberOfTouches(_deviceType);

  for (uint8_t pos = 0; pos < maxNumberOfTouches; pos++) {
    if (_rawBuffer[sizeof(TouchType) * pos] == 0xFF &&
        _rawBuffer[sizeof(TouchType) * pos + 1] == 0xFF)
      break;

    _detectedVerticalTouches++;
  }

  if (_deviceType == DeviceType::TRILL_SQUARE) {
    uint8_t horizontalOffset = 2 * sizeof(TouchType) * maxNumberOfTouches;
    // Look for the number of horizontal touches in 2D sliders
    // which might be different from number of vertical touches
    for (uint8_t pos = 0; pos < maxNumberOfTouches; pos++) {

      if (_rawBuffer[sizeof(TouchType) * pos + horizontalOffset] == 0xFF &&
          _rawBuffer[sizeof(TouchType) * pos + horizontalOffset + 1] == 0xFF)
        break;

      _detectedHorizontalTouches++;
    }
  }
}

} // namespace TrillDaisy