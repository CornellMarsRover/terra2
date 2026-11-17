// C++ port of cmr_imu protocol_485_resolver.py / jy901s_dataProcessor.py.
#include <chrono>
#include <string>
#include <thread>

#include "cmr_imu_cpp/device_model.hpp"
#include "cmr_imu_cpp/py_compat.hpp"

namespace cmr_imu_cpp
{

namespace
{

// region  CRC tables (verbatim from protocol_485_resolver.py)
const uint8_t auchCRCHi[256] = {
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
  0x00, 0xC1, 0x81, 0x40,
};

const uint8_t auchCRCLo[256] = {
  0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
  0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
  0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
  0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
  0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
  0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
  0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
  0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
  0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
  0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
  0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
  0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
  0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
  0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
  0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
  0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
  0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
  0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
  0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
  0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
  0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
  0x41, 0x81, 0x80, 0x40,
};
// endregion

void sleep_s(double s)
{
  std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

}  // namespace

// ---------------------------------------------------------------------------
// JY901SDataProcessor

std::vector<JY901SDataProcessor::Listener> & JY901SDataProcessor::onVarChanged()
{
  static std::vector<Listener> listeners;
  return listeners;
}

void JY901SDataProcessor::onUpdate(DeviceModel & deviceModel)
{
  for (auto & fun : onVarChanged()) {
    fun(deviceModel);
  }
}

// ---------------------------------------------------------------------------
// Protocol485Resolver

int Protocol485Resolver::get_crc(const std::vector<uint8_t> & datas, size_t dlen) const
{
  int tempH = 0xff;  // high CRC byte initialization
  int tempL = 0xff;  // low CRC byte initialization
  for (size_t i = 0; i < dlen; ++i) {
    const int tempIndex = (tempH ^ datas[i]) & 0xff;
    tempH = (tempL ^ auchCRCHi[tempIndex]) & 0xff;
    tempL = auchCRCLo[tempIndex];
  }
  return (tempH << 8) | tempL;
}

void Protocol485Resolver::sendData(const std::vector<uint8_t> & data, DeviceModel & deviceModel)
{
  deviceModel.serialWrite(data);
}

void Protocol485Resolver::passiveReceiveData(
  const uint8_t * data, size_t len, DeviceModel & deviceModel)
{
  // NOTE(port): identical framing state machine to the Python. Two quirks are kept:
  //  * only one leading byte is dropped per received byte when re-syncing;
  //  * if the buffer starts with ADDR,0x03 and has already grown past
  //    TempBytes[2]+5 bytes (possible after a CRC failure), it never matches
  //    again and grows without bound -> the IMU stream stalls until restart.
  for (size_t k = 0; k < len; ++k) {
    TempBytes.push_back(data[k]);
    if (TempBytes[0] != deviceModel.ADDR) {  // first byte is not the device ID
      TempBytes.erase(TempBytes.begin());
      continue;
    }
    if (TempBytes.size() > 2) {
      if (TempBytes[1] != 0x03) {  // second byte is not the read command
        TempBytes.erase(TempBytes.begin());
        continue;
      }
      const size_t tlen = TempBytes.size();
      if (tlen == static_cast<size_t>(TempBytes[2]) + 5) {  // one full packet
        const int tempCrc = get_crc(TempBytes, tlen - 2);
        if ((tempCrc >> 8) == TempBytes[tlen - 2] && (tempCrc & 0xff) == TempBytes[tlen - 1]) {
          if (PackSize == tlen) {  // acceleration, angular velocity, angle...
            get_data(TempBytes, deviceModel);
            deviceModel.dataProcessor->onUpdate(deviceModel);
          }
          get_find(TempBytes, deviceModel);
          TempBytes.clear();
        } else {  // CRC failed
          TempBytes.erase(TempBytes.begin());
        }
      }
    }
  }
}

std::vector<uint8_t> Protocol485Resolver::get_readbytes(int devid, int regAddr, int regCount) const
{
  std::vector<uint8_t> tempBytes(8, 0);
  tempBytes[0] = static_cast<uint8_t>(devid);
  tempBytes[1] = 0x03;
  tempBytes[2] = static_cast<uint8_t>(regAddr >> 8);
  tempBytes[3] = static_cast<uint8_t>(regAddr & 0xff);
  tempBytes[4] = static_cast<uint8_t>(regCount >> 8);
  tempBytes[5] = static_cast<uint8_t>(regCount & 0xff);
  const int tempCrc = get_crc(tempBytes, tempBytes.size() - 2);
  tempBytes[6] = static_cast<uint8_t>(tempCrc >> 8);
  tempBytes[7] = static_cast<uint8_t>(tempCrc & 0xff);
  return tempBytes;
}

std::vector<uint8_t> Protocol485Resolver::get_writebytes(int devid, int regAddr, int sValue) const
{
  std::vector<uint8_t> tempBytes(8, 0);
  tempBytes[0] = static_cast<uint8_t>(devid);
  tempBytes[1] = 0x06;
  tempBytes[2] = static_cast<uint8_t>(regAddr >> 8);
  tempBytes[3] = static_cast<uint8_t>(regAddr & 0xff);
  tempBytes[4] = static_cast<uint8_t>(sValue >> 8);
  tempBytes[5] = static_cast<uint8_t>(sValue & 0xff);
  const int tempCrc = get_crc(tempBytes, tempBytes.size() - 2);
  tempBytes[6] = static_cast<uint8_t>(tempCrc >> 8);
  tempBytes[7] = static_cast<uint8_t>(tempCrc & 0xff);
  return tempBytes;
}

void Protocol485Resolver::get_data(const std::vector<uint8_t> & datahex, DeviceModel & deviceModel)
{
  using cmr_port::py_round;
  int tempReg = 0x30;                 // start register
  const int dlen = datahex[2] / 2;    // number of registers
  std::vector<double> tempVals;
  for (int i = 0; i < dlen; ++i) {
    const int tempIndex = 3 + i * 2;
    const int raw = (datahex[tempIndex] << 8) | datahex[tempIndex + 1];  // unsigned 16 bit
    double tempVal = raw;
    if (0x30 <= tempReg && tempReg <= 0x33) {  // chip time
      tempVals.push_back(raw);
      if (tempReg == 0x33) {
        const int v0 = static_cast<int>(tempVals[0]);
        const int v1 = static_cast<int>(tempVals[1]);
        const int v2 = static_cast<int>(tempVals[2]);
        const int v3 = static_cast<int>(tempVals[3]);
        const int _year = 2000 + (v0 & 0xff);
        const int _moth = (v0 >> 8) & 0xff;
        const int _day = v1 & 0xff;
        const int _hour = (v1 >> 8) & 0xff;
        const int _minute = v2 & 0xff;
        const int _second = (v2 >> 8) & 0xff;
        const int _millisecond = v3;
        deviceModel.setDeviceString(
          "Chiptime",
          std::to_string(_year) + "-" + std::to_string(_moth) + "-" + std::to_string(_day) +
          " " + std::to_string(_hour) + ":" + std::to_string(_minute) + ":" +
          std::to_string(_second) + "." + std::to_string(_millisecond));
        tempVals.clear();
      }
    } else if (tempReg >= 0x34 && tempReg <= 0x36) {  // acceleration X Y Z
      tempVal = tempVal / 32768.0 * accRange;
      if (tempVal >= accRange) {
        tempVal -= 2 * accRange;
      }
      tempVals.push_back(py_round(tempVal, 4));
      if (tempReg == 0x36) {
        deviceModel.setDeviceData("accX", tempVals[0]);
        deviceModel.setDeviceData("accY", tempVals[1]);
        deviceModel.setDeviceData("accZ", tempVals[2]);
        tempVals.clear();
      }
    } else if (tempReg == 0x40) {  // temperature
      const double temperature = py_round(tempVal / 100.0, 2);
      // NOTE(port): unsigned register, so sub-zero temperatures read as ~655 C (as in Python).
      deviceModel.setDeviceData("temperature", temperature);
    } else if (tempReg >= 0x37 && tempReg <= 0x39) {  // angular velocity X Y Z
      tempVal = tempVal / 32768.0 * gyroRange;
      if (tempVal >= gyroRange) {
        tempVal -= 2 * gyroRange;
      }
      tempVals.push_back(py_round(tempVal, 4));
      if (tempReg == 0x39) {
        deviceModel.setDeviceData("gyroX", tempVals[0]);
        deviceModel.setDeviceData("gyroY", tempVals[1]);
        deviceModel.setDeviceData("gyroZ", tempVals[2]);
        tempVals.clear();
      }
    } else if (tempReg >= 0x3d && tempReg <= 0x3f) {  // angle X Y Z
      tempVal = tempVal / 32768.0 * angleRange;
      if (tempVal >= angleRange) {
        tempVal -= 2 * angleRange;
      }
      tempVals.push_back(py_round(tempVal, 3));
      if (tempReg == 0x3f) {
        deviceModel.setDeviceData("angleX", tempVals[0]);
        deviceModel.setDeviceData("angleY", tempVals[1]);
        deviceModel.setDeviceData("angleZ", tempVals[2]);
        tempVals.clear();
      }
    } else if (tempReg >= 0x3a && tempReg <= 0x3c) {  // magnetic field X Y Z
      // NOTE(port): round(int, 0) is a no-op and the raw register is never
      // sign-converted, so negative field values come out as 32768..65535 (as in Python).
      tempVals.push_back(raw);
      if (tempReg == 0x3c) {
        deviceModel.setDeviceData("magX", tempVals[0]);
        deviceModel.setDeviceData("magY", tempVals[1]);
        deviceModel.setDeviceData("magZ", tempVals[2]);
        tempVals.clear();
      }
    }
    tempReg += 1;  // next register
  }
}

std::vector<int> Protocol485Resolver::readReg(int regAddr, int regCount, DeviceModel & deviceModel)
{
  {
    std::lock_guard<std::mutex> lk(find_mutex_);
    TempFindValues.clear();
    TempReadRegCount = regCount;
  }
  const auto tempBytes = get_readbytes(deviceModel.ADDR, regAddr, regCount);
  deviceModel.serialWrite(tempBytes);
  for (int i = 0; i < 15; ++i) {  // wait up to 150 ms in 10 ms steps
    sleep_s(0.01);
    std::lock_guard<std::mutex> lk(find_mutex_);
    if (!TempFindValues.empty()) {
      break;
    }
  }
  std::lock_guard<std::mutex> lk(find_mutex_);
  return TempFindValues;
}

void Protocol485Resolver::writeReg(int regAddr, int sValue, DeviceModel & deviceModel)
{
  deviceModel.serialWrite(get_writebytes(deviceModel.ADDR, regAddr, sValue));
}

void Protocol485Resolver::get_find(const std::vector<uint8_t> & datahex, DeviceModel &)
{
  std::vector<int> tempArr;
  const int dlen = datahex[2] / 2;
  for (int i = 0; i < dlen; ++i) {
    const int tempIndex = 3 + i * 2;
    tempArr.push_back((datahex[tempIndex] << 8) | datahex[tempIndex + 1]);
  }
  std::lock_guard<std::mutex> lk(find_mutex_);
  TempFindValues.insert(TempFindValues.end(), tempArr.begin(), tempArr.end());
}

void Protocol485Resolver::unlock(DeviceModel & deviceModel)
{
  deviceModel.serialWrite(get_writebytes(deviceModel.ADDR, 0x69, 0xb588));
}

void Protocol485Resolver::save(DeviceModel & deviceModel)
{
  deviceModel.serialWrite(get_writebytes(deviceModel.ADDR, 0x00, 0x00));
}

void Protocol485Resolver::AccelerationCalibration(DeviceModel & deviceModel)
{
  unlock(deviceModel);
  sleep_s(0.1);
  deviceModel.serialWrite(get_writebytes(deviceModel.ADDR, 0x01, 0x01));
  sleep_s(5.5);
}

void Protocol485Resolver::BeginFiledCalibration(DeviceModel & deviceModel)
{
  unlock(deviceModel);
  sleep_s(0.1);
  deviceModel.serialWrite(get_writebytes(deviceModel.ADDR, 0x01, 0x07));
}

void Protocol485Resolver::EndFiledCalibration(DeviceModel & deviceModel)
{
  unlock(deviceModel);
  sleep_s(0.1);
  save(deviceModel);
}

}  // namespace cmr_imu_cpp
