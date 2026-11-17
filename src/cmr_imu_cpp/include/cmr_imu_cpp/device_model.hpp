// C++ port of the WitMotion JY901S Python SDK used by cmr_imu:
//   device_model.py, i_protocol_resolver.py, i_data_processor.py,
//   protocol_485_resolver.py, jy901s_dataProcessor.py
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cmr_imu_cpp/serial_port.hpp"

namespace cmr_imu_cpp
{

class DeviceModel;

// i_data_processor.py
class IDataProcessor
{
public:
  virtual ~IDataProcessor() = default;
  virtual void onOpen(DeviceModel & deviceModel) = 0;
  virtual void onClose() = 0;
  virtual void onUpdate(DeviceModel & deviceModel) = 0;
};

// jy901s_dataProcessor.py
class JY901SDataProcessor : public IDataProcessor
{
public:
  using Listener = std::function<void(DeviceModel &)>;
  // Python keeps this as a class-level (shared) list.
  static std::vector<Listener> & onVarChanged();

  void onOpen(DeviceModel &) override {}
  void onClose() override {}
  void onUpdate(DeviceModel & deviceModel) override;
};

// i_protocol_resolver.py
class IProtocolResolver
{
public:
  virtual ~IProtocolResolver() = default;
  virtual void setConfig(DeviceModel & deviceModel) = 0;
  virtual void sendData(const std::vector<uint8_t> & data, DeviceModel & deviceModel) = 0;
  virtual std::vector<int> readReg(int regAddr, int regCount, DeviceModel & deviceModel) = 0;
  virtual void writeReg(int regAddr, int sValue, DeviceModel & deviceModel) = 0;
  virtual void passiveReceiveData(const uint8_t * data, size_t len, DeviceModel & deviceModel) = 0;
  virtual void unlock(DeviceModel & deviceModel) = 0;
  virtual void save(DeviceModel & deviceModel) = 0;
  virtual void AccelerationCalibration(DeviceModel & deviceModel) = 0;
  virtual void BeginFiledCalibration(DeviceModel & deviceModel) = 0;
  virtual void EndFiledCalibration(DeviceModel & deviceModel) = 0;
};

// protocol_485_resolver.py
class Protocol485Resolver : public IProtocolResolver
{
public:
  int get_crc(const std::vector<uint8_t> & datas, size_t dlen) const;
  void setConfig(DeviceModel &) override {}
  void sendData(const std::vector<uint8_t> & data, DeviceModel & deviceModel) override;
  void passiveReceiveData(const uint8_t * data, size_t len, DeviceModel & deviceModel) override;
  std::vector<uint8_t> get_readbytes(int devid, int regAddr, int regCount) const;
  std::vector<uint8_t> get_writebytes(int devid, int regAddr, int sValue) const;
  void get_data(const std::vector<uint8_t> & datahex, DeviceModel & deviceModel);
  std::vector<int> readReg(int regAddr, int regCount, DeviceModel & deviceModel) override;
  void writeReg(int regAddr, int sValue, DeviceModel & deviceModel) override;
  void get_find(const std::vector<uint8_t> & datahex, DeviceModel & deviceModel);
  void unlock(DeviceModel & deviceModel) override;
  void save(DeviceModel & deviceModel) override;
  void AccelerationCalibration(DeviceModel & deviceModel) override;
  void BeginFiledCalibration(DeviceModel & deviceModel) override;
  void EndFiledCalibration(DeviceModel & deviceModel) override;

  static constexpr size_t PackSize = 87;     // size of one data packet
  static constexpr double gyroRange = 2000.0;
  static constexpr double accRange = 16.0;
  static constexpr double angleRange = 180.0;

private:
  std::vector<uint8_t> TempBytes;  // only touched by the receive thread
  std::mutex find_mutex_;
  std::vector<int> TempFindValues;
  int TempReadRegCount = 0;
};

struct SerialConfig
{
  std::string portName;
  int baud = 9600;
};

// device_model.py
class DeviceModel
{
public:
  DeviceModel(
    std::string deviceName, IProtocolResolver * protocolResolver,
    IDataProcessor * dataProcessor, std::string dataUpdateListener);
  ~DeviceModel();

  void setDeviceData(const std::string & key, double value);
  void setDeviceString(const std::string & key, const std::string & value);
  // Returns std::nullopt for unknown keys (Python returns None).
  std::optional<double> getDeviceData(const std::string & key) const;
  std::optional<std::string> getDeviceString(const std::string & key) const;
  void removeDeviceData(const std::string & key);

  void openDevice();
  void closeDevice();
  void onDataReceived(const uint8_t * data, size_t len);

  void sendData(const std::vector<uint8_t> & data);
  std::vector<int> readReg(int regAddr, int regCount);
  void writeReg(int regAddr, int sValue);
  void unlock();
  void save();
  void AccelerationCalibration();
  void BeginFiledCalibration();
  void EndFiledCalibration();

  // serialPort.write(...)
  void serialWrite(const std::vector<uint8_t> & data);

  std::string deviceName;
  int ADDR = 0x50;
  SerialConfig serialConfig;
  std::atomic<bool> isOpen{false};
  std::string dataUpdateListener;
  IDataProcessor * dataProcessor = nullptr;
  IProtocolResolver * protocolResolver = nullptr;

private:
  void readDataTh(const std::string & threadName);

  mutable std::mutex data_mutex_;
  std::map<std::string, double> deviceData_;
  std::map<std::string, std::string> deviceStrings_;
  cmr_port::SerialPort serialPort_;
  std::mutex write_mutex_;
  std::thread read_thread_;
};

}  // namespace cmr_imu_cpp
