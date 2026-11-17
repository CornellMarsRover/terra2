// C++ port of cmr_imu/device_model.py.
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

#include "cmr_imu_cpp/device_model.hpp"

namespace cmr_imu_cpp
{

DeviceModel::DeviceModel(
  std::string deviceName_, IProtocolResolver * protocolResolver_,
  IDataProcessor * dataProcessor_, std::string dataUpdateListener_)
: deviceName(std::move(deviceName_)),
  dataUpdateListener(std::move(dataUpdateListener_)),
  dataProcessor(dataProcessor_),
  protocolResolver(protocolResolver_)
{
  std::cout << "初始化设备模型" << std::endl;
}

DeviceModel::~DeviceModel()
{
  isOpen = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  serialPort_.close();
}

void DeviceModel::setDeviceData(const std::string & key, double value)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  deviceData_[key] = value;
}

void DeviceModel::setDeviceString(const std::string & key, const std::string & value)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  deviceStrings_[key] = value;
}

std::optional<double> DeviceModel::getDeviceData(const std::string & key) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  auto it = deviceData_.find(key);
  if (it == deviceData_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::optional<std::string> DeviceModel::getDeviceString(const std::string & key) const
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  auto it = deviceStrings_.find(key);
  if (it == deviceStrings_.end()) {
    return std::nullopt;
  }
  return it->second;
}

void DeviceModel::removeDeviceData(const std::string & key)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  deviceData_.erase(key);
  deviceStrings_.erase(key);
}

void DeviceModel::readDataTh(const std::string & threadName)
{
  std::cout << "启动" << threadName << std::endl;
  uint8_t buf[1024];
  while (true) {
    if (isOpen) {
      try {
        // NOTE(port): Python busy-polls inWaiting() at 100% CPU; this blocks in
        // poll() for up to 100 ms instead.
        const size_t tlen = serialPort_.read_some(buf, sizeof(buf), 100);
        if (tlen > 0) {
          onDataReceived(buf, tlen);
        }
      } catch (const std::exception & ex) {
        std::cout << ex.what() << std::endl;
        // NOTE(port): Python retries immediately in a tight loop; back off 100 ms.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::cout << "暂停" << std::endl;
      break;
    }
  }
}

void DeviceModel::openDevice()
{
  // close the port first
  closeDevice();
  try {
    // Exclusive flock + TIOCEXCL taken before any byte is written.
    serialPort_.open(serialConfig.portName, serialConfig.baud);
  } catch (const cmr_port::PortBusyError &) {
    throw;  // another node owns the IMU port: caller logs and retries
  } catch (const std::exception & ex) {
    // NOTE(port): Python's handler builds "打开" + portName + baud + "失败" with an int
    // baud, raising TypeError, so a failed open crashes the node. Keep the crash
    // (the fabric fault handler restarts it), but with a readable message.
    const std::string msg =
      "打开" + serialConfig.portName + std::to_string(serialConfig.baud) + "失败";
    std::cout << msg << std::endl;
    throw std::runtime_error(msg + ": " + ex.what());
  }
  isOpen = true;
  read_thread_ = std::thread(&DeviceModel::readDataTh, this, "Data-Received-Thread");
}

void DeviceModel::closeDevice()
{
  const bool was_running = read_thread_.joinable();
  isOpen = false;
  if (was_running) {
    read_thread_.join();
  }
  if (serialPort_.is_open()) {
    serialPort_.close();
    std::cout << "端口关闭了" << std::endl;
  }
  std::cout << "设备关闭了" << std::endl;
}

void DeviceModel::onDataReceived(const uint8_t * data, size_t len)
{
  if (protocolResolver != nullptr) {
    protocolResolver->passiveReceiveData(data, len, *this);
  }
}

void DeviceModel::serialWrite(const std::vector<uint8_t> & data)
{
  std::lock_guard<std::mutex> lk(write_mutex_);
  serialPort_.write_all(data.data(), data.size());
}

void DeviceModel::sendData(const std::vector<uint8_t> & data)
{
  if (protocolResolver != nullptr) {
    protocolResolver->sendData(data, *this);
  }
}

std::vector<int> DeviceModel::readReg(int regAddr, int regCount)
{
  if (protocolResolver != nullptr) {
    return protocolResolver->readReg(regAddr, regCount, *this);
  }
  return {};
}

void DeviceModel::writeReg(int regAddr, int sValue)
{
  if (protocolResolver != nullptr) {
    protocolResolver->writeReg(regAddr, sValue, *this);
  }
}

void DeviceModel::unlock()
{
  if (protocolResolver != nullptr) {
    protocolResolver->unlock(*this);
  }
}

void DeviceModel::save()
{
  if (protocolResolver != nullptr) {
    protocolResolver->save(*this);
  }
}

void DeviceModel::AccelerationCalibration()
{
  if (protocolResolver != nullptr) {
    protocolResolver->AccelerationCalibration(*this);
  }
}

void DeviceModel::BeginFiledCalibration()
{
  if (protocolResolver != nullptr) {
    protocolResolver->BeginFiledCalibration(*this);
  }
}

void DeviceModel::EndFiledCalibration()
{
  if (protocolResolver != nullptr) {
    protocolResolver->EndFiledCalibration(*this);
  }
}

}  // namespace cmr_imu_cpp
