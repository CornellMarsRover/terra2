// C++ port of cmr_imu/imu.py (console script `imu_node`).
//
// Polls a WitMotion JY901S over Modbus-RTU (RS485, /dev/ttyUSB0 @ 9600) and
// publishes cmr_msgs/IMUSensorData on /imu at 10 Hz.
#include <atomic>
#include <chrono>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "cmr_imu_cpp/device_model.hpp"
#include "cmr_msgs/msg/imu_sensor_data.hpp"
#include "rclcpp/rclcpp.hpp"

using cmr_imu_cpp::DeviceModel;
using cmr_imu_cpp::JY901SDataProcessor;
using cmr_imu_cpp::Protocol485Resolver;

namespace
{

// data_list = [temp, accX, accY, accZ, gyroX, gyroY, gyroZ, angleX, angleY, angleZ, magX, magY, magZ]
using DataList = std::vector<std::optional<double>>;

// NOTE(port): intentional change. Python used an unbounded queue.Queue() and
// popped one sample per 100 ms tick, so if the device loop ran slightly faster
// than 10 Hz /imu lagged further and further behind real time. Here each tick
// publishes only the newest sample and drops any older ones.
std::mutex g_queue_mutex;
std::deque<DataList> g_data_queue;

std::string py_list_repr(const std::vector<int> & v)
{
  std::string s = "[";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) {s += ", ";}
    s += std::to_string(v[i]);
  }
  return s + "]";
}

}  // namespace

class IMUPublisher : public rclcpp::Node
{
public:
  IMUPublisher()
  : Node("number_publisher")
  {
    publisher_ = create_publisher<cmr_msgs::msg::IMUSensorData>("/imu", 10);
    // Node clock (honours use_sim_time), like rclpy create_timer.
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(100),
      std::bind(&IMUPublisher::publish_msg, this));
  }

private:
  void publish_msg()
  {
    DataList data_list;
    {
      std::lock_guard<std::mutex> lk(g_queue_mutex);
      if (g_data_queue.empty()) {
        return;
      }
      data_list = std::move(g_data_queue.back());  // newest sample
      g_data_queue.clear();                         // drop the stale backlog
    }
    // Every key is set by a full 87-byte packet before onUpdate fires, so the
    // Python None case cannot occur; fall back to 0 defensively.
    auto v = [&data_list](size_t i) {return data_list[i].value_or(0.0);};
    cmr_msgs::msg::IMUSensorData msg;
    msg.temp = v(0);
    msg.accx = v(1);
    msg.accy = v(2);
    msg.accz = v(3);
    msg.gyrox = v(4);
    msg.gyroy = v(5);
    msg.gyroz = v(6);
    msg.anglex = v(7);
    msg.angley = v(8);
    msg.anglez = v(9);
    msg.magx = static_cast<int32_t>(v(10));
    msg.magy = static_cast<int32_t>(v(11));
    msg.magz = static_cast<int32_t>(v(12));
    publisher_->publish(msg);
  }

  rclcpp::Publisher<cmr_msgs::msg::IMUSensorData>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Example of reading configuration information
void readConfig(DeviceModel & device)
{
  auto tVals = device.readReg(0x02, 3);  // data content, return rate, communication rate
  if (!tVals.empty()) {
    std::cout << "返回结果：" << py_list_repr(tVals) << std::endl;
  } else {
    std::cout << "无返回" << std::endl;
  }
  tVals = device.readReg(0x23, 2);  // installation direction and algorithm
  if (!tVals.empty()) {
    std::cout << "返回结果：" << py_list_repr(tVals) << std::endl;
  } else {
    std::cout << "无返回" << std::endl;
  }
}

// Start recording data (the header string is built but never written, as in Python).
void startRecord()
{
  std::cout << "开始记录数据" << std::endl;
}

// Data update event
void onUpdate(DeviceModel & deviceModel)
{
  DataList data = {
    deviceModel.getDeviceData("temperature"),
    deviceModel.getDeviceData("accX"), deviceModel.getDeviceData("accY"),
    deviceModel.getDeviceData("accZ"),
    deviceModel.getDeviceData("gyroX"), deviceModel.getDeviceData("gyroY"),
    deviceModel.getDeviceData("gyroZ"),
    deviceModel.getDeviceData("angleX"), deviceModel.getDeviceData("angleY"),
    deviceModel.getDeviceData("angleZ"),
    deviceModel.getDeviceData("magX"), deviceModel.getDeviceData("magY"),
    deviceModel.getDeviceData("magZ"),
  };
  std::lock_guard<std::mutex> lk(g_queue_mutex);
  g_data_queue.push_back(std::move(data));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto number_publisher = std::make_shared<IMUPublisher>();

  Protocol485Resolver resolver;
  JY901SDataProcessor processor;
  DeviceModel device("我的JY901", &resolver, &processor, "51_0");
  device.ADDR = 0x50;  // sensor ID
  device.serialConfig.portName = "/dev/ttyUSB0";  // platform.system() == 'linux'
  device.serialConfig.baud = 9600;
  try {
    // A busy port (locked by another node) is retried: Python would have opened
    // it anyway and kept running. Any other open failure exits, as Python crashed.
    while (true) {
      try {
        device.openDevice();
        break;
      } catch (const cmr_port::PortBusyError & e) {
        if (!rclcpp::ok()) {throw;}
        RCLCPP_WARN_THROTTLE(
          number_publisher->get_logger(), *number_publisher->get_clock(), 5000,
          "%s; retrying", e.what());
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(number_publisher->get_logger(), "%s", ex.what());
    number_publisher.reset();
    rclcpp::shutdown();
    return 1;
  }
  readConfig(device);
  JY901SDataProcessor::onVarChanged().push_back(onUpdate);

  startRecord();
  // LoopReadThead: continuously request registers 0x30..0x58.
  std::atomic<bool> running{true};
  std::thread t([&device, &running]() {
      while (running.load()) {
        try {
          device.readReg(0x30, 41);
        } catch (const std::exception & ex) {
          // Python: an exception here ends the read thread (data stops, node keeps spinning).
          std::cerr << "IMU read loop stopped: " << ex.what() << std::endl;
          return;
        }
      }
    });

  // NOTE(port): Python spins in a separate thread and its non-daemon read threads
  // keep the process alive after Ctrl-C; here spin runs in main and shuts down cleanly.
  rclcpp::spin(number_publisher);

  running = false;
  if (t.joinable()) {
    t.join();
  }
  device.closeDevice();
  number_publisher.reset();
  rclcpp::shutdown();
  return 0;
}
