// C++ port of cmr_controller_remote/connect.py (console script `connect_node`).
//
// Receives joystick packets over non-blocking UDP sockets and republishes them
// as ROS messages every 20 ms. Same node name, topics, message types, QoS
// depths and packet layout as the Python original.

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "cmr_msgs/msg/controller_reading.hpp"
#include "cmr_msgs/msg/mini_arm_degree.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace
{

constexpr const char * UDP_IP = "0.0.0.0";

constexpr int UDP_PORT_DRIVES = 5010;
constexpr int UDP_PORT_DRIVES_LOCAL = 5005;
constexpr int UDP_PORT_ARM = 5020;
constexpr int UDP_PORT_MINI_ARM = 5030;

constexpr size_t DRIVE_PACKET_LENGTH = 13;
// struct.calcsize("ffffff") with native alignment.
constexpr size_t MINI_ARM_PACKET_LENGTH = 6 * sizeof(float);

constexpr int DIRECTION_NEUTRAL = 8;  // directions["neutral"]

// Python recvfrom(1024): longer datagrams are truncated to 1024 bytes.
constexpr size_t RECV_BUFSIZE = 1024;

int create_udp_socket(int port)
{
  int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    throw std::runtime_error(std::string("socket(): ") + std::strerror(errno));
  }
  int one = 1;
  if (::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) < 0) {
    int err = errno;
    ::close(fd);
    throw std::runtime_error(std::string("setsockopt(SO_REUSEADDR): ") + std::strerror(err));
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  ::inet_pton(AF_INET, UDP_IP, &addr.sin_addr);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    int err = errno;
    ::close(fd);
    throw std::runtime_error(
            "bind(" + std::string(UDP_IP) + ":" + std::to_string(port) + "): " +
            std::strerror(err));
  }
  return fd;
}

std::string describe_socket(int fd, int port)
{
  return "<socket fd=" + std::to_string(fd) + ", family=AF_INET, type=SOCK_DGRAM, laddr=('" +
         std::string(UDP_IP) + "', " + std::to_string(port) + ")>";
}

struct Packet
{
  std::vector<uint8_t> data;
  std::string ip;
  int port{0};
};

struct Sockets
{
  // Insertion order of the Python dict: 5010 then 5005.
  std::vector<std::pair<int, int>> drive;  // (port, fd)
  int arm{-1};
  int mini_arm{-1};

  ~Sockets()
  {
    for (auto & p : drive) {
      if (p.second >= 0) {::close(p.second);}
    }
    if (arm >= 0) {::close(arm);}
    if (mini_arm >= 0) {::close(mini_arm);}
  }
};

// Module-level socket creation (done at import time in Python, before rclpy.init()).
std::shared_ptr<Sockets> create_sockets()
{
  auto s = std::make_shared<Sockets>();
  std::cout << "Connecting Drive UDP sockets" << std::endl;
  s->drive.emplace_back(UDP_PORT_DRIVES, create_udp_socket(UDP_PORT_DRIVES));
  s->drive.emplace_back(UDP_PORT_DRIVES_LOCAL, create_udp_socket(UDP_PORT_DRIVES_LOCAL));
  for (const auto & p : s->drive) {
    std::cout << "Drive port " << p.first << ": " << describe_socket(p.second, p.first) <<
      std::endl;
  }
  std::cout << "Connecting Arm" << std::endl;
  s->arm = create_udp_socket(UDP_PORT_ARM);
  std::cout << describe_socket(s->arm, UDP_PORT_ARM) << std::endl;
  std::cout << "Connecting Mini Arm" << std::endl;
  s->mini_arm = create_udp_socket(UDP_PORT_MINI_ARM);
  std::cout << describe_socket(s->mini_arm, UDP_PORT_MINI_ARM) << std::endl;
  return s;
}

std::string format_py_list(const std::vector<std::string> & items)
{
  std::string out = "[";
  for (size_t i = 0; i < items.size(); ++i) {
    if (i) {out += ", ";}
    out += "'" + items[i] + "'";
  }
  out += "]";
  return out;
}

}  // namespace

class CmdVelPublisher : public rclcpp::Node
{
public:
  explicit CmdVelPublisher(std::shared_ptr<Sockets> sockets)
  : Node("cmd_vel_publisher"), sockets_(std::move(sockets))
  {
    publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/controller/drives/axes", 10);
    button_publisher_ = create_publisher<cmr_msgs::msg::ControllerReading>(
      "/controller/drives/buttons", 10);
    arm_publisher_ = create_publisher<sensor_msgs::msg::Joy>("/arm_controller/cmd_vel", 10);
    arm_button_publisher_ = create_publisher<cmr_msgs::msg::ControllerReading>(
      "/arm_controller/cmd_buttons", 10);
    mini_arm_publisher_ = create_publisher<cmr_msgs::msg::MiniArmDegree>(
      "/mini_arm_controller/cmd_pos", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&CmdVelPublisher::publish_msg, this));

    std::vector<int> ports;
    for (const auto & p : sockets_->drive) {ports.push_back(p.first);}
    std::sort(ports.begin(), ports.end());
    std::string drive_ports;
    for (size_t i = 0; i < ports.size(); ++i) {
      if (i) {drive_ports += ", ";}
      drive_ports += std::to_string(ports[i]);
    }
    RCLCPP_INFO(
      get_logger(),
      "Listening for drive UDP packets on 0.0.0.0 ports [%s], arm on %d, mini-arm on %d",
      drive_ports.c_str(), UDP_PORT_ARM, UDP_PORT_MINI_ARM);
    RCLCPP_INFO(
      get_logger(),
      "Joystick UDP bridge publishes manual drive topics only: "
      "/controller/drives/axes and /controller/drives/buttons");
  }

private:
  void publish_msg()
  {
    for (const auto & [port, sock] : sockets_->drive) {
      auto packet = recv_latest(sock);
      if (!packet) {
        continue;
      }
      if (!validate_drive_packet(*packet, port, "drive")) {
        continue;
      }
      const auto [lx, ly, rx, ry] = parse_controller_data(packet->data);
      auto [button_array, dpad] = parse_button_data(packet->data);

      publisher_->publish(
        create_twist_stamped({lx, ly, rx, ry}, "udp:" + std::to_string(port)));
      button_publisher_->publish(create_button_message(button_array, dpad));

      log_drive_button_changes(button_array);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Drive packet from %s:%d on %d -> lx=%.3f ly=%.3f rx=%.3f ry=%.3f",
        packet->ip.c_str(), packet->port, port, lx, ly, rx, ry);
    }

    auto arm_packet = recv_latest(sockets_->arm);
    if (arm_packet) {
      if (validate_drive_packet(*arm_packet, UDP_PORT_ARM, "arm")) {
        const auto [lx, ly, rx, ry] = parse_controller_data(arm_packet->data);
        auto [arm_button_array, dpad] = parse_button_data(arm_packet->data);
        const std::array<double, 12> v = {
          lx,
          ly,
          static_cast<double>(arm_button_array[2]),
          rx,
          ry,
          static_cast<double>(arm_button_array[3]),
          static_cast<double>(arm_button_array[5]),
          static_cast<double>(arm_button_array[6]),
          static_cast<double>(arm_button_array[7]),
          static_cast<double>(arm_button_array[4]),
          static_cast<double>(arm_button_array[0]),
          static_cast<double>(arm_button_array[1]),
        };
        arm_publisher_->publish(
          create_arm_joy_message(v, "udp:" + std::to_string(UDP_PORT_ARM)));
        arm_button_publisher_->publish(create_button_message(arm_button_array, dpad));
      }
    }

    auto mini_arm_packet = recv_latest(sockets_->mini_arm);
    if (mini_arm_packet) {
      if (mini_arm_packet->data.size() != MINI_ARM_PACKET_LENGTH) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring mini-arm packet from %s:%d on %d: expected %zu bytes, got %zu",
          mini_arm_packet->ip.c_str(), mini_arm_packet->port, UDP_PORT_MINI_ARM,
          MINI_ARM_PACKET_LENGTH, mini_arm_packet->data.size());
        return;
      }
      // struct.unpack("ffffff") -> native byte order floats.
      std::array<float, 6> mini{};
      std::memcpy(mini.data(), mini_arm_packet->data.data(), MINI_ARM_PACKET_LENGTH);
      mini_arm_publisher_->publish(create_mini_arm_message(mini));
    }
  }

  // Drain the socket and return only the most recent datagram (or nothing).
  std::optional<Packet> recv_latest(int fd)
  {
    std::optional<Packet> latest;
    uint8_t buf[RECV_BUFSIZE];
    while (true) {
      sockaddr_in from{};
      socklen_t fromlen = sizeof(from);
      ssize_t n = ::recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &fromlen);
      if (n < 0) {
        if (errno == EINTR) {
          continue;
        }
        // Python only catches BlockingIOError (EAGAIN). Any other socket error would
        // propagate and kill the node; here we just stop draining for this cycle.
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_ERROR(get_logger(), "recvfrom failed: %s", std::strerror(errno));
        }
        return latest;
      }
      Packet p;
      p.data.assign(buf, buf + n);
      char ip[INET_ADDRSTRLEN] = {0};
      ::inet_ntop(AF_INET, &from.sin_addr, ip, sizeof(ip));
      p.ip = ip;
      p.port = ntohs(from.sin_port);
      latest = std::move(p);
    }
  }

  bool validate_drive_packet(const Packet & packet, int port, const char * kind)
  {
    if (packet.data.size() != DRIVE_PACKET_LENGTH) {
      // Single call site, shared by drive and arm packets (same as Python).
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring %s packet from %s:%d on %d: expected exactly %zu bytes, got %zu",
        kind, packet.ip.c_str(), packet.port, port, DRIVE_PACKET_LENGTH, packet.data.size());
      return false;
    }
    return true;
  }

  cmr_msgs::msg::MiniArmDegree create_mini_arm_message(const std::array<float, 6> & d)
  {
    cmr_msgs::msg::MiniArmDegree msg;
    msg.base_angle = d[0];
    msg.shoulder_angle = d[1];
    msg.elbow_angle = d[2];
    msg.first_rotate_angle = d[4];
    msg.tilt_angle = d[3];
    msg.second_rotate_angle = d[5];
    return msg;
  }

  static int axis_map_for_arm(double axis)
  {
    if (axis >= 1) {return -1;}
    if (axis <= -1) {return 1;}
    return 0;
  }

  geometry_msgs::msg::TwistStamped create_twist_stamped(
    const std::array<double, 4> & velocities, const std::string & frame_id)
  {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = get_clock()->now();
    twist_msg.header.frame_id = frame_id;
    twist_msg.twist.linear.x = velocities[0];
    twist_msg.twist.linear.y = velocities[1];
    twist_msg.twist.angular.x = velocities[2];
    twist_msg.twist.angular.y = velocities[3];
    return twist_msg;
  }

  sensor_msgs::msg::Joy create_arm_joy_message(
    const std::array<double, 12> & v, const std::string & frame_id)
  {
    sensor_msgs::msg::Joy joy_msg;
    const double lx = v[0], ly = v[1], rx = v[3], ry = v[4];
    const double l2_raw = v[2], r2_raw = v[5];
    const int x = static_cast<int>(v[6]);
    const int circle = static_cast<int>(v[7]);
    const int triangle = static_cast<int>(v[8]);
    const int square = static_cast<int>(v[9]);
    const int l1 = static_cast<int>(v[10]);
    const int r1 = static_cast<int>(v[11]);

    const int lx_val = axis_map_for_arm(lx);
    const int ly_val = axis_map_for_arm(ly);
    const int rx_val = axis_map_for_arm(rx);
    const int ry_val = axis_map_for_arm(ry);

    const int l2 = (l2_raw != 0.0) ? -1 : 1;
    const int r2 = (r2_raw != 0.0) ? -1 : 1;

    joy_msg.header.stamp = get_clock()->now();
    joy_msg.header.frame_id = frame_id.empty() ? "joy" : frame_id;
    joy_msg.axes = {
      static_cast<float>(lx_val),
      static_cast<float>(ly_val),
      static_cast<float>(l2),
      static_cast<float>(rx_val),
      static_cast<float>(ry_val),
      static_cast<float>(r2),
      0.0f,
      0.0f,
    };
    joy_msg.buttons = {x, circle, triangle, square, l1, r1, 0, 0, 0, 0, 0, 0, 0};
    return joy_msg;
  }

  cmr_msgs::msg::ControllerReading create_button_message(
    const std::vector<int32_t> & button_array, int32_t dpad)
  {
    cmr_msgs::msg::ControllerReading button_msg;
    button_msg.button_array = button_array;
    button_msg.dpad = dpad;
    return button_msg;
  }

  static constexpr std::array<const char *, 8> kButtonNames = {
    "L1", "R1", "L2", "R2", "SQUARE", "X", "CIRCLE", "TRIANGLE"};

  static std::array<int, 8> decode_drive_buttons(const std::vector<int32_t> & button_array)
  {
    std::array<int, 8> state{};
    for (size_t i = 0; i < kButtonNames.size(); ++i) {
      if (i < button_array.size()) {
        state[i] = button_array[i];
      }
    }
    return state;
  }

  void log_drive_button_changes(const std::vector<int32_t> & button_array)
  {
    const auto state = decode_drive_buttons(button_array);
    if (!last_drive_button_state_) {
      last_drive_button_state_ = state;
      std::vector<std::string> pressed;
      for (size_t i = 0; i < state.size(); ++i) {
        if (state[i]) {pressed.emplace_back(kButtonNames[i]);}
      }
      if (pressed.empty()) {pressed.emplace_back("none");}
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Drive buttons initialized: pressed=%s L2=%d R2=%d",
        format_py_list(pressed).c_str(), state[2], state[3]);
      return;
    }

    std::string changed;
    for (size_t i = 0; i < state.size(); ++i) {
      const int value = state[i];
      const int previous = (*last_drive_button_state_)[i];
      if (value != previous) {
        std::string name = kButtonNames[i];
        std::string entry;
        if (name == "L2" || name == "R2") {
          entry = name + "=" + std::to_string(value);
        } else {
          entry = name + "=" + (value ? "pressed" : "released");
        }
        if (!changed.empty()) {changed += ", ";}
        changed += entry;
      }
    }
    if (!changed.empty()) {
      RCLCPP_INFO(get_logger(), "Drive button update: %s", changed.c_str());
    }
    last_drive_button_state_ = state;
  }

  static double scale_value(
    double value, double old_min, double old_max, double new_min, double new_max)
  {
    return ((value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min;
  }

  static std::array<double, 4> parse_controller_data(const std::vector<uint8_t> & raw)
  {
    return {
      scale_value(raw[0], 0, 255, -2.5, 2.5),
      scale_value(raw[1], 0, 255, -2.5, 2.5),
      scale_value(raw[2], 0, 255, -2.5, 2.5),
      scale_value(raw[3], 0, 255, -2.5, 2.5),
    };
  }

  static std::pair<std::vector<int32_t>, int32_t> parse_button_data(
    const std::vector<uint8_t> & raw)
  {
    std::vector<int32_t> buttons(raw.begin() + 4, raw.begin() + 12);
    const int32_t dpad_raw = raw[12];
    const int32_t dpad = (dpad_raw >= 0 && dpad_raw <= 8) ? dpad_raw : DIRECTION_NEUTRAL;
    return {buttons, dpad};
  }

  std::shared_ptr<Sockets> sockets_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Publisher<cmr_msgs::msg::ControllerReading>::SharedPtr button_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr arm_publisher_;
  rclcpp::Publisher<cmr_msgs::msg::ControllerReading>::SharedPtr arm_button_publisher_;
  rclcpp::Publisher<cmr_msgs::msg::MiniArmDegree>::SharedPtr mini_arm_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<std::array<int, 8>> last_drive_button_state_;
};

int main(int argc, char ** argv)
{
  // Python binds the sockets at module import, before rclpy.init().
  auto sockets = create_sockets();

  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelPublisher>(sockets);
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
