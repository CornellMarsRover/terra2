// C++ port of cmr_rtkgps/rover.py (console script `gps_rover`).
//
// Configures a u-blox ZED-F9P as an RTK rover, forwards RTCM corrections received
// over TCP from the basestation into the receiver, and publishes NAV position
// fixes as sensor_msgs/NavSatFix on /rtk/navsatfix_data.
#include <arpa/inet.h>
#include <dirent.h>
#include <glob.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "cmr_rtkgps_cpp/py_compat.hpp"
#include "cmr_rtkgps_cpp/serial_port.hpp"
#include "cmr_rtkgps_cpp/ubx.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace ubx = cmr_rtkgps_cpp;

namespace
{

std::vector<std::string> glob_sorted(const std::string & pattern)
{
  std::vector<std::string> out;
  glob_t g{};
  if (::glob(pattern.c_str(), 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; ++i) {
      out.emplace_back(g.gl_pathv[i]);
    }
  }
  ::globfree(&g);
  std::sort(out.begin(), out.end());
  return out;
}

bool is_dir(const std::string & path)
{
  struct stat st{};
  return ::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

std::string lower(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(), [](unsigned char c) {return std::tolower(c);});
  return s;
}

std::string py_list_repr(const std::vector<std::string> & v)
{
  std::string s = "[";
  for (size_t i = 0; i < v.size(); ++i) {
    if (i) {s += ", ";}
    s += "'" + v[i] + "'";
  }
  return s + "]";
}

}  // namespace

class GPSRover : public rclcpp::Node
{
public:
  GPSRover()
  : Node("gps_rover")
  {
    declare_parameter<std::string>("serial_path", "");

    // Navsatfix data publisher
    pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/rtk/navsatfix_data", 10);
    const std::string requested_serial_path = get_parameter("serial_path").as_string();

    // 1. Open local serial port for the rover's ZED-F9P. The port is locked
    // exclusively (flock + TIOCEXCL) before any byte is written. If another node
    // holds it, log (throttled) and retry: Python would have opened it anyway and
    // kept running. Autodetect skips busy devices and is re-run on each retry.
    while (true) {
      try {
        serial_path_ = !requested_serial_path.empty() ? requested_serial_path :
          detect_gps_serial_path();
        ser_.open(serial_path_, 115200);
        RCLCPP_INFO(
          get_logger(), "Rover serial port %s opened successfully.", serial_path_.c_str());
        break;
      } catch (const cmr_port::PortBusyError & e) {
        if (!rclcpp::ok()) {return;}
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s; retrying", e.what());
        std::this_thread::sleep_for(std::chrono::seconds(1));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", e.what());
        return;
      }
    }

    // 2. Configure local ZED-F9P for RTK rover
    configure_rover();

    // 3. Set up TCP socket to receive RTCM corrections
    // server_ip_ = "192.168.1.101";  // Basestation IP
    sock_ = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(server_port_));
    ::inet_pton(AF_INET, server_ip_.c_str(), &addr.sin_addr);
    // Blocking connect with the OS default timeout, as in Python.
    if (sock_ < 0 || ::connect(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      const int err = errno;
      RCLCPP_ERROR(
        get_logger(), "Failed to connect to basestation: [Errno %d] %s", err,
        std::strerror(err));
      if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
      }
      // NOTE(port): as in Python, without a basestation connection no position is
      // ever published (the read timer is only created after a successful connect).
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Connected to basestation at %s:%d", server_ip_.c_str(), server_port_);

    running_ = true;
    // Start a background thread to listen for corrections
    rx_thread_ = std::thread(&GPSRover::listen_for_corrections, this);

    // 4. NOTE(port): Python reads ONE message per 100 ms timer tick, blocking the
    // executor for up to the 1 s serial timeout. Here a dedicated thread parses
    // the stream continuously and publishes every NAV fix as it arrives.
    read_thread_ = std::thread(&GPSRover::read_loop, this);

    RCLCPP_INFO(get_logger(), "GPS Rover node started.");
  }

  ~GPSRover() override
  {
    running_ = false;
    if (sock_ >= 0) {
      ::shutdown(sock_, SHUT_RDWR);
    }
    if (rx_thread_.joinable()) {rx_thread_.join();}
    if (read_thread_.joinable()) {read_thread_.join();}
    if (sock_ >= 0) {::close(sock_);}
  }

private:
  std::string detect_gps_serial_path()
  {
    const std::string by_id_dir = "/dev/serial/by-id";
    if (is_dir(by_id_dir)) {
      const auto candidates = glob_sorted(by_id_dir + "/*");
      RCLCPP_INFO_ONCE(get_logger(), "Serial by-id candidates: %s", py_list_repr(candidates).c_str());
      std::vector<std::string> preferred, fallback;
      for (const auto & path : candidates) {
        const std::string base = lower(path.substr(path.find_last_of('/') + 1));
        if (base.find("fdcanusb") != std::string::npos) {
          continue;
        }
        bool match = false;
        for (const char * token : {"u-blox", "ublox", "zed", "gnss", "gps"}) {
          if (base.find(token) != std::string::npos) {match = true;}
        }
        (match ? preferred : fallback).push_back(path);
      }
      if (!preferred.empty()) {
        // If any GPS-named device exists, only GPS-named devices are ever used
        // (never fall back to an unrelated tty, e.g. the IMU or an Arduino).
        // Busy ones (locked by another node) are skipped; if all are busy, wait.
        // Only GPS-named devices are probed: opening a tty toggles DTR.
        std::vector<std::string> free_preferred;
        for (const auto & path : preferred) {
          if (cmr_port::SerialPort::try_probe(path)) {free_preferred.push_back(path);}
        }
        if (free_preferred.empty()) {
          throw cmr_port::PortBusyError(
                  "all GPS serial devices are busy (locked by another node): " +
                  py_list_repr(preferred));
        }
        preferred = free_preferred;
        RCLCPP_INFO(get_logger(), "Detected GPS serial device by-id: %s", preferred[0].c_str());
        return preferred[0];
      }
      if (!fallback.empty()) {
        RCLCPP_WARN(
          get_logger(), "No explicit GPS serial device found by-id, falling back to %s",
          fallback[0].c_str());
        return fallback[0];
      }
    }

    auto tty_candidates = glob_sorted("/dev/ttyACM*");
    const auto usb = glob_sorted("/dev/ttyUSB*");
    tty_candidates.insert(tty_candidates.end(), usb.begin(), usb.end());
    std::sort(tty_candidates.begin(), tty_candidates.end());
    if (!tty_candidates.empty()) {
      RCLCPP_WARN(
        get_logger(), "No by-id GPS match found, falling back to %s", tty_candidates[0].c_str());
      return tty_candidates[0];
    }

    RCLCPP_WARN(
      get_logger(),
      "No GPS serial device found; falling back to /dev/ttyACM2. "
      "Pass --ros-args -p serial_path:=/path/to/device to override.");
    return "/dev/ttyACM2";
  }

  void serial_write(const std::vector<uint8_t> & data)
  {
    std::lock_guard<std::mutex> lk(write_mutex_);
    ser_.write_all(data.data(), data.size());
  }

  // Configure the local ZED-F9P to accept RTCM input on USB and output UBX-NAV-PVT.
  void configure_rover()
  {
    const int transaction = 0;
    const int layers = 1;  // volatile RAM
    // NOTE(port): CFG-VALDEL only applies to the BBR/Flash layers, so with
    // layers=1 (RAM) the receiver NAKs this message. Sent anyway, as in Python.
    const std::vector<std::string> clear_keys = {
      "CFG_USBOUTPROT_NMEA", "CFG_USBOUTPROT_UBX", "CFG_MSGOUT_UBX_NAV_PVT_USB"};
    serial_write(ubx::config_del(layers, transaction, clear_keys));

    std::vector<std::pair<std::string, int64_t>> cfgData;
    cfgData.emplace_back("CFG_UART1_BAUDRATE", 115200);
    // (A) Rover mode
    cfgData.emplace_back("CFG_TMODE_MODE", 0);
    cfgData.emplace_back("CFG_TMODE_POS_TYPE", 0);
    // (C) Enable RTCM input on USB
    cfgData.emplace_back("CFG_USBINPROT_RTCM3X", 1);
    // (D) Enable UBX output on USB, so we get NAV_PVT
    cfgData.emplace_back("CFG_USBOUTPROT_UBX", 1);
    cfgData.emplace_back("CFG_USBOUTPROT_NMEA", 0);
    cfgData.emplace_back("CFG_USBOUTPROT_RTCM3X", 0);
    // (E) NAV_PVT output
    cfgData.emplace_back("CFG_MSGOUT_UBX_NAV_PVT_USB", 1);
    cfgData.emplace_back("CFG_MSGOUT_UBX_NAV_SVIN_USB", 0);
    cfgData.emplace_back("CFG_MSGOUT_NMEA_NAV2_ID_GGA_USB", 0);
    cfgData.emplace_back("CFG_MSGOUT_NMEA_NAV2_ID_GLL_USB", 0);
    cfgData.emplace_back("CFG_MSGOUT_NMEA_NAV2_ID_GNS_USB", 0);
    cfgData.emplace_back("CFG_MSGOUT_NMEA_NAV2_ID_GSA_USB", 0);
    cfgData.emplace_back("CFG_RATE_MEAS", 200);  // 5 Hz (200 ms)

    serial_write(ubx::config_set(layers, transaction, cfgData));
    RCLCPP_INFO(get_logger(), "Configured rover for RTCM input + UBX NAV_PVT output.");
  }

  // Continuously receive RTCM data from the basestation (TCP) and feed it to the GNSS.
  void listen_for_corrections()
  {
    uint8_t buf[2048];
    while (rclcpp::ok() && running_) {
      const ssize_t n = ::recv(sock_, buf, sizeof(buf), 0);
      if (n < 0) {
        if (errno == EINTR) {continue;}
        if (!running_) {break;}
        // Python logs and retries immediately; same here.
        RCLCPP_ERROR(
          get_logger(), "Error receiving UDP data: [Errno %d] %s", errno, std::strerror(errno));
        continue;
      }
      if (n == 0) {
        // NOTE(port): on basestation disconnect Python's recv() keeps returning b''
        // and the loop spins forever logging "Received RTCM Correction". Stop instead.
        if (running_) {
          RCLCPP_ERROR(get_logger(), "Error receiving UDP data: basestation closed the connection");
        }
        break;
      }
      try {
        serial_write(std::vector<uint8_t>(buf, buf + n));
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Error writing RTCM data to GNSS: %s", e.what());
      }
      RCLCPP_INFO(get_logger(), "Received RTCM Correction");
    }
  }

  // pyubx2 _read_bytes(): read exactly n bytes with the 1 s port timeout.
  // Returns 0 = EOF/timeout (nothing read), 1 = ok, -1 = truncated.
  int read_bytes(uint8_t * buf, size_t n)
  {
    const size_t got = ser_.read_exact(buf, n, 1000);
    if (got == 0) {return 0;}
    if (got < n) {
      std::cerr << "Serial stream terminated unexpectedly. " << n << " bytes requested, " <<
        got << " bytes returned." << std::endl;
      return -1;
    }
    return 1;
  }

  // Mirrors UBXReader.read(): sync on UBX / NMEA / RTCM3 headers, skip junk.
  void read_loop()
  {
    while (running_ && rclcpp::ok()) {
      try {
        uint8_t b1;
        if (read_bytes(&b1, 1) != 1) {continue;}
        if (b1 != 0xB5 && b1 != 0x24 && b1 != 0xD3) {continue;}
        uint8_t b2;
        if (read_bytes(&b2, 1) != 1) {continue;}

        if (b1 == ubx::UBX_SYNC1 && b2 == ubx::UBX_SYNC2) {
          handle_ubx();
        } else if (b1 == 0x24 && (b2 == 'G' || b2 == 'P')) {
          // NMEA sentence: read to LF and discard (never a NAV identity).
          std::string line;
          if (!ser_.read_line(line, 1000)) {continue;}
        } else if (b1 == 0xD3 && (b2 & ~0x03) == 0) {
          // RTCM3 frame: skip payload + CRC24 (never a NAV identity).
          uint8_t b3;
          if (read_bytes(&b3, 1) != 1) {continue;}
          const size_t size = b3 | (static_cast<size_t>(b2) << 8);
          std::vector<uint8_t> skip(size + 3);
          read_bytes(skip.data(), skip.size());
        } else {
          char msg[64];
          std::snprintf(msg, sizeof(msg), "Unknown protocol header b'\\x%02x\\x%02x'.", b1, b2);
          std::cerr << msg << std::endl;
        }
      } catch (const std::exception & e) {
        // Python's timer swallows every exception from the read.
        RCLCPP_DEBUG(get_logger(), "Error reading local GPS data: %s", e.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  void handle_ubx()
  {
    uint8_t hdr[4];
    if (read_bytes(hdr, 4) != 1) {return;}
    const uint8_t cls = hdr[0], id = hdr[1];
    const size_t len = hdr[2] | (static_cast<size_t>(hdr[3]) << 8);
    std::vector<uint8_t> body(len + 2);
    if (read_bytes(body.data(), body.size()) != 1) {return;}

    // Validate checksum over class, id, length and payload.
    std::vector<uint8_t> ck_data(hdr, hdr + 4);
    ck_data.insert(ck_data.end(), body.begin(), body.begin() + len);
    uint8_t a, b;
    ubx::ubx_checksum(ck_data.data(), ck_data.size(), a, b);
    if (a != body[len] || b != body[len + 1]) {
      std::cerr << "Message checksum invalid" << std::endl;
      return;
    }

    const std::vector<uint8_t> payload(body.begin(), body.begin() + len);
    const auto pos = ubx::decode_nav_position(cls, id, payload);
    if (!pos) {
      return;
    }

    sensor_msgs::msg::NavSatFix msg;
    msg.latitude = pos->lat;
    msg.longitude = pos->lon;
    // NOTE(port): Python stores hAcc**2/1000 in altitude and calls it "variance in
    // m^2"; with hAcc in mm that is actually mm^2/1000. Kept as-is.
    msg.altitude = (pos->hAcc * pos->hAcc) / 1000.0;
    msg.header.stamp = get_clock()->now();
    pub_->publish(msg);

    const std::string hacc = pos->hAcc_is_int ?
      std::to_string(static_cast<uint64_t>(pos->hAcc)) : cmr_port::py_float_repr(pos->hAcc);
    RCLCPP_INFO(
      get_logger(), "Rover: lat=%.7f, lon=%.7f, hAcc=%s mm", pos->lat, pos->lon, hacc.c_str());
  }

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
  std::string serial_path_;
  cmr_port::SerialPort ser_;
  std::mutex write_mutex_;

  const std::string server_ip_ = "192.168.1.103";  // Basestation IP or  84
  const int server_port_ = 4990;                    // Same port as the basestation
  int sock_{-1};

  std::atomic<bool> running_{false};
  std::thread rx_thread_;
  std::thread read_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<GPSRover>();
    rclcpp::spin(node);
  }
  rclcpp::shutdown();
  return 0;
}
