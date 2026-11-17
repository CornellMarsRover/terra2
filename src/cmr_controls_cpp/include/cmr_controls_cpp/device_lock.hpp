// Exclusive fdcanusb ownership, copied from cmr_rovernet_cpp (moteus_bus.hpp /
// moteus_bus.cpp: DeviceLock, PortBusyError, autodetect skipping busy adapters)
// so two nodes can never share one CAN adapter.
#pragma once

#include <fcntl.h>
#include <glob.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace cmr_controls_cpp
{

/// Thrown when another process holds the exclusive lock on a CAN adapter.
class PortBusyError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// Exclusive, non-blocking flock(2) on a device node, held for the object's
/// lifetime. flock is per open-file-description, so always release (destroy)
/// the old lock before re-acquiring from this process.
class DeviceLock
{
public:
  explicit DeviceLock(const std::string & path)
  {
    int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0 && (errno == EACCES || errno == EISDIR)) {
      fd = ::open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    }
    if (fd < 0) {
      if (errno == EBUSY) {  // TIOCEXCL held by another process
        throw PortBusyError("port busy: " + path + " is held exclusively by another process");
      }
      throw std::runtime_error("could not open " + path + ": " + std::strerror(errno));
    }
    while (::flock(fd, LOCK_EX | LOCK_NB) != 0) {
      if (errno == EINTR) {
        continue;
      }
      const int err = errno;
      ::close(fd);
      if (err == EWOULDBLOCK) {
        throw PortBusyError(
                "port busy: " + path + " is locked by another process (another node owns it)");
      }
      throw std::runtime_error("flock " + path + ": " + std::strerror(err));
    }
    fd_ = fd;
  }

  ~DeviceLock()
  {
    if (fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
      ::close(fd_);
    }
  }

  DeviceLock(const DeviceLock &) = delete;
  DeviceLock & operator=(const DeviceLock &) = delete;

  /// true if @p path could be locked right now (the lock is released again).
  static bool try_probe(const std::string & path)
  {
    try {
      DeviceLock lock(path);
      return true;
    } catch (const std::exception &) {
      return false;
    }
  }

private:
  int fd_ = -1;
};

/// moteus.Fdcanusb() autodetect candidates, in its search order:
/// /dev/fdcanusb, then sorted /dev/serial/by-id/*fdcanusb*. Nothing is opened.
inline std::vector<std::string> fdcanusb_candidates()
{
  std::vector<std::string> candidates;
  if (::access("/dev/fdcanusb", F_OK) == 0) {
    candidates.emplace_back("/dev/fdcanusb");
  }
  glob_t g{};
  if (::glob("/dev/serial/by-id/*fdcanusb*", 0, nullptr, &g) == 0) {
    std::vector<std::string> by_id;
    for (size_t i = 0; i < g.gl_pathc; ++i) {by_id.emplace_back(g.gl_pathv[i]);}
    std::sort(by_id.begin(), by_id.end());
    candidates.insert(candidates.end(), by_id.begin(), by_id.end());
  }
  ::globfree(&g);
  return candidates;
}

}  // namespace cmr_controls_cpp
