// LD_PRELOAD shim (test only): let TIOCGSERIAL/TIOCSSERIAL succeed on a pty,
// and make tty writes block forever while /tmp/ptyshim_stall exists (simulates
// a stalled adapter write).
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdarg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#include <unistd.h>
int ioctl(int fd, unsigned long req, ...) {
  static int (*real)(int, unsigned long, ...) = 0;
  if (!real) real = dlsym(RTLD_NEXT, "ioctl");
  va_list ap; va_start(ap, req); void *arg = va_arg(ap, void *); va_end(ap);
  int rc = real(fd, req, arg);
  if (rc < 0 && (req == TIOCGSERIAL || req == TIOCSSERIAL)) {
    if (req == TIOCGSERIAL && arg) memset(arg, 0, sizeof(struct serial_struct));
    errno = 0;
    return 0;
  }
  return rc;
}

ssize_t write(int fd, const void *buf, size_t n) {
  static ssize_t (*real)(int, const void *, size_t) = 0;
  if (!real) real = dlsym(RTLD_NEXT, "write");
  if (fd > 2 && isatty(fd) && access("/tmp/ptyshim_stall", F_OK) == 0) {
    for (;;) sleep(1000);
  }
  return real(fd, buf, n);
}
