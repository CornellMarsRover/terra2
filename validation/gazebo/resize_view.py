"""Resize the Gazebo window on the isolated Xvfb display, without desktop access."""
import ctypes as c
import time

x = c.CDLL('libX11.so.6')
x.XOpenDisplay.restype = c.c_void_p
x.XDefaultRootWindow.argtypes = [c.c_void_p]
x.XDefaultRootWindow.restype = c.c_ulong
x.XQueryTree.argtypes = [c.c_void_p, c.c_ulong, c.POINTER(c.c_ulong),
                        c.POINTER(c.c_ulong), c.POINTER(c.POINTER(c.c_ulong)),
                        c.POINTER(c.c_uint)]
x.XFetchName.argtypes = [c.c_void_p, c.c_ulong, c.POINTER(c.c_char_p)]
x.XResizeWindow.argtypes = [c.c_void_p, c.c_ulong, c.c_uint, c.c_uint]
x.XFlush.argtypes = [c.c_void_p]
x.XCloseDisplay.argtypes = [c.c_void_p]
x.XFree.argtypes = [c.c_void_p]
display = x.XOpenDisplay(None)
if not display:
    raise SystemExit('Cannot open simulation display')
try:
    for _ in range(30):
        root, parent, count = c.c_ulong(), c.c_ulong(), c.c_uint()
        children = c.POINTER(c.c_ulong)()
        x.XQueryTree(display, x.XDefaultRootWindow(display), c.byref(root),
                     c.byref(parent), c.byref(children), c.byref(count))
        found = False
        for window in list(children[:count.value]):
            name = c.c_char_p()
            x.XFetchName(display, window, c.byref(name))
            if name.value and b'gazebo' in name.value.lower():
                x.XResizeWindow(display, window, 780, 720)
                x.XFlush(display)
                found = True
            if name:
                x.XFree(name)
        if children:
            x.XFree(children)
        if found:
            break
        time.sleep(1)
    else:
        raise SystemExit('Gazebo window not found within 30 seconds')
finally:
    x.XCloseDisplay(display)
