#!/usr/bin/env python3
import ctypes,re,subprocess
x=ctypes.CDLL('libX11.so.6');x.XOpenDisplay.restype=ctypes.c_void_p;d=x.XOpenDisplay(None)
x.XMoveResizeWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint];x.XIconifyWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int];x.XFlush.argtypes=[ctypes.c_void_p]
for line in subprocess.check_output(['xwininfo','-root','-tree'],text=True).splitlines():
    m=re.search(r'(0x[0-9a-f]+) "([^"]+)"',line)
    if not m:continue
    wid=int(m[1],16);name=m[2]
    if name.endswith(' - RViz'):x.XMoveResizeWindow(d,wid,1020,65,890,850)
    elif name=='Gazebo':x.XMoveResizeWindow(d,wid,75,65,925,850)
    elif 'Visual Studio Code' in name:x.XIconifyWindow(d,wid,0)
x.XFlush(d)
