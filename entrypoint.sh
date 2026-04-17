#!/bin/bash

export DISPLAY=:1
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR

# Start virtual framebuffer with all extensions Qt needs
Xvfb :1 -screen 0 1920x1080x24 +extension RANDR +extension GLX +extension RENDER &
sleep 2

# Window manager
openbox &

# VNC server sharing the Xvfb display
x11vnc -display :1 -forever -nopw -shared -rfbport 5900 &

# noVNC web access
websockify --web /usr/share/novnc 6080 localhost:5900 &

echo ""
echo "============================================"
echo "  Open in your browser:"
echo "  http://localhost:6080/vnc.html"
echo "============================================"
echo ""

exec bash
