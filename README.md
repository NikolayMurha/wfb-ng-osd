This project started from https://github.com/TobiasBales/PlayuavOSD.git

Supported platforms:
-------------------
  * Raspberry Pi 0-3 -- use hardware overlay mode (OpenVG)
  * Radxa Zero 3W/3E -- use hardware overlay mode (libdrm)
  * OrangePi 5 -- use hardware overlay mode (libdrm)
  * Any other Linux with X11/Wayland and GPU -- use GStreamer OpenGL mixer
  * macOS with Homebrew GStreamer -- use GStreamer OpenGL mixer (`mode=gst`)

Supported autopilots:
---------------------

   * PX4 -- full support
   * Ardupilot -- should work, but not tested
   * any mavlink-based -- should work with small fixes

Building:
---------

0. Initialize submodules:
  * `git submodule update --init --recursive`

1. Build for Linux (X11 or Wayland) (native build):
  * `apt-get install gstreamer1.0-tools pkg-config libgstreamer1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-bad gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev`
  * `make osd`

2. Build for macOS (Homebrew) (native build):
  * `brew install pkg-config glib gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly`
  * `make osd mode=gst`

3. Build for Raspberry PI 0-3 (OpenVG) (native build):
  * `make osd mode=rpi3`

4. Build for Radxa or OrangePi (libdrm) (native build):
  * `apt-get install libdrm-dev pkg-config`
  * `make osd mode=rockchip`

5. Build Docker images for Linux x86/64 and ARM64:
  * `make osd_image_all`
  * images: `wfb-ng-osd:latest-amd64`, `wfb-ng-osd:latest-arm64`
  * optional multi-arch push: `make osd_image_push OSD_DOCKER_IMAGE=<repo/name> OSD_DOCKER_TAG=<tag>`

6. Build binary artifacts into `dist/` (macOS ARM + Linux amd64/arm64):
  * script: `./build_dist.sh`
  * outputs: `dist/mac.arm/osd.gst`, `dist/x86/osd.gst`, `dist/aarch64/osd.gst`
  * optional modes: `DIST_LINUX_MODES="gst rockchip" ./build_dist.sh`

Running:
--------

Default mavlink port is UDP 14551.
Default RTP video port is UDP 5600.
Optional input URI via `-R`:
  * optimized paths for `srt://...` and `rtsp(s)://...`
  * generic URI fallback via GStreamer `uridecodebin` (for example `file://...`, `http://...`, `https://...`), if matching plugins are installed
Optional MAVLink TX heartbeat:
  * `-H` enable periodic OSD heartbeat transmission to the selected target
  * `-i <ms>` heartbeat interval in milliseconds (default: `1000`)
Optional OSD element disable via CLI:
  * `-D <items>` comma-separated list of groups/items to disable at runtime
  * groups: `gps`, `wfb`
  * items: `wfb_state`, `wfb_alarm`, `gps_status`, `gps_coords`, `gps2`, `home`, `cwh`, `warnings`
  * examples: `-D gps,wfb`, `-D wfb_state`, `-D gps_status,gps_coords`

   * For `mode=gst`, run `./osd.gst`
   * You should got screen like this:
     ![gstreamer](scr1.png)


Screenshots:
------------
![px4](scr2.png)

Wiki: [![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/svpcom/wfb-ng-osd)
------------
