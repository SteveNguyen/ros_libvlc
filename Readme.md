# ROS libvlc wrapper

Have you ever thought "I can easily show this camera stream in vlc, why is there no ROS driver for it?". Then this is exactly the right place for you.

This package provides a simple way to convert any video input(-stream) that is supported by `vlc` into ros `sensor_msgs/Image` topics. It might not be the best integration of your camera one can imagine, but it mostly gets the job done `¯\_(ツ)_/¯`

## Install

```bash
cd <your-catkin-workspace>/src
git clone https://github.com/dreuter/ros_libvlc
sudo apt-get install libvlc-dev libvlccore-dev
cd ..
catkin_make # or catkin build or whatever you fancy
```

## Example

You can try a small demo of this project by running

```bash
roslaunch ros_libvlc test.launch
```

Pro tip: Open the `/screen_capture/image` stream and enjoy the [Droste effect](https://en.wikipedia.org/wiki/Droste_effect).

## Usage

```bash
rosrun ros_libvlc ros_libvlc _src:="<your video source>"
# where <your video source> is any string, that vlc accepts as a stream url
# URL syntax:
#   file:///path/file              Plain media file
#   http://host[:port]/file        HTTP URL
#   ftp://host[:port]/file         FTP URL
#   mms://host[:port]/file         MMS URL
#   screen://                      Screen capture
#   dvd://[device]                 DVD device
#   vcd://[device]                 VCD device
#   cdda://[device]                Audio CD device
#   udp://[[<source address>]@[<bind address>][:<bind port>]]
#                                  UDP stream sent by a streaming server
#   vlc://pause:<seconds>          Pause the playlist for a certain time
#   vlc://quit                     Special item to quit VLC
```