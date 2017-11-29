#pragma once
#include <vlc/vlc.h>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class RosLibVlc {
public:

  RosLibVlc() = delete; // Better explicit than backward compatible :)
  static RosLibVlc* getInstance(const ros::NodeHandle& nh_);
  ~RosLibVlc();
  
  void spin();


private:

  // Sad enough, but I can't find a better way to create a C callback than this
  // ( Basically the problem is that it would require C closures...:
  //     http://www.haible.de/bruno/packages-ffcall.html
  //     https://en.wikipedia.org/wiki/Blocks_%28C_language_extension%29
  // )
  RosLibVlc(const ros::NodeHandle& nh_);
  // probably it's better to use a scoped ptr
  static RosLibVlc* instance;


  ros::NodeHandle nh;
  ros::Publisher pub;
  
  
  // VLC pointers
  libvlc_instance_t *vlcInstance;
  libvlc_media_player_t *mp;
  uint8_t *videoBuffer;
  
  boost::mutex img_lock;
  
  static void cbVideoPrerender(void *p_video_data, uint8_t **pp_pixel_buffer, int size);
  static void cbVideoPostrender(void *p_video_data, uint8_t *p_pixel_buffer, int width, int height, int pixel_pitch, int size, int64_t pts);

};