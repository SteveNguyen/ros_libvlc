#include "ros_libvlc/RosLibVlc.h"

#include <sstream>

RosLibVlc::RosLibVlc(const ros::NodeHandle &nh_) : nh(nh_), videoBuffer(nullptr)
{

  pub = nh.advertise<sensor_msgs::Image>("image", 100);

  // VLC options
  std::stringstream smem_options_stream;

  smem_options_stream << "#transcode{vcodec=RV24}" // Make stream smem compatible
                      << ":"
                      // Insert addresses of callbacks
                      << "smem{video-prerender-callback=" << (long long int)(intptr_t)(void *)&cbVideoPrerender
                      << ",video-postrender-callback=" << (long long int)(intptr_t)(void *)&cbVideoPostrender
                      << ",no-time-sync"
                      << "}";

  std::string smem_options = smem_options_stream.str();
  const char *const vlc_args[] = {
      "-I", "dummy",                 // Don't use any interface
      "--ignore-config",             // Don't use VLC's config
      "--extraintf=logger",          // Log anything
      "--verbose=2",                 // Be much more verbose then normal for debugging purpose
      "--sout", smem_options.c_str() // Stream to memory
  };

  ROS_INFO_STREAM("vlc args" << smem_options.c_str());

  // We launch VLC
  vlcInstance = libvlc_new(sizeof(vlc_args) / sizeof(vlc_args[0]), vlc_args);

  libvlc_media_t *media;

  std::string source_url;

  if (!nh.getParam("src", source_url))
  {
    ROS_FATAL("ros_libvlc needs a ~src parameter to work.");
    ros::shutdown();
  }

  media = libvlc_media_new_location(vlcInstance, source_url.c_str()); //, "v4l2:///dev/video0");, "http://walterebert.com/playground/video/hls/sintel-trailer.m3u8"

  // create a media play playing environment
  mp = libvlc_media_player_new_from_media(media);

  // no need to keep the media now
  libvlc_media_release(media);

  //mp = libvlc_media_player_new(vlcInstance);
  libvlc_media_player_play(mp);
}

RosLibVlc *RosLibVlc::instance = nullptr;

RosLibVlc *RosLibVlc::getInstance(const ros::NodeHandle &nh_)
{
  ROS_FATAL_COND(instance, "This class is not supposed to be constructed more than once.");

  if (!instance)
  {
    instance = new RosLibVlc(nh_);
  }
  return instance;
}

RosLibVlc::~RosLibVlc()
{
}

void RosLibVlc::spin()
{
  ros::spin();
}

void RosLibVlc::cbVideoPrerender(void *p_video_data, uint8_t **pp_pixel_buffer, int size)
{
  //   ROS_INFO("Yeah cbVideoPrerender got called.");
  instance->img_lock.lock();
  instance->videoBuffer = (uint8_t *)realloc(instance->videoBuffer, size);
  *pp_pixel_buffer = instance->videoBuffer;
}

void RosLibVlc::cbVideoPostrender(void *p_video_data, uint8_t *p_pixel_buffer, int width, int height, int pixel_pitch, int size, int64_t pts)
{
  //   ROS_INFO("Yeah cbVideoPostrender got called.");
  instance->img_lock.unlock();

  sensor_msgs::Image msg;
  msg.header.stamp = ros::Time::now(); // FIXME

  msg.encoding = "bgr8";
  msg.width = width;
  msg.height = height;
  msg.step = sizeof(uint8_t) * 3 * width;

  msg.data.resize(size);
  std::copy(p_pixel_buffer, p_pixel_buffer + size, msg.data.begin());

  instance->pub.publish(msg);
}
