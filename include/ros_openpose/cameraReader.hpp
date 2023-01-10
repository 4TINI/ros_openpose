#pragma once

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// CV brigge header
#include <cv_bridge/cv_bridge.h>

// OpenCV header
#include <opencv2/core/core.hpp>

// c++ headers
#include <mutex>
#include <vector>

// define a few datatype
typedef unsigned long long ullong;

namespace ros_openpose
{
  class CameraReader
  {
  private:
    cv::Mat mColorImage;
    cv::Mat mColorImageUsed;
    std::string mColorTopic;
    std::mutex mMutex;
    ros::NodeHandle mNh;
    ros::Subscriber mCamInfoSubscriber;
    ros::Subscriber mColorImgSubscriber;

    ullong mFrameNumber = 0ULL;

    inline void subscribe();
    void colorImgCallback(const sensor_msgs::ImageConstPtr& colorMsg);

  public:
    // we don't want to instantiate using deafult constructor
    CameraReader() = delete;

    // copy constructor
    CameraReader(const CameraReader& other);

    // copy assignment operator
    CameraReader& operator=(const CameraReader& other);

    // main constructor
    CameraReader(ros::NodeHandle& nh, const std::string& colorTopic);

    // we are okay with default destructor
    ~CameraReader() = default;

    // returns the current frame number
    // the frame number starts from 0 and increments
    // by 1 each time a frame (color) is received
    ullong getFrameNumber()
    {
      return mFrameNumber;
    }

    // returns the latest color frame from camera
    // it locks the color frame. remember that we
    // are just passing the pointer instead of copying whole data
    const cv::Mat& getColorFrame()
    {
      mMutex.lock();
      mColorImageUsed = mColorImage;
      mMutex.unlock();
      return mColorImageUsed;
    }

  };
}
