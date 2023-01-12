#include <ros_openpose/cameraReader.hpp>

namespace ros_openpose
{
  CameraReader::CameraReader(ros::NodeHandle& nh, const std::string& colorTopic)
    : mNh(nh), mColorTopic(colorTopic)
  {
    // std::cout << "[" << this << "] constructor called" << std::endl;
    subscribe();
  }

  CameraReader::CameraReader(const CameraReader& other)
    : mNh(other.mNh), mColorTopic(other.mColorTopic)
  {
    // std::cout << "[" << this << "] copy constructor called" << std::endl;
    subscribe();
  }

  CameraReader& CameraReader::operator=(const CameraReader& other)
  {
    // std::cout << "[" << this << "] copy assignment called" << std::endl;
    mNh = other.mNh;
    mColorTopic = other.mColorTopic;

    subscribe();
    return *this;
  }

  // we define the subscriber here. we are using TimeSynchronizer filter to receive the synchronized data
  inline void CameraReader::subscribe()
  {
    // create a subscriber to read the camera parameters from the ROS
    mColorImgSubscriber = mNh.subscribe<sensor_msgs::Image>(mColorTopic, 1, &CameraReader::colorImgCallback, this, ros::TransportHints().reliable().tcpNoDelay());
  }

  void CameraReader::colorImgCallback(const sensor_msgs::ImageConstPtr& colorMsg)
  {
    try
    {
      auto colorPtr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);

      // it is very important to lock the below assignment operation.
      // remember that we are accessing it from another thread too.
      std::lock_guard<std::mutex> lock(mMutex);
      mColorImage = colorPtr->image;
      mFrameNumber++;
    }
    catch (cv_bridge::Exception& e)
    {
      // display the error at most once per 10 seconds
      ROS_ERROR_THROTTLE(10, "cv_bridge exception %s at line number %d on function %s in file %s", e.what(), __LINE__,
                         __FUNCTION__, __FILE__);
    }
  }

}
